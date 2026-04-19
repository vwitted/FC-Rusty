// logger.rs — defmt global logger over USART1 TX (PA9)
//
// The SpeedyBee F7 V3 exposes no SWD pads, so probe-rs + RTT isn't
// an option. Instead we pipe defmt frames out of USART1 TX on PA9
// — the T1 pad (VTX) on the board. Wire a USB-UART dongle to that
// pad and decode with `defmt-print` on the host:
//
//   defmt-print -e target/thumbv7em-none-eabihf/debug/fc-firmware \
//               serial --path /dev/ttyUSB0 --baud 115200
//
// Implementation notes:
//   * We bypass Embassy's USART driver and poke USART1 registers
//     directly. The logger is called from many tasks and interrupts;
//     keeping it raw avoids lock contention with any Embassy-owned
//     DMA transfers and keeps the Logger trait implementation
//     entirely synchronous.
//   * `init_usart1()` MUST be called after `embassy_stm32::init()`
//     has configured the clock tree — it assumes APB2 = 108 MHz.
//   * Defmt calls made before `init_usart1()` silently drop their
//     bytes (guarded by the INITIALIZED flag).

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

// ---- USART1 registers (STM32F7 — note: different layout from F4) ----
const USART1_BASE: usize = 0x4001_1000;
const USART1_CR1: *mut u32 = (USART1_BASE + 0x00) as *mut u32;
const USART1_BRR: *mut u32 = (USART1_BASE + 0x0C) as *mut u32;
const USART1_ISR: *mut u32 = (USART1_BASE + 0x1C) as *mut u32;
const USART1_TDR: *mut u32 = (USART1_BASE + 0x28) as *mut u32;

const CR1_UE: u32 = 1 << 0;
const CR1_TE: u32 = 1 << 3;
const ISR_TXE: u32 = 1 << 7; // TX data register empty
const ISR_TC:  u32 = 1 << 6; // Transmission complete

// ---- RCC and GPIOA registers ----
const RCC_BASE: usize = 0x4002_3800;
const RCC_AHB1ENR: *mut u32 = (RCC_BASE + 0x30) as *mut u32;
const RCC_APB2ENR: *mut u32 = (RCC_BASE + 0x44) as *mut u32;
const RCC_AHB1ENR_GPIOAEN: u32 = 1 << 0;
const RCC_APB2ENR_USART1EN: u32 = 1 << 4;

const GPIOA_BASE: usize = 0x4002_0000;
const GPIOA_MODER:   *mut u32 = (GPIOA_BASE + 0x00) as *mut u32;
const GPIOA_OTYPER:  *mut u32 = (GPIOA_BASE + 0x04) as *mut u32;
const GPIOA_OSPEEDR: *mut u32 = (GPIOA_BASE + 0x08) as *mut u32;
const GPIOA_PUPDR:   *mut u32 = (GPIOA_BASE + 0x0C) as *mut u32;
const GPIOA_AFRH:    *mut u32 = (GPIOA_BASE + 0x24) as *mut u32;

/// Configure USART1 TX on PA9 at 115200 baud 8N1.
///
/// Call once after `embassy_stm32::init()` — this function assumes
/// the clock tree is already set up with APB2 = 108 MHz.
pub fn init_usart1() {
    const APB2_HZ: u32 = 108_000_000;
    const BAUD: u32 = 115_200;
    // BRR = fck / baud for OVER8=0 (the reset default).
    // 108 MHz / 115200 ≈ 937.5 → 938 (rounded) → actual 115 139 baud (0.05% err).
    let brr = (APB2_HZ + BAUD / 2) / BAUD;

    unsafe {
        // Enable peripheral clocks
        let ahb1 = core::ptr::read_volatile(RCC_AHB1ENR);
        core::ptr::write_volatile(RCC_AHB1ENR, ahb1 | RCC_AHB1ENR_GPIOAEN);
        let apb2 = core::ptr::read_volatile(RCC_APB2ENR);
        core::ptr::write_volatile(RCC_APB2ENR, apb2 | RCC_APB2ENR_USART1EN);

        // PA9 → alternate function mode (MODER = 0b10)
        let moder = core::ptr::read_volatile(GPIOA_MODER);
        let moder = (moder & !(0b11u32 << (9 * 2))) | (0b10u32 << (9 * 2));
        core::ptr::write_volatile(GPIOA_MODER, moder);

        // PA9 push-pull (OTYPER bit=0), high speed (OSPEEDR=0b11), no pull
        let otyper = core::ptr::read_volatile(GPIOA_OTYPER);
        core::ptr::write_volatile(GPIOA_OTYPER, otyper & !(1u32 << 9));
        let ospeedr = core::ptr::read_volatile(GPIOA_OSPEEDR);
        let ospeedr = (ospeedr & !(0b11u32 << (9 * 2))) | (0b11u32 << (9 * 2));
        core::ptr::write_volatile(GPIOA_OSPEEDR, ospeedr);
        let pupdr = core::ptr::read_volatile(GPIOA_PUPDR);
        core::ptr::write_volatile(GPIOA_PUPDR, pupdr & !(0b11u32 << (9 * 2)));

        // PA9 AF7 = USART1 (pin 9 lives in AFRH bits [7:4])
        let afrh = core::ptr::read_volatile(GPIOA_AFRH);
        let afrh = (afrh & !(0b1111u32 << ((9 - 8) * 4))) | (7u32 << ((9 - 8) * 4));
        core::ptr::write_volatile(GPIOA_AFRH, afrh);

        // USART1: set BRR, then enable (UE) and transmitter (TE).
        core::ptr::write_volatile(USART1_BRR, brr);
        core::ptr::write_volatile(USART1_CR1, CR1_UE | CR1_TE);
    }

    INITIALIZED.store(true, Ordering::Release);
}

// ---- Low-level byte push ----

static INITIALIZED: AtomicBool = AtomicBool::new(false);

#[inline(always)]
fn putc(byte: u8) {
    if !INITIALIZED.load(Ordering::Relaxed) {
        return; // pre-init defmt calls are dropped
    }
    unsafe {
        while (core::ptr::read_volatile(USART1_ISR) & ISR_TXE) == 0 {}
        core::ptr::write_volatile(USART1_TDR, byte as u32);
    }
}

fn do_write(bytes: &[u8]) {
    for &b in bytes {
        putc(b);
    }
}

// ---- defmt global logger ----

static TAKEN: AtomicBool = AtomicBool::new(false);
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut RESTORE: MaybeUninit<critical_section::RestoreState> = MaybeUninit::uninit();

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // Safety: `critical_section::acquire` disables interrupts,
        // so nothing else on this core can race us until `release`.
        let restore = unsafe { critical_section::acquire() };

        // Nested defmt call (e.g. from a panic inside a log) — bail
        // to avoid corrupting the encoder state.
        if TAKEN.load(Ordering::Relaxed) {
            unsafe { critical_section::release(restore) };
            return;
        }
        TAKEN.store(true, Ordering::Relaxed);

        unsafe {
            (*(&raw mut RESTORE)).write(restore);
            (*(&raw mut ENCODER)).start_frame(do_write);
        }
    }

    unsafe fn flush() {
        // Busy-wait until the last byte has left the shift register.
        while unsafe { core::ptr::read_volatile(USART1_ISR) } & ISR_TC == 0 {}
    }

    unsafe fn release() {
        unsafe { (*(&raw mut ENCODER)).end_frame(do_write); }
        TAKEN.store(false, Ordering::Relaxed);
        let restore = unsafe { (*(&raw const RESTORE)).assume_init() };
        unsafe { critical_section::release(restore) };
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe { (*(&raw mut ENCODER)).write(bytes, do_write); }
    }
}
