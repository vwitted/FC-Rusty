// logger.rs — defmt global logger over USART3 TX (PB10)
//
// Target board is the Radiolink F722, which exposes a full set of
// UART pads; defmt gets its own dedicated UART (USART3 TX on PB10,
// T3 pad) so it no longer needs to share a peripheral with CRSF.
// Decode on the host with:
//
//   defmt-print -e target/thumbv7em-none-eabihf/debug/fc-firmware \
//               serial --path /dev/ttyUSB0 --baud 115200
//
// Implementation notes:
//   * We bypass Embassy's USART driver and poke USART3 registers
//     directly. The logger is called from many tasks and interrupts;
//     keeping it raw avoids lock contention with any Embassy-owned
//     DMA transfers and keeps the Logger trait implementation
//     entirely synchronous.
//   * `init_usart3()` MUST be called after `embassy_stm32::init()`
//     has configured the clock tree — it assumes APB1 = 54 MHz.
//   * Defmt calls made before `init_usart3()` silently drop their
//     bytes (guarded by the INITIALIZED flag).

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

// ---- USART3 registers (STM32F7 — note: different layout from F4) ----
const USART3_BASE: usize = 0x4000_4800;
const USART3_CR1: *mut u32 = (USART3_BASE + 0x00) as *mut u32;
const USART3_BRR: *mut u32 = (USART3_BASE + 0x0C) as *mut u32;
const USART3_ISR: *mut u32 = (USART3_BASE + 0x1C) as *mut u32;
const USART3_TDR: *mut u32 = (USART3_BASE + 0x28) as *mut u32;

const CR1_UE: u32 = 1 << 0;
const CR1_TE: u32 = 1 << 3;
const ISR_TXE: u32 = 1 << 7; // TX data register empty
const ISR_TC:  u32 = 1 << 6; // Transmission complete

// ---- RCC and GPIOB registers ----
const RCC_BASE: usize = 0x4002_3800;
const RCC_AHB1ENR: *mut u32 = (RCC_BASE + 0x30) as *mut u32;
const RCC_APB1ENR: *mut u32 = (RCC_BASE + 0x40) as *mut u32;
const RCC_AHB1ENR_GPIOBEN: u32 = 1 << 1;
const RCC_APB1ENR_USART3EN: u32 = 1 << 18;

const GPIOB_BASE: usize = 0x4002_0400;
const GPIOB_MODER:   *mut u32 = (GPIOB_BASE + 0x00) as *mut u32;
const GPIOB_OTYPER:  *mut u32 = (GPIOB_BASE + 0x04) as *mut u32;
const GPIOB_OSPEEDR: *mut u32 = (GPIOB_BASE + 0x08) as *mut u32;
const GPIOB_PUPDR:   *mut u32 = (GPIOB_BASE + 0x0C) as *mut u32;
const GPIOB_AFRH:    *mut u32 = (GPIOB_BASE + 0x24) as *mut u32;

/// Configure USART3 TX on PB10 at 115200 baud 8N1.
///
/// Call once after `embassy_stm32::init()` — this function assumes
/// the clock tree is already set up with APB1 = 54 MHz.
pub fn init_usart3() {
    const APB1_HZ: u32 = 54_000_000;
    const BAUD: u32 = 115_200;
    // BRR = fck / baud for OVER8=0 (the reset default).
    // 54 MHz / 115200 ≈ 468.75 → 469 (rounded) → 0.06% error.
    let brr = (APB1_HZ + BAUD / 2) / BAUD;

    unsafe {
        // Enable peripheral clocks
        let ahb1 = core::ptr::read_volatile(RCC_AHB1ENR);
        core::ptr::write_volatile(RCC_AHB1ENR, ahb1 | RCC_AHB1ENR_GPIOBEN);
        let apb1 = core::ptr::read_volatile(RCC_APB1ENR);
        core::ptr::write_volatile(RCC_APB1ENR, apb1 | RCC_APB1ENR_USART3EN);

        // PB10 → alternate function mode (MODER = 0b10)
        let moder = core::ptr::read_volatile(GPIOB_MODER);
        let moder = (moder & !(0b11u32 << (10 * 2))) | (0b10u32 << (10 * 2));
        core::ptr::write_volatile(GPIOB_MODER, moder);

        // PB10 push-pull (OTYPER bit=0), high speed (OSPEEDR=0b11), no pull
        let otyper = core::ptr::read_volatile(GPIOB_OTYPER);
        core::ptr::write_volatile(GPIOB_OTYPER, otyper & !(1u32 << 10));
        let ospeedr = core::ptr::read_volatile(GPIOB_OSPEEDR);
        let ospeedr = (ospeedr & !(0b11u32 << (10 * 2))) | (0b11u32 << (10 * 2));
        core::ptr::write_volatile(GPIOB_OSPEEDR, ospeedr);
        let pupdr = core::ptr::read_volatile(GPIOB_PUPDR);
        core::ptr::write_volatile(GPIOB_PUPDR, pupdr & !(0b11u32 << (10 * 2)));

        // PB10 AF7 = USART3 (pin 10 lives in AFRH bits [11:8])
        let afrh = core::ptr::read_volatile(GPIOB_AFRH);
        let afrh = (afrh & !(0b1111u32 << ((10 - 8) * 4))) | (7u32 << ((10 - 8) * 4));
        core::ptr::write_volatile(GPIOB_AFRH, afrh);

        // USART3: set BRR, then enable (UE) and transmitter (TE).
        core::ptr::write_volatile(USART3_BRR, brr);
        core::ptr::write_volatile(USART3_CR1, CR1_UE | CR1_TE);
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
        while (core::ptr::read_volatile(USART3_ISR) & ISR_TXE) == 0 {}
        core::ptr::write_volatile(USART3_TDR, byte as u32);
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
        while unsafe { core::ptr::read_volatile(USART3_ISR) } & ISR_TC == 0 {}
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
