// logger.rs — defmt global logger over USART6 TX (PC6)
//
// Target board: DAKEFPVH743
// USART6 TX is on PC6 (AF7). This maps to SERIAL6 / T6 pad — a
// physically accessible user/GP port, leaving all protocol-assigned
// ports free:
//   USART1 → GPS, USART2 → MAVLink/Telem, USART3 → ESC telem,
//   UART4 → VTX/DisplayPort, UART5 → RC input, UART7/8 → user.
//
// Implementation notes:
//   * We bypass Embassy's UART driver and poke USART6 registers
//     directly using the `pac` module. The logger is called from many
//     tasks and interrupts; keeping it raw avoids lock contention with
//     any Embassy-owned DMA transfers and keeps the Logger trait
//     implementation entirely synchronous.
//   * `init_usart6()` MUST be called after `embassy_stm32::init()`
//     has configured the clock tree — it assumes APB2 = 120 MHz.
//   * Defmt calls made before `init_usart6()` silently drop their
//     bytes (guarded by the INITIALIZED flag).

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::pac;

/// Configure USART6 TX on PC6 at 115200 baud 8N1.
///
/// Call once after `embassy_stm32::init()` — this function assumes
/// the clock tree is already set up with APB2 = 120 MHz.
pub fn init_usart6() {
    // USART6 is on APB2 (not APB1 like UART4–8).
    const APB2_HZ: u32 = 120_000_000;
    const BAUD: u32 = 115_200;
    // BRR = fck / baud for OVER8=0 (the reset default).
    let brr = (APB2_HZ + BAUD / 2) / BAUD;

    // Enable peripheral clocks using PAC
    // For STM32H7, GPIOC is on AHB4. USART6 is on APB2.
    pac::RCC.ahb4enr().modify(|w| w.set_gpiocen(true));
    pac::RCC.apb2enr().modify(|w| w.set_usart6en(true));

    let gpioc = pac::GPIOC;

    // PC6 -> AF7 (USART6_TX)
    // PC6 is pin 6, which lives in AFRL (afr(0)), position 6.
    gpioc.moder().modify(|w| w.set_moder(6, pac::gpio::vals::Moder::ALTERNATE));
    gpioc.otyper().modify(|w| w.set_ot(6, pac::gpio::vals::Ot::PUSH_PULL));
    gpioc.ospeedr().modify(|w| w.set_ospeedr(6, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));
    gpioc.pupdr().modify(|w| w.set_pupdr(6, pac::gpio::vals::Pupdr::FLOATING));
    gpioc.afr(0).modify(|w| w.set_afr(6, 7)); // AF7

    // USART6
    let usart = pac::USART6;
    usart.brr().write(|w| w.set_brr(brr as u16));
    usart.cr1().write(|w| {
        w.set_ue(true);
        w.set_te(true);
    });

    INITIALIZED.store(true, Ordering::Release);
}

// ---- Low-level byte push ----

static INITIALIZED: AtomicBool = AtomicBool::new(false);

#[inline(always)]
fn putc(byte: u8) {
    if !INITIALIZED.load(Ordering::Relaxed) {
        return; // pre-init defmt calls are dropped
    }
    let usart = pac::USART6;
    while !usart.isr().read().txe() {}
    usart.tdr().write(|w| w.set_dr(byte as u16));
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
        let restore = unsafe { critical_section::acquire() };

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
        let usart = pac::USART6;
        while !usart.isr().read().tc() {}
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
