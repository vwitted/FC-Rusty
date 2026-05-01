// logger.rs — defmt global logger over USART1 TX (PA9)
//
// Target board: DAKEFPVH743
// USART1 TX is on PA9 (T1 pad). USART3 was previously used but PD8
// is either not broken out or conflicts with ESC telemetry on this board.
//
// Implementation notes:
//   * We bypass Embassy's USART driver and poke USART1 registers
//     directly using the `pac` module. The logger is called from many
//     tasks and interrupts; keeping it raw avoids lock contention with
//     any Embassy-owned DMA transfers and keeps the Logger trait
//     implementation entirely synchronous.
//   * `init_usart1()` MUST be called after `embassy_stm32::init()`
//     has configured the clock tree — it assumes APB2 = 120 MHz.
//   * Defmt calls made before `init_usart1()` silently drop their
//     bytes (guarded by the INITIALIZED flag).

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::pac;

/// Configure USART1 TX on PA9 at 115200 baud 8N1.
///
/// Call once after `embassy_stm32::init()` — this function assumes
/// the clock tree is already set up with APB2 = 120 MHz.
pub fn init_usart1() {
    const APB2_HZ: u32 = 120_000_000;
    const BAUD: u32 = 115_200;
    // BRR = fck / baud for OVER8=0 (the reset default).
    let brr = (APB2_HZ + BAUD / 2) / BAUD;

    // Enable peripheral clocks using PAC
    // For STM32H7, GPIOA is on AHB4. USART1 is on APB2.
    pac::RCC.ahb4enr().modify(|w| w.set_gpioaen(true));
    pac::RCC.apb2enr().modify(|w| w.set_usart1en(true));

    let gpioa = pac::GPIOA;
    
    // PA9 -> AF7 (USART1_TX)
    gpioa.moder().modify(|w| w.set_moder(9, pac::gpio::vals::Moder::ALTERNATE));
    gpioa.otyper().modify(|w| w.set_ot(9, pac::gpio::vals::Ot::PUSH_PULL));
    gpioa.ospeedr().modify(|w| w.set_ospeedr(9, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));
    gpioa.pupdr().modify(|w| w.set_pupdr(9, pac::gpio::vals::Pupdr::FLOATING));
    gpioa.afr(1).modify(|w| w.set_afr(9 - 8, 7)); // AF7

    // USART1
    let usart = pac::USART1;
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
    let usart = pac::USART1;
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
        let usart = pac::USART1;
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
