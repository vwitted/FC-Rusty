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
//
// Buffering. A log call encodes its frame into RING inside a critical
// section (all interrupts masked), which costs the encoding time only.
// The USART6 TXE interrupt, at the lowest priority, drains RING one byte
// per TXE. The logger used to busy-wait on TXE (~87 us/byte at 115200)
// inside that critical section, so every line masked interrupts for
// milliseconds -- including the fast executor in main.rs.
//
// Three cases still transmit synchronously, busy-waiting on TXE inside
// the critical section exactly as before:
//   * RING is full. Output sustained above the line rate (~11.5 KB/s),
//     such as the blackbox dump, falls back to blocking; bytes are never
//     dropped, because the dump's CSV must arrive intact.
//   * Interrupts were already masked when the log call began (the
//     panic handler disables them before printing), so the drain
//     interrupt cannot run. The whole ring is emptied on release.
//   * `defmt::flush()`.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::pac;

/// Priority of the USART6 drain interrupt. Lowest, so draining the log
/// never delays anything else; it only preempts thread mode.
const DRAIN_PRIORITY: Priority = Priority::P15;

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

    // USART6. FIFO mode stays disabled (reset default), so TXE/TXEIE are
    // the single-byte transmit-register flags.
    let usart = pac::USART6;
    usart.brr().write(|w| w.set_brr(brr as u16));
    usart.cr1().write(|w| {
        w.set_ue(true);
        w.set_te(true);
    });

    interrupt::USART6.set_priority(DRAIN_PRIORITY);
    interrupt::USART6.unpend();
    // SAFETY: the handler below touches only RING and USART6, both of
    // which it accesses inside a critical section.
    unsafe { interrupt::USART6.enable() };

    INITIALIZED.store(true, Ordering::Release);
}

// ---- Ring buffer ----

/// Bytes of encoded defmt frames awaiting transmission. 8 KB is about
/// 0.7 s of line time at 115200 baud.
const RING_LEN: usize = 8192;

struct Ring {
    buf: [u8; RING_LEN],
    /// Next slot to write. Only touched inside a critical section.
    head: usize,
    /// Next byte to transmit. Only touched inside a critical section.
    tail: usize,
}

/// Interior-mutable holder for RING. Every access is made inside a
/// critical section (single-core, so that excludes all other access).
struct RingCell(UnsafeCell<Ring>);
// SAFETY: see RingCell -- accesses are serialised by critical sections.
unsafe impl Sync for RingCell {}

static RING: RingCell = RingCell(UnsafeCell::new(Ring {
    buf: [0; RING_LEN],
    head: 0,
    tail: 0,
}));

impl Ring {
    fn is_empty(&self) -> bool {
        self.head == self.tail
    }

    fn is_full(&self) -> bool {
        (self.head + 1) % RING_LEN == self.tail
    }

    /// Transmit one queued byte, busy-waiting for TXE. Caller ensures the
    /// ring is non-empty.
    fn send_one_blocking(&mut self) {
        let usart = pac::USART6;
        while !usart.isr().read().txe() {}
        usart.tdr().write(|w| w.set_dr(self.buf[self.tail] as u16));
        self.tail = (self.tail + 1) % RING_LEN;
    }

    fn drain_blocking(&mut self) {
        while !self.is_empty() {
            self.send_one_blocking();
        }
    }
}

/// Caller must hold a critical section.
unsafe fn ring() -> &'static mut Ring {
    unsafe { &mut *RING.0.get() }
}

#[interrupt]
fn USART6() {
    critical_section::with(|_| {
        // SAFETY: inside a critical section.
        let r = unsafe { ring() };
        let usart = pac::USART6;
        if !r.is_empty() && usart.isr().read().txe() {
            usart.tdr().write(|w| w.set_dr(r.buf[r.tail] as u16));
            r.tail = (r.tail + 1) % RING_LEN;
        }
        if r.is_empty() {
            usart.cr1().modify(|w| w.set_txeie(false));
        }
    });
}

// ---- Low-level byte push ----

static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Caller must hold a critical section (the logger's, from `acquire`).
fn do_write(bytes: &[u8]) {
    if !INITIALIZED.load(Ordering::Relaxed) {
        return; // pre-init defmt calls are dropped
    }
    // SAFETY: called only between acquire() and release().
    let r = unsafe { ring() };
    for &b in bytes {
        if r.is_full() {
            r.send_one_blocking();
        }
        r.buf[r.head] = b;
        r.head = (r.head + 1) % RING_LEN;
    }
    // TXE is set whenever the transmitter is idle, so enabling the
    // interrupt pends it at once; it runs when the critical section ends.
    pac::USART6.cr1().modify(|w| w.set_txeie(true));
}

// ---- defmt global logger ----

static TAKEN: AtomicBool = AtomicBool::new(false);
/// Interrupts were already masked when the current frame was acquired,
/// so the drain interrupt cannot run until something unmasks them --
/// which in the panic handler is never. release() then drains inline.
static MASKED_ON_ACQUIRE: AtomicBool = AtomicBool::new(false);
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut RESTORE: MaybeUninit<critical_section::RestoreState> = MaybeUninit::uninit();

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let masked = cortex_m::register::primask::read().is_inactive();
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            unsafe { critical_section::release(restore) };
            return;
        }
        TAKEN.store(true, Ordering::Relaxed);
        MASKED_ON_ACQUIRE.store(masked, Ordering::Relaxed);

        unsafe {
            (*(&raw mut RESTORE)).write(restore);
            (*(&raw mut ENCODER)).start_frame(do_write);
        }
    }

    unsafe fn flush() {
        // Called with the logger acquired; nesting the critical section
        // is allowed and keeps this sound if that ever changes.
        critical_section::with(|_| {
            // SAFETY: inside a critical section.
            unsafe { ring() }.drain_blocking();
        });
        let usart = pac::USART6;
        while !usart.isr().read().tc() {}
    }

    unsafe fn release() {
        unsafe { (*(&raw mut ENCODER)).end_frame(do_write); }
        if MASKED_ON_ACQUIRE.load(Ordering::Relaxed) && INITIALIZED.load(Ordering::Relaxed) {
            // SAFETY: still inside the logger's critical section.
            unsafe { ring() }.drain_blocking();
        }
        TAKEN.store(false, Ordering::Relaxed);
        let restore = unsafe { (*(&raw const RESTORE)).assume_init() };
        unsafe { critical_section::release(restore) };
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe { (*(&raw mut ENCODER)).write(bytes, do_write); }
    }
}
