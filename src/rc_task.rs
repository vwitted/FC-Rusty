// rc_task.rs — Embassy async task for reading CRSF from a UART
//
// This module wires the CRSF parser to an actual UART peripheral
// using Embassy's async UART driver. It runs as a spawned task
// and publishes parsed RC channel data and link statistics via
// Embassy Signals so other tasks (mainly the control loop) can
// read the latest values without blocking.
//
// CRSF between receiver and FC uses:
//   - 416666 baud, 8N1, non-inverted
//   - Rx-only from the FC's perspective (we just listen)
//   - Frames arrive at ~150 Hz (every ~6.6ms)
//   - Max frame size 64 bytes
//
// We use DMA-backed `read()` to avoid busy-waiting on bytes.
// The UART peripheral and DMA channel are passed in at spawn time.

// ---- NOTE ----
// This file won't compile standalone — it needs the full Embassy
// project setup with embassy-stm32, the right chip feature, and
// linker scripts. It's written to show you the structure and how
// everything connects. You'd drop this into your fc-firmware/src/
// alongside main.rs.

use embassy_stm32::usart::{Config as UartConfig, UartRx};
use embassy_stm32::peripherals;
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};

use crate::drivers::crsf::{CrsfEvent, CrsfParser, LinkStatistics, RcChannels};

// ---- Shared signals ----
// These are how the RC task communicates with the control task.
//
// Signal is "latest value wins" — if the control task hasn't
// read the previous value yet, it gets overwritten. This is
// exactly what we want: the control loop should always see
// the most recent RC data, not queue up stale frames.

/// Latest RC channel data, updated ~150 Hz
pub static RC_CHANNELS: Signal<CriticalSectionRawMutex, RcChannels> = Signal::new();

/// Latest link statistics, updated ~150 Hz (alternates with channels)
pub static RC_LINK: Signal<CriticalSectionRawMutex, LinkStatistics> = Signal::new();

/// Link health — the control task reads this to detect failsafe.
/// Updated every time we receive any valid CRSF frame.
pub static RC_LAST_SEEN: Signal<CriticalSectionRawMutex, Instant> = Signal::new();

/// CRSF baud rate (416666 is the spec default for FC<->Rx)
const CRSF_BAUDRATE: u32 = 416_666;

/// How we configure the UART for CRSF.
///
/// Call this to get a UartConfig you can pass to Embassy's
/// UART constructor. Separated out so main.rs stays clean.
pub fn crsf_uart_config() -> UartConfig {
    let mut config = UartConfig::default();
    config.baudrate = CRSF_BAUDRATE;
    // 8N1 is the default, so we don't need to change
    // data_bits, stop_bits, or parity.
    config
}

/// The RC receiver task.
///
/// Reads bytes from the UART, feeds them through the CRSF parser,
/// and signals fresh data to other tasks.
///
/// # Arguments
/// * `uart_rx` — the Rx half of a UART peripheral, already configured
///   with `crsf_uart_config()`. The Tx half is unused (receive-only).
///
/// # Example (in main.rs)
/// ```ignore
/// // In your #[embassy_executor::main] fn:
/// let uart = embassy_stm32::usart::UartRx::new(
///     p.USART1,        // whichever USART your Rx is on
///     Irqs,            // interrupt binding
///     p.PA10,          // Rx pin
///     p.DMA2_CH2,      // DMA channel for Rx
///     rc_task::crsf_uart_config(),
/// ).unwrap();
///
/// spawner.spawn(rc_task::run(uart)).unwrap();
/// ```
///
/// # Panics
/// This task runs forever and never returns. If the UART errors
/// (framing, overrun), it logs the error and keeps going — we
/// don't want a single bad byte to kill RC input.
#[embassy_executor::task]
pub async fn run(
    mut uart_rx: UartRx<'static, embassy_stm32::mode::Async>,
) {
    let mut parser = CrsfParser::new();

    // Read buffer — we read in small chunks. CRSF frames are
    // max 64 bytes and arrive every ~6.6ms at 150 Hz. At 416666
    // baud that's ~1.5ms of wire time per frame, so a 64-byte
    // buffer is plenty.
    let mut buf = [0u8; 64];

    loop {
        // Async read: this yields the task until DMA delivers
        // bytes. No CPU time wasted polling.
        match uart_rx.read(&mut buf).await {
            Ok(()) => {
                // read() fills the entire buffer, so process
                // all bytes we got.
                for &byte in &buf {
                    if let Some(event) = parser.push_byte(byte) {
                        // Mark that we've seen a valid frame
                        RC_LAST_SEEN.signal(Instant::now());

                        match event {
                            CrsfEvent::Channels(ch) => {
                                RC_CHANNELS.signal(ch);
                            }
                            CrsfEvent::Link(link) => {
                                RC_LINK.signal(link);
                            }
                        }
                    }
                }
            }
            Err(_e) => {
                // UART error (framing, overrun, noise).
                // On a real FC you'd increment an error counter.
                // Don't panic — just keep trying. The parser
                // will re-sync on the next valid sync byte.

                // Small delay to avoid tight-looping on persistent errors
                embassy_time::Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

// ---- Failsafe helper ----

/// Milliseconds since last valid CRSF frame.
///
/// Returns u32::MAX if no frame has ever been received.
pub fn rc_last_seen_ms() -> u32 {
    match RC_LAST_SEEN.try_take() {
        Some(last) => {
            let ms = (Instant::now() - last).as_millis() as u32;
            RC_LAST_SEEN.signal(last); // put it back
            ms
        }
        None => u32::MAX,
    }
}

/// Check if we've lost the RC link.
///
/// Returns true if we haven't received a valid CRSF frame
/// within the given timeout. The control task should call this
/// every loop iteration and engage failsafe if it returns true.
///
/// Typical timeout: 500ms-1000ms (CRSF sends frames every ~6.6ms,
/// so missing ~75-150 frames means the link is truly gone).
pub fn rc_link_lost(timeout: Duration) -> bool {
    // If we've never received a frame, the signal won't have
    // been set yet, so try_take returns None -> assume lost.
    match RC_LAST_SEEN.try_take() {
        Some(last) => {
            let lost = Instant::now() - last > timeout;
            // Put it back so other callers can check too
            RC_LAST_SEEN.signal(last);
            lost
        }
        None => true,
    }
}
