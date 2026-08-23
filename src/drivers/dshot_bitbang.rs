// dshot_bitbang.rs — bit-banged DShot for the DAKEFPVH743.
//
// Reference: betaflight/src/platform/STM32/dshot_bitbang.c
//
// TIM1 is a pacer only: it drives no pin, it just generates a DMA request
// every state period. DMA writes 32-bit BSRR words to GPIOA, producing the
// waveform directly. The motor pins are plain GPIO throughout — never in
// alternate-function mode — which is why none of the compare-register or
// AF-handover failure modes of the retired timer-output-compare driver
// exist here. That driver (a port of BF's `pwm_output_dshot_hal.c`) never
// got bidir working and was removed in the 2026-08-08 cutover; it survives
// at `bbf2d2b:src/drivers/dshot_hw.rs` if the history is ever needed (that
// commit is an ancestor of this branch, so it is reachable in any clone —
// unlike a branch name, which may be local to one machine).
//
// M1..M4 are PA0..PA3, one port, so all four motors share one buffer and one
// DMA stream. Per-pin data lives in the middle state of each symbol.
//
// Timing (DShot600, TIM1 at 240 MHz): 3 states per symbol → 1.8 MHz pacer,
// ARR = 240e6/1.8e6 - 1 = 132. See TX_ARR/RX_ARR for why 600 and not 300.
//
// --- Step 1 probe result (embassy-stm32 0.4.0) ---
// `UpDma` lives at `embassy_stm32::timer::UpDma` (defined via the
// `dma_trait!(UpDma, BasicInstance)` macro in src/macros.rs) and is
// implemented per-chip by generated code from build.rs (the
// `("timer", "UP") -> crate::timer::UpDma` table entry), so whether
// `DMA2_CH2: UpDma<TIM1>` holds depends on the stm32-data peripheral tables,
// not anything hand-written here. For the H743VI it does hold. The trait
// method is `fn request(&self) -> Request`, implemented on the bare
// peripheral type (`impl UpDma<TIM1> for DMA2_CH2`), not on `Peri<DMA2_CH2>`.
// Calling `<DMA2_CH2 as UpDma<TIM1>>::request(&peri)` where `peri: &Peri<'_,
// DMA2_CH2>` compiles because plain deref coercion (`Peri: Deref<Target =
// DMA2_CH2>`) applies at ordinary function-argument positions, not just to
// method-call receivers. Confirmed by `cargo build --release --features
// motor-test` with a throwaway probe function before this file existed.

use embassy_stm32::Peri;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{DMA2_CH2, PA0, PA1, PA2, PA3, TIM1};

use super::dshot_bb_decode::{BbTelemetry, RX_BUF_LEN, decode};
use super::dshot_bb_frame::{BB_BUF_LEN, output_data_clear, output_data_init, output_data_set};
use super::dshot_frame::DshotFrame;

/// PA0..PA3 = M1..M4.
const PORT_MASK: u16 = 0b1111;
/// Motor index -> the GPIOA pad it drives, or `None` for "not driven".
/// Default is M1..M4 = PA0..PA3, all four active.
///
/// Set with `MOTOR_PIN_ORDER`: four digits, one per motor, naming the pad it
/// drives. `0` means that motor is left unassigned — its pad is never claimed
/// as an output, never flipped to input, and sits at high-Z on its pull-up, so
/// the ESC there sees no command and transmits nothing.
///
///     MOTOR_PIN_ORDER=1234   default, all four (same as unset)
///     MOTOR_PIN_ORDER=4231   swap M1 and M4, leave M2/M3 alone
///     MOTOR_PIN_ORDER=0004   drive M4 alone -- what ONLY_MOTOR=4 expands to
///     MOTOR_PIN_ORDER=0204   M2 and M4 only, M2 moved to the M2 pad
///
/// Two distinct uses, and it is worth keeping them apart:
///
///   * Remapping (a permutation) pairs with a *physical* lead swap. Moving the
///     software mapping alone cannot tell a bad pin from a bad ESC — the ESC
///     stays soldered to the same pad — but after re-plugging two ESC leads it
///     keeps motor numbering honest end to end, so throttle, telemetry decode
///     and the RX probe all still refer to the motor they claim to.
///
///   * Unassigning motors isolates a line electrically, which is what
///     separates "this ESC never answers" from "its answer is swamped by its
///     neighbours".
///
/// Unassigning is deliberately high-Z rather than a driven-idle output: an ESC
/// pulling the line low against a push-pull output driving it high shorts both
/// drivers together. High-Z can never fight.
///
/// A malformed value is a compile error, not a silent fallback to the default.
/// A typo'd bench mapping that quietly reverted would cost a whole session of
/// confused readings.
const MOTOR_PADS: [Option<u8>; 4] = parse_mapping();

/// The pad each motor is associated with, ignoring whether it is driven.
/// Unassigned motors keep their default pad so logs can still name it.
const MOTOR_PINS: [u8; 4] = {
    let mut out = [0u8; 4];
    let mut i = 0;
    while i < 4 {
        out[i] = match MOTOR_PADS[i] {
            Some(p) => p,
            None => i as u8,
        };
        i += 1;
    }
    out
};

const fn parse_mapping() -> [Option<u8>; 4] {
    // ONLY_MOTOR is shorthand for the common isolation case. Accepting both
    // knobs at once would leave which one wins undefined, so refuse.
    let order = option_env!("MOTOR_PIN_ORDER");
    let only = option_env!("ONLY_MOTOR");
    if order.is_some() && only.is_some() {
        panic!("set MOTOR_PIN_ORDER or ONLY_MOTOR, not both (ONLY_MOTOR=4 is MOTOR_PIN_ORDER=0004)");
    }

    if let Some(s) = only {
        let b = s.as_bytes();
        if b.len() != 1 || b[0] < b'1' || b[0] > b'4' {
            panic!("ONLY_MOTOR must be a single digit 1-4");
        }
        let m = (b[0] - b'1') as usize;
        let mut out = [None; 4];
        out[m] = Some(m as u8);
        return out;
    }

    let Some(s) = order else {
        return [Some(0), Some(1), Some(2), Some(3)];
    };
    let b = s.as_bytes();
    if b.len() != 4 {
        panic!("MOTOR_PIN_ORDER must be exactly 4 digits, e.g. MOTOR_PIN_ORDER=4231");
    }
    let mut out = [None; 4];
    let mut seen = [false; 4];
    let mut any = false;
    let mut i = 0;
    while i < 4 {
        let d = b[i];
        if d == b'0' {
            i += 1;
            continue; // unassigned
        }
        if d < b'1' || d > b'4' {
            panic!("MOTOR_PIN_ORDER digits must each be 0-4 (0 = motor not driven)");
        }
        let pad = (d - b'1') as usize;
        if seen[pad] {
            panic!("MOTOR_PIN_ORDER must not use the same pad twice");
        }
        seen[pad] = true;
        out[i] = Some(pad as u8);
        any = true;
        i += 1;
    }
    if !any {
        panic!("MOTOR_PIN_ORDER leaves every motor unassigned — nothing would be driven");
    }
    out
}

/// Is this motor driven in this build?
const fn is_active(i: usize) -> bool {
    MOTOR_PADS[i].is_some()
}

/// Port mask covering only the driven pads.
const ACTIVE_MASK: u16 = {
    let mut mask = 0u16;
    let mut i = 0;
    while i < 4 {
        if let Some(p) = MOTOR_PADS[i] {
            mask |= 1u16 << p;
        }
        i += 1;
    }
    mask
};

/// The pads driven in this build, in motor order.
fn active_pins() -> impl Iterator<Item = u8> {
    MOTOR_PADS.into_iter().flatten()
}

/// TIM1 counter period for the transmit pacer. 3 states per bit at 600 kbit/s
/// is a 1.8 MHz state rate; 240 MHz / 1.8 MHz - 1 = 132.3 → 132, giving
/// 240e6/133 = 1.8045 MHz (601.5 kbit/s, +0.25%). Frame = 51 states = 28.3 µs.
///
/// DShot600, not 300, because the flight loop runs at 8 kHz = 125 µs and a
/// bidirectional frame must fit inside that with room for MEKF and PID. At
/// DShot300 the frame was ~181 µs and the loop could not hold its rate.
/// ESCs auto-detect the bit rate, so this needs no ESC-side change.
const TX_ARR: u32 = 132;

/// TIM1 counter period for the receive pacer. The reply runs at 5/4 the DShot
/// bit rate and we oversample 3×, so 600 kHz × 5/4 × 3 = 2.25 MHz.
/// 240 MHz / 2.25 MHz - 1 = 105.7 → 106. (BF derives this as
/// `outputFreq * 5 * 2 * OVER_SAMPLE / 24`, which is `outputFreq × 5/4`.)
///
/// Window budget, and the thing to watch on the bench: the ESC's turnaround
/// is a fixed delay, so halving the sample period doubles how much of the
/// buffer it eats. At DShot300 the first falling edge landed consistently at
/// sample 26-27 = 23-24 µs; at this rate (445.8 ns/sample) that same delay is
/// ~53 samples, plus 21 GCR bits × 3 = 63, leaving ~24 of RX_BUF_LEN spare.
/// Comfortable, but if a slower ESC pushes the edge past ~sample 77 the reply
/// gets truncated and decodes as InvalidGcr — raise RX_BUF_LEN if that shows up.
const RX_ARR: u32 = 106;

/// Frame counts at which `send_and_receive` dumps an RX probe, so the bench
/// run reports what arrived without needing a scope.
///
/// These are deliberately late and repeating. The original one-shot at frame
/// 100 fired ~50 ms in, inside the ESC's power-on/beep sequence, and read
/// "0 transitions" on a link that was in fact healthy — a false "ESC silent"
/// that only went uncaught because another motor happened to work. At
/// LOOP_KHZ=2 the arming stream is 3 s = 6000 frames, so 8000 lands ~1 s into
/// the drive phase and every 4000 after is one probe per 2 s. Repeating also
/// catches *intermittent* replies, which a one-shot cannot.
const RX_PROBE_START: u32 = 8000;
const RX_PROBE_EVERY: u32 = 4000;

/// Cache-line-aligned TX buffer. H7 DMA requires 32-byte alignment for clean
/// cache maintenance; do not assume D-cache is disabled.
#[repr(C, align(32))]
struct TxBuf([u32; BB_BUF_LEN]);

static mut TX_BUF: TxBuf = TxBuf([0; BB_BUF_LEN]);

/// Cache-line-aligned RX buffer, same rationale as `TxBuf`.
#[repr(C, align(32))]
struct RxBuf([u16; RX_BUF_LEN]);

static mut RX_BUF: RxBuf = RxBuf([0; RX_BUF_LEN]);

pub struct DshotBitbang<'d> {
    dma: Peri<'d, DMA2_CH2>,
    bidir: bool,
    frame_count: u32,
}

impl<'d> DshotBitbang<'d> {
    pub fn new(
        _tim1: Peri<'d, TIM1>,
        dma: Peri<'d, DMA2_CH2>,
        _pa0: Peri<'d, PA0>,
        _pa1: Peri<'d, PA1>,
        _pa2: Peri<'d, PA2>,
        _pa3: Peri<'d, PA3>,
        bidir: bool,
    ) -> Self {
        // GPIO: plain push-pull output, low slew. ArduPilot notes bidir DShot
        // needs push-pull and below-MID2 slew to avoid noise on the
        // output→input transition; BF uses GPIO_SPEED_FREQ_LOW.
        // Pull direction follows the idle level, not a fixed PULL_UP: for
        // BIDIR=0 the line idles LOW, so pulling it UP while the pins are
        // still analog inputs (before MODER below) contradicts the idle
        // contract. Becomes load-bearing in Task 4, when the pin returns to
        // being an input for the telemetry read.
        let pupd = if bidir {
            pac::gpio::vals::Pupdr::PULL_UP
        } else {
            pac::gpio::vals::Pupdr::PULL_DOWN
        };
        pac::GPIOA.pupdr().modify(|w| {
            for p in MOTOR_PINS {
                w.set_pupdr(p as usize, pupd);
            }
        });
        pac::GPIOA.otyper().modify(|w| {
            for p in MOTOR_PINS {
                w.set_ot(p as usize, pac::gpio::vals::Ot::PUSH_PULL);
            }
        });
        pac::GPIOA.ospeedr().modify(|w| {
            for p in MOTOR_PINS {
                w.set_ospeedr(p as usize, pac::gpio::vals::Ospeedr::LOW_SPEED);
            }
        });
        // Idle level before the pins become outputs: bidir idles HIGH.
        set_idle_level(bidir);
        pac::GPIOA.moder().modify(|w| {
            for p in active_pins() {
                w.set_moder(p as usize, pac::gpio::vals::Moder::OUTPUT);
            }
        });

        // TIM1 as pacer: no output, update event only.
        //
        // Must go through embassy's enable_and_reset, not a raw APB2ENR
        // write: `modify()` reads-then-writes so there is no read-after-
        // write barrier, and the peripheral clock needs ~2 cycles to start
        // before register writes to it land (embassy's own RCC path does
        // this enable + a dummy read + dsb() for exactly this reason — see
        // embassy-stm32 rcc/mod.rs). Also resets TIM1 via APB2RSTR, which
        // matters here because TIM1 is an advanced-control timer with RCR
        // (a repetition counter TIM2 doesn't have): left nonzero from a
        // stale prior config, the update event — and therefore every DMA
        // request driving this waveform — would only fire once every
        // RCR+1 overflows.
        embassy_stm32::rcc::enable_and_reset::<TIM1>();
        pac::TIM1.cr1().write(|_| {}); // counter disabled while configuring
        pac::TIM1.psc().write_value(0);
        pac::TIM1
            .arr()
            .write_value(pac::timer::regs::ArrCore(TX_ARR));
        pac::TIM1.egr().write(|w| w.set_ug(true)); // load PSC/ARR
        pac::TIM1.cr1().modify(|w| w.set_cen(true));

        defmt::info!(
            "DShot bitbang init: TIM1 pacer ARR={=u32} bidir={=bool} port_mask={=u16:04b}",
            TX_ARR,
            bidir,
            ACTIVE_MASK,
        );
        // Announce any non-default mapping. A stale flash whose mapping does
        // not match what the bench notes assume is the kind of thing that
        // silently invalidates a whole session of readings.
        if ACTIVE_MASK != PORT_MASK || MOTOR_PINS[0] != 0 || MOTOR_PINS[1] != 1
            || MOTOR_PINS[2] != 2 || MOTOR_PINS[3] != 3
        {
            for i in 0..4usize {
                match MOTOR_PADS[i] {
                    Some(p) => defmt::warn!("motor map: M{=usize} -> PA{=u8}", i + 1, p),
                    None => defmt::warn!("motor map: M{=usize} -> unassigned (high-Z)", i + 1),
                }
            }
        }

        Self { dma, bidir, frame_count: 0 }
    }

    /// Emit one frame on all four motors.
    pub async fn send(&mut self, frames: [DshotFrame; 4]) {
        use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
        use embassy_stm32::timer::UpDma;

        // SAFETY: single owner; the DMA transfer is awaited to completion
        // before this function returns, so no aliasing outlives the borrow.
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF.0) };

        if self.frame_count == 0 {
            output_data_init(buf, ACTIVE_MASK, self.bidir);
        } else {
            output_data_clear(buf);
        }
        for (i, pin) in MOTOR_PINS.iter().enumerate() {
            if is_active(i) {
                output_data_set(buf, *pin, frames[i].raw, self.bidir);
            }
        }

        let mut opts = TransferOptions::default();
        opts.fifo_threshold = Some(FifoThreshold::Quarter);
        opts.mburst = Burst::Single;
        opts.pburst = Burst::Single;

        let bsrr = pac::GPIOA.bsrr().as_ptr() as *mut u32;

        unsafe {
            // Pacer first, DMA stream armed last — BF's ordering.
            pac::TIM1.cnt().write_value(pac::timer::regs::CntCore(0));
            pac::TIM1.dier().modify(|w| w.set_ude(true));

            let req = <DMA2_CH2 as UpDma<TIM1>>::request(&self.dma);
            let t = Transfer::new_write(self.dma.reborrow(), req, &buf[..], bsrr, opts);
            t.await;

            pac::TIM1.dier().modify(|w| w.set_ude(false));
        }

        self.frame_count = self.frame_count.wrapping_add(1);
    }

    /// Send one frame, then release the line and sample the ESC's reply.
    /// Returns the raw port samples; decoding is the caller's job.
    pub async fn send_and_receive(&mut self, frames: [DshotFrame; 4]) -> [u16; RX_BUF_LEN] {
        use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
        use embassy_stm32::timer::UpDma;

        self.send(frames).await;

        // Release the line. The three hold states at the end of the frame have
        // already given the ESC time to sample the last bit, so this
        // transition is safe here and only here.
        pac::GPIOA.moder().modify(|w| {
            for p in active_pins() {
                w.set_moder(p as usize, pac::gpio::vals::Moder::INPUT);
            }
        });

        // SAFETY: as for TX — awaited to completion before returning.
        let rx = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF.0) };
        rx.fill(0);

        let mut opts = TransferOptions::default();
        opts.fifo_threshold = Some(FifoThreshold::Quarter);
        opts.mburst = Burst::Single;
        opts.pburst = Burst::Single;

        let idr = pac::GPIOA.idr().as_ptr() as *mut u16;

        unsafe {
            pac::TIM1
                .arr()
                .write_value(pac::timer::regs::ArrCore(RX_ARR));
            pac::TIM1.cnt().write_value(pac::timer::regs::CntCore(0));
            pac::TIM1.egr().write(|w| w.set_ug(true));
            pac::TIM1.dier().modify(|w| w.set_ude(true));

            let req = <DMA2_CH2 as UpDma<TIM1>>::request(&self.dma);
            let t = Transfer::new_read(self.dma.reborrow(), req, idr, &mut rx[..], opts);
            t.await;

            pac::TIM1.dier().modify(|w| w.set_ude(false));
            pac::TIM1
                .arr()
                .write_value(pac::timer::regs::ArrCore(TX_ARR));
            pac::TIM1.egr().write(|w| w.set_ug(true));
        }

        // Back to driving the line at its idle level.
        set_idle_level(self.bidir);
        pac::GPIOA.moder().modify(|w| {
            for p in active_pins() {
                w.set_moder(p as usize, pac::gpio::vals::Moder::OUTPUT);
            }
        });

        // Bench only, and it must stay that way. `logger::putc` busy-waits on
        // USART6 TXE at 115200 baud (~87 us/byte) inside a global
        // `critical_section`, so these eight lines hold interrupts off for
        // milliseconds. At 2 kHz on the bench that is harmless; in the armed
        // 8 kHz flight loop it would stall the inner loop for tens of
        // iterations, and because IMU_DATA is a latest-value Signal the gyro
        // samples produced during the stall are lost outright.
        #[cfg(feature = "motor-test")]
        if self.frame_count >= RX_PROBE_START && self.frame_count % RX_PROBE_EVERY == 0 {
            // Per-pin, because a NoSignal on one motor and clean eRPM on
            // another is the case worth telling apart: zero transitions means
            // nothing came back at all (ESC config or wiring), whereas
            // transitions plus a failed decode means the samples arrived but
            // the reconstruction is off.
            for (i, p) in MOTOR_PINS.iter().enumerate() {
                let m = 1u16 << p;
                let low = rx.iter().filter(|&&s| s & m == 0).count();
                let transitions = rx.windows(2).filter(|w| (w[0] ^ w[1]) & m != 0).count();
                defmt::info!(
                    "bitbang RX probe M{=usize}: {=usize}/{=usize} low, {=usize} transitions",
                    i + 1,
                    low,
                    RX_BUF_LEN,
                    transitions,
                );
                // Whole-window run lengths. A 16-sample keyhole cannot tell
                // "reply arriving at half the expected bit rate" from "reply
                // truncated by the end of the buffer" — both look like a
                // fragment. Run lengths show bit width and structure directly:
                // a healthy reply is ~21 runs averaging OVERSAMPLE samples,
                // ending well inside RX_BUF_LEN. Double-width runs mean a rate
                // mismatch; a final run that hits the buffer end means the
                // window is too short.
                {
                    let mut runs = [0u8; 40];
                    let mut levels: u64 = 0;
                    let mut n = 0usize;
                    let mut cur = rx[0] & m;
                    let mut len = 0u8;
                    for &s in rx.iter() {
                        if (s & m) == cur {
                            len = len.saturating_add(1);
                        } else {
                            if n < runs.len() {
                                runs[n] = len;
                                if cur != 0 {
                                    levels |= 1 << n;
                                }
                                n += 1;
                            }
                            cur = s & m;
                            len = 1;
                        }
                    }
                    if n < runs.len() {
                        runs[n] = len;
                        if cur != 0 {
                            levels |= 1 << n;
                        }
                        n += 1;
                    }
                    defmt::info!(
                        "  M{=usize} runs n={=usize} hi={=u64:b} lens={=[u8]}",
                        i + 1,
                        n,
                        levels,
                        &runs[..n],
                    );
                }
                match rx.iter().position(|&s| s & m == 0) {
                    Some(idx) => defmt::info!("  M{=usize} first edge @{=usize}", i + 1, idx),
                    None => defmt::info!("  M{=usize}: no falling edge in window", i + 1),
                }
            }
        }

        *rx
    }

    /// Send one frame and decode all four replies.
    pub async fn send_and_decode(&mut self, frames: [DshotFrame; 4]) -> [BbTelemetry; 4] {
        let rx = self.send_and_receive(frames).await;
        core::array::from_fn(|i| match MOTOR_PADS[i] {
            Some(pad) => decode(&rx[..], pad),
            // Not driven and not sampled — report absence rather than
            // decoding an idle line and calling it a dead ESC.
            None => BbTelemetry::NoSignal,
        })
    }
}

/// Drive the four motor pins to their idle level via BSRR.
fn set_idle_level(bidir: bool) {
    pac::GPIOA.bsrr().write(|w| {
        for p in active_pins() {
            if bidir {
                w.set_bs(p as usize, true); // bidir idles HIGH
            } else {
                w.set_br(p as usize, true); // plain DShot idles LOW
            }
        }
    });
}
