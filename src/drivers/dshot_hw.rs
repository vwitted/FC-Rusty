// dshot_hw.rs — direct port of Betaflight's H7 DShot driver (both
// non-bidir and bidir telemetry paths), targeting the DAKEFPVH743.
//
// Reference:
//   betaflight/src/platform/STM32/pwm_output_dshot_hal.c
//   betaflight/src/platform/common/stm32/pwm_output_dshot_shared.c
//   betaflight/src/platform/common/stm32/dshot_dpwm.c
//   betaflight/src/main/drivers/dshot.{c,h}
//
// Architecture is per-channel CC DMA (NOT burst-DMA via DMAR): each of
// the four motor channels gets its own DMA1 stream that writes directly
// to TIM2.CCRn for TX, then reverses direction to read TIM2.CCRn for
// RX (input-capture timestamps).
//
// Trigger is `CCxDE` (channel-DMA-enable in DIER) — DMA fires on CC
// match for TX, on CC capture for RX. Cell rate is 12 MHz (PSC=19),
// period 20 ticks (ARR=19), CCR values 7 ("0" bit, 35 % HIGH) and 14
// ("1" bit, 70 % HIGH).
//
// ---- Non-bidir vs bidir ----
//
//   *Non-bidir*: line idles LOW. TX only. After 16 data cells + 2
//   trailing zero cells (line stays LOW), nothing else happens until
//   the next frame.
//
//   *Bidir*: line idles HIGH (CCER.CCxP = 1 inverts polarity; CCR=0
//   leaves OC inactive = HIGH). After TX, channel mode flips from
//   output-compare to input-capture (both edges, filter=2), ARR
//   stretched so CNT becomes a free-running timestamp source, and the
//   same DMA stream is reconfigured P2M to read CCRn timestamps into
//   an edge buffer. After the deadtime, we read NDTR for the edge
//   count, decode 21 GCR bits → 16-bit eRPM payload → period in µs,
//   then flip the channel back to output-compare for the next TX.
//
// ---- Pin map (DAKEFPVH743 — all four motors on TIM2) ----
//   TIM2 CH1 → PA0 → M1   (DMA1_CH2)
//   TIM2 CH2 → PA1 → M2   (DMA1_CH3)
//   TIM2 CH3 → PA2 → M3   (DMA1_CH4)
//   TIM2 CH4 → PA3 → M4   (DMA1_CH7)
//
// Channel→motor mapping is 1:1 here (unlike the GEPRC TAKER port,
// where the four motors were scrambled across TIM3 channels by the
// board's pad layout).
//
// ---- TIM2 is 32-bit ----
// Unlike the GEPRC port's TIM3 (16-bit), TIM2 is a 32-bit general-
// purpose timer. Its CNT/ARR/CCR registers are plain 32-bit values
// (`read()` returns `u32`, writes via `write_value`), where TIM3's
// were 16-bit struct registers (`read().ccr()` / `modify(set_arr)`).
// The bidir RX phase uses this: ARR is stretched to 0xFFFF_FFFF so
// the 32-bit counter never wraps within the response window, removing
// the 16-bit wraparound bookkeeping the TIM3 decoder needed.

use embassy_futures::join::join4;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{OutputType, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH7, PA0, PA1, PA2, PA3, TIM2,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, PwmPinConfig, SimplePwm};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4, Channel, Dma as TimChDma};

use super::dshot_diag;
use super::dshot_frame::{DshotFrame, DshotSpeed};

/// 16 data bits + 2 trailing zero cells (DPWM.h: `DSHOT_DMA_BUFFER_SIZE = 18`).
const STEPS_PER_FRAME: usize = 18;
/// Capture buffer length per motor (BF: `GCR_TELEMETRY_INPUT_LEN = 22`).
const RX_BUF_LEN: usize = 22;
/// Minimum edges to even attempt a GCR decode (BF: `MIN_GCR_EDGES = 7`).
const MIN_GCR_EDGES: usize = 7;

/// 20-tick cell period (DPWM.h: `MOTOR_BITLENGTH = 20`). With PSC=19 on
/// a 240 MHz timer clock, cell clock = 12 MHz, one cell = 1.667 µs.
const MOTOR_BITLENGTH: u32 = 20;

/// Per-cell CCR values (DPWM.h: `MOTOR_BIT_0 = 7`, `MOTOR_BIT_1 = 14`).
const MOTOR_BIT_0: u32 = 7;
const MOTOR_BIT_1: u32 = 14;

/// Bidir response window deadtime in microseconds. BF computes this as
/// `DSHOT_TELEMETRY_DEADTIME_US (=35) + 1e6 * (16*20)/12e6 ≈ 62 µs`
/// for DShot600 — guard + 16-bit response duration.
///
/// Overridable at build time via `DEADTIME_US=<n>` so the window can be
/// widened on the bench. Two things depend on it, which makes it a useful
/// probe: how long we listen for the ESC's reply, and *when* the switch back
/// to output-compare happens. If a scope artefact moves when this moves, it
/// belongs to our direction switch; if it stays put, it belongs to the ESC.
const DEADTIME_US: u64 = parse_deadtime_us();

const DEFAULT_DEADTIME_US: u64 = 80;

/// Parse `DEADTIME_US` at compile time. Unset or blank keeps the default;
/// anything unparseable fails the build rather than silently reverting (the
/// same convention `motor_test.rs` uses for its env vars).
const fn parse_deadtime_us() -> u64 {
    let Some(s) = option_env!("DEADTIME_US") else {
        return DEFAULT_DEADTIME_US;
    };
    let b = s.as_bytes();
    let (mut i, j) = {
        let (mut lo, mut hi) = (0usize, b.len());
        while lo < hi && (b[lo] == b' ' || b[lo] == b'\t') {
            lo += 1;
        }
        while hi > lo && (b[hi - 1] == b' ' || b[hi - 1] == b'\t') {
            hi -= 1;
        }
        (lo, hi)
    };
    if i == j {
        return DEFAULT_DEADTIME_US;
    }
    let mut v: u64 = 0;
    while i < j {
        assert!(
            b[i] >= b'0' && b[i] <= b'9',
            "DEADTIME_US must be a positive integer (microseconds)"
        );
        v = v * 10 + (b[i] - b'0') as u64;
        i += 1;
    }
    assert!(v > 0, "DEADTIME_US must be greater than zero");
    v
}

/// Frame number on which the one-shot bidir idle probe runs. Sits inside
/// motor_test's 3 s MotorStop arming stream (loop_khz × 3000 frames), so the
/// motors are stopped when it perturbs a frame.
const PROBE_FRAME: u32 = 100;

/// Frame on which the RX-path self-test drives its own edges on M1. Also
/// inside the MotorStop arming stream, and far enough from `PROBE_FRAME` that
/// the two diagnostics can't interact.
const RX_SELFTEST_FRAME: u32 = 200;

/// BF: in `decodeTelemetryPacket` the per-bit divisor is 16 ticks. At
/// our 12 MHz cell clock with ARR=0xFFFF_FFFF during RX, 16 ticks =
/// 1.33 µs.
const RX_BIT_TICKS: u32 = 16;

// SRAM1 layout (D2 domain, DMA1-accessible).
//   TX buffers: 4 × 72 bytes (18 u32 each), packed 0x3000_0000..0x120
//   RX buffers: 4 × 88 bytes (22 u32 each), packed 0x3000_0120..0x280
const BUF_M1_ADDR: usize = 0x3000_0000;
const BUF_M2_ADDR: usize = 0x3000_0048;
const BUF_M3_ADDR: usize = 0x3000_0090;
const BUF_M4_ADDR: usize = 0x3000_00D8;
const RX_M1_ADDR: usize = 0x3000_0120;
const RX_M2_ADDR: usize = 0x3000_0178;
const RX_M3_ADDR: usize = 0x3000_01D0;
const RX_M4_ADDR: usize = 0x3000_0228;

/// One motor's telemetry result. `period_us` is the BLHeli-style raw
/// eRPM period; convert to mechanical RPM with
/// `60_000_000 / (period_us * pole_pairs)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum TelemetryResult {
    Erpm { period_us: u32 },
    NoEdge,
    InvalidGcr,
    InvalidCrc,
}

pub struct DshotQuad<'d> {
    _tim: SimplePwm<'d, TIM2>,

    /// DMA1_CH2 → TIM2 CC1 → CCR1 → M1
    dma_m1: Peri<'d, DMA1_CH2>,
    /// DMA1_CH3 → TIM2 CC2 → CCR2 → M2
    dma_m2: Peri<'d, DMA1_CH3>,
    /// DMA1_CH4 → TIM2 CC3 → CCR3 → M3
    dma_m3: Peri<'d, DMA1_CH4>,
    /// DMA1_CH7 → TIM2 CC4 → CCR4 → M4
    dma_m4: Peri<'d, DMA1_CH7>,

    buf_m1: &'static mut [u32; STEPS_PER_FRAME],
    buf_m2: &'static mut [u32; STEPS_PER_FRAME],
    buf_m3: &'static mut [u32; STEPS_PER_FRAME],
    buf_m4: &'static mut [u32; STEPS_PER_FRAME],

    rx_buf_m1: &'static mut [u32; RX_BUF_LEN],
    rx_buf_m2: &'static mut [u32; RX_BUF_LEN],
    rx_buf_m3: &'static mut [u32; RX_BUF_LEN],
    rx_buf_m4: &'static mut [u32; RX_BUF_LEN],

    /// Bidir mode: when true, line idles HIGH (CCxP=1), trailing
    /// zero cells hold the line high, and each frame is followed by
    /// the input-capture telemetry phase.
    bidir: bool,

    /// True while the four channels are left configured for input capture.
    ///
    /// BF switches back to output from `pwmTelemetryDecode`, which runs at the
    /// start of the next motor update — so the channel stays an input for the
    /// whole idle gap and the direction switch happens adjacent to the frame.
    /// This port used to switch back the moment the response window closed,
    /// leaving ~200 µs of idle driven push-pull and putting the switch's
    /// disturbance alone in the middle of the gap, where an edge-triggered ESC
    /// receiver can read it as a frame start.
    channels_in_input: bool,

    frame_count: u32,
}

impl<'d> DshotQuad<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tim2: Peri<'d, TIM2>,
        pa0: Peri<'d, PA0>, // CH1 → M1
        pa1: Peri<'d, PA1>, // CH2 → M2
        pa2: Peri<'d, PA2>, // CH3 → M3
        pa3: Peri<'d, PA3>, // CH4 → M4
        dma_m1: Peri<'d, DMA1_CH2>,
        dma_m2: Peri<'d, DMA1_CH3>,
        dma_m3: Peri<'d, DMA1_CH4>,
        dma_m4: Peri<'d, DMA1_CH7>,
        _speed: DshotSpeed,
        bidir: bool,
    ) -> Self {
        // GPIO: AF push-pull, Speed::Low, internal pull-up (BF:
        // `IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP)`).
        // PullUp matters for bidir specifically — during the brief input
        // window before the ESC drives the line, the internal pull-up
        // keeps it at idle-high.
        let pin_cfg = PwmPinConfig {
            output_type: OutputType::PushPull,
            speed: Speed::Low,
            pull: Pull::Up,
        };

        let tim_pwm = SimplePwm::new(
            tim2,
            Some(PwmPin::new_with_config(pa0, pin_cfg)), // CH1 → M1
            Some(PwmPin::new_with_config(pa1, pin_cfg)), // CH2 → M2
            Some(PwmPin::new_with_config(pa2, pin_cfg)), // CH3 → M3
            Some(PwmPin::new_with_config(pa3, pin_cfg)), // CH4 → M4
            Hertz(600_000),
            CountingMode::EdgeAlignedUp,
        );
        let mut tim_pwm = tim_pwm;

        // BF timer init (HAL:298-306):
        //   PSC = 240e6 / 12e6 - 1 = 19
        //   ARR = MOTOR_BITLENGTH - 1 = 19
        // TIM2's PSC is 16-bit (write_value); ARR/CNT/CCR are plain
        // 32-bit registers (write_value, not modify+set_arr).
        pac::TIM2.psc().write_value(19);
        pac::TIM2.arr().write_value(MOTOR_BITLENGTH - 1);

        // ARPE on (BF: LL_TIM_EnableARRPreload, HAL:466).
        pac::TIM2.cr1().modify(|w| w.set_arpe(true));

        // CCR=0, CCxE=1 (BF: HAL:457-459). SimplePwm's enable()
        // toggles CCxE; we never touch it again.
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            let mut c = tim_pwm.channel(ch);
            c.set_duty_cycle(0);
            c.enable();
        }

        // Bidir polarity: invert CCER.CCxP so OC active = LOW, OC
        // inactive = HIGH. With trailing CCR=0 the line idles HIGH.
        // BF: `OCPolarity = LL_TIM_OCPOLARITY_LOW` when output inverted
        // (HAL:330 + the XOR with TIMER_OUTPUT_INVERTED at HAL:288).
        if bidir {
            pac::TIM2.ccer().modify(|w| {
                w.set_ccp(0, true);
                w.set_ccp(1, true);
                w.set_ccp(2, true);
                w.set_ccp(3, true);
            });
            // (No CCR=0xFFFF bringup quirk — see the GEPRC port's note;
            // SimplePwm already left CCR=0, which with CCxP=1 idles the
            // line HIGH from boot.)
        }

        let buf_m1: &'static mut [u32; STEPS_PER_FRAME] =
            unsafe { &mut *(BUF_M1_ADDR as *mut [u32; STEPS_PER_FRAME]) };
        let buf_m2: &'static mut [u32; STEPS_PER_FRAME] =
            unsafe { &mut *(BUF_M2_ADDR as *mut [u32; STEPS_PER_FRAME]) };
        let buf_m3: &'static mut [u32; STEPS_PER_FRAME] =
            unsafe { &mut *(BUF_M3_ADDR as *mut [u32; STEPS_PER_FRAME]) };
        let buf_m4: &'static mut [u32; STEPS_PER_FRAME] =
            unsafe { &mut *(BUF_M4_ADDR as *mut [u32; STEPS_PER_FRAME]) };
        buf_m1.fill(0);
        buf_m2.fill(0);
        buf_m3.fill(0);
        buf_m4.fill(0);

        let rx_buf_m1: &'static mut [u32; RX_BUF_LEN] =
            unsafe { &mut *(RX_M1_ADDR as *mut [u32; RX_BUF_LEN]) };
        let rx_buf_m2: &'static mut [u32; RX_BUF_LEN] =
            unsafe { &mut *(RX_M2_ADDR as *mut [u32; RX_BUF_LEN]) };
        let rx_buf_m3: &'static mut [u32; RX_BUF_LEN] =
            unsafe { &mut *(RX_M3_ADDR as *mut [u32; RX_BUF_LEN]) };
        let rx_buf_m4: &'static mut [u32; RX_BUF_LEN] =
            unsafe { &mut *(RX_M4_ADDR as *mut [u32; RX_BUF_LEN]) };
        rx_buf_m1.fill(0);
        rx_buf_m2.fill(0);
        rx_buf_m3.fill(0);
        rx_buf_m4.fill(0);

        defmt::info!(
            "DShot build: [{=str}] deadtime={=u64}us",
            dshot_diag::BUILD_TAG,
            DEADTIME_US,
        );
        defmt::info!(
            "DShot init (BF-port, TIM2): bidir={=bool} PSC={=u16} ARR={=u32} bit0={=u32} bit1={=u32}",
            bidir,
            pac::TIM2.psc().read(),
            pac::TIM2.arr().read(),
            MOTOR_BIT_0,
            MOTOR_BIT_1,
        );

        Self {
            _tim: tim_pwm,
            dma_m1,
            dma_m2,
            dma_m3,
            dma_m4,
            buf_m1,
            buf_m2,
            buf_m3,
            buf_m4,
            rx_buf_m1,
            rx_buf_m2,
            rx_buf_m3,
            rx_buf_m4,
            bidir,
            channels_in_input: false,
            frame_count: 0,
        }
    }

    pub fn log_config(&self) {
        let addr = self.buf_m1.as_ptr() as u32;
        defmt::info!(
            "DShot buffer base: {=u32:08x} (4 TX × 72 + 4 RX × 88 bytes)",
            addr
        );
        defmt::assert!(addr >= 0x2400_0000);
        dshot_diag::log_caches();
        dshot_diag::log_timpre();
        dshot_diag::log_gpio_pins();
        dshot_diag::log_timer_running();
        dshot_diag::log_tim2_config();
    }

    pub fn log_runtime_state(&self) {
        dshot_diag::log_dma1_stream("TIM2_CH1 (M1)", 2);
        defmt::info!(
            "post-send CCR: ch1={=u32} ch2={=u32} ch3={=u32} ch4={=u32}",
            pac::TIM2.ccr(0).read(),
            pac::TIM2.ccr(1).read(),
            pac::TIM2.ccr(2).read(),
            pac::TIM2.ccr(3).read(),
        );
    }

    pub async fn send_throttles_and_receive(
        &mut self,
        frames: [DshotFrame; 4],
    ) -> [TelemetryResult; 4] {
        // ---- Return the channels to output, immediately before transmitting
        //      (BF does this from pwmTelemetryDecode at update start, not when
        //      the response window closes) ----
        if self.channels_in_input {
            let probe = self.frame_count == PROBE_FRAME + 1;
            let idr_during_rx = if probe {
                dshot_diag::read_pa_low()
            } else {
                0
            };

            switch_channels_to_output_compare();
            self.channels_in_input = false;

            if probe {
                dshot_diag::probe_output_idle(idr_during_rx);
            }
        }

        // ---- TX phase (same as non-bidir, modulo polarity which is
        //                set once at init via CCER.CCxP) ----
        fill_buffer(self.buf_m1, &frames[0]);
        fill_buffer(self.buf_m2, &frames[1]);
        fill_buffer(self.buf_m3, &frames[2]);
        fill_buffer(self.buf_m4, &frames[3]);

        // Periodic encoder dump for bidir bring-up. Fires on frame 1
        // (pristine init state, before any RX-phase mode-switch has
        // happened) and every 5000 frames thereafter so the dump
        // appears in the log regardless of when capture starts.
        if self.frame_count == 1 || self.frame_count.is_multiple_of(5000) {
            let bits = frames[0].bits_msb_first();
            let mut bits_u16: u16 = 0;
            for (i, b) in bits.iter().enumerate() {
                if *b {
                    bits_u16 |= 1 << (15 - i);
                }
            }
            defmt::info!(
                "DShot M1 frame dump (bidir={=bool}): raw=0x{=u16:04x} data12=0x{=u16:03x} crc=0x{=u8:01x} bits_msb=0x{=u16:04x}",
                self.bidir,
                frames[0].raw,
                frames[0].data_12(),
                frames[0].crc(),
                bits_u16,
            );
            defmt::info!(
                "DShot M1 buf cells (expect 7=`0` 14=`1`, 0 trailing): [{=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} {=u32} | {=u32} {=u32}]",
                self.buf_m1[0], self.buf_m1[1], self.buf_m1[2], self.buf_m1[3],
                self.buf_m1[4], self.buf_m1[5], self.buf_m1[6], self.buf_m1[7],
                self.buf_m1[8], self.buf_m1[9], self.buf_m1[10], self.buf_m1[11],
                self.buf_m1[12], self.buf_m1[13], self.buf_m1[14], self.buf_m1[15],
                self.buf_m1[16], self.buf_m1[17],
            );
            defmt::info!(
                "DShot CCER=0x{=u32:08x} CCMR1=0x{=u32:08x} CCMR2=0x{=u32:08x} ARR=0x{=u32:08x}",
                pac::TIM2.ccer().read().0 as u32,
                pac::TIM2.ccmr_output(0).read().0 as u32,
                pac::TIM2.ccmr_output(1).read().0 as u32,
                pac::TIM2.arr().read(),
            );
        }

        use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
        let mut tx_opts = TransferOptions::default();
        tx_opts.fifo_threshold = Some(FifoThreshold::Quarter);
        tx_opts.mburst = Burst::Single;
        tx_opts.pburst = Burst::Single;

        let ccr1 = pac::TIM2.ccr(0).as_ptr();
        let ccr2 = pac::TIM2.ccr(1).as_ptr();
        let ccr3 = pac::TIM2.ccr(2).as_ptr();
        let ccr4 = pac::TIM2.ccr(3).as_ptr();

        unsafe {
            let m1_req = <DMA1_CH2 as TimChDma<TIM2, Ch1>>::request(&self.dma_m1);
            let t_m1 = Transfer::new_write(
                self.dma_m1.reborrow(),
                m1_req,
                &self.buf_m1[..],
                ccr1,
                tx_opts,
            );
            let m2_req = <DMA1_CH3 as TimChDma<TIM2, Ch2>>::request(&self.dma_m2);
            let t_m2 = Transfer::new_write(
                self.dma_m2.reborrow(),
                m2_req,
                &self.buf_m2[..],
                ccr2,
                tx_opts,
            );
            let m3_req = <DMA1_CH4 as TimChDma<TIM2, Ch3>>::request(&self.dma_m3);
            let t_m3 = Transfer::new_write(
                self.dma_m3.reborrow(),
                m3_req,
                &self.buf_m3[..],
                ccr3,
                tx_opts,
            );
            let m4_req = <DMA1_CH7 as TimChDma<TIM2, Ch4>>::request(&self.dma_m4);
            let t_m4 = Transfer::new_write(
                self.dma_m4.reborrow(),
                m4_req,
                &self.buf_m4[..],
                ccr4,
                tx_opts,
            );

            pac::TIM2.cr1().modify(|w| w.set_arpe(false));
            pac::TIM2.arr().write_value(MOTOR_BITLENGTH - 1);
            pac::TIM2.cnt().write_value(0);

            pac::TIM2.dier().modify(|w| {
                w.set_ccde(0, true);
                w.set_ccde(1, true);
                w.set_ccde(2, true);
                w.set_ccde(3, true);
            });

            join4(t_m1, t_m2, t_m3, t_m4).await;
        }

        pac::TIM2.dier().modify(|w| {
            w.set_ccde(0, false);
            w.set_ccde(1, false);
            w.set_ccde(2, false);
            w.set_ccde(3, false);
        });
        pac::TIM2.cr1().modify(|w| w.set_arpe(true));

        // ---- Non-bidir path returns now ----
        if !self.bidir {
            self.frame_count = self.frame_count.wrapping_add(1);
            if self.frame_count.is_multiple_of(800) {
                self.log_runtime_state();
            }
            return [TelemetryResult::NoEdge; 4];
        }

        // ---- Bidir RX phase ----
        //
        // BF flow (pwmDshotSetDirectionInput in dshot_hal.c):
        //   - ARR ← max (we use 0xFFFF_FFFF — TIM2 is 32-bit)
        //   - For each channel: CCMR.CCxS = TI input, ICxF = 2,
        //     ICxPSC = 1, CCER.CCxNP = 1 (with CCxP=1 → both edges)
        //   - DMA reprogrammed P2M (Transfer::new_read with the same
        //     stream)
        //   - NDTR = 22, EN = 1, CCxDE = 1
        //   - Mainloop gate: wait `dshotTelemetryDeadtimeUs`
        //   - Then read NDTR, decode, switch back to output-compare
        pac::TIM2.arr().write_value(0xFFFF_FFFF);
        switch_channels_to_input_capture();

        let mut rx_opts = TransferOptions::default();
        rx_opts.fifo_threshold = Some(FifoThreshold::Quarter);
        rx_opts.mburst = Burst::Single;
        rx_opts.pburst = Burst::Single;

        let ccr1_src = pac::TIM2.ccr(0).as_ptr();
        let ccr2_src = pac::TIM2.ccr(1).as_ptr();
        let ccr3_src = pac::TIM2.ccr(2).as_ptr();
        let ccr4_src = pac::TIM2.ccr(3).as_ptr();

        // Race-safe edge counts: we'll read NDTR after the wait to find
        // out how many words landed in each buffer.
        unsafe {
            let m1_req = <DMA1_CH2 as TimChDma<TIM2, Ch1>>::request(&self.dma_m1);
            let r_m1 = Transfer::new_read(
                self.dma_m1.reborrow(),
                m1_req,
                ccr1_src,
                &mut self.rx_buf_m1[..],
                rx_opts,
            );
            let m2_req = <DMA1_CH3 as TimChDma<TIM2, Ch2>>::request(&self.dma_m2);
            let r_m2 = Transfer::new_read(
                self.dma_m2.reborrow(),
                m2_req,
                ccr2_src,
                &mut self.rx_buf_m2[..],
                rx_opts,
            );
            let m3_req = <DMA1_CH4 as TimChDma<TIM2, Ch3>>::request(&self.dma_m3);
            let r_m3 = Transfer::new_read(
                self.dma_m3.reborrow(),
                m3_req,
                ccr3_src,
                &mut self.rx_buf_m3[..],
                rx_opts,
            );
            let m4_req = <DMA1_CH7 as TimChDma<TIM2, Ch4>>::request(&self.dma_m4);
            let r_m4 = Transfer::new_read(
                self.dma_m4.reborrow(),
                m4_req,
                ccr4_src,
                &mut self.rx_buf_m4[..],
                rx_opts,
            );

            pac::TIM2.dier().modify(|w| {
                w.set_ccde(0, true);
                w.set_ccde(1, true);
                w.set_ccde(2, true);
                w.set_ccde(3, true);
            });

            // RX-path self-test: drive our own edges on M1 so the capture
            // chain can be validated without the ESC participating.
            if self.frame_count == RX_SELFTEST_FRAME {
                dshot_diag::rx_loopback_pulse_pa0();
            }

            // Race the DMA transfers against a deadtime timer. We
            // expect partial fills (BLHeli sends ≤ 22 edges; the rest
            // never arrive), so any-completes-first is the wrong
            // semantic. Instead let them ALL race a single timeout;
            // whichever returns first cancels the others on drop.
            use embassy_futures::select::{select5, Either5};
            let _ = select5(
                r_m1,
                r_m2,
                r_m3,
                r_m4,
                embassy_time::Timer::after_micros(DEADTIME_US),
            )
            .await;
            // (Drop of the unawaited transfers stops their streams.)
            let _ = (Either5::<(), (), (), (), ()>::First(()),); // suppress unused-variant warning
        }

        pac::TIM2.dier().modify(|w| {
            w.set_ccde(0, false);
            w.set_ccde(1, false);
            w.set_ccde(2, false);
            w.set_ccde(3, false);
        });

        // Read remaining transfer count per stream → edge count.
        // We get edges from `RX_BUF_LEN - NDTR`.
        //   M1=DMA1_CH2=stream2, M2=CH3=stream3, M3=CH4=stream4,
        //   M4=CH7=stream7.
        let n1 = read_ndtr_stream(2);
        let n2 = read_ndtr_stream(3);
        let n3 = read_ndtr_stream(4);
        let n4 = read_ndtr_stream(7);
        let edges_m1 = (RX_BUF_LEN as u32).saturating_sub(n1);
        let edges_m2 = (RX_BUF_LEN as u32).saturating_sub(n2);
        let edges_m3 = (RX_BUF_LEN as u32).saturating_sub(n3);
        let edges_m4 = (RX_BUF_LEN as u32).saturating_sub(n4);

        if self.frame_count == RX_SELFTEST_FRAME {
            dshot_diag::log_rx_loopback_result(
                [edges_m1, edges_m2, edges_m3, edges_m4],
                &self.rx_buf_m1[..],
            );
        }

        let res_m1 = decode_telemetry(self.rx_buf_m1, edges_m1 as usize);
        let res_m2 = decode_telemetry(self.rx_buf_m2, edges_m2 as usize);
        let res_m3 = decode_telemetry(self.rx_buf_m3, edges_m3 as usize);
        let res_m4 = decode_telemetry(self.rx_buf_m4, edges_m4 as usize);

        // ---- Leave the channels as inputs ----
        // The switch back to output-compare now happens at the top of the
        // next send, immediately before transmitting. The line stays
        // released (input + pull-up) for the whole idle gap, matching BF.
        self.channels_in_input = true;

        self.frame_count = self.frame_count.wrapping_add(1);
        if self.frame_count.is_multiple_of(800) {
            self.log_runtime_state();
        }

        [res_m1, res_m2, res_m3, res_m4]
    }
}

/// Fill one motor's TX DMA buffer (BF: `loadDmaBufferDshot`).
fn fill_buffer(buf: &mut [u32; STEPS_PER_FRAME], frame: &DshotFrame) {
    let bits = frame.bits_msb_first();
    for i in 0..16 {
        buf[i] = if bits[i] { MOTOR_BIT_1 } else { MOTOR_BIT_0 };
    }
    buf[16] = 0;
    buf[17] = 0;
}

/// STM32H7 glitch guard: temporarily switch the four motor pins from
/// AF mode to pure GPIO output (push-pull) before reconfiguring CCMR.
/// BF: `pwmDshotSetDirectionInput/Output` do this on H7 to prevent
/// the pad glitching during the alt-function rewire — without it, the
/// pad can briefly drop LOW between the old and new AF source taking
/// effect, which corrupts the trailing-HIGH the ESC's frame-end
/// detector is watching for.
///
/// DAKEFPVH743: all four pins are PA0..PA3, so we touch GPIOA.
fn gpio_glitch_guard_to_output() {
    // ODR first: the pad takes ODR's value the instant MODER says OUTPUT,
    // so setting the level afterwards means the pad briefly shows whatever
    // ODR happened to hold. Reset value is 0, i.e. LOW — the opposite of
    // the bidir idle we are trying to protect.
    pac::GPIOA.bsrr().write(|w| {
        w.set_bs(0, true);
        w.set_bs(1, true);
        w.set_bs(2, true);
        w.set_bs(3, true);
    });
    // Now hold the pins HIGH (idle for bidir) so the line state doesn't
    // change while CCMR is being rewritten.
    pac::GPIOA.moder().modify(|w| {
        w.set_moder(0, pac::gpio::vals::Moder::OUTPUT);
        w.set_moder(1, pac::gpio::vals::Moder::OUTPUT);
        w.set_moder(2, pac::gpio::vals::Moder::OUTPUT);
        w.set_moder(3, pac::gpio::vals::Moder::OUTPUT);
    });
}

fn gpio_glitch_guard_to_af() {
    pac::GPIOA.moder().modify(|w| {
        w.set_moder(0, pac::gpio::vals::Moder::ALTERNATE);
        w.set_moder(1, pac::gpio::vals::Moder::ALTERNATE);
        w.set_moder(2, pac::gpio::vals::Moder::ALTERNATE);
        w.set_moder(3, pac::gpio::vals::Moder::ALTERNATE);
    });
}

/// Switch all four TIM2 channels from output-compare to input-capture
/// mode (BF: `pwmDshotSetDirectionInput`). Both edges, filter=2.
///
/// Order matters: disable CCxE → glitch-guard pins to output → rewrite
/// CCMR → restore AF → re-enable CCxE.
fn switch_channels_to_input_capture() {
    // 1. Guard FIRST — before any timer register is touched. Disabling CCxE
    //    while the pad is still on AF exposes the pad to the output stage at
    //    the exact moment its enable changes, which is the transition the
    //    guard exists to hide.
    gpio_glitch_guard_to_output();

    // 2. Disable channel enables so CCMR modification is atomic.
    pac::TIM2.ccer().modify(|w| {
        w.set_cce(0, false);
        w.set_cce(1, false);
        w.set_cce(2, false);
        w.set_cce(3, false);
    });

    // 3. CCMR for input capture (CCxS=01 = normal TI mapping, ICxF=2,
    //    ICxPSC=0 = capture every edge).
    //
    //    ICxPSC must be written explicitly. Its bits alias onto OCxPE and
    //    OCxFE from the output configuration, which are 1 and 0, so a
    //    read-modify-write that skips them leaves ICxPSC = 0b10 —
    //    "capture once every 4 events", i.e. three quarters of the ESC's
    //    edges silently dropped. BF sets LL_TIM_ICPSC_DIV1 here.
    for reg in [0usize, 1] {
        pac::TIM2.ccmr_input(reg).modify(|w| {
            for ch in [0usize, 1] {
                w.set_ccs(ch, pac::timer::vals::CcmrInputCcs::TI4);
                w.set_icf(ch, pac::timer::vals::FilterValue::FCK_INT_N2);
                w.set_icpsc(ch, 0);
            }
        });
    }

    // 4. CCER.CCxNP = 1 (with CCxP=1 from bidir TX → both edges).
    pac::TIM2.ccer().modify(|w| {
        w.set_ccnp(0, true);
        w.set_ccnp(1, true);
        w.set_ccnp(2, true);
        w.set_ccnp(3, true);
    });

    // 5. Re-enable channels (now in input mode) while the pads are still
    //    held HIGH by the GPIO guard. Enabling capture here is safe: the
    //    guard is driving a steady level, so there is no edge to capture.
    pac::TIM2.ccer().modify(|w| {
        w.set_cce(0, true);
        w.set_cce(1, true);
        w.set_cce(2, true);
        w.set_cce(3, true);
    });

    // 6. Only now hand the pads back to the AF — the peripheral is fully
    //    configured, and both the guard level and the released-to-pull-up
    //    level are HIGH, so the handover produces no edge.
    gpio_glitch_guard_to_af();
}

/// Switch all four TIM2 channels back to output-compare PWM Mode 1 with
/// preload (BF: `pwmDshotSetDirectionOutput`).
fn switch_channels_to_output_compare() {
    // NO GPIO glitch guard in this direction — deliberately.
    //
    // BF's H7 guard exists only in `pwmDshotSetDirectionInput`, wrapped
    // around `LL_TIM_IC_Init`; `pwmDshotSetDirectionOutput` touches no GPIO
    // at all and leaves the pin in ALTERNATE mode throughout. This port
    // originally applied the guard symmetrically to both directions, and
    // that extra OUTPUT↔AF handover was measured on hardware 2026-07-26 as
    // a LOW glitch on the idle line, tracking DEADTIME_US (110 µs after the
    // frame at 80 µs, 272 µs at 250 µs) — i.e. landing exactly here. Three
    // attempts to make the handover glitch-free by reordering all failed;
    // the handover itself is the defect, so it is gone.
    //
    // Safe because CCRn is cleared below *before* CCxE is re-enabled, so
    // the output is never enabled while the compare register still holds an
    // RX capture value. That is the same ordering LL_TIM_OC_Init gives BF:
    // clear CC1E, configure CCMR/CCR, then restore CCER last.
    pac::TIM2.ccer().modify(|w| {
        w.set_cce(0, false);
        w.set_cce(1, false);
        w.set_cce(2, false);
        w.set_cce(3, false);
    });

    pac::TIM2.ccmr_output(0).modify(|w| {
        w.set_ccs(0, pac::timer::vals::CcmrOutputCcs::OUTPUT);
        w.set_ocm(0, pac::timer::vals::Ocm::PWM_MODE1);
        w.set_ocpe(0, true);
        w.set_ccs(1, pac::timer::vals::CcmrOutputCcs::OUTPUT);
        w.set_ocm(1, pac::timer::vals::Ocm::PWM_MODE1);
        w.set_ocpe(1, true);
    });
    pac::TIM2.ccmr_output(1).modify(|w| {
        w.set_ccs(0, pac::timer::vals::CcmrOutputCcs::OUTPUT);
        w.set_ocm(0, pac::timer::vals::Ocm::PWM_MODE1);
        w.set_ocpe(0, true);
        w.set_ccs(1, pac::timer::vals::CcmrOutputCcs::OUTPUT);
        w.set_ocm(1, pac::timer::vals::Ocm::PWM_MODE1);
        w.set_ocpe(1, true);
    });

    // Clear CCxNP so bidir TX polarity (CCxP=1 active-low) is restored.
    pac::TIM2.ccer().modify(|w| {
        w.set_ccnp(0, false);
        w.set_ccnp(1, false);
        w.set_ccnp(2, false);
        w.set_ccnp(3, false);
    });

    // Clear the residual input-capture timestamp out of CCRn *before*
    // the pin is re-attached to the OC output. During the RX phase the
    // hardware wrote capture values into CCRn; those are 32-bit CNT
    // snapshots, so typically hundreds-to-thousands of ticks. Left in
    // place, the OC comparator sees CNT < CCRn and holds the output in
    // its active state — LOW in bidir — from here until the TX DMA
    // finally overwrites CCRn a few cells into the next frame. Scoped
    // on 2026-07-26: ~8 µs (≈5 cells) of solid LOW ahead of every
    // frame, swallowing the falling edge of bit 0. BLHeli syncs its bit
    // timing on that edge, which is why bidir never gets a reply while
    // non-bidir (no RX phase, so CCRn is never polluted) is fine.
    //
    // CCRn is two registers behind one address: with OCxPE=1 a write
    // lands in the *preload* register and only reaches the *active*
    // (shadow) one at the next update event. Which of the two the
    // capture hardware wrote during RX isn't something we can observe
    // from software — reads return the preload register — so clear
    // BOTH and leave no path for a stale value to come back:
    //
    //   OCxPE=0, CCRn←0   → active register, immediately
    //   OCxPE=1, CCRn←0   → preload register, so the next UEV (which
    //                       lands at the start of the next frame)
    //                       can't shadow junk back in
    //   EGR.UG            → reset CNT, reload shadows
    //
    // BF brackets its `LL_TIM_OC_Init` in `pwmDshotSetDirectionOutput`
    // with DisablePreload/EnablePreload for the same reason.
    //
    // CCxE is still 0 here, so the output isn't driving the pin yet —
    // no glitch leaks out.
    for reg in [0usize, 1] {
        pac::TIM2.ccmr_output(reg).modify(|w| {
            w.set_ocpe(0, false);
            w.set_ocpe(1, false);
        });
    }
    pac::TIM2.ccr(0).write_value(0);
    pac::TIM2.ccr(1).write_value(0);
    pac::TIM2.ccr(2).write_value(0);
    pac::TIM2.ccr(3).write_value(0);
    for reg in [0usize, 1] {
        pac::TIM2.ccmr_output(reg).modify(|w| {
            w.set_ocpe(0, true);
            w.set_ocpe(1, true);
        });
    }
    pac::TIM2.ccr(0).write_value(0);
    pac::TIM2.ccr(1).write_value(0);
    pac::TIM2.ccr(2).write_value(0);
    pac::TIM2.ccr(3).write_value(0);
    pac::TIM2.egr().write(|w| w.set_ug(true));

    pac::TIM2.ccer().modify(|w| {
        w.set_cce(0, true);
        w.set_cce(1, true);
        w.set_cce(2, true);
        w.set_cce(3, true);
    });

    // The update event above is NOT enough: measured on hardware
    // 2026-07-26, all four pads sit LOW after this function returns, and
    // stay there for milliseconds — until an update event is issued with
    // the channels *enabled*. The bench probe isolated it: writing CCR1
    // alone changed nothing, but a single EGR.UG released all four pads to
    // idle HIGH at once, so what matters is the timer-wide update, not the
    // per-channel compare value.
    //
    // The earlier UG runs while CCxE=0 and reloads CNT but evidently
    // leaves the CCRn shadow registers holding what the RX phase left in
    // them, so the comparator keeps OCxREF asserted (= LOW under the bidir
    // CCxP=1 inversion). Re-issuing it here, after CCxE=1 and after the
    // pins are back on AF, reproduces exactly the sequence that was proven
    // to release the line.
    //
    // ARR must be restored to the cell period *before* the UG, since the
    // UG is what transfers the preload into the shadow — this is why the
    // caller no longer does it.
    pac::TIM2.arr().write_value(MOTOR_BITLENGTH - 1);
    pac::TIM2.egr().write(|w| w.set_ug(true));

}

/// Read NDTR for one DMA1 stream (0..7). Returns the remaining
/// transfer count. NDTR = configured count - actual transferred count.
fn read_ndtr_stream(stream: usize) -> u32 {
    pac::DMA1.st(stream).ndtr().read().ndt() as u32
}

/// Decode a captured edge-timestamp buffer into an eRPM period.
/// BF: `decodeTelemetryPacket` in pwm_output_dshot_shared.c.
fn decode_telemetry(buf: &[u32; RX_BUF_LEN], edges: usize) -> TelemetryResult {
    if edges < MIN_GCR_EDGES {
        return TelemetryResult::NoEdge;
    }

    // Reconstruct the 21-bit GCR value from edge intervals.
    // Each interval gap rounds to a nearest multiple of RX_BIT_TICKS;
    // the result is the number of bit-times that elapsed since the
    // previous edge. For each interval we shift `value` left by `len`
    // and OR in a 1 at bit (len-1) — i.e. a 1 followed by (len-1)
    // zeros, MSB-first. (BF: `dshot_shared.c` decode loop.)
    //
    // TIM2 is 32-bit and ARR was 0xFFFF_FFFF during RX, so the counter
    // never wraps within the ~50 µs response window — a plain
    // `wrapping_sub` recovers the interval with no 16-bit masking.
    let mut value: u32 = 0;
    let mut bits: u32 = 0;
    let mut prev = buf[0];
    for i in 1..edges {
        let cur = buf[i];
        let diff = cur.wrapping_sub(prev);
        let len = (diff + RX_BIT_TICKS / 2) / RX_BIT_TICKS;
        if len == 0 || bits + len > 21 {
            return TelemetryResult::InvalidGcr;
        }
        value <<= len;
        value |= 1 << (len - 1);
        bits += len;
        prev = cur;
    }
    // Final padding interval up to 21 bits (BF: `len = 21 - bits`).
    if bits < 21 {
        let len = 21 - bits;
        value <<= len;
        value |= 1 << (len - 1);
        bits += len;
    }
    if bits != 21 {
        return TelemetryResult::InvalidGcr;
    }

    // 5-to-4 GCR symbol decode (BF table, 4 symbols × 5 bits each).
    const GCR_DECODE: [u32; 32] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0,
    ];
    let s0 = GCR_DECODE[(value & 0x1F) as usize];
    let s1 = GCR_DECODE[((value >> 5) & 0x1F) as usize];
    let s2 = GCR_DECODE[((value >> 10) & 0x1F) as usize];
    let s3 = GCR_DECODE[((value >> 15) & 0x1F) as usize];
    let decoded = s0 | (s1 << 4) | (s2 << 8) | (s3 << 12);

    // BLHeli CRC check (BF: `csum = decoded ^ (decoded >> 8); csum ^=
    // csum >> 4; if ((csum & 0xf) != 0xf) return invalid`).
    let mut csum = decoded ^ (decoded >> 8);
    csum ^= csum >> 4;
    if (csum & 0xF) != 0xF {
        return TelemetryResult::InvalidCrc;
    }

    // Top 12 bits = eRPM payload: 3-bit exponent + 9-bit mantissa.
    let payload = (decoded >> 4) & 0xFFF;
    if payload == 0x0FFF {
        return TelemetryResult::Erpm { period_us: 0 };
    }
    let exponent = (payload >> 9) & 0x7;
    let mantissa = payload & 0x1FF;
    let period_us = mantissa << exponent;
    if period_us == 0 {
        return TelemetryResult::Erpm { period_us: 0 };
    }
    TelemetryResult::Erpm { period_us }
}
