// dshot_hw.rs — multi-timer DMA-driven DShot600 for Radiolink F722.
//
// The motor pins span three different timers, so we cannot drive
// all four channels from a single timer like the F405 port did.
// Instead we run three DMA streams in parallel — one TIMx_UP
// stream per timer — and `join3!` on the three transfers.
//
// All four outputs go through Embassy's burst-DMA variant
// (`waveform_up_multi_channel`). TIM2 carries two channels
// interleaved (M1+M2). TIM3 and TIM4 only drive one channel each,
// but we still use the multi-channel API in its degenerate form
// (Ch1 → Ch1, DBL=0) so every motor sits on the same DMAR-burst
// mechanism.
//
// The single-channel `waveform_up` variant looks simpler but
// saves/restores CCR around the transfer and hits a FIFO-close
// race at the frame tail — Embassy flags this in its own source
// at `simple_pwm.rs:357` ("this can almost always trigger a DMA
// FIFO error"). That corrupted tail caused two of four ESC
// channels to fail motor bring-up on 2026-04-22 (one stuck at
// protocol-detect, one weakly spinning with no arm command),
// while the M1/M2 pair on the TIM2 burst path armed cleanly.
// Swapping the M3 signal wire onto ESC-channel 1 moved the fault
// with it, confirming the signal — not the ESCs — was at fault.
// Unifying all four onto the burst path fixed it.
//
// Pin / peripheral assignment (see main.rs header for the full map):
//   TIM2 CH1 → PA15 → M1 (rear-right,  CW)     AF1
//   TIM2 CH2 → PB3  → M2 (front-right, CCW)    AF1
//   TIM3 CH1 → PB4  → M3 (rear-left,   CCW)    AF2
//   TIM4 CH1 → PB6  → M4 (front-left,  CW)     AF2
//   TIM2_UP  → DMA1 Stream 7 (request 3)
//   TIM3_UP  → DMA1 Stream 2 (request 5)
//   TIM4_UP  → DMA1 Stream 6 (request 2)
//
// Timer clock: TIM2/3/4 live on APB1. With our F722 RCC config
// (APB1 = 54 MHz) the timer clock is APB1 × 2 = 108 MHz.
//
// Motor skew: `join3` polls the three futures in sequence, so one
// DMA stream is kicked off before the next. At 216 MHz CPU and a
// few instructions per poll this is tens of nanoseconds — well
// under 1 % of a 1.67 µs DShot600 bit cell, and each ESC decodes
// its own frame independently, so this is a non-issue.

use embassy_futures::join::join3;
use embassy_stm32::Peri;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH6, DMA1_CH7,
    PA15, PB3, PB4, PB6,
    TIM2, TIM3, TIM4,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};

use super::dshot::{DshotFrame, DshotSpeed};
use super::dshot_diag;

/// 16 data bits + 8 trailing low cells.
///
/// Two reasons for 8 trailing zeros (we'd only need 2 to idle low):
/// Embassy's `waveform_up_multi_channel` configures the F7 DMA with
/// `mburst=Incr4` (4-beat memory bursts) and `fifo_threshold=Full`
/// (16-byte FIFO), which together require the total transfer length
/// in bytes to be a multiple of 16. With 16-bit cells that's a
/// multiple of 8 cells. Misalignment causes a FIFO error and the
/// trailing partial-burst is dropped — corrupting the last bits of
/// every frame (proven on bench 2026-04-25, see motor-bringup-log).
/// 24 cells × 2 bytes = 48 bytes (TIM3/4) and × 4 = 96 bytes (TIM2)
/// — both clean multiples of 16.
const STEPS_PER_FRAME: usize = 24;

/// TIM2 carries two channels interleaved row-major:
/// `[m1_s0, m2_s0, m1_s1, m2_s1, …]`, one pair per timer update event.
const TIM2_BUF_LEN: usize = STEPS_PER_FRAME * 2;

// SRAM2 layout (1 KB reserved by memory.x; we use 192 bytes):
//   0x2003_FC00..FC60  buf_tim2  (48 cells = 96 bytes)
//   0x2003_FC60..FC90  buf_tim3  (24 cells = 48 bytes)
//   0x2003_FC90..FCC0  buf_tim4  (24 cells = 48 bytes)
const BUF_TIM2_ADDR: usize = 0x2003_FC00;
const BUF_TIM3_ADDR: usize = 0x2003_FC60;
const BUF_TIM4_ADDR: usize = 0x2003_FC90;

pub struct DshotQuad<'d> {
    tim2: SimplePwm<'d, TIM2>,
    tim3: SimplePwm<'d, TIM3>,
    tim4: SimplePwm<'d, TIM4>,

    dma_tim2_up: Peri<'d, DMA1_CH7>,
    dma_tim3_up: Peri<'d, DMA1_CH2>,
    dma_tim4_up: Peri<'d, DMA1_CH6>,

    t1h: u16,
    t0h: u16,

    buf_tim2: &'static mut [u16; TIM2_BUF_LEN],
    buf_tim3: &'static mut [u16; STEPS_PER_FRAME],
    buf_tim4: &'static mut [u16; STEPS_PER_FRAME],

    /// Wraps every 200 sends. Used to gate the 1 Hz runtime diag log.
    frame_count: u32,
}

impl<'d> DshotQuad<'d> {
    /// Configure the three timers, their pins, and their DMA streams.
    /// All four motor lines idle low at 0 % duty until the first
    /// `send()` call.
    pub fn new(
        tim2: Peri<'d, TIM2>,
        tim3: Peri<'d, TIM3>,
        tim4: Peri<'d, TIM4>,
        pa15: Peri<'d, PA15>,
        pb3: Peri<'d, PB3>,
        pb4: Peri<'d, PB4>,
        pb6: Peri<'d, PB6>,
        dma_tim2_up: Peri<'d, DMA1_CH7>,
        dma_tim3_up: Peri<'d, DMA1_CH2>,
        dma_tim4_up: Peri<'d, DMA1_CH6>,
        speed: DshotSpeed,
    ) -> Self {
        const TIMER_CLOCK_HZ: u32 = 108_000_000;
        let t1h = speed.t1h_ticks(TIMER_CLOCK_HZ);
        let t0h = speed.t0h_ticks(TIMER_CLOCK_HZ);
        let freq = Hertz(speed.bitrate());

        // ---- TIM2: CH1 (PA15 = M1), CH2 (PB3 = M2) ----
        let tim2_ch1 = PwmPin::new(pa15, OutputType::PushPull);
        let tim2_ch2 = PwmPin::new(pb3,  OutputType::PushPull);
        let mut tim2_pwm = SimplePwm::new(
            tim2,
            Some(tim2_ch1),
            Some(tim2_ch2),
            None,
            None,
            freq,
            CountingMode::EdgeAlignedUp,
        );
        for ch in [Channel::Ch1, Channel::Ch2] {
            let mut c = tim2_pwm.channel(ch);
            c.set_duty_cycle(0);
            c.enable();
        }

        // ---- TIM3: CH1 (PB4 = M3) ----
        let tim3_ch1 = PwmPin::new(pb4, OutputType::PushPull);
        let mut tim3_pwm = SimplePwm::new(
            tim3,
            Some(tim3_ch1),
            None,
            None,
            None,
            freq,
            CountingMode::EdgeAlignedUp,
        );
        {
            let mut c = tim3_pwm.channel(Channel::Ch1);
            c.set_duty_cycle(0);
            c.enable();
        }

        // ---- TIM4: CH1 (PB6 = M4) ----
        let tim4_ch1 = PwmPin::new(pb6, OutputType::PushPull);
        let mut tim4_pwm = SimplePwm::new(
            tim4,
            Some(tim4_ch1),
            None,
            None,
            None,
            freq,
            CountingMode::EdgeAlignedUp,
        );
        {
            let mut c = tim4_pwm.channel(Channel::Ch1);
            c.set_duty_cycle(0);
            c.enable();
        }

        // DShot DMA buffers — placed at a fixed address in SRAM2
        // (outside the linker's `RAM` region) so DMA1 can read them.
        // Safety: we're the sole owner of this memory, and
        // `DshotQuad::new` is called exactly once.
        let buf_tim2: &'static mut [u16; TIM2_BUF_LEN] = unsafe {
            &mut *(BUF_TIM2_ADDR as *mut [u16; TIM2_BUF_LEN])
        };
        let buf_tim3: &'static mut [u16; STEPS_PER_FRAME] = unsafe {
            &mut *(BUF_TIM3_ADDR as *mut [u16; STEPS_PER_FRAME])
        };
        let buf_tim4: &'static mut [u16; STEPS_PER_FRAME] = unsafe {
            &mut *(BUF_TIM4_ADDR as *mut [u16; STEPS_PER_FRAME])
        };

        // Verify the SRAM2 cells are real and coherent before zeroing.
        dshot_diag::canary_check("buf_tim2", &mut buf_tim2[..]);
        dshot_diag::canary_check("buf_tim3", &mut buf_tim3[..]);
        dshot_diag::canary_check("buf_tim4", &mut buf_tim4[..]);

        Self {
            tim2: tim2_pwm,
            tim3: tim3_pwm,
            tim4: tim4_pwm,
            dma_tim2_up,
            dma_tim3_up,
            dma_tim4_up,
            t1h,
            t0h,
            buf_tim2,
            buf_tim3,
            buf_tim4,
            frame_count: 0,
        }
    }

    /// Dump caches, every timer's PWM/DMA configuration, and assert
    /// the DMA buffers landed in SRAM (not DTCM, which DMA1 can't
    /// reach). Call once at boot, immediately after `new`. Cheap
    /// (~10 defmt lines) but high-signal: PSC/ARR/CCMR/DCR mismatches
    /// would produce exactly the malformed-signal symptoms seen on
    /// 2026-04-22, so verifying them up front rules them out.
    pub fn log_config(&self) {
        let (a2, a3, a4) = self.buffer_addresses();
        defmt::info!("DShot buffers: tim2={=u32:08x} tim3={=u32:08x} tim4={=u32:08x}",
                     a2, a3, a4);
        defmt::assert!(a2 >= 0x2001_0000 && a3 >= 0x2001_0000 && a4 >= 0x2001_0000,
                       "DShot buffer in DTCM — DMA1 can't reach it");
        defmt::info!("DShot bit-cell ticks: t0h={=u16} t1h={=u16}", self.t0h, self.t1h);
        dshot_diag::log_caches();
        dshot_diag::log_timpre();
        dshot_diag::log_gpio_pins();
        dshot_diag::log_timer_running();
        dshot_diag::log_tim2_config();
        dshot_diag::log_tim3_config();
        dshot_diag::log_tim4_config();
    }

    /// Post-transfer DMA + timer state for all three streams. Called
    /// from `send()` every 200 frames (~1 Hz at the 200 Hz outer
    /// loop). Intended to surface mid-flight regressions: an NDTR
    /// stuck > 0, an error flag set, or a non-zero CCR1 at frame end
    /// would all explain ESCs failing to decode.
    pub fn log_runtime_state(&self) {
        dshot_diag::log_dma1_stream("TIM2_UP", 7);
        dshot_diag::log_dma1_stream("TIM3_UP", 2);
        dshot_diag::log_dma1_stream("TIM4_UP", 6);
        defmt::info!(
            "post-send CCR: tim2_ch1={=u32} tim2_ch2={=u32} tim3_ch1={=u16} tim4_ch1={=u16}",
            pac::TIM2.ccr(0).read(),
            pac::TIM2.ccr(1).read(),
            pac::TIM3.ccr(0).read().ccr(),
            pac::TIM4.ccr(0).read().ccr(),
        );
    }

    /// Transmit one DShot frame on each of the four motors in parallel.
    /// Completes in ~30 µs at DShot600. The caller must invoke this at
    /// least every ~20 ms or the ESCs will hit their signal-loss failsafe.
    pub async fn send(&mut self, frames: [DshotFrame; 4]) {
        // TIM2: interleave M1/M2 bits per step, MSB first.
        for step in 0..16 {
            let m1 = (frames[0].raw >> (15 - step)) & 1;
            let m2 = (frames[1].raw >> (15 - step)) & 1;
            self.buf_tim2[step * 2]     = if m1 == 1 { self.t1h } else { self.t0h };
            self.buf_tim2[step * 2 + 1] = if m2 == 1 { self.t1h } else { self.t0h };
        }
        for step in 16..STEPS_PER_FRAME {
            self.buf_tim2[step * 2]     = 0;
            self.buf_tim2[step * 2 + 1] = 0;
        }

        // TIM3, TIM4: flat per-channel buffers.
        frames[2].fill_dma_buffer(&mut self.buf_tim3[..], self.t1h, self.t0h);
        frames[3].fill_dma_buffer(&mut self.buf_tim4[..], self.t1h, self.t0h);

        // Three DMA streams launched in near-lockstep; each timer runs
        // its own frame out independently and the await resolves once
        // all three have finished.
        join3(
            self.tim2.waveform_up_multi_channel(
                self.dma_tim2_up.reborrow(),
                Channel::Ch1,
                Channel::Ch2,
                &self.buf_tim2[..],
            ),
            self.tim3.waveform_up_multi_channel(
                self.dma_tim3_up.reborrow(),
                Channel::Ch1,
                Channel::Ch1,
                &self.buf_tim3[..],
            ),
            self.tim4.waveform_up_multi_channel(
                self.dma_tim4_up.reborrow(),
                Channel::Ch1,
                Channel::Ch1,
                &self.buf_tim4[..],
            ),
        ).await;

        self.frame_count = self.frame_count.wrapping_add(1);
        if self.frame_count.is_multiple_of(200) {
            self.log_runtime_state();
        }
    }

    /// Returns the memory addresses of the three DMA buffers.
    /// Used to verify buffers live in DMA-accessible memory.
    pub fn buffer_addresses(&self) -> (u32, u32, u32) {
        (
            self.buf_tim2.as_ptr() as u32,
            self.buf_tim3.as_ptr() as u32,
            self.buf_tim4.as_ptr() as u32,
        )
    }
}
