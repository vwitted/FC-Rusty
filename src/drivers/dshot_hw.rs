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

/// 16 data bits + 2 trailing low cells so lines idle low after the frame.
const STEPS_PER_FRAME: usize = 18;

/// TIM2 carries two channels interleaved row-major:
/// `[m1_s0, m2_s0, m1_s1, m2_s1, …]`, one pair per timer update event.
const TIM2_BUF_LEN: usize = STEPS_PER_FRAME * 2;

pub struct DshotQuad<'d> {
    tim2: SimplePwm<'d, TIM2>,
    tim3: SimplePwm<'d, TIM3>,
    tim4: SimplePwm<'d, TIM4>,

    dma_tim2_up: Peri<'d, DMA1_CH7>,
    dma_tim3_up: Peri<'d, DMA1_CH2>,
    dma_tim4_up: Peri<'d, DMA1_CH6>,

    t1h: u16,
    t0h: u16,

    buf_tim2: [u16; TIM2_BUF_LEN],
    buf_tim3: [u16; STEPS_PER_FRAME],
    buf_tim4: [u16; STEPS_PER_FRAME],
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

        Self {
            tim2: tim2_pwm,
            tim3: tim3_pwm,
            tim4: tim4_pwm,
            dma_tim2_up,
            dma_tim3_up,
            dma_tim4_up,
            t1h,
            t0h,
            buf_tim2: [0; TIM2_BUF_LEN],
            buf_tim3: [0; STEPS_PER_FRAME],
            buf_tim4: [0; STEPS_PER_FRAME],
        }
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
        frames[2].fill_dma_buffer(&mut self.buf_tim3, self.t1h, self.t0h);
        frames[3].fill_dma_buffer(&mut self.buf_tim4, self.t1h, self.t0h);

        // Three DMA streams launched in near-lockstep; each timer runs
        // its own frame out independently and the await resolves once
        // all three have finished.
        join3(
            self.tim2.waveform_up_multi_channel(
                self.dma_tim2_up.reborrow(),
                Channel::Ch1,
                Channel::Ch2,
                &self.buf_tim2,
            ),
            self.tim3.waveform_up_multi_channel(
                self.dma_tim3_up.reborrow(),
                Channel::Ch1,
                Channel::Ch1,
                &self.buf_tim3,
            ),
            self.tim4.waveform_up_multi_channel(
                self.dma_tim4_up.reborrow(),
                Channel::Ch1,
                Channel::Ch1,
                &self.buf_tim4,
            ),
        ).await;
    }
}
