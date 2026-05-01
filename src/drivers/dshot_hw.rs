// dshot_hw.rs — single-timer DMA-driven DShot600 for DAKEFPVH743.
//
// The DAKEFPVH743 places all four motor outputs on TIM2:
//   TIM2 CH1 → PA0 → M1
//   TIM2 CH2 → PA1 → M2
//   TIM2 CH3 → PA2 → M3
//   TIM2 CH4 → PA3 → M4
//   TIM2_UP  → DMA1 (managed via DMAMUX on H7)
//
// We drive all four channels simultaneously using a 4-beat burst DMA
// transfer triggered by the TIM2 Update event. This is done via the
// `DMAR` register (`DBL=3` -> 4 transfers to CCR1..CCR4).
//
// Timer clock: APB1 timers on our H743 run at 240 MHz.

use embassy_stm32::Peri;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{
    DMA1_CH7, PA0, PA1, PA2, PA3, TIM2,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::UpDma;

use uf_dshot::{DshotSpeed, EncodedFrame};
use super::dshot_diag;

/// 16 data bits + 8 trailing low cells.
/// 24 cells × 4 channels = 96 words.
const STEPS_PER_FRAME: usize = 24;
const TIM2_BUF_LEN: usize = STEPS_PER_FRAME * 4;

// SRAM1 layout (D2 domain, for DMA1 access):
//   0x3000_0000..0180  buf_tim2  (96 cells = 384 bytes)
const BUF_TIM2_ADDR: usize = 0x3000_0000;

pub struct DshotQuad<'d> {
    tim2: SimplePwm<'d, TIM2>,
    dma_tim2_up: Peri<'d, DMA1_CH7>,

    t1h: u32,
    t0h: u32,

    buf_tim2: &'static mut [u32; TIM2_BUF_LEN],

    frame_count: u32,
}

impl<'d> DshotQuad<'d> {
    pub fn new(
        tim2: Peri<'d, TIM2>,
        pa0: Peri<'d, PA0>,
        pa1: Peri<'d, PA1>,
        pa2: Peri<'d, PA2>,
        pa3: Peri<'d, PA3>,
        dma_tim2_up: Peri<'d, DMA1_CH7>,
        speed: DshotSpeed,
    ) -> Self {
        const TIMER_CLOCK_HZ: u32 = 240_000_000;
        let bitrate = speed.timing_hints().nominal_bitrate_hz;
        let period = TIMER_CLOCK_HZ / bitrate;
        let t1h = period * 3 / 4;
        let t0h = period * 3 / 8;
        let freq = Hertz(bitrate);

        // ---- TIM2: CH1 (PA0), CH2 (PA1), CH3 (PA2), CH4 (PA3) ----
        let mut tim2_pwm = SimplePwm::new(
            tim2,
            Some(PwmPin::new(pa0, OutputType::PushPull)),
            Some(PwmPin::new(pa1, OutputType::PushPull)),
            Some(PwmPin::new(pa2, OutputType::PushPull)),
            Some(PwmPin::new(pa3, OutputType::PushPull)),
            freq,
            CountingMode::EdgeAlignedUp,
        );
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            let mut c = tim2_pwm.channel(ch);
            c.set_duty_cycle(0);
            c.enable();
        }

        // DShot DMA buffer — placed in SRAM1
        let buf_tim2: &'static mut [u32; TIM2_BUF_LEN] =
            unsafe { &mut *(BUF_TIM2_ADDR as *mut [u32; TIM2_BUF_LEN]) };

        Self {
            tim2: tim2_pwm,
            dma_tim2_up,
            t1h,
            t0h,
            buf_tim2,
            frame_count: 0,
        }
    }

    pub fn log_config(&self) {
        let a2 = self.buffer_addresses();
        defmt::info!("DShot buffer: tim2={=u32:08x}", a2);
        defmt::assert!(
            a2 >= 0x2400_0000,
            "DShot buffer in DTCM/ITCM — DMA1 can't reach it"
        );
        defmt::info!("DShot bit-cell ticks: t0h={=u32} t1h={=u32}", self.t0h, self.t1h);
        dshot_diag::log_caches();
        dshot_diag::log_timpre();
        dshot_diag::log_gpio_pins();
        dshot_diag::log_timer_running();
        dshot_diag::log_tim2_config();
    }

    pub fn log_runtime_state(&self) {
        dshot_diag::log_dma1_stream("TIM2_UP", 7);
        defmt::info!(
            "post-send CCR: ch1={=u32} ch2={=u32} ch3={=u32} ch4={=u32}",
            pac::TIM2.ccr(0).read(),
            pac::TIM2.ccr(1).read(),
            pac::TIM2.ccr(2).read(),
            pac::TIM2.ccr(3).read(),
        );
    }

    pub async fn send(&mut self, frames: [EncodedFrame; 4]) {
        let m1_bits = frames[0].bits_msb_first();
        let m2_bits = frames[1].bits_msb_first();
        let m3_bits = frames[2].bits_msb_first();
        let m4_bits = frames[3].bits_msb_first();
        
        for step in 0..16 {
            self.buf_tim2[step * 4] = if m1_bits[step] { self.t1h } else { self.t0h };
            self.buf_tim2[step * 4 + 1] = if m2_bits[step] { self.t1h } else { self.t0h };
            self.buf_tim2[step * 4 + 2] = if m3_bits[step] { self.t1h } else { self.t0h };
            self.buf_tim2[step * 4 + 3] = if m4_bits[step] { self.t1h } else { self.t0h };
        }
        for step in 16..STEPS_PER_FRAME {
            self.buf_tim2[step * 4] = 0;
            self.buf_tim2[step * 4 + 1] = 0;
            self.buf_tim2[step * 4 + 2] = 0;
            self.buf_tim2[step * 4 + 3] = 0;
        }

        let cr1_addr = pac::TIM2.cr1().as_ptr() as u32;
        let ccr1_addr = pac::TIM2.ccr(0).as_ptr() as u32;
        
        pac::TIM2.dcr().modify(|w| {
            w.set_dba(((ccr1_addr - cr1_addr) / 4) as u8);
            w.set_dbl(3); // 4 transfers per update event (CCR1..CCR4)
        });

        let req = <DMA1_CH7 as UpDma<TIM2>>::request(&self.dma_tim2_up);
        pac::TIM2.dier().modify(|w| w.set_ude(true)); // Enable Update DMA
        
        unsafe {
            use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
            let mut dma_transfer_option = TransferOptions::default();
            dma_transfer_option.fifo_threshold = Some(FifoThreshold::Full);
            dma_transfer_option.mburst = Burst::Incr4;
            
            Transfer::new_write(
                self.dma_tim2_up.reborrow(),
                req,
                self.buf_tim2,
                pac::TIM2.dmar().as_ptr() as *mut u32,
                dma_transfer_option,
            )
            .await;
        }
        pac::TIM2.dier().modify(|w| w.set_ude(false)); // Disable Update DMA

        self.frame_count = self.frame_count.wrapping_add(1);
        if self.frame_count.is_multiple_of(200) {
            self.log_runtime_state();
        }
    }

    pub fn buffer_addresses(&self) -> u32 {
        self.buf_tim2.as_ptr() as u32
    }
}
