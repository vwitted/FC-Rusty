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
    DMA1_CH7, PB0, PB1, PB5, PB4, TIM3,
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
const TIM3_BUF_LEN: usize = STEPS_PER_FRAME * 4;

// SRAM1 layout (D2 domain, for DMA1 access):
//   0x3000_0000..0180  buf_tim3  (96 cells = 384 bytes)
const BUF_TIM3_ADDR: usize = 0x3000_0000;

pub struct DshotQuad<'d> {
    tim3: SimplePwm<'d, TIM3>,
    dma_tim3_up: Peri<'d, DMA1_CH7>,

    t1h: u32,
    t0h: u32,

    buf_tim3: &'static mut [u32; TIM3_BUF_LEN],

    frame_count: u32,
}

impl<'d> DshotQuad<'d> {
    pub fn new(
        tim3: Peri<'d, TIM3>,
        pb0: Peri<'d, PB0>, // CH3
        pb1: Peri<'d, PB1>, // CH4
        pb5: Peri<'d, PB5>, // CH2
        pb4: Peri<'d, PB4>, // CH1
        dma_tim3_up: Peri<'d, DMA1_CH7>,
        speed: DshotSpeed,
    ) -> Self {
        const TIMER_CLOCK_HZ: u32 = 240_000_000;
        let bitrate = speed.timing_hints().nominal_bitrate_hz;
        let period = TIMER_CLOCK_HZ / bitrate;
        let t1h = period * 3 / 4;
        let t0h = period * 3 / 8;
        let freq = Hertz(bitrate);

        // ---- TIM3: CH3 (PB0), CH4 (PB1), CH2 (PB5), CH1 (PB4) ----
        let mut tim3_pwm = SimplePwm::new(
            tim3,
            Some(PwmPin::new(pb4, OutputType::PushPull)), // CH1
            Some(PwmPin::new(pb5, OutputType::PushPull)), // CH2
            Some(PwmPin::new(pb0, OutputType::PushPull)), // CH3
            Some(PwmPin::new(pb1, OutputType::PushPull)), // CH4
            freq,
            CountingMode::EdgeAlignedUp,
        );
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            let mut c = tim3_pwm.channel(ch);
            c.set_duty_cycle(0);
            c.enable();
        }

        // DShot DMA buffer — placed in SRAM1
        let buf_tim3: &'static mut [u32; TIM3_BUF_LEN] =
            unsafe { &mut *(BUF_TIM3_ADDR as *mut [u32; TIM3_BUF_LEN]) };

        Self {
            tim3: tim3_pwm,
            dma_tim3_up,
            t1h,
            t0h,
            buf_tim3,
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
        dshot_diag::log_dma1_stream("TIM3_UP", 7);
        defmt::info!(
            "post-send CCR: ch1={=u32} ch2={=u32} ch3={=u32} ch4={=u32}",
            pac::TIM3.ccr(0).read().ccr() as u32,
            pac::TIM3.ccr(1).read().ccr() as u32,
            pac::TIM3.ccr(2).read().ccr() as u32,
            pac::TIM3.ccr(3).read().ccr() as u32,
        );
    }

    pub async fn send(&mut self, frames: [EncodedFrame; 4]) {
        let m1_bits = frames[0].bits_msb_first();
        let m2_bits = frames[1].bits_msb_first();
        let m3_bits = frames[2].bits_msb_first();
        let m4_bits = frames[3].bits_msb_first();
        
        // M1=CH3(PB0), M2=CH4(PB1), M3=CH2(PB5), M4=CH1(PB4)
        // DMA writes CCR1..CCR4 sequentially in burst
        for step in 0..16 {
            self.buf_tim3[step * 4] = if m4_bits[step] { self.t1h } else { self.t0h };     // CCR1
            self.buf_tim3[step * 4 + 1] = if m3_bits[step] { self.t1h } else { self.t0h }; // CCR2
            self.buf_tim3[step * 4 + 2] = if m1_bits[step] { self.t1h } else { self.t0h }; // CCR3
            self.buf_tim3[step * 4 + 3] = if m2_bits[step] { self.t1h } else { self.t0h }; // CCR4
        }
        for step in 16..STEPS_PER_FRAME {
            self.buf_tim3[step * 4] = 0;
            self.buf_tim3[step * 4 + 1] = 0;
            self.buf_tim3[step * 4 + 2] = 0;
            self.buf_tim3[step * 4 + 3] = 0;
        }

        let cr1_addr = pac::TIM3.cr1().as_ptr() as u32;
        let ccr1_addr = pac::TIM3.ccr(0).as_ptr() as u32;
        
        pac::TIM3.dcr().modify(|w| {
            w.set_dba(((ccr1_addr - cr1_addr) / 4) as u8);
            w.set_dbl(3); // 4 transfers per update event (CCR1..CCR4)
        });

        let req = <DMA1_CH7 as UpDma<TIM3>>::request(&self.dma_tim3_up);
        pac::TIM3.dier().modify(|w| w.set_ude(true)); // Enable Update DMA
        
        unsafe {
            use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
            let mut dma_transfer_option = TransferOptions::default();
            dma_transfer_option.fifo_threshold = Some(FifoThreshold::Full);
            dma_transfer_option.mburst = Burst::Incr4;
            
            Transfer::new_write(
                self.dma_tim3_up.reborrow(),
                req,
                self.buf_tim3,
                pac::TIM3.dmar().as_ptr() as *mut u32,
                dma_transfer_option,
            )
            .await;
        }
        pac::TIM3.dier().modify(|w| w.set_ude(false)); // Disable Update DMA

        self.frame_count = self.frame_count.wrapping_add(1);
        if self.frame_count.is_multiple_of(200) {
            self.log_runtime_state();
        }
    }

    pub fn buffer_addresses(&self) -> u32 {
        self.buf_tim3.as_ptr() as u32
    }
}
