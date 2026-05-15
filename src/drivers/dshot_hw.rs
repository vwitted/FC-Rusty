// dshot_hw.rs — single-timer DMA-driven bidirectional DShot600 for the
// GEPRC TAKER H743 BT.
//
// Motor pinout on this board (TIM3, not TIM2):
//   TIM3 CH1 → PB4 → M4
//   TIM3 CH2 → PB5 → M3
//   TIM3 CH3 → PB0 → M1
//   TIM3 CH4 → PB1 → M2
//   TIM3_UP  → DMA1 (managed via DMAMUX on H7)
//
// We drive all four channels simultaneously using a 4-beat burst DMA
// transfer triggered by the TIM3 Update event, via the timer's `DMAR`
// register (`DBL = 3` → 4 transfers per UEV to CCR1..CCR4).
//
// ---- Bidirectional DShot wiring conventions ----
//
// Bidir DShot inverts the on-wire signal vs standard DShot:
//   - Line idles HIGH between frames (pull-up + open-drain).
//   - A "1" data bit is a long-LOW pulse (75 % of the cell).
//   - A "0" data bit is a short-LOW pulse (37.5 % of the cell).
//   - After the 16-bit frame the ESC waits ~30 µs then drives the
//     same wire LOW to transmit a 21-bit GCR-encoded response.
//
// Two consequences for this driver:
//   1. **Pins are open-drain with a pull-up.** Push-pull on the FC
//      side would short its PMOS through the ESC's NMOS during the
//      TX→RX handoff if the ESC starts responding earlier than the
//      reconfiguration completes. Same hazard pattern as the I²C
//      bus-recovery push-pull issue (see CLAUDE.md).
//   2. **TIM_CCER.CCxP is set to 1 (active-low) for all four
//      channels.** PWM Mode 1 keeps "OC active" while counter < CCR
//      and "OC inactive" otherwise; inverting the polarity means
//      "active" = LOW and "inactive" = HIGH. With open-drain that
//      becomes: CCR ticks of driven-LOW, then pull-up-HIGH for the
//      remainder. Setting CCR = t1h (75 % of period) produces the
//      bidir "1" pulse; CCR = 0 in the trailing slots leaves the
//      line at idle-HIGH.
//
// Timer clock: APB1 timers on this board run at 240 MHz.

use embassy_stm32::Peri;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{
    DMA1_CH6, DMA1_CH7, PB0, PB1, PB5, PB4, TIM3,
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
//   0x3000_0200..0400  rx_buffer (256 samples * 2 bytes = 512 bytes)
const BUF_TIM3_ADDR: usize = 0x3000_0000;
const RX_BUFFER_ADDR: usize = 0x3000_0200;

pub struct DshotQuad<'d> {
    tim3: SimplePwm<'d, TIM3>,
    dma_tim3_up: Peri<'d, DMA1_CH7>,

    t1h: u32,
    t0h: u32,
    /// TIM3 ARR for TX (one bit-cell at the configured speed). Cached
    /// so the RX path can restore it after the inverted-direction
    /// sample-rate change.
    tx_arr: u32,
    /// TIM3 ARR for RX. Picked so the bidir telemetry response
    /// (5/4 × TX bit period) is sampled at ≈3 samples per bit, which
    /// is what `uf_dshot::OversamplingConfig::default()` expects.
    rx_arr: u32,
    /// Bidir mode toggle: enables CCER polarity inversion, open-drain
    /// + pull-up idle, and the RX phase. With `bidir = false` the
    /// driver emits standard DShot (HIGH-pulse, idle LOW, push-pull)
    /// and the RX phase is a no-op.
    bidir: bool,

    buf_tim3: &'static mut [u32; TIM3_BUF_LEN],
    rx_buffer: &'static mut [u16; 256],
    dma_tim3_rx: Peri<'d, DMA1_CH6>,

    // Decoders for telemetry (unused when bidir == false).
    decoders: [uf_dshot::BidirDecoder; 4],

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
        dma_tim3_rx: Peri<'d, DMA1_CH6>,
        speed: DshotSpeed,
        bidir: bool,
    ) -> Self {
        const TIMER_CLOCK_HZ: u32 = 240_000_000;
        let bitrate = speed.timing_hints().nominal_bitrate_hz;
        let period = TIMER_CLOCK_HZ / bitrate;
        let t1h = period * 3 / 4;
        let t0h = period * 3 / 8;
        let freq = Hertz(bitrate);

        // Cached ARR values. The bidir telemetry response uses a bit
        // period 5/4× longer than the TX bit period (so it's *slower*,
        // not faster). `OversamplingConfig::default()` wants 3 samples
        // per response bit, so sample period in timer ticks
        // = period_TX × 5/4 / 3 = period_TX × 5/12. ARR = ticks − 1.
        let tx_arr = period - 1;
        let rx_arr = (period * 5 / 12).saturating_sub(1);

        // ---- TIM3: CH3 (PB0), CH4 (PB1), CH2 (PB5), CH1 (PB4) ----
        // Output type depends on mode:
        //   - Bidir: open-drain + pull-up. The ESC also drives the
        //     line during its telemetry response phase; OD avoids
        //     push-pull contention if our cleanup races the response.
        //   - Standard: push-pull. We own the wire end-to-end, and a
        //     push-pull HIGH is crisper for the ESC's RC-time-constant
        //     detection on the rising edge of each bit cell.
        let pin_otype = if bidir { OutputType::OpenDrain } else { OutputType::PushPull };
        let mut tim3_pwm = SimplePwm::new(
            tim3,
            Some(PwmPin::new(pb4, pin_otype)), // CH1 → M4
            Some(PwmPin::new(pb5, pin_otype)), // CH2 → M3
            Some(PwmPin::new(pb0, pin_otype)), // CH3 → M1
            Some(PwmPin::new(pb1, pin_otype)), // CH4 → M2
            freq,
            CountingMode::EdgeAlignedUp,
        );
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            let mut c = tim3_pwm.channel(ch);
            c.set_duty_cycle(0);
            c.enable();
        }

        // Bidir-only: invert CCER polarity and engage pull-up.
        // With CCxP=1 and PWM Mode 1 the timer drives LOW while
        // counter < CCR and goes high-Z otherwise; the pull-up takes
        // the line back HIGH. CCR = t1h → 75 % LOW pulse ("1"),
        // CCR = t0h → 37.5 % LOW pulse ("0"), CCR = 0 → idle HIGH.
        //
        // Standard DShot keeps CCxP=0 (default after SimplePwm::new)
        // so the push-pull timer drives HIGH while counter < CCR and
        // LOW otherwise — exactly the conventional bit shape.
        if bidir {
            pac::TIM3.ccer().modify(|w| {
                w.set_ccp(0, true); // CH1
                w.set_ccp(1, true); // CH2
                w.set_ccp(2, true); // CH3
                w.set_ccp(3, true); // CH4
            });
            pac::GPIOB.pupdr().modify(|w| {
                w.set_pupdr(0, pac::gpio::vals::Pupdr::PULL_UP);
                w.set_pupdr(1, pac::gpio::vals::Pupdr::PULL_UP);
                w.set_pupdr(4, pac::gpio::vals::Pupdr::PULL_UP);
                w.set_pupdr(5, pac::gpio::vals::Pupdr::PULL_UP);
            });
        }

        // DShot DMA buffer — placed in SRAM1
        let buf_tim3: &'static mut [u32; TIM3_BUF_LEN] =
            unsafe { &mut *(BUF_TIM3_ADDR as *mut [u32; TIM3_BUF_LEN]) };

        let rx_buffer: &'static mut [u16; 256] =
            unsafe { &mut *(RX_BUFFER_ADDR as *mut [u16; 256]) };

        defmt::info!(
            "DShot init: speed={=?} bidir={=bool} tx_arr={=u32} rx_arr={=u32} t0h={=u32} t1h={=u32}",
            speed,
            bidir,
            tx_arr,
            rx_arr,
            t0h,
            t1h,
        );

        Self {
            tim3: tim3_pwm,
            dma_tim3_up,
            dma_tim3_rx,
            t1h,
            t0h,
            tx_arr,
            rx_arr,
            bidir,
            buf_tim3,
            rx_buffer,
            decoders: [
                uf_dshot::BidirDecoder::new(uf_dshot::OversamplingConfig::default()),
                uf_dshot::BidirDecoder::new(uf_dshot::OversamplingConfig::default()),
                uf_dshot::BidirDecoder::new(uf_dshot::OversamplingConfig::default()),
                uf_dshot::BidirDecoder::new(uf_dshot::OversamplingConfig::default()),
            ],
            frame_count: 0,
        }
    }

    pub fn log_config(&self) {
        let a = self.buffer_addresses();
        defmt::info!("DShot buffer: tim3={=u32:08x}", a);
        // DMA1 lives in the D2 domain on the H7. SRAM1/2/3 (0x3000…)
        // and AXI_SRAM (0x2400…) are both reachable; DTCM/ITCM
        // (0x2000…/0x0000…) are D1-domain TCM and *not* reachable
        // by DMA1.
        defmt::assert!(
            a >= 0x2400_0000,
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

    pub async fn send_throttles_and_receive(&mut self, frames: [EncodedFrame; 4]) -> [Result<uf_dshot::TelemetryFrame, uf_dshot::TelemetryError>; 4] {
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

        // Wait for the trailing zero cells to finish (8 cells at the
        // current TX bit period). At DShot600 that's ~13 µs; at
        // DShot300 ~27 µs; at DShot150 ~53 µs. Pad by 2 µs and scale.
        let trailing_us: u32 = ((self.tx_arr + 1) * 8) / 240 + 2;
        embassy_time::Timer::after_micros(trailing_us as u64).await;

        if !self.bidir {
            // Standard DShot: no telemetry. Skip the entire RX phase
            // and return a placeholder per channel so the caller's
            // tally still updates (NoEdge == "we never saw an edge"
            // is the cleanest match — semantically there was no
            // response to start with).
            self.frame_count = self.frame_count.wrapping_add(1);
            if self.frame_count.is_multiple_of(800) {
                self.log_runtime_state();
            }
            return [
                Err(uf_dshot::TelemetryError::NoEdge),
                Err(uf_dshot::TelemetryError::NoEdge),
                Err(uf_dshot::TelemetryError::NoEdge),
                Err(uf_dshot::TelemetryError::NoEdge),
            ];
        }

        // ---- Rx Phase (bidir only) ----
        // 1. Reconfigure pins to Input PullUp.
        pac::GPIOB.moder().modify(|w| {
            w.set_moder(0, pac::gpio::vals::Moder::INPUT);
            w.set_moder(1, pac::gpio::vals::Moder::INPUT);
            w.set_moder(4, pac::gpio::vals::Moder::INPUT);
            w.set_moder(5, pac::gpio::vals::Moder::INPUT);
        });
        pac::GPIOB.pupdr().modify(|w| {
            w.set_pupdr(0, pac::gpio::vals::Pupdr::PULL_UP);
            w.set_pupdr(1, pac::gpio::vals::Pupdr::PULL_UP);
            w.set_pupdr(4, pac::gpio::vals::Pupdr::PULL_UP);
            w.set_pupdr(5, pac::gpio::vals::Pupdr::PULL_UP);
        });

        // 2. Switch TIM3 ARR to the cached RX sample rate.
        // `OversamplingConfig::default()` expects 3 samples per
        // response bit. The bidir response bit cell is 5/4 × the TX
        // bit period, so the matching sample period = period × 4/15;
        // `rx_arr` is that minus one. Cached at construction so this
        // tracks whatever DShot speed was selected.
        pac::TIM3.arr().modify(|w| w.set_arr(self.rx_arr as u16));

        // 3. Start Rx DMA reading GPIOB_IDR.
        let rx_req = <DMA1_CH6 as UpDma<TIM3>>::request(&self.dma_tim3_rx);
        pac::TIM3.dier().modify(|w| w.set_ude(true));

        unsafe {
            use embassy_stm32::dma::{Transfer, TransferOptions};
            Transfer::new_read(
                self.dma_tim3_rx.reborrow(),
                rx_req,
                pac::GPIOB.idr().as_ptr() as *mut u16,
                self.rx_buffer.as_mut_slice(),
                TransferOptions::default()
            ).await;
        }
        pac::TIM3.dier().modify(|w| w.set_ude(false));

        // 4. Restore pins and timer for the next cycle.
        // PUPDR stays PULL_UP — with open-drain alternate-function
        // output the HIGH idle is provided by the pull-up.
        pac::TIM3.arr().modify(|w| w.set_arr(self.tx_arr as u16));
        pac::GPIOB.moder().modify(|w| {
            w.set_moder(0, pac::gpio::vals::Moder::ALTERNATE);
            w.set_moder(1, pac::gpio::vals::Moder::ALTERNATE);
            w.set_moder(4, pac::gpio::vals::Moder::ALTERNATE);
            w.set_moder(5, pac::gpio::vals::Moder::ALTERNATE);
        });

        // 5. Decode telemetry.
        // M1=CH3(PB0), M2=CH4(PB1), M3=CH2(PB5), M4=CH1(PB4)
        let frame1 = self.decoders[0].decode_frame_tuned_port_samples_u16(self.rx_buffer, 1 << 0);
        let frame2 = self.decoders[1].decode_frame_tuned_port_samples_u16(self.rx_buffer, 1 << 1);
        let frame3 = self.decoders[2].decode_frame_tuned_port_samples_u16(self.rx_buffer, 1 << 5);
        let frame4 = self.decoders[3].decode_frame_tuned_port_samples_u16(self.rx_buffer, 1 << 4);

        self.frame_count = self.frame_count.wrapping_add(1);

        if self.frame_count.is_multiple_of(800) {
            self.log_runtime_state();
        }

        [frame1, frame2, frame3, frame4]
    }

    pub fn buffer_addresses(&self) -> u32 {
        self.buf_tim3.as_ptr() as u32
    }
}
