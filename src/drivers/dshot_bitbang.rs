// dshot_bitbang.rs — bit-banged DShot for the DAKEFPVH743.
//
// Reference: betaflight/src/platform/STM32/dshot_bitbang.c
//
// TIM1 is a pacer only: it drives no pin, it just generates a DMA request
// every state period. DMA writes 32-bit BSRR words to GPIOA, producing the
// waveform directly. The motor pins are plain GPIO throughout — never in
// alternate-function mode — which is why none of the compare-register or
// AF-handover failure modes of dshot_hw.rs exist here.
//
// M1..M4 are PA0..PA3, one port, so all four motors share one buffer and one
// DMA stream. Per-pin data lives in the middle state of each symbol.
//
// Timing (DShot300, TIM1 at 240 MHz): 3 states per symbol → 900 kHz pacer,
// ARR = 240e6/900e3 - 1 = 265.
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

use super::dshot_bb_decode::RX_BUF_LEN;
use super::dshot_bb_frame::{BB_BUF_LEN, output_data_clear, output_data_init, output_data_set};
use super::dshot_frame::DshotFrame;

/// PA0..PA3 = M1..M4.
const PORT_MASK: u16 = 0b1111;
const MOTOR_PINS: [u8; 4] = [0, 1, 2, 3];

/// TIM1 counter period for the transmit pacer. 240 MHz / 900 kHz - 1.
const TX_ARR: u32 = 265;

/// TIM1 counter period for the receive pacer. The reply runs at 5/4 the DShot
/// bit rate and we oversample 3×, so 300 kHz × 5/4 × 3 = 1.125 MHz.
/// 240 MHz / 1.125 MHz - 1 = 212. (BF derives this as
/// `outputFreq * 5 * 2 * OVER_SAMPLE / 24`, which is `outputFreq × 5/4`.)
const RX_ARR: u32 = 212;

/// Frame count at which `send_and_receive` dumps a one-shot RX probe, so the
/// bench run reports what arrived without needing a scope.
const RX_PROBE_FRAME: u32 = 100;

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
            for p in MOTOR_PINS {
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
            PORT_MASK,
        );

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
            output_data_init(buf, PORT_MASK, self.bidir);
        } else {
            output_data_clear(buf);
        }
        for (i, pin) in MOTOR_PINS.iter().enumerate() {
            output_data_set(buf, *pin, frames[i].raw, self.bidir);
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
            for p in MOTOR_PINS {
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
            for p in MOTOR_PINS {
                w.set_moder(p as usize, pac::gpio::vals::Moder::OUTPUT);
            }
        });

        if self.frame_count == RX_PROBE_FRAME {
            let m1_low = rx.iter().filter(|&&s| s & 1 == 0).count();
            let transitions = rx.windows(2).filter(|w| (w[0] ^ w[1]) & 1 != 0).count();
            defmt::info!(
                "bitbang RX probe: {=usize} of {=usize} samples low on M1, {=usize} transitions",
                m1_low, RX_BUF_LEN, transitions,
            );
            defmt::info!(
                "  first 16 samples (M1 bit): {=u16:016b}",
                rx.iter().take(16).enumerate()
                    .fold(0u16, |acc, (i, &s)| acc | (((s & 1) as u16) << i)),
            );
        }

        *rx
    }
}

/// Drive the four motor pins to their idle level via BSRR.
fn set_idle_level(bidir: bool) {
    pac::GPIOA.bsrr().write(|w| {
        for p in MOTOR_PINS {
            if bidir {
                w.set_bs(p as usize, true); // bidir idles HIGH
            } else {
                w.set_br(p as usize, true); // plain DShot idles LOW
            }
        }
    });
}
