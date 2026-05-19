// dshot_diag.rs — DShot peripheral-state instrumentation.
//
// All four motors are still failing to decode on hardware (2026-04-22
// session, see docs/motor-bringup-log.md). DTCM-buffer hypothesis was
// disproved 2026-04-25: relocating to SRAM2 didn't change behaviour.
// We were debugging the loudest peripheral on the board with zero
// driver-level instrumentation, so this module fills that gap until
// the oscilloscope arrives.
//
// Two entry points:
//   - log_boot_state(): one-shot dump of caches, all three timers'
//     full PWM/DMA register state, and a write-then-readback canary
//     on the SRAM2 DMA buffers. Run once after DshotQuad::new.
//   - log_runtime_state(): post-transfer DMA stream state + the last
//     CCR value the timer holds. Cheap; call ~1 Hz.
//
// Anything surprising here narrows the root cause before scope work.

use embassy_stm32::pac;

/// RCC TIMPRE bit. F7-specific: when 0 the timer clock is APBx*2
/// (when APB prescaler > 1); when 1 it's APBx*4. With our APB1
/// prescaler of /4, TIMPRE=1 doubles the timer clock to 216 MHz —
/// our PSC=0/ARR=179 would then produce DShot1200 timing, which
/// BLHeli_S can't decode (max DShot600).
pub fn log_timpre() {
    use pac::rcc::vals::Timpre;
    let t = pac::RCC.cfgr().read().timpre();
    let raw = t.to_bits();
    let label: &str = match t {
        Timpre::DEFAULT_X2 => "DEFAULT_X2 (timer = APB*2 = 240 MHz expected)",
        Timpre::DEFAULT_X4 => "DEFAULT_X4 (timer = HCLK = 480 MHz)",
    };
    defmt::info!("RCC.CFGR.TIMPRE = {=u8} ({=str})", raw, label);
    if !matches!(t, Timpre::DEFAULT_X2) {
        defmt::error!("TIMPRE != DEFAULT_X2 — actual bit rate is double what we configured");
    }
}

/// Dump full per-pin state for the four DShot motor pins on this
/// board (TIM3 CH1..4 → PB4/PB5/PB0/PB1). Reports MODER, OTYPER,
/// OSPEEDR, PUPDR, and AFR so we can verify the pad matches what
/// the embassy `PwmPin` config asked for. With the push-pull TX
/// fix in place, the expected steady-state is:
///   MODER=2 (AF), OTYPER=0 (push-pull), OSPEEDR=1 (Medium),
///   PUPDR=1 (PullUp, bidir mode), AF=2 (TIM3_CH1..4).
pub fn log_gpio_pins() {
    let b = pac::GPIOB;
    let b_moder = b.moder().read().0;
    let b_otype = b.otyper().read().0;
    let b_ospd  = b.ospeedr().read().0;
    let b_pupd  = b.pupdr().read().0;
    let b_afrl  = b.afr(0).read().0;

    // Per-pin field extraction. MODER/OSPEEDR/PUPDR are 2 bits/pin,
    // OTYPER is 1 bit/pin, AFR is 4 bits/pin (first 8 pins in AFRL).
    let pin_state = |p: u32| -> (u32, u32, u32, u32, u32) {
        let mode = (b_moder >> (p * 2)) & 0b11;
        let otype = (b_otype >> p) & 0b1;
        let ospd  = (b_ospd  >> (p * 2)) & 0b11;
        let pupd  = (b_pupd  >> (p * 2)) & 0b11;
        let af    = (b_afrl  >> (p * 4)) & 0xF;
        (mode, otype, ospd, pupd, af)
    };

    let (m0, t0, s0, u0, a0) = pin_state(0);  // PB0 → CH3 → M1
    let (m1, t1, s1, u1, a1) = pin_state(1);  // PB1 → CH4 → M2
    let (m4, t4, s4, u4, a4) = pin_state(4);  // PB4 → CH1 → M4
    let (m5, t5, s5, u5, a5) = pin_state(5);  // PB5 → CH2 → M3

    defmt::info!(
        "GPIO PB0 (M1/CH3): MODER={=u32} OTYPER={=u32} OSPEEDR={=u32} PUPDR={=u32} AF={=u32} (want 2/0/1/1/2)",
        m0, t0, s0, u0, a0
    );
    defmt::info!(
        "GPIO PB1 (M2/CH4): MODER={=u32} OTYPER={=u32} OSPEEDR={=u32} PUPDR={=u32} AF={=u32} (want 2/0/1/1/2)",
        m1, t1, s1, u1, a1
    );
    defmt::info!(
        "GPIO PB4 (M4/CH1): MODER={=u32} OTYPER={=u32} OSPEEDR={=u32} PUPDR={=u32} AF={=u32} (want 2/0/1/1/2)",
        m4, t4, s4, u4, a4
    );
    defmt::info!(
        "GPIO PB5 (M3/CH2): MODER={=u32} OTYPER={=u32} OSPEEDR={=u32} PUPDR={=u32} AF={=u32} (want 2/0/1/1/2)",
        m5, t5, s5, u5, a5
    );
}

/// Read each timer's CNT three times in a row. If CNT is changing,
/// the timer is alive; if stuck, CEN is being cleared somewhere.
pub fn log_timer_running() {
    let a2 = pac::TIM2.cnt().read();
    let b2 = pac::TIM2.cnt().read();
    let c2 = pac::TIM2.cnt().read();
    let a3 = pac::TIM3.cnt().read().cnt();
    let b3 = pac::TIM3.cnt().read().cnt();
    let c3 = pac::TIM3.cnt().read().cnt();
    let a4 = pac::TIM4.cnt().read().cnt();
    let b4 = pac::TIM4.cnt().read().cnt();
    let c4 = pac::TIM4.cnt().read().cnt();
    defmt::info!("TIM2 CNT samples: {=u32} {=u32} {=u32}", a2, b2, c2);
    defmt::info!("TIM3 CNT samples: {=u16} {=u16} {=u16}", a3, b3, c3);
    defmt::info!("TIM4 CNT samples: {=u16} {=u16} {=u16}", a4, b4, c4);
}

/// SCB_CCR cache enable bits — D-cache uncoherent with DMA is a
/// classic F7 footgun, so verify it's off (Embassy default).
pub fn log_caches() {
    let ccr = unsafe { core::ptr::read_volatile(0xE000_ED14 as *const u32) };
    let ic = (ccr >> 17) & 1;
    let dc = (ccr >> 16) & 1;
    defmt::info!("SCB_CCR = {=u32:08x}  IC={=u32}  DC={=u32}", ccr, ic, dc);
    if dc == 1 {
        defmt::warn!("D-cache enabled — DMA buffers need MPU non-cacheable region");
    }
}

/// Dump TIM2 (32-bit) PWM+DMA configuration registers.
pub fn log_tim2_config() {
    let t = pac::TIM2;
    defmt::info!(
        "TIM2: PSC={=u16} ARR={=u32} CR1={=u32:08x} DIER={=u32:08x} DCR={=u32:08x} CCMR1={=u32:08x} CCER={=u32:08x} CCR1={=u32} CCR2={=u32}",
        t.psc().read(),
        t.arr().read(),
        t.cr1().read().0,
        t.dier().read().0,
        t.dcr().read().0,
        t.ccmr_output(0).read().0,
        t.ccer().read().0,
        t.ccr(0).read(),
        t.ccr(1).read(),
    );
}

/// Dump TIM3 (16-bit) PWM+DMA configuration registers.
///
/// CCMR1 covers CH1/CH2; CCMR2 covers CH3/CH4 — we drive all four,
/// so both matter. Per-channel OCxM (PWM mode 1 = 0b110) and OCxPE
/// (preload enable) live inside CCMRx.
pub fn log_tim3_config() {
    let t = pac::TIM3;
    defmt::info!(
        "TIM3: PSC={=u16} ARR={=u16} CR1={=u32:08x} DIER={=u32:08x} DCR={=u32:08x} CCMR1={=u32:08x} CCMR2={=u32:08x} CCER={=u32:08x}",
        t.psc().read(),
        t.arr().read().arr(),
        t.cr1().read().0,
        t.dier().read().0,
        t.dcr().read().0,
        t.ccmr_output(0).read().0,
        t.ccmr_output(1).read().0,
        t.ccer().read().0,
    );
    defmt::info!(
        "TIM3 CCR: ch1={=u16} ch2={=u16} ch3={=u16} ch4={=u16}",
        t.ccr(0).read().ccr(),
        t.ccr(1).read().ccr(),
        t.ccr(2).read().ccr(),
        t.ccr(3).read().ccr(),
    );
}

/// Dump TIM4 (16-bit) PWM+DMA configuration registers.
pub fn log_tim4_config() {
    let t = pac::TIM4;
    defmt::info!(
        "TIM4: PSC={=u16} ARR={=u16} CR1={=u32:08x} DIER={=u32:08x} DCR={=u32:08x} CCMR1={=u32:08x} CCER={=u32:08x} CCR1={=u16}",
        t.psc().read(),
        t.arr().read().arr(),
        t.cr1().read().0,
        t.dier().read().0,
        t.dcr().read().0,
        t.ccmr_output(0).read().0,
        t.ccer().read().0,
        t.ccr(0).read().ccr(),
    );
}

/// Decode + log the post-transfer state of a DMA1 stream.
///
/// `name`: "TIM2_UP", "TIM3_UP", "TIM4_UP" etc.
/// `stream`: 0..=7 within DMA1.
pub fn log_dma1_stream(name: &'static str, stream: usize) {
    let dma = pac::DMA1;
    let st = dma.st(stream);
    let cr = st.cr().read().0;
    let ndtr = st.ndtr().read().0 & 0xFFFF;
    let par = st.par().read();
    let m0ar = st.m0ar().read();
    let fcr = st.fcr().read().0;

    // Per-stream flag bits in LISR (streams 0..=3) / HISR (4..=7).
    // Within each stream's 6-bit slot: FE(0) - DME(2) TE(3) HT(4) TC(5).
    let isr_word = dma.isr(if stream < 4 { 0 } else { 1 }).read().0;
    let shift = match stream % 4 {
        0 => 0, 1 => 6, 2 => 16, 3 => 22, _ => 0,
    };
    let bits = (isr_word >> shift) & 0x3F;
    let fe  = (bits & 0x01) != 0;
    let dme = (bits & 0x04) != 0;
    let te  = (bits & 0x08) != 0;
    let ht  = (bits & 0x10) != 0;
    let tc  = (bits & 0x20) != 0;

    defmt::info!(
        "DMA1 S{=usize} {=str}: EN={=bool} NDTR={=u32} PAR={=u32:08x} M0AR={=u32:08x} CR={=u32:08x} FCR={=u32:08x} | TC={=bool} HT={=bool} TE={=bool} DME={=bool} FE={=bool}",
        stream, name,
        (cr & 1) != 0,
        ndtr, par, m0ar, cr, fcr,
        tc, ht, te, dme, fe,
    );
    if te || dme {
        defmt::warn!("DMA1 S{=usize} {=str}: error flags set (TE/DME)", stream, name);
    }
    if fe && !te && !dme {
        // FE (FIFO Error) alone is benign on H7 burst DMA — fires when
        // the FIFO drains at transfer completion. Not a data-integrity issue.
        defmt::trace!("DMA1 S{=usize} {=str}: FE flag (benign)", stream, name);
    }

    // Clear all six flags for this stream so the next 1 Hz log
    // reflects only the past second, not everything since boot.
    let mut ifcr = 0u32;
    ifcr |= 0x3D << shift;  // CTCIF | CHTIF | CTEIF | CDMEIF | CFEIF (bits 5,4,3,2,0)
    pac::DMA1.ifcr(if stream < 4 { 0 } else { 1 }).write_value(
        pac::dma::regs::Ixr(ifcr)
    );
}

/// Write a known pattern into a buffer, read it back, log mismatches.
/// Catches MPU/aliasing/bus issues at the hardcoded SRAM2 addresses.
pub fn canary_check(name: &'static str, buf: &mut [u16]) {
    const PATTERN_A: u16 = 0xA5A5;
    const PATTERN_B: u16 = 0x5A5A;

    for (i, cell) in buf.iter_mut().enumerate() {
        *cell = if i & 1 == 0 { PATTERN_A } else { PATTERN_B };
    }
    // Fence to make sure stores are committed before the readback.
    cortex_m::asm::dsb();

    let mut bad = 0u32;
    for (i, cell) in buf.iter().enumerate() {
        let expect = if i & 1 == 0 { PATTERN_A } else { PATTERN_B };
        if *cell != expect {
            if bad < 4 {
                defmt::warn!(
                    "canary {=str}[{=usize}] = {=u16:04x} (expected {=u16:04x})",
                    name, i, *cell, expect,
                );
            }
            bad += 1;
        }
    }
    if bad == 0 {
        defmt::info!("canary {=str}: ok ({=usize} cells)", name, buf.len());
    } else {
        defmt::error!("canary {=str}: {=u32} mismatches in {=usize} cells",
                       name, bad, buf.len());
    }

    // Restore zeros so first send() starts from a clean buffer.
    buf.fill(0);
    cortex_m::asm::dsb();
}
