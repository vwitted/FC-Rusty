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
    let t = pac::RCC.dckcfgr1().read().timpre();
    let raw = t.to_bits();
    let label: &str = match t {
        Timpre::MUL2 => "MUL2 (timer = APB*2 = 108 MHz expected)",
        Timpre::MUL4 => "MUL4 (timer = HCLK = 216 MHz — DShot600 would be DShot1200)",
    };
    defmt::info!("RCC.DCKCFGR1.TIMPRE = {=u8} ({=str})", raw, label);
    if !matches!(t, Timpre::MUL2) {
        defmt::error!("TIMPRE != MUL2 — actual bit rate is double what we configured");
    }
}

/// Dump GPIO MODER/OSPEEDR/AFR for the four DShot pins. If any pin
/// isn't in AF mode at the right AF number with VeryHigh slew, the
/// signal at the pad won't match what the timer is generating.
pub fn log_gpio_pins() {
    let a = pac::GPIOA;
    let b = pac::GPIOB;
    let a_moder = a.moder().read().0;
    let a_ospd  = a.ospeedr().read().0;
    let a_afrh  = a.afr(1).read().0;
    let b_moder = b.moder().read().0;
    let b_ospd  = b.ospeedr().read().0;
    let b_afrl  = b.afr(0).read().0;

    // (pin, name, expected_af, raw moder, raw ospd, raw afr nibble)
    let pa15_mode = (a_moder >> 30) & 0b11;
    let pa15_ospd = (a_ospd  >> 30) & 0b11;
    let pa15_af   = (a_afrh  >> 28) & 0xF;     // PA15 in AFRH bits 28..=31
    let pb3_mode  = (b_moder >> 6)  & 0b11;
    let pb3_ospd  = (b_ospd  >> 6)  & 0b11;
    let pb3_af    = (b_afrl  >> 12) & 0xF;
    let pb4_mode  = (b_moder >> 8)  & 0b11;
    let pb4_ospd  = (b_ospd  >> 8)  & 0b11;
    let pb4_af    = (b_afrl  >> 16) & 0xF;
    let pb6_mode  = (b_moder >> 12) & 0b11;
    let pb6_ospd  = (b_ospd  >> 12) & 0b11;
    let pb6_af    = (b_afrl  >> 24) & 0xF;

    // Expected: MODER=2 (AF), OSPEEDR=3 (VeryHigh), AF: PA15=1 PB3=1 PB4=2 PB6=2
    defmt::info!("GPIO PA15 (M1/TIM2_CH1): MODER={=u32} OSPEEDR={=u32} AF={=u32} (want 2/3/1)",
                  pa15_mode, pa15_ospd, pa15_af);
    defmt::info!("GPIO PB3  (M2/TIM2_CH2): MODER={=u32} OSPEEDR={=u32} AF={=u32} (want 2/3/1)",
                  pb3_mode, pb3_ospd, pb3_af);
    defmt::info!("GPIO PB4  (M3/TIM3_CH1): MODER={=u32} OSPEEDR={=u32} AF={=u32} (want 2/3/2)",
                  pb4_mode, pb4_ospd, pb4_af);
    defmt::info!("GPIO PB6  (M4/TIM4_CH1): MODER={=u32} OSPEEDR={=u32} AF={=u32} (want 2/3/2)",
                  pb6_mode, pb6_ospd, pb6_af);
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
pub fn log_tim3_config() {
    let t = pac::TIM3;
    defmt::info!(
        "TIM3: PSC={=u16} ARR={=u16} CR1={=u32:08x} DIER={=u32:08x} DCR={=u32:08x} CCMR1={=u32:08x} CCER={=u32:08x} CCR1={=u16}",
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
    if te || dme || fe {
        defmt::warn!("DMA1 S{=usize} {=str}: error flags set", stream, name);
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
