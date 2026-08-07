//! Bench motor-test mode.
//!
//! Drives the DShot driver directly with build-time per-motor throttles,
//! fully decoupled from arming and the flight stack. The firmware `run()`
//! is compiled only into the binary under the `motor-test` cargo feature;
//! the pure config layer below is exercised by host tests.
//!
//! Spec: docs/superpowers/specs/2026-06-21-dshot-motor-test-design.md

/// Resolved, clamped motor-test configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorTestConfig {
    /// Per-motor throttle percent, post-clamp. 0 = stopped.
    pub motor_pct: [u8; 4],
    /// Bidirectional DShot.
    pub bidir: bool,
    /// Send-loop frequency in kHz (2..=8).
    pub loop_khz: u8,
    /// Which DShot driver to exercise: false = timer-DMA (dshot_hw), true = bit-bang.
    pub use_bitbang: bool,
}

/// Hard safety ceiling on any motor's throttle. Raising it requires a
/// deliberate source edit + reflash — intentional.
const MAX_PCT: u8 = 25;
/// Per-motor throttle when the env var is unset: a gentle spin, so a bare
/// motor-test flash actually tests motors. Explicit `Mx_PCT=0` still stops.
const DEFAULT_PCT: u8 = 5;
const DEFAULT_BIDIR: bool = true;
const DEFAULT_LOOP_KHZ: u8 = 8;
const MIN_LOOP_KHZ: u8 = 2;
const MAX_LOOP_KHZ: u8 = 8;
const DEFAULT_USE_BITBANG: bool = false;

/// Parse + clamp raw string inputs into a `MotorTestConfig`. Pure (no env
/// IO) so it can be unit-tested with arbitrary inputs.
fn parse_config(
    motor: [Option<&str>; 4],
    bidir: Option<&str>,
    loop_khz: Option<&str>,
    driver: Option<&str>,
) -> MotorTestConfig {
    let motor_pct = core::array::from_fn(|i| {
        motor[i]
            .and_then(|s| s.trim().parse::<u8>().ok())
            .unwrap_or(DEFAULT_PCT)
            .min(MAX_PCT)
    });
    let bidir = match bidir.map(|s| s.trim()) {
        Some("0") => false,
        Some("1") => true,
        _ => DEFAULT_BIDIR,
    };
    let loop_khz = loop_khz
        .and_then(|s| s.trim().parse::<u8>().ok())
        .unwrap_or(DEFAULT_LOOP_KHZ)
        .clamp(MIN_LOOP_KHZ, MAX_LOOP_KHZ);
    let use_bitbang = match driver.map(|s| s.trim()) {
        Some("bitbang") => true,
        Some("timer") => false,
        _ => DEFAULT_USE_BITBANG,
    };
    MotorTestConfig {
        motor_pct,
        bidir,
        loop_khz,
        use_bitbang,
    }
}

/// Byte range [i, j) of `b` after trimming ASCII whitespace; i == j if blank.
const fn trimmed(b: &[u8]) -> (usize, usize) {
    let (mut i, mut j) = (0, b.len());
    while i < j && (b[i] == b' ' || b[i] == b'\t' || b[i] == b'\n' || b[i] == b'\r') {
        i += 1;
    }
    while j > i && (b[j - 1] == b' ' || b[j - 1] == b'\t' || b[j - 1] == b'\n' || b[j - 1] == b'\r') {
        j -= 1;
    }
    (i, j)
}

/// Build-time env validation: unset or blank is fine (defaults apply), but a
/// set value must be a decimal u8. Rejecting garbage at compile time stops a
/// typo'd value from silently building default-configured firmware.
const fn env_u8_ok(v: Option<&str>) -> bool {
    let Some(s) = v else { return true };
    let b = s.as_bytes();
    let (mut i, j) = trimmed(b);
    if i == j {
        return true; // set-but-empty ≈ unset
    }
    let mut val: u32 = 0;
    while i < j {
        if b[i] < b'0' || b[i] > b'9' {
            return false;
        }
        val = val * 10 + (b[i] - b'0') as u32;
        if val > u8::MAX as u32 {
            return false;
        }
        i += 1;
    }
    true
}

/// Build-time env validation for BIDIR: unset, blank, "0" or "1" only.
const fn env_bidir_ok(v: Option<&str>) -> bool {
    let Some(s) = v else { return true };
    let b = s.as_bytes();
    let (i, j) = trimmed(b);
    i == j || (j - i == 1 && (b[i] == b'0' || b[i] == b'1'))
}

/// Build-time validation for DRIVER: unset, blank, "timer" or "bitbang" only.
const fn env_driver_ok(v: Option<&str>) -> bool {
    let Some(s) = v else { return true };
    let b = s.as_bytes();
    let (i, j) = trimmed(b);
    if i == j {
        return true;
    }
    let n = j - i;
    if n == 5 {
        return b[i] == b't' && b[i+1] == b'i' && b[i+2] == b'm' && b[i+3] == b'e' && b[i+4] == b'r';
    }
    if n == 7 {
        return b[i] == b'b' && b[i+1] == b'i' && b[i+2] == b't' && b[i+3] == b'b'
            && b[i+4] == b'a' && b[i+5] == b'n' && b[i+6] == b'g';
    }
    false
}

// Fail the motor-test build outright on unparseable values for the env vars
// it knows about. Unset vars still default; misspelt *names* remain
// undetectable — the startup banner logging the resolved config covers those.
#[cfg(feature = "motor-test")]
const _: () = {
    assert!(env_u8_ok(option_env!("M1_PCT")), "M1_PCT must be an integer 0-255");
    assert!(env_u8_ok(option_env!("M2_PCT")), "M2_PCT must be an integer 0-255");
    assert!(env_u8_ok(option_env!("M3_PCT")), "M3_PCT must be an integer 0-255");
    assert!(env_u8_ok(option_env!("M4_PCT")), "M4_PCT must be an integer 0-255");
    assert!(env_bidir_ok(option_env!("BIDIR")), "BIDIR must be 0 or 1");
    assert!(env_u8_ok(option_env!("LOOP_KHZ")), "LOOP_KHZ must be an integer (kHz)");
    assert!(env_driver_ok(option_env!("DRIVER")), "DRIVER must be `timer` or `bitbang`");
};

/// Read the build-time env values and resolve them into a clamped config.
#[allow(dead_code)] // used by the firmware `run()` in the binary build only
pub fn resolve_config() -> MotorTestConfig {
    parse_config(
        [
            option_env!("M1_PCT"),
            option_env!("M2_PCT"),
            option_env!("M3_PCT"),
            option_env!("M4_PCT"),
        ],
        option_env!("BIDIR"),
        option_env!("LOOP_KHZ"),
        option_env!("DRIVER"),
    )
}

/// Bench motor-test entry point. Never returns. Drives the DShot driver
/// directly from the build-time config; no arming, RC, PID, or mixer.
#[cfg(feature = "firmware")]
pub async fn run(p: embassy_stm32::Peripherals) -> ! {
    use crate::drivers::dshot_frame::{DshotFrame, DshotSpeed};
    use crate::drivers::dshot_hw::DshotQuad;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_time::{Duration, Ticker, Timer};

    let cfg = resolve_config();

    defmt::warn!(
        "MOTOR TEST — REMOVE PROPS. Spinning in 5s: M1={}% M2={}% M3={}% M4={}% bidir={} loop={}kHz",
        cfg.motor_pct[0],
        cfg.motor_pct[1],
        cfg.motor_pct[2],
        cfg.motor_pct[3],
        cfg.bidir,
        cfg.loop_khz,
    );

    // Countdown with LED blink (PD10, active-low on the DAKEFPV) as an
    // "alive" indicator before any frame is sent.
    let mut led = Output::new(p.PD10, Level::High, Speed::Low);
    for s in (1..=5).rev() {
        defmt::info!("motor-test: spinning in {}s", s);
        for _ in 0..5 {
            led.toggle();
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    // Per-motor frames are constant for the run; 0% maps to MotorStop.
    let frames: [DshotFrame; 4] =
        core::array::from_fn(|i| DshotFrame::from_normalised(cfg.motor_pct[i] as f32 / 100.0, cfg.bidir));

    let mut ticker = Ticker::every(Duration::from_micros(1000 / cfg.loop_khz as u64));
    let log_every: u32 = cfg.loop_khz as u32 * 100; // ~10 Hz
    let mut n: u32 = 0;

    // ESCs arm only after a sustained stream of valid zero-throttle frames;
    // a nonzero first frame locks them out. Stream MotorStop for 3s (expect
    // the ESC arm beeps during this window) before any real throttle.
    let stop: [DshotFrame; 4] = [DshotFrame::motor_stop(cfg.bidir); 4];

    if cfg.use_bitbang {
        let mut dshot = crate::drivers::dshot_bitbang::DshotBitbang::new(
            p.TIM1, p.DMA2_CH2, p.PA0, p.PA1, p.PA2, p.PA3, cfg.bidir,
        );
        defmt::info!("motor-test: arming ESCs (zero throttle, 3s) [bitbang]");
        for _ in 0..(cfg.loop_khz as u32 * 3000) {
            dshot.send(stop).await;
            ticker.next().await;
        }
        defmt::info!("motor-test: driving motors [bitbang]");
        loop {
            dshot.send(frames).await;
            ticker.next().await;
        }
    } else {
        let mut dshot = DshotQuad::new(
            p.TIM2,
            p.PA0,
            p.PA1,
            p.PA2,
            p.PA3,
            p.DMA1_CH2,
            p.DMA1_CH3,
            p.DMA1_CH4,
            p.DMA1_CH7,
            DshotSpeed::Dshot600,
            cfg.bidir,
        );
        defmt::info!("motor-test: arming ESCs (zero throttle, 3s)");
        for _ in 0..(cfg.loop_khz as u32 * 3000) {
            dshot.send_throttles_and_receive(stop).await;
            ticker.next().await;
        }

        defmt::info!("motor-test: driving motors");
        loop {
            let telem = dshot.send_throttles_and_receive(frames).await;
            n = n.wrapping_add(1);
            if cfg.bidir && n % log_every == 0 {
                defmt::info!(
                    "motor-test RX: M1={=?} M2={=?} M3={=?} M4={=?}",
                    telem[0],
                    telem[1],
                    telem[2],
                    telem[3],
                );
            }
            ticker.next().await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_when_all_unset() {
        let c = parse_config([None; 4], None, None, None);
        assert_eq!(c.motor_pct, [5, 5, 5, 5]); // bare motor test spins gently
        assert!(c.bidir);
        assert_eq!(c.loop_khz, 8);
    }

    #[test]
    fn explicit_zero_still_stops_motor() {
        let c = parse_config([Some("0"), None, None, None], None, None, None);
        assert_eq!(c.motor_pct[0], 0);
    }

    #[test]
    fn parses_each_motor_independently() {
        let c = parse_config([Some("3"), None, Some("7"), None], None, None, None);
        assert_eq!(c.motor_pct, [3, 5, 7, 5]);
    }

    #[test]
    fn clamps_motor_to_max_pct() {
        let c = parse_config([Some("90"), Some("25"), Some("26"), Some("0")], None, None, None);
        assert_eq!(c.motor_pct, [25, 25, 25, 0]);
    }

    #[test]
    fn bidir_explicit_zero_one_else_default() {
        assert!(!parse_config([None; 4], Some("0"), None, None).bidir);
        assert!(parse_config([None; 4], Some("1"), None, None).bidir);
        assert!(parse_config([None; 4], Some("yes"), None, None).bidir); // garbage → default true
    }

    #[test]
    fn driver_defaults_to_timer() {
        let c = parse_config([None; 4], None, None, None);
        assert!(!c.use_bitbang);
    }

    #[test]
    fn driver_selects_bitbang() {
        let c = parse_config([None; 4], None, None, Some("bitbang"));
        assert!(c.use_bitbang);
    }

    #[test]
    fn driver_garbage_falls_back_to_timer() {
        let c = parse_config([None; 4], None, None, Some("nonsense"));
        assert!(!c.use_bitbang);
    }

    #[test]
    fn env_driver_ok_accepts_only_known_drivers() {
        assert!(env_driver_ok(None));
        assert!(env_driver_ok(Some("")));
        assert!(env_driver_ok(Some(" timer ")));
        assert!(env_driver_ok(Some("bitbang")));
        assert!(!env_driver_ok(Some("bit-bang")));
        assert!(!env_driver_ok(Some("dma")));
    }

    #[test]
    fn env_u8_ok_accepts_unset_blank_and_integers() {
        assert!(env_u8_ok(None));
        assert!(env_u8_ok(Some(""))); // set-but-empty ≈ unset
        assert!(env_u8_ok(Some("  ")));
        assert!(env_u8_ok(Some("0")));
        assert!(env_u8_ok(Some(" 10 ")));
        assert!(env_u8_ok(Some("255")));
    }

    #[test]
    fn env_u8_ok_rejects_garbage() {
        assert!(!env_u8_ok(Some("ten")));
        assert!(!env_u8_ok(Some("10%")));
        assert!(!env_u8_ok(Some("-1")));
        assert!(!env_u8_ok(Some("256")));
        assert!(!env_u8_ok(Some("1.5")));
    }

    #[test]
    fn env_bidir_ok_accepts_only_unset_blank_zero_one() {
        assert!(env_bidir_ok(None));
        assert!(env_bidir_ok(Some("")));
        assert!(env_bidir_ok(Some("0")));
        assert!(env_bidir_ok(Some(" 1 ")));
        assert!(!env_bidir_ok(Some("false")));
        assert!(!env_bidir_ok(Some("true")));
        assert!(!env_bidir_ok(Some("2")));
    }

    #[test]
    fn loop_khz_clamped_to_range() {
        assert_eq!(parse_config([None; 4], None, Some("1"), None).loop_khz, 2);
        assert_eq!(parse_config([None; 4], None, Some("9"), None).loop_khz, 8);
        assert_eq!(parse_config([None; 4], None, Some("4"), None).loop_khz, 4);
        assert_eq!(parse_config([None; 4], None, Some("x"), None).loop_khz, 8); // garbage → default
    }
}
