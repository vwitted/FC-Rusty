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
    /// Run the plant-characterisation step profile instead of holding a
    /// constant throttle, capture the response, and dump it afterwards.
    ///
    /// Ignores `motor_pct` entirely: the profile drives all four motors
    /// together from `plant_capture::PROFILE`, so a capture is the same
    /// experiment every time and the four fits are comparable.
    pub profile: bool,
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

/// Parse + clamp raw string inputs into a `MotorTestConfig`. Pure (no env
/// IO) so it can be unit-tested with arbitrary inputs.
fn parse_config(
    motor: [Option<&str>; 4],
    bidir: Option<&str>,
    loop_khz: Option<&str>,
    profile: Option<&str>,
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
    // Opt-in only. An unset PROFILE must never start a scripted run that
    // ramps the motors on its own.
    let profile = matches!(profile.map(|s| s.trim()), Some("1"));
    MotorTestConfig {
        motor_pct,
        bidir,
        loop_khz,
        profile,
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
        option_env!("PROFILE"),
    )
}

/// Bench motor-test entry point. Never returns. Drives the DShot driver
/// directly from the build-time config; no arming, RC, PID, or mixer.
#[cfg(feature = "firmware")]
pub async fn run(p: embassy_stm32::Peripherals) -> ! {
    use crate::drivers::dshot_bitbang::DshotBitbang;
    use crate::drivers::dshot_frame::DshotFrame;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_time::{Duration, Ticker, Timer};

    let cfg = resolve_config();

    // Build stamp first, before anything else can scroll it away. A stale
    // board decoded against a fresh ELF yields garbage — defmt resolves
    // format strings by index, so a shifted index prints one log line's
    // arguments through another's format string, which reads as a plausible
    // but nonsensical hardware result rather than as an error. Compare this
    // against the stamp echoed by scripts/flash-motor-test.sh.
    defmt::info!("motor-test build: [{=str}]", env!("FC_BUILD_STAMP"));

    if cfg.profile {
        // Deliberately a different warning. The ordinary motor test wants
        // props OFF; a plant capture is worthless with them off, because
        // the lag being measured is dominated by the aerodynamic load on
        // the prop. So this run needs props ON, which makes it the more
        // dangerous of the two, and it must not read as the same thing.
        defmt::warn!(
            "PLANT CAPTURE — PROPS ON, AIRCRAFT SECURED. All four motors, {=u32} ms profile to {=u8}% in 5s",
            crate::plant_capture::profile_ms(),
            crate::plant_capture::PROFILE.iter().fold(0u8, |m, h| if h.pct > m { h.pct } else { m }),
        );
    } else {
        defmt::warn!(
            "MOTOR TEST — REMOVE PROPS. Spinning in 5s: M1={}% M2={}% M3={}% M4={}% bidir={} loop={}kHz",
            cfg.motor_pct[0],
            cfg.motor_pct[1],
            cfg.motor_pct[2],
            cfg.motor_pct[3],
            cfg.bidir,
            cfg.loop_khz,
        );
    }

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

    let mut dshot = DshotBitbang::new(
        p.TIM1, p.DMA2_CH2, p.PA0, p.PA1, p.PA2, p.PA3, cfg.bidir,
    );

    defmt::info!("motor-test: arming ESCs (zero throttle, 3s)");
    for _ in 0..(cfg.loop_khz as u32 * 3000) {
        // Under bidir the ESC replies to every frame, so the RX phase (which
        // returns the pins to INPUT) must run even when the reply is not
        // wanted — otherwise our push-pull output fights the ESC's.
        if cfg.bidir {
            dshot.send_and_receive(stop).await;
        } else {
            dshot.send(stop).await;
        }
        ticker.next().await;
    }

    if cfg.profile {
        run_profile(&mut dshot, &mut ticker, cfg).await;
    }

    defmt::info!("motor-test: driving motors");
    loop {
        if cfg.bidir {
            let telem = dshot.send_and_decode(frames).await;
            n = n.wrapping_add(1);
            if n % log_every == 0 {
                defmt::info!(
                    "motor-test RX: M1={=?} M2={=?} M3={=?} M4={=?}",
                    telem[0],
                    telem[1],
                    telem[2],
                    telem[3],
                );
            }
        } else {
            dshot.send(frames).await;
        }
        ticker.next().await;
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
    fn profile_is_opt_in_and_only_on_an_exact_one() {
        // An unset or fat-fingered PROFILE must not start a scripted run
        // that ramps the motors by itself. Anything but "1" is off.
        assert!(!parse_config([None; 4], None, None, None).profile);
        assert!(parse_config([None; 4], None, None, Some("1")).profile);
        for junk in ["0", "", "yes", "true", "2", " "] {
            assert!(
                !parse_config([None; 4], None, None, Some(junk)).profile,
                "PROFILE={junk:?} must not enable the profile"
            );
        }
        // Whitespace around a real 1 is still a 1 -- shell quoting adds it.
        assert!(parse_config([None; 4], None, None, Some(" 1 ")).profile);
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

/// Fly the characterisation profile, capture the response to RAM, stop
/// the motors, then dump.
///
/// The three phases are separate on purpose. `logger::putc` busy-waits on
/// USART6 TXE (~87 us/byte) inside a critical_section, so emitting a
/// sample as it is taken would stall the DShot loop for milliseconds per
/// line -- at 500 Hz that is not a measurement, it is a different
/// experiment. Capture is a struct copy into RAM; nothing goes on the
/// wire until the motors are stopped and the timing no longer matters.
///
/// This is also why the buffer is RAM and not flash: an H7 flash program
/// stalls the bus it is issued from, and while bank 2 could be written
/// from bank-1 code without that penalty, a bench run that lasts seconds
/// and ends next to a USB cable has no need of it. Flight is the case
/// that needs flash, and `plant_log::RECORD_LEN` is already one flash
/// word so the format will not have to change.
#[cfg(feature = "motor-test")]
async fn run_profile(
    dshot: &mut crate::drivers::dshot_bitbang::DshotBitbang<'static>,
    ticker: &mut embassy_time::Ticker,
    cfg: MotorTestConfig,
) {
    use crate::drivers::dshot_bb_decode::BbTelemetry;
    use crate::drivers::dshot_frame::DshotFrame;
    use crate::plant_capture::{self, Capture};
    use crate::plant_log::{PlantSample, NO_TELEMETRY};

    const N: usize = plant_capture::capacity();
    // Static, not a local: N samples at 32 bytes is tens of kilobytes and
    // an async fn's locals live in its future, which the executor holds
    // in a fixed-size arena.
    static mut BUF: Capture<N> = Capture::new();
    // SAFETY: the motor-test build runs exactly one task and this is the
    // only reference taken anywhere in it.
    let cap: &mut Capture<N> = unsafe { &mut *(&raw mut BUF) };

    let loop_hz = cfg.loop_khz as u32 * 1000;
    // Frames between captured samples. The DShot loop must keep running at
    // full rate -- the ESC needs a continuous frame stream -- so sampling
    // is decimation, not a slower loop.
    let decim = (loop_hz / plant_capture::SAMPLE_HZ).max(1);

    defmt::info!(
        "plant capture: {=u32} ms profile, {=u32} Hz sampling, {=usize} sample buffer",
        plant_capture::profile_ms(),
        plant_capture::SAMPLE_HZ,
        N,
    );

    let mut frame: u32 = 0;
    loop {
        let t_ms = frame * 1000 / loop_hz;
        let Some(pct) = plant_capture::command_at(t_ms) else {
            break;
        };

        let f: [DshotFrame; 4] =
            core::array::from_fn(|_| DshotFrame::from_normalised(pct as f32 / 100.0, cfg.bidir));

        if cfg.bidir {
            let telem = dshot.send_and_decode(f).await;
            if frame % decim == 0 {
                let period_us = core::array::from_fn(|i| match telem[i] {
                    // Periods above u16 mean the rotor is barely turning,
                    // which is not a regime this fit covers. Recorded as
                    // absent rather than wrapped into a plausible number.
                    BbTelemetry::Erpm { period_us } if period_us > 0 && period_us <= u16::MAX as u32 => {
                        period_us as u16
                    }
                    _ => NO_TELEMETRY,
                });
                cap.push(PlantSample {
                    t_ms,
                    cmd: [plant_capture::cmd_units(pct); 4],
                    period_us,
                    // No IMU in this build; see PlantSample::gyro_dps10.
                    gyro_dps10: [0; 3],
                });
            }
        } else {
            // Without bidir there is no telemetry and therefore nothing to
            // fit. Still drive the profile so the run is not silently
            // different, but say so.
            dshot.send(f).await;
        }

        frame = frame.wrapping_add(1);
        ticker.next().await;
    }

    // Motors off BEFORE anything touches the UART. The dump takes seconds
    // of blocked interrupts and the ESCs must not be holding throttle
    // through it.
    let stop: [DshotFrame; 4] = [DshotFrame::motor_stop(cfg.bidir); 4];
    for _ in 0..(cfg.loop_khz as u32 * 500) {
        if cfg.bidir {
            dshot.send_and_receive(stop).await;
        } else {
            dshot.send(stop).await;
        }
        ticker.next().await;
    }

    if !cfg.bidir {
        defmt::error!("plant capture: BIDIR=0, so no telemetry was captured — nothing to fit");
        return;
    }

    let n = cap.len();
    let with_telem = cap
        .as_slice()
        .iter()
        .filter(|s| s.period_us.iter().any(|&p| p != NO_TELEMETRY))
        .count();
    defmt::info!(
        "plant capture done: {=usize} samples ({=usize} with telemetry), {=u32} dropped. Dumping.",
        n,
        with_telem,
        cap.dropped(),
    );
    defmt::info!("PLANT_HEADER,{=str}", crate::plant_log::CSV_HEADER);

    for s in cap.as_slice().iter() {
        defmt::info!(
            "PLANT,{=u32},{=u16},{=u16},{=u16},{=u16},{=u16},{=u16},{=u16},{=u16},{=i16},{=i16},{=i16}",
            s.t_ms,
            s.cmd[0], s.cmd[1], s.cmd[2], s.cmd[3],
            s.period_us[0], s.period_us[1], s.period_us[2], s.period_us[3],
            s.gyro_dps10[0], s.gyro_dps10[1], s.gyro_dps10[2],
        );
    }
    defmt::info!("PLANT_END");
}
