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
}

/// Hard safety ceiling on any motor's throttle. Raising it requires a
/// deliberate source edit + reflash — intentional.
const MAX_PCT: u8 = 25;
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
) -> MotorTestConfig {
    let motor_pct = core::array::from_fn(|i| {
        motor[i]
            .and_then(|s| s.trim().parse::<u8>().ok())
            .unwrap_or(0)
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
    MotorTestConfig {
        motor_pct,
        bidir,
        loop_khz,
    }
}

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
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_when_all_unset() {
        let c = parse_config([None; 4], None, None);
        assert_eq!(c.motor_pct, [0, 0, 0, 0]);
        assert!(c.bidir);
        assert_eq!(c.loop_khz, 8);
    }

    #[test]
    fn parses_each_motor_independently() {
        let c = parse_config([Some("3"), None, Some("7"), None], None, None);
        assert_eq!(c.motor_pct, [3, 0, 7, 0]);
    }

    #[test]
    fn clamps_motor_to_max_pct() {
        let c = parse_config([Some("90"), Some("25"), Some("26"), Some("0")], None, None);
        assert_eq!(c.motor_pct, [25, 25, 25, 0]);
    }

    #[test]
    fn bidir_explicit_zero_one_else_default() {
        assert!(!parse_config([None; 4], Some("0"), None).bidir);
        assert!(parse_config([None; 4], Some("1"), None).bidir);
        assert!(parse_config([None; 4], Some("yes"), None).bidir); // garbage → default true
    }

    #[test]
    fn loop_khz_clamped_to_range() {
        assert_eq!(parse_config([None; 4], None, Some("1")).loop_khz, 2);
        assert_eq!(parse_config([None; 4], None, Some("9")).loop_khz, 8);
        assert_eq!(parse_config([None; 4], None, Some("4")).loop_khz, 4);
        assert_eq!(parse_config([None; 4], None, Some("x")).loop_khz, 8); // garbage → default
    }
}
