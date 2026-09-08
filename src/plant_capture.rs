//! The throttle profile a plant capture flies, and the RAM buffer it
//! lands in.
//!
//! Split out of `motor_test` and kept pure so the profile itself is
//! host-testable. What the profile does is a safety question as much as a
//! measurement one -- every step is real motors changing speed -- and
//! "does this ever command more than the ceiling" should be answerable by
//! a test, not by reading the loop.

use crate::plant_log::{PlantSample, CMD_SCALE};

/// One hold in the profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hold {
    /// Throttle percent for this hold.
    pub pct: u8,
    /// How long to stay there, milliseconds.
    pub ms: u16,
}

/// Hard ceiling on any commanded throttle, percent.
///
/// Same value as `motor_test::MAX_PCT` and asserted against the profile
/// below. Duplicated deliberately rather than imported: this module is
/// host-compiled and that one is not, and a ceiling that silently follows
/// another module's edit is not a ceiling.
pub const MAX_PCT: u8 = 25;

/// Lowest throttle that still spins reliably under load, percent.
///
/// Steps are taken FROM here rather than from zero. A step from a stopped
/// motor measures the ESC's startup ramp -- a commutation-detection
/// behaviour with its own logic -- rather than the closed-loop lag the
/// sim models, and it would fit a much longer time constant that is real
/// but irrelevant to flight.
pub const IDLE_PCT: u8 = 8;

/// The capture profile: alternating steps up and back to idle.
///
/// Up AND down matter. `omega^2` is not symmetric about a step (see
/// plant_fit), so a profile that only ever accelerates leaves the
/// deceleration lag unmeasured -- and deceleration is what a quad does
/// when it corrects an over-rotation, which is the case the rate loop's
/// stability actually depends on.
///
/// Returning to idle between steps also gives every step the same
/// starting point, so the four fits are comparable rather than each
/// starting wherever the last one ended.
///
/// 400 ms holds: long enough for ~13 time constants at the 30 ms the sim
/// assumes, so the asymptote is genuinely reached even if the real lag is
/// several times longer than expected.
pub const PROFILE: [Hold; 9] = [
    Hold { pct: IDLE_PCT, ms: 600 }, // settle before the first step
    Hold { pct: 15, ms: 400 },
    Hold { pct: IDLE_PCT, ms: 400 },
    Hold { pct: 20, ms: 400 },
    Hold { pct: IDLE_PCT, ms: 400 },
    Hold { pct: 25, ms: 400 },
    Hold { pct: IDLE_PCT, ms: 400 },
    Hold { pct: 12, ms: 400 }, // a small step: lag can vary with step size
    Hold { pct: IDLE_PCT, ms: 400 },
];

/// Capture sample rate.
///
/// 500 Hz is 2 ms resolution, ~15 points inside one 30 ms time constant.
/// Higher would cost buffer for very little: the telemetry itself only
/// updates once per DShot frame and the eRPM period is quantised to whole
/// microseconds, so the information content stops rising well before the
/// sample rate does.
pub const SAMPLE_HZ: u32 = 500;

/// Total profile duration, milliseconds.
pub const fn profile_ms() -> u32 {
    let mut total = 0u32;
    let mut i = 0;
    while i < PROFILE.len() {
        total += PROFILE[i].ms as u32;
        i += 1;
    }
    total
}

/// Samples the buffer must hold for the whole profile.
pub const fn capacity() -> usize {
    (profile_ms() * SAMPLE_HZ / 1000) as usize + 64 // margin for jitter
}

/// Commanded throttle percent at a given time into the profile, and
/// whether the profile has finished.
pub fn command_at(t_ms: u32) -> Option<u8> {
    let mut acc = 0u32;
    let mut i = 0;
    while i < PROFILE.len() {
        acc += PROFILE[i].ms as u32;
        if t_ms < acc {
            return Some(PROFILE[i].pct);
        }
        i += 1;
    }
    None
}

/// Fixed-point command value for a throttle percent, as stored in a
/// `PlantSample`.
pub fn cmd_units(pct: u8) -> u16 {
    (pct as f32 / 100.0 * CMD_SCALE) as u16
}

/// Fixed-size capture buffer.
///
/// A plain array rather than a ring: the profile has a known length and
/// wrapping would silently discard the beginning of the run, which is
/// where the first step is.
pub struct Capture<const N: usize> {
    samples: [PlantSample; N],
    len: usize,
    dropped: u32,
}

impl<const N: usize> Default for Capture<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Capture<N> {
    pub const fn new() -> Self {
        Self {
            samples: [PlantSample {
                t_ms: 0,
                cmd: [0; 4],
                period_us: [0; 4],
                gyro_dps10: [0; 3],
            }; N],
            len: 0,
            dropped: 0,
        }
    }

    /// Record one sample. Counts overruns instead of overwriting, so a
    /// buffer that turned out too small is visible in the dump rather
    /// than being a truncation nobody notices.
    pub fn push(&mut self, s: PlantSample) {
        if self.len < N {
            self.samples[self.len] = s;
            self.len += 1;
        } else {
            self.dropped = self.dropped.wrapping_add(1);
        }
    }

    pub fn as_slice(&self) -> &[PlantSample] {
        &self.samples[..self.len]
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn dropped(&self) -> u32 {
        self.dropped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_hold_exceeds_the_ceiling() {
        // The safety property. If a future edit raises a hold above the
        // ceiling this fails at test time rather than on a bench with
        // props on.
        for h in PROFILE.iter() {
            assert!(h.pct <= MAX_PCT, "hold at {}% exceeds {}%", h.pct, MAX_PCT);
        }
    }

    #[test]
    fn the_profile_steps_both_up_and_down() {
        // A profile that only accelerates cannot measure the deceleration
        // lag, and omega^2 is not symmetric about a step.
        let mut up = 0;
        let mut down = 0;
        for w in PROFILE.windows(2) {
            if w[1].pct > w[0].pct {
                up += 1;
            } else if w[1].pct < w[0].pct {
                down += 1;
            }
        }
        assert!(up >= 3, "only {up} steps up");
        assert!(down >= 3, "only {down} steps down");
    }

    #[test]
    fn every_step_is_large_enough_for_the_fitter_to_see() {
        // plant_fit::MIN_STEP is 0.02 normalised, i.e. 2%. A hold pair
        // closer than that would be skipped and the profile would take
        // longer than it needs to for no data.
        for w in PROFILE.windows(2) {
            if w[0].pct != w[1].pct {
                let delta = (w[1].pct as i16 - w[0].pct as i16).unsigned_abs();
                assert!(delta >= 2, "step of {delta}% is below the detection floor");
            }
        }
    }

    #[test]
    fn steps_start_from_idle_not_from_a_stop() {
        // A step from zero measures the ESC's startup ramp, not the
        // closed-loop lag. No hold may be below the idle floor.
        for h in PROFILE.iter() {
            assert!(h.pct >= IDLE_PCT, "hold at {}% is below idle", h.pct);
        }
    }

    #[test]
    fn command_at_walks_the_profile_and_then_ends() {
        assert_eq!(command_at(0), Some(IDLE_PCT));
        assert_eq!(command_at(599), Some(IDLE_PCT));
        assert_eq!(command_at(600), Some(15));
        assert_eq!(command_at(999), Some(15));
        assert_eq!(command_at(1000), Some(IDLE_PCT));
        assert_eq!(command_at(profile_ms() - 1), Some(IDLE_PCT));
        assert_eq!(command_at(profile_ms()), None, "profile must terminate");
        assert_eq!(command_at(profile_ms() + 10_000), None);
    }

    #[test]
    fn the_buffer_holds_the_whole_profile() {
        // The capture is worthless if it stops before the last step, and
        // "it fitted three of four motors" is a confusing way to discover
        // an undersized buffer.
        let needed = (profile_ms() * SAMPLE_HZ / 1000) as usize;
        assert!(capacity() > needed, "{} <= {needed}", capacity());
    }

    #[test]
    fn the_profile_fits_in_a_u16_millisecond_timestamp() {
        // PlantSample::t_ms is u16. A profile longer than 65.5 s would
        // wrap it and produce a log that fits a nonsense time constant
        // without looking wrong.
        assert!(profile_ms() < u16::MAX as u32, "{} ms", profile_ms());
    }

    #[test]
    fn overrun_is_counted_not_silently_dropped() {
        let mut c: Capture<2> = Capture::new();
        for i in 0..5 {
            c.push(PlantSample { t_ms: i, ..PlantSample::default() });
        }
        assert_eq!(c.len(), 2);
        assert_eq!(c.dropped(), 3);
        // And what it kept is the START of the run, where the first step
        // is -- not the end.
        assert_eq!(c.as_slice()[0].t_ms, 0);
    }

    #[test]
    fn cmd_units_round_trip_through_the_sample() {
        let s = PlantSample { cmd: [cmd_units(25); 4], ..PlantSample::default() };
        assert!((s.cmd_f32(0) - 0.25).abs() < 1e-3);
    }
}
