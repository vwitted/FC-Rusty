// gps_accel.rs — horizontal acceleration from GPS Doppler velocity.
//
// Exists to give the attitude estimator an acceleration reference that is
// INDEPENDENT of its own attitude. The MEKF's accel update treats the
// accelerometer as reading gravity, which is false whenever the aircraft
// accelerates; subtracting a known acceleration first restores that
// assumption. See attitude_mekf.rs.
//
// Independence is the whole point and it is easy to lose. Deriving the
// acceleration from the IMU (a_world = R(q)*f_body + g, which main.rs
// already computes for the position KF) is DEGENERATE: substituting it back
// gives f - R^T(R f + g) = -R^T g, exactly the predicted gravity, so the
// innovation is identically zero and the accel update silently stops
// working. GPS Doppler velocity is the only independent source available.
//
// Horizontal only, matching PosKF::update_gps_velocity: consumer vertical
// GPS velocity is noisy, and the tilt-induced error this corrects is
// horizontal by construction.
//
// The signal-to-noise looks unusable on paper -- 0.05 m/s of Doppler noise
// at 10 Hz differentiates to ~0.7 m/s2, against the ~2.6 m/s2 a 15 deg tilt
// produces. It works because the error being removed is QUASI-STATIC: a
// sustained tilt holding against wind, or the sustained acceleration of an
// upset. Both persist for seconds, so the estimate can be filtered hard and
// the lag costs nothing. Measured in examples/sim_sweep.rs.

/// Smoothing time constant, seconds. Chosen so ~5 fixes at 10 Hz are
/// averaged: enough to halve the differentiation noise, short enough that
/// a multi-second manoeuvre is still tracked.
pub const DEFAULT_TAU_S: f32 = 0.5;

/// Seconds without a fix after which the estimate is no longer trusted and
/// decays away.
pub const STALE_AFTER_S: f32 = 1.0;

/// Seconds over which a stale estimate decays to zero.
///
/// Decayed rather than dropped. A step change in the accel update's input
/// is a step in its innovation, which is its own transient -- precisely the
/// disturbance this whole mechanism exists to avoid.
pub const DECAY_S: f32 = 1.0;

/// Horizontal acceleration estimated by differentiating GPS velocity.
#[derive(Debug, Clone, Copy)]
pub struct GpsAccelEstimator {
    last_vel: Option<[f32; 2]>,
    filtered: [f32; 2],
    tau_s: f32,
    since_fix_s: f32,
}

impl GpsAccelEstimator {
    pub const fn new(tau_s: f32) -> Self {
        Self { last_vel: None, filtered: [0.0; 2], tau_s, since_fix_s: 0.0 }
    }

    /// Advance time without a fix. Decays the estimate once GPS goes stale,
    /// so losing the receiver fades the compensation out instead of
    /// stepping it to zero.
    pub fn tick(&mut self, dt: f32) {
        self.since_fix_s += dt;
        if self.since_fix_s > STALE_AFTER_S {
            let k = (dt / DECAY_S).clamp(0.0, 1.0);
            self.filtered[0] -= self.filtered[0] * k;
            self.filtered[1] -= self.filtered[1] * k;
            // Drop the anchor too: differencing against a velocity from
            // before the outage would manufacture a huge false acceleration
            // on the first fix back.
            self.last_vel = None;
        }
    }

    /// Feed a fresh GPS velocity fix. `dt_s` is the interval since the
    /// previous one.
    pub fn update(&mut self, vn: f32, ve: f32, dt_s: f32) {
        let v = [vn, ve];
        if let Some(prev) = self.last_vel {
            if dt_s > 1e-3 {
                let alpha = (dt_s / self.tau_s).clamp(0.0, 1.0);
                for k in 0..2 {
                    let raw = (v[k] - prev[k]) / dt_s;
                    self.filtered[k] += (raw - self.filtered[k]) * alpha;
                }
            }
        }
        self.last_vel = Some(v);
        self.since_fix_s = 0.0;
    }

    /// Horizontal acceleration, world NED (m/s²). Zero until two fixes have
    /// arrived, and decaying to zero if GPS is lost.
    pub fn accel_ned(&self) -> [f32; 2] {
        self.filtered
    }

    /// Whether the estimate is currently backed by fresh fixes.
    pub fn is_fresh(&self) -> bool {
        self.last_vel.is_some() && self.since_fix_s <= STALE_AFTER_S
    }
}

impl Default for GpsAccelEstimator {
    fn default() -> Self {
        Self::new(DEFAULT_TAU_S)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.1; // 10 Hz fixes

    #[test]
    fn constant_velocity_gives_zero_acceleration() {
        let mut e = GpsAccelEstimator::default();
        for _ in 0..50 {
            e.update(12.0, -3.0, DT);
        }
        let a = e.accel_ned();
        assert!(a[0].abs() < 1e-4 && a[1].abs() < 1e-4, "got {a:?}");
    }

    #[test]
    fn steady_acceleration_is_tracked() {
        let mut e = GpsAccelEstimator::default();
        let want = 2.5f32; // m/s^2 north
        let mut v = 0.0f32;
        for _ in 0..100 {
            v += want * DT;
            e.update(v, 0.0, DT);
        }
        let a = e.accel_ned();
        assert!((a[0] - want).abs() < 0.05, "north {a:?} should approach {want}");
        assert!(a[1].abs() < 0.01);
    }

    /// The first fix cannot produce an acceleration -- there is nothing to
    /// difference against. Returning something would be inventing data.
    #[test]
    fn a_single_fix_yields_nothing() {
        let mut e = GpsAccelEstimator::default();
        e.update(50.0, 50.0, DT);
        assert_eq!(e.accel_ned(), [0.0, 0.0]);
        assert!(e.is_fresh());
    }

    /// Losing GPS must FADE the estimate, not step it to zero: a
    /// discontinuous input to the accel update is its own transient.
    #[test]
    fn losing_gps_decays_the_estimate_rather_than_dropping_it() {
        let mut e = GpsAccelEstimator::default();
        let mut v = 0.0f32;
        for _ in 0..100 {
            v += 3.0 * DT;
            e.update(v, 0.0, DT);
        }
        let settled = e.accel_ned()[0];
        assert!(settled > 2.5, "should have tracked ~3, got {settled}");

        // Just past the staleness threshold: still substantial, not zeroed.
        for _ in 0..((STALE_AFTER_S / DT) as usize + 2) {
            e.tick(DT);
        }
        let just_stale = e.accel_ned()[0];
        assert!(!e.is_fresh());
        assert!(just_stale > settled * 0.7, "must not step to zero: {just_stale}");

        // And well past it, gone.
        for _ in 0..100 {
            e.tick(DT);
        }
        assert!(e.accel_ned()[0].abs() < 0.05, "should have faded out");
    }

    /// After an outage, the first fix back must not be differenced against a
    /// velocity from before it -- that would manufacture an enormous false
    /// acceleration at the worst moment.
    #[test]
    fn a_fix_after_an_outage_does_not_manufacture_acceleration() {
        let mut e = GpsAccelEstimator::default();
        e.update(0.0, 0.0, DT);
        for _ in 0..50 {
            e.tick(DT); // long gap
        }
        // Reappears moving fast, having accelerated unobserved.
        e.update(40.0, 0.0, DT);
        let a = e.accel_ned()[0];
        assert!(a.abs() < 0.5, "differenced across the gap: {a} m/s^2");
    }

    /// Noise is attenuated, which is the reason for filtering at all: raw
    /// differentiation of Doppler noise swamps the signal.
    #[test]
    fn filtering_attenuates_differentiation_noise() {
        use crate::sim::sensors::Rng;
        let mut rng = Rng::new(11);
        let mut e = GpsAccelEstimator::default();
        let (mut raw_sq, mut n) = (0.0f64, 0usize);
        for _ in 0..2000 {
            // Stationary, so all of this is noise.
            let v = 0.05 * rng.normal();
            let prev_raw = v / DT;
            raw_sq += (prev_raw * prev_raw) as f64;
            n += 1;
            e.update(v, 0.0, DT);
        }
        let raw_rms = (raw_sq / n as f64).sqrt() as f32;
        let filt = e.accel_ned()[0].abs();
        assert!(
            filt < raw_rms * 0.5,
            "filtered {filt} should be well under raw noise {raw_rms}"
        );
    }
}
