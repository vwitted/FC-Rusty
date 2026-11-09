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
// produces. The error being removed is largely QUASI-STATIC -- a sustained
// tilt holding against wind, or the sustained acceleration of an upset --
// so the estimate can be filtered hard.
//
// The lag is not free. sim_sweep's measurement rotated the compensation by
// the TRUE attitude with no position loop closed on the estimate. In the
// firmware-faithful closed loop (sim-eval, sim/REPORT.md) the position loop
// produces real accelerations on the timescale of the filter, and a LONGER
// tau is worse: 1.0 s took nominal-hover tilt RMS from 2.4 to 8.2 deg.
// Shorter tau helps hover but costs the fault and manoeuvre scenarios, so
// DEFAULT_TAU_S stays 0.5 (sim/ITERATIONS.md, iterations 11 and 22).

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

/// Largest horizontal acceleration treated as physically real, m/s².
///
/// Bounds the one failure mode that losing GPS does not have. A lost
/// receiver fades the estimate out and leaves the aircraft exactly where it
/// was before this existed -- bounded, and no worse than the prior status
/// quo. A receiver that is WRONG has no such property: a multipath glitch
/// of 10 m/s across one 10 Hz fix differentiates to 100 m/s², over 10 g,
/// and would be handed to the attitude estimator as truth.
///
/// 20 m/s² is about 2 g, comfortably above anything the airframe can
/// produce horizontally (at 45 deg of tilt it is g·tan45 = 9.8) and far
/// below the numbers a bad fix generates. The gate upstream only checks
/// fix_mode and satellite count, which a corrupted fix passes.
pub const MAX_PLAUSIBLE_ACCEL_MS2: f32 = 20.0;

/// Remove a GPS-derived horizontal acceleration from a body-frame specific
/// force reading, leaving (approximately) the gravity direction that
/// `AttitudeMekf::update_accel` assumes.
///
/// `accel_g` is body specific force in g. `accel_ned` is world NED m/s²
/// from `GpsAccelEstimator::accel_ned`. `euler_rad` is the MEKF's own
/// [roll, pitch, yaw] (3-2-1), used only to rotate the correction into the
/// body frame; the magnitude and direction come from GPS, so an attitude
/// error mis-rotates a small correction rather than cancelling the
/// innovation. A zero `accel_ned` returns the input unchanged.
pub fn compensate_specific_force_g(
    accel_g: [f32; 3],
    accel_ned: [f32; 2],
    euler_rad: [f32; 3],
) -> [f32; 3] {
    if accel_ned == [0.0; 2] {
        return accel_g;
    }
    let e = euler_rad;
    let rot = nalgebra::Rotation3::from_euler_angles(e[0], e[1], e[2]);
    let ab = rot.inverse() * nalgebra::Vector3::new(accel_ned[0], accel_ned[1], 0.0);
    let g = crate::attitude_mekf::G_MPS2;
    [accel_g[0] - ab.x / g, accel_g[1] - ab.y / g, accel_g[2] - ab.z / g]
}

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
                    // Clamp before filtering, not after: an unclamped
                    // outlier would otherwise contaminate the filter state
                    // and take several fixes to wash out.
                    let raw = ((v[k] - prev[k]) / dt_s)
                        .clamp(-MAX_PLAUSIBLE_ACCEL_MS2, MAX_PLAUSIBLE_ACCEL_MS2);
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

    /// A bad fix is the failure mode losing GPS does not have: fading out
    /// leaves the aircraft at the prior status quo, whereas a wrong
    /// velocity actively misleads the attitude estimator. Bound it.
    #[test]
    fn an_implausible_velocity_jump_is_clamped() {
        let mut e = GpsAccelEstimator::default();
        e.update(0.0, 0.0, DT);
        // 10 m/s in one 10 Hz fix -> 100 m/s^2, over 10 g. No quad does
        // this; a multipath glitch does.
        e.update(10.0, 10.0, DT);
        for k in 0..2 {
            assert!(
                e.accel_ned()[k].abs() <= MAX_PLAUSIBLE_ACCEL_MS2,
                "axis {k} unclamped: {}",
                e.accel_ned()[k]
            );
        }
    }

    /// And the clamp must not corrupt the filter for long afterwards --
    /// clamping happens before filtering precisely so one outlier does not
    /// linger in the state.
    #[test]
    fn the_filter_recovers_quickly_after_a_glitch() {
        let mut e = GpsAccelEstimator::default();
        let mut v = 0.0f32;
        for _ in 0..50 {
            e.update(v, 0.0, DT); // stationary
        }
        e.update(10.0, 0.0, DT); // glitch
        v = 0.0;
        for _ in 0..40 {
            e.update(v, 0.0, DT); // back to stationary
        }
        assert!(
            e.accel_ned()[0].abs() < 0.2,
            "should have washed out, got {}",
            e.accel_ned()[0]
        );
    }

    /// Level, accelerating north at 2 m/s²: the accelerometer reads
    /// R^T(a - g) = (2, 0, -g)/g in g. Compensation must leave pure gravity.
    #[test]
    fn compensation_removes_a_known_acceleration_at_level() {
        let g = crate::attitude_mekf::G_MPS2;
        let f = [2.0 / g, 0.0, -1.0];
        let out = compensate_specific_force_g(f, [2.0, 0.0], [0.0; 3]);
        assert!(out[0].abs() < 1e-6 && out[1].abs() < 1e-6, "{out:?}");
        assert!((out[2] + 1.0).abs() < 1e-6, "{out:?}");
        assert_eq!(compensate_specific_force_g(f, [0.0, 0.0], [0.0; 3]), f);
    }

    /// The compensation rotates with `Rotation3::from_euler_angles(euler)`,
    /// not with the MEKF quaternion. The two must be the same rotation, or
    /// the correction is applied along the wrong body axes once tilted.
    #[test]
    fn euler_rotation_matches_the_mekf_quaternion() {
        use crate::attitude_mekf::{AttitudeMekf, MekfParams};
        let mut m = AttitudeMekf::new(MekfParams::default());
        // Drive to roll 20 / pitch -15 / yaw 70 deg-ish by integrating rates.
        for _ in 0..1000 {
            m.predict([0.35, -0.26, 1.22], 1e-3);
        }
        let e = m.euler();
        let q = m.quaternion();
        let rq = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            q[0], q[1], q[2], q[3],
        ));
        let re = nalgebra::Rotation3::from_euler_angles(e[0], e[1], e[2]);
        let v = nalgebra::Vector3::new(0.3, -1.1, 0.7);
        let d = rq.inverse() * v - re.inverse() * v;
        assert!(d.norm() < 1e-4, "euler/quaternion rotations disagree by {}", d.norm());
    }

    /// The circularity audit, as arithmetic rather than prose.
    ///
    /// An acceleration reference built from the IMU itself -- the PosKF
    /// predict input a = R(q_est) f + g -- makes the corrected measurement
    /// f - R(q_est)^T a / g = -R(q_est)^T e3, which IS the predicted gravity:
    /// the innovation is zero and the update cannot move the estimate.
    /// The GPS-derived reference has no such identity: from the same wrong
    /// estimate, the update moves the tilt toward the truth.
    #[test]
    fn imu_derived_reference_is_circular_and_gps_derived_is_not() {
        use crate::attitude_mekf::{AttitudeMekf, MekfParams};
        use nalgebra::{Rotation3, UnitQuaternion, Vector3};
        let tilt_err = |m: &AttitudeMekf, truth: &Rotation3<f32>| {
            let q = m.quaternion();
            let rq = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]));
            (rq * Vector3::z()).angle(&(truth * Vector3::z()))
        };
        // Truth: rolled 15 deg, accelerating 3 m/s2 east. Estimate: level.
        let truth = Rotation3::from_euler_angles(15f32.to_radians(), 0.0, 0.0);
        let g = crate::attitude_mekf::G_MPS2;
        let a_true = Vector3::new(0.0, 3.0, 0.0);
        let f_g = (truth.inverse() * (a_true - Vector3::new(0.0, 0.0, g))) / g;
        let f = [f_g.x, f_g.y, f_g.z];

        let mut circ = AttitudeMekf::new(MekfParams::default());
        let before = tilt_err(&circ, &truth);
        let e = circ.euler();
        let r_est = Rotation3::from_euler_angles(e[0], e[1], e[2]);
        let a_imu = r_est * f_g * g + Vector3::new(0.0, 0.0, g);
        let ab = r_est.inverse() * a_imu / g;
        let q0 = circ.quaternion();
        for _ in 0..50 {
            circ.update_accel([f[0] - ab.x, f[1] - ab.y, f[2] - ab.z]);
        }
        let q1 = circ.quaternion();
        for i in 0..4 {
            assert!((q0[i] - q1[i]).abs() < 1e-6, "circular reference moved q: {q0:?} -> {q1:?}");
        }

        let mut gps = AttitudeMekf::new(MekfParams::default());
        for _ in 0..50 {
            let c = compensate_specific_force_g(f, [a_true.x, a_true.y], gps.euler());
            gps.update_accel(c);
        }
        let after = tilt_err(&gps, &truth);
        assert!(after < 0.2 * before, "GPS reference failed to correct tilt: {before} -> {after} rad");
    }

    /// A real manoeuvre must still pass. The bound is above anything the
    /// airframe can do, not a limit on normal flight.
    #[test]
    fn a_hard_but_real_manoeuvre_is_not_clamped() {
        let mut e = GpsAccelEstimator::default();
        let want = 8.0f32; // ~0.8 g, a genuinely aggressive quad
        let mut v = 0.0f32;
        for _ in 0..100 {
            v += want * DT;
            e.update(v, 0.0, DT);
        }
        assert!((e.accel_ned()[0] - want).abs() < 0.2, "got {}", e.accel_ned()[0]);
    }
}
