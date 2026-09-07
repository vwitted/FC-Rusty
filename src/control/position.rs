// position.rs — Position controller for GPS rescue / return-to-home
//
// Outer-most loop in the cascade:
//   Position PD (this) → attitude MPC → rate PID → mixer → motors
//
// Takes the NED position + velocity estimate from the Kalman filter
// and a target waypoint, and outputs the roll/pitch angles the MPC
// should track to fly there.
//
// Physics:
//   To accelerate horizontally at (ax, ay) the quad must tilt so that
//   a component of its thrust vector points in that direction.
//
//   Convention (3-2-1 Tait-Bryan, NED) — see src/conventions.rs, which
//   enforces this for every stage:
//     +X = north, +Y = east, +Z = down
//     +pitch = nose UP, +roll = right wing down
//
//   Projecting body -Z thrust into the world gives
//
//     a_north = -(T/m) · cos(roll) · sin(pitch)
//     a_east  = +(T/m) · sin(roll)              (at zero yaw)
//
//   so the tilt needed for a demanded acceleration is
//
//     pitch_desired = -atan2(ax_desired, g)   (nose DOWN to fly north)
//     roll_desired  =  atan2(ay_desired, g)   (roll RIGHT to fly east)
//
//   For small angles atan2(a, g) ≈ a/g, but we keep the atan2 so the
//   controller remains valid up to the tilt clamp (typically 15–20°).
//
// The controller runs at the same rate as the MPC (100 Hz — pos_ctrl.update
// sits inside main.rs's MPC_PERIOD_US ticker). Its output feeds
// `AttitudeMpc::set_reference()` unnegated, at all three call sites.

use core::f32::consts::PI;

const GRAVITY: f32 = 9.81;

/// Tuning gains for the position controller.
#[derive(Debug, Clone)]
pub struct PositionGains {
    /// Proportional gain (m/s² per metre of position error).
    /// Higher = more aggressive homing. 0.5 is gentle, 2.0 is sporty.
    pub kp: f32,
    /// Derivative gain (m/s² per m/s of velocity).
    /// Damps overshoot. Should be roughly 2·√(kp) for critical damping.
    pub kd: f32,
    /// Maximum tilt angle in radians. Limits how aggressively the quad can
    /// tilt to correct position error — and therefore bounds horizontal
    /// acceleration to g·tan(tilt), and station-keeping speed to
    /// sqrt(m·a/drag).
    ///
    /// Was 15°, described as "conservative (GPS rescue)". It is not
    /// conservative: 15° caps wind-holding at ~15.6 m/s, so it is the
    /// setting that makes GPS rescue unable to get home in a gale. Measured
    /// station-keeping with the estimator and GPS accel compensation in the
    /// loop (att_rms / pos_rms, 8 seeds):
    ///
    ///     wind      15°                30°           45°
    ///     20 m/s    3.3 / 51  (5/8 drift)  4.7 / 5.3   5.3 / 5.3
    ///     25 m/s    8/8 drift              5.3 / 28    8.5 / 8.3
    ///     33 m/s    8/8 drift              8/8 drift  13.4 / 44 (1/8)
    ///
    /// 15° DRIFTS AWAY at 20 m/s — an ordinary gusty day — while 45° holds
    /// through 25 and mostly through 33. Losing the aircraft downwind
    /// during a rescue is the worse failure and it happens in far more
    /// common conditions, so 45° is the safer presumptive default.
    ///
    /// Two caveats carried deliberately:
    ///
    /// The benefit above depends on the GPS-derived acceleration
    /// compensation (see gps_accel.rs). WITHOUT it, 45° at 33 m/s crashes
    /// where 15° merely drifts. Gating tilt on
    /// GpsAccelEstimator::is_fresh() is the principled fix and is not yet
    /// wired.
    ///
    /// And 45° now coincides exactly with mpc::MAX_ANGLE_RAD, so the
    /// reference sits on the solver's own state constraint with no margin.
    /// That matters less than it sounds — MAX_ANGLE_RAD is a bare number
    /// with no stated derivation, unlike MAX_CMD_RAD beside it — but both
    /// deserve the same scrutiny the tilt limit has now had.
    pub max_tilt_rad: f32,

    /// Tilt limit used while the GPS-derived acceleration reference is
    /// NOT fresh, radians.
    ///
    /// The mechanism the note above calls "not yet wired" is now wired --
    /// `set_compensation` selects between the two limits -- but it DEFAULTS
    /// TO INERT (equal to `max_tilt_rad`), because measuring it did not
    /// support switching it on. Setting this lower is a one-constant
    /// decision; the evidence for and against is here.
    ///
    /// Station-keeping with the estimator in the loop, 8 seeds, 30 s
    /// flights, as att_rms / pos_rms:
    ///
    ///                  NO GPS ACCEL              WITH GPS ACCEL
    ///     wind      15 deg      45 deg        15 deg       45 deg
    ///     20 m/s   8/8 drift  7.8 / 5.3    7/8 drift    5.2 / 5.3
    ///     25 m/s   8/8 drift 17.3 / 10.0   8/8 drift   10.8 / 8.6
    ///     33 m/s   8/8 drift  8/8 DIVERGE  8/8 drift   15.8 / 36 (2/8)
    ///
    /// The 33 m/s cell is the one that motivated a fallback, and it is
    /// real: uncompensated, 45 deg diverges in attitude where 15 deg only
    /// drifts, and divergence is the worse failure. But it is the ONLY
    /// cell that favours 15 deg. At 20 and 25 m/s -- far commoner
    /// conditions -- the fallback is what loses the aircraft, drifting
    /// 8/8 where 45 deg holds station. So as an instantaneous gate it
    /// trades a rare failure for a common one.
    ///
    /// It is also measuring the wrong thing, which is the more important
    /// point. Those columns model the reference being absent for the
    /// WHOLE FLIGHT. The gate fires on staleness, which in flight is a
    /// transient -- and the divergence above takes ~15 s to develop, far
    /// longer than a plausible GPS dropout. So the benefit needs a long
    /// outage while the cost lands on the first tick.
    ///
    /// If this is enabled, it should therefore be on SUSTAINED staleness
    /// (several seconds), not on `is_fresh()` directly -- which needs a
    /// scenario that actually interrupts GPS mid-flight to tune, and the
    /// wind axis is not it.
    pub max_tilt_rad_degraded: f32,
}

impl Default for PositionGains {
    /// Defaults suitable for GPS rescue. See `max_tilt_rad` — the tilt
    /// limit is set for the aircraft to actually reach home in wind, which
    /// is the opposite trade from the 15° this used to carry.
    fn default() -> Self {
        Self {
            kp: 0.8,
            kd: 1.2,
            max_tilt_rad: 45.0 * PI / 180.0,
            // Inert by default -- see the field. Not 15 deg: that setting
            // was measured to lose the aircraft at 20 and 25 m/s to save
            // it at 33.
            max_tilt_rad_degraded: 45.0 * PI / 180.0,
        }
    }
}

/// Output of one position controller step.
pub struct PositionOutput {
    /// Desired roll angle (radians). Positive = right wing down.
    pub roll_rad: f32,
    /// Desired pitch angle (radians). Positive = nose up.
    pub pitch_rad: f32,
}

/// Horizontal position PD controller.
///
/// Stateless — no integrator. The altitude axis is handled separately
/// by `AltitudeController`; this only produces roll/pitch references.
pub struct PositionController {
    pub gains: PositionGains,
    /// Tilt limit actually in force this tick, radians. Slewed by
    /// `set_compensation` between the two limits in `gains`.
    tilt_limit_rad: f32,
}

/// Rate at which the tilt limit is allowed to RISE, rad/s.
///
/// Only the upward direction is rate-limited. Lowering the clamp can only
/// reduce a commanded angle, which is the safe direction and wants no
/// delay. Raising it can step the attitude reference by the full 30 deg
/// difference the instant a fix returns -- but only if the demand was
/// already saturated, which during a rescue in wind it usually is. So the
/// rise is spread over about a second, which the MPC tracks without a
/// transient.
const TILT_LIMIT_RISE_RAD_S: f32 = 30.0 * PI / 180.0;

impl PositionController {
    /// Starts in the degraded (low-tilt) limit: at construction no GPS
    /// velocity fix has been seen, so the acceleration reference cannot be
    /// fresh, and the safe limit is the one that needs no justification.
    pub fn new(gains: PositionGains) -> Self {
        let tilt_limit_rad = gains.max_tilt_rad_degraded;
        Self { gains, tilt_limit_rad }
    }

    /// Select the tilt limit for this tick.
    ///
    /// `accel_fresh` is `GpsAccelEstimator::is_fresh()` -- whether the
    /// attitude estimator is currently receiving a GPS-derived
    /// acceleration to subtract from the accelerometer. The full 45 deg is
    /// only defensible while that holds; see `PositionGains::max_tilt_rad`
    /// for the measurements, and `max_tilt_rad_degraded` for why the
    /// fallback is a tilt limit rather than anything cleverer.
    ///
    /// Call once per outer-loop tick, before `update`. `dt` is that
    /// period, seconds.
    pub fn set_compensation(&mut self, accel_fresh: bool, dt: f32) {
        if accel_fresh {
            let target = self.gains.max_tilt_rad;
            let step = TILT_LIMIT_RISE_RAD_S * dt;
            self.tilt_limit_rad = (self.tilt_limit_rad + step).min(target);
        } else {
            // Immediate, not slewed -- see TILT_LIMIT_RISE_RAD_S. Capped by
            // max_tilt_rad so the "degraded" limit can never RAISE the
            // clamp: a caller that lowers only the ceiling, as
            // sim_gps_rescue does, must not be handed extra authority by a
            // fallback it never asked for.
            self.tilt_limit_rad =
                self.gains.max_tilt_rad_degraded.min(self.gains.max_tilt_rad);
        }
    }

    /// The tilt limit currently in force, radians. For logging and tests.
    pub fn tilt_limit_rad(&self) -> f32 {
        self.tilt_limit_rad
    }

    /// Compute desired roll/pitch to fly from the current position
    /// toward the target.
    ///
    /// All inputs are in the **NED world frame** (metres, m/s).
    ///
    /// `yaw_rad` is the current heading — needed to rotate the
    /// world-frame acceleration demand into the body-frame tilt axes
    /// (otherwise a quad pointing east would pitch when it should roll).
    pub fn update(
        &self,
        pos_ned: [f32; 2],     // [north, east] current estimate
        vel_ned: [f32; 2],     // [vn, ve] current estimate
        target_ned: [f32; 2],  // [north, east] target
        yaw_rad: f32,
    ) -> PositionOutput {
        // ---- World-frame desired acceleration ----
        let err_n = target_ned[0] - pos_ned[0];
        let err_e = target_ned[1] - pos_ned[1];

        // PD: a_desired = kp * error - kd * velocity
        // (velocity term is negative because we want to damp, not chase)
        let ax_world = self.gains.kp * err_n - self.gains.kd * vel_ned[0];
        let ay_world = self.gains.kp * err_e - self.gains.kd * vel_ned[1];

        // ---- Rotate into body frame using yaw ----
        // Body X = forward (pitch axis), Body Y = right (roll axis)
        //   a_body_x =  cos(ψ)·ax_world + sin(ψ)·ay_world
        //   a_body_y = -sin(ψ)·ax_world + cos(ψ)·ay_world
        let cos_yaw = libm::cosf(yaw_rad);
        let sin_yaw = libm::sinf(yaw_rad);
        let ax_body = cos_yaw * ax_world + sin_yaw * ay_world;
        let ay_body = -sin_yaw * ax_world + cos_yaw * ay_world;

        // ---- Acceleration → tilt angle ----
        // Inverted from the physics above: nose DOWN to fly north, roll
        // RIGHT to fly east.
        //
        // These two signs were previously the other way round. The comment
        // that stood here derived them from QuadSim's thrust projection
        // ("positive pitch = north accel") rather than from physics, and
        // that projection was itself wrong — it dropped yaw entirely and
        // carried inverted lateral signs. Sim and controller therefore
        // agreed with each other and disagreed with the aircraft, so
        // sim_gps_rescue converged while hardware would have departed at
        // full tilt. Fixing the sim exposed it; src/conventions.rs now pins
        // every stage so the two cannot drift apart again silently.
        let pitch_raw = -libm::atan2f(ax_body, GRAVITY);
        let roll_raw = libm::atan2f(ay_body, GRAVITY);

        // ---- Clamp to max tilt ----
        // The limit in force, not the gains' ceiling: it is degraded when
        // the GPS acceleration reference is stale. See set_compensation.
        let max = self.tilt_limit_rad;
        let pitch = clamp(pitch_raw, -max, max);
        let roll = clamp(roll_raw, -max, max);

        PositionOutput { roll_rad: roll, pitch_rad: pitch }
    }
}

fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo { lo } else if x > hi { hi } else { x }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A controller with the acceleration reference live, which is the
    /// state the sign and gain tests below are about. Construction starts
    /// DEGRADED on purpose (see `PositionController::new`), so these must
    /// say so explicitly rather than inherit whichever limit the default
    /// happens to carry.
    fn ctrl() -> PositionController {
        let mut c = PositionController::new(PositionGains::default());
        c.set_compensation(true, 10.0); // one long tick: straight to the top
        c
    }

    #[test]
    fn at_target_zero_tilt() {
        let out = ctrl().update([0.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.roll_rad.abs() < 1e-6);
        assert!(out.pitch_rad.abs() < 1e-6);
    }

    // NOTE: these four assertions were inverted until 2026-09-05, and their
    // comments cited "in sim" rather than physics -- written against the
    // same wrong QuadSim projection the implementation was. That is exactly
    // why the sign error survived: the tests agreed with the bug. They now
    // reason from a_north = -(T/m)*cos(roll)*sin(pitch). See
    // src/conventions.rs.

    #[test]
    fn north_of_target_pitches_to_go_south() {
        // 10 m north of target, heading north: must fly SOUTH, which needs
        // NOSE-UP, i.e. positive pitch.
        let out = ctrl().update([10.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.pitch_rad > 0.0, "pitch={}", out.pitch_rad);
        assert!(out.roll_rad.abs() < 1e-6, "roll={}", out.roll_rad);
    }

    #[test]
    fn south_of_target_pitches_to_go_north() {
        // 10 m south of target: must fly NORTH, which needs NOSE-DOWN.
        let out = ctrl().update([-10.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.pitch_rad < 0.0, "pitch={}", out.pitch_rad);
    }

    #[test]
    fn east_of_target_rolls_to_go_west() {
        // 10 m east of target, heading north: must fly WEST, which needs a
        // LEFT roll (negative, left wing down).
        let out = ctrl().update([0.0, 10.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.roll_rad < 0.0, "roll={}", out.roll_rad);
        assert!(out.pitch_rad.abs() < 1e-6, "pitch={}", out.pitch_rad);
    }

    #[test]
    fn velocity_damps_command() {
        // 2 m north of target but already flying south at 1 m/s. The
        // command is nose-up (positive) to keep going south; damping must
        // make it LESS positive, i.e. closer to zero.
        let no_vel = ctrl().update([2.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        let with_vel = ctrl().update([2.0, 0.0], [-1.0, 0.0], [0.0, 0.0], 0.0);
        assert!(with_vel.pitch_rad < no_vel.pitch_rad,
            "damped {} should be closer to zero than undamped {}",
            with_vel.pitch_rad, no_vel.pitch_rad);
        assert!(with_vel.pitch_rad > 0.0, "still commanding south");
    }

    #[test]
    fn tilt_clamped() {
        // Huge error should hit the tilt clamp.
        let out = ctrl().update([1000.0, 1000.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        let max = PositionGains::default().max_tilt_rad;
        assert!((out.roll_rad.abs() - max).abs() < 1e-5);
        assert!((out.pitch_rad.abs() - max).abs() < 1e-5);
    }

    #[test]
    fn yaw_rotates_command() {
        // Quad is 10m north of target. If heading east (yaw = π/2),
        // "forward" is east, so it should *roll* to go south, not pitch.
        let yaw = core::f32::consts::FRAC_PI_2;
        let out = ctrl().update([10.0, 0.0], [0.0, 0.0], [0.0, 0.0], yaw);
        // Roll significant, pitch near zero -- and the SIGN matters, which
        // this test used to ignore by taking abs() on both. Heading east,
        // the right wing points south, so flying south means rolling RIGHT.
        assert!(out.roll_rad > 0.01, "roll={}", out.roll_rad);
        assert!(out.pitch_rad.abs() < 0.01, "pitch={}", out.pitch_rad);
    }

    // ---- Tilt authority gate ----
    //
    // These are about the ONE thing 45 deg is conditional on. The
    // measurement that motivated the whole gate is that without a GPS
    // acceleration reference, 45 deg crashes at 33 m/s where 15 deg only
    // drifts -- so "the limit fell back" is a flight-safety assertion, not
    // a tidiness one.

    /// Gains with the fallback ACTUALLY LOWER than the ceiling. The
    /// shipped default has them equal -- the gate is inert until someone
    /// decides otherwise (see `max_tilt_rad_degraded`) -- so these tests
    /// build the configuration they are about rather than inheriting one
    /// that would make every assertion below pass vacuously.
    fn gains() -> PositionGains {
        PositionGains {
            max_tilt_rad_degraded: 15.0 * PI / 180.0,
            ..PositionGains::default()
        }
    }

    /// The same, already wound up to the full limit.
    fn gated_ctrl() -> PositionController {
        let mut c = PositionController::new(gains());
        c.set_compensation(true, 10.0);
        c
    }

    #[test]
    fn starts_degraded_because_no_fix_has_arrived_yet() {
        let c = PositionController::new(gains());
        assert_eq!(c.tilt_limit_rad(), gains().max_tilt_rad_degraded);
    }

    #[test]
    fn stale_gps_drops_the_limit_immediately() {
        let mut c = gated_ctrl();
        assert_eq!(c.tilt_limit_rad(), gains().max_tilt_rad);
        // One tick of staleness, and the fallback is already in force --
        // no slew on the way DOWN.
        c.set_compensation(false, 0.01);
        assert_eq!(c.tilt_limit_rad(), gains().max_tilt_rad_degraded);
    }

    #[test]
    fn the_shipped_default_leaves_the_gate_inert() {
        // Guards the decision, not the mechanism: if someone lowers the
        // default fallback, this fails and they have to come and read the
        // measurements on `max_tilt_rad_degraded` first.
        let d = PositionGains::default();
        assert_eq!(d.max_tilt_rad_degraded, d.max_tilt_rad);
    }

    #[test]
    fn a_degraded_limit_above_the_ceiling_cannot_raise_it() {
        // A caller that lowers only the ceiling -- examples/sim_gps_rescue
        // does -- must not be handed authority by a fallback it never
        // asked for.
        let mut c = PositionController::new(PositionGains {
            max_tilt_rad: 10.0 * PI / 180.0,
            max_tilt_rad_degraded: 15.0 * PI / 180.0,
            ..PositionGains::default()
        });
        c.set_compensation(false, 0.01);
        assert!((c.tilt_limit_rad() - 10.0 * PI / 180.0).abs() < 1e-6);
    }

    #[test]
    fn the_limit_rises_at_a_bounded_rate() {
        let mut c = PositionController::new(gains());
        // A single 10 ms tick must not hand back the full 30 deg of extra
        // authority: that is a step in the attitude reference whenever the
        // demand is saturated, which in the wind case it always is.
        c.set_compensation(true, 0.01);
        let after_one = c.tilt_limit_rad();
        assert!(
            after_one < gains().max_tilt_rad_degraded + 0.01,
            "one tick jumped to {} rad",
            after_one
        );
        // ...and it does get there. 30 deg of travel at 30 deg/s is one
        // second; 150 ticks of 10 ms leaves margin without hiding a stall.
        for _ in 0..150 {
            c.set_compensation(true, 0.01);
        }
        assert!((c.tilt_limit_rad() - gains().max_tilt_rad).abs() < 1e-5);
    }

    #[test]
    fn degraded_limit_actually_clamps_the_output() {
        // The state is only worth anything if `update` reads it. Same
        // saturating input as `tilt_clamped`, opposite compensation.
        let mut c = PositionController::new(gains());
        c.set_compensation(false, 0.01);
        let out = c.update([1000.0, 1000.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        let deg_max = gains().max_tilt_rad_degraded;
        assert!((out.roll_rad.abs() - deg_max).abs() < 1e-5, "roll={}", out.roll_rad);
        assert!((out.pitch_rad.abs() - deg_max).abs() < 1e-5, "pitch={}", out.pitch_rad);
    }
}
