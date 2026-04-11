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
//   pitch_desired =  atan2(ax_desired, g)   (tilt forward → accelerate north)
//   roll_desired  = -atan2(ay_desired, g)   (tilt right   → accelerate east)
//
//   The signs follow the NED + Betaflight convention:
//     +X = north, +Y = east, +Z = down
//     +pitch = nose up (decelerates northward motion)
//     +roll  = right wing down (decelerates eastward motion)
//
//   For small angles atan2(a, g) ≈ a/g, but we keep the atan2 so the
//   controller remains valid up to the tilt clamp (typically 15–20°).
//
// The controller runs at the same rate as the MPC (50 Hz). Its output
// is fed directly to `AttitudeMpc::set_reference()`.

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
    /// Maximum tilt angle in radians. Limits how aggressively the quad
    /// can tilt to correct position error. 15° is conservative (GPS
    /// rescue), 30° is reasonable for autonomous flight.
    pub max_tilt_rad: f32,
}

impl Default for PositionGains {
    /// Conservative defaults suitable for GPS rescue.
    fn default() -> Self {
        Self {
            kp: 0.8,
            kd: 1.2,
            max_tilt_rad: 15.0 * PI / 180.0,
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
}

impl PositionController {
    pub fn new(gains: PositionGains) -> Self {
        Self { gains }
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
        // Matched to the QuadSim physics model:
        //   ax_thrust =  T · sin(pitch) / m   → positive pitch = north accel
        //   ay_thrust = −T · sin(roll)  / m   → positive roll  = west accel
        //
        // So to produce desired body-frame acceleration:
        //   pitch =  atan2(ax_body, g)   (positive ax → positive pitch → north)
        //   roll  = −atan2(ay_body, g)   (positive ay → negative roll  → east)
        let pitch_raw = libm::atan2f(ax_body, GRAVITY);
        let roll_raw = -libm::atan2f(ay_body, GRAVITY);

        // ---- Clamp to max tilt ----
        let max = self.gains.max_tilt_rad;
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

    fn ctrl() -> PositionController {
        PositionController::new(PositionGains::default())
    }

    #[test]
    fn at_target_zero_tilt() {
        let out = ctrl().update([0.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.roll_rad.abs() < 1e-6);
        assert!(out.pitch_rad.abs() < 1e-6);
    }

    #[test]
    fn north_of_target_pitches_to_go_south() {
        // Quad is 10m north of target, heading north (yaw=0).
        // In sim: negative pitch → south. So pitch should be negative.
        let out = ctrl().update([10.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.pitch_rad < 0.0, "pitch={}", out.pitch_rad);
        assert!(out.roll_rad.abs() < 1e-6, "roll={}", out.roll_rad);
    }

    #[test]
    fn south_of_target_pitches_to_go_north() {
        // Quad is 10m south of target → in sim, positive pitch → north.
        let out = ctrl().update([-10.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.pitch_rad > 0.0, "pitch={}", out.pitch_rad);
    }

    #[test]
    fn east_of_target_rolls_to_go_west() {
        // Quad is 10m east of target, heading north (yaw=0).
        // In sim: positive roll → west. So roll should be positive.
        let out = ctrl().update([0.0, 10.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(out.roll_rad > 0.0, "roll={}", out.roll_rad);
        assert!(out.pitch_rad.abs() < 1e-6, "pitch={}", out.pitch_rad);
    }

    #[test]
    fn velocity_damps_command() {
        // 2m north of target but already flying south at good speed.
        // Pitch is negative (go south). Velocity damping should make
        // the pitch *less negative* (closer to zero = less aggressive).
        let no_vel = ctrl().update([2.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        let with_vel = ctrl().update([2.0, 0.0], [-1.0, 0.0], [0.0, 0.0], 0.0);
        assert!(with_vel.pitch_rad > no_vel.pitch_rad,
            "damped {} should be closer to zero than undamped {}", with_vel.pitch_rad, no_vel.pitch_rad);
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
        // Roll should be significant, pitch should be near zero.
        assert!(out.roll_rad.abs() > 0.01, "roll={}", out.roll_rad);
        assert!(out.pitch_rad.abs() < 0.01, "pitch={}", out.pitch_rad);
    }
}
