//! conventions.rs — the frame conventions, stated once and enforced.
//!
//! This module contains no runtime code. It exists because a sign-convention
//! error is invisible until something moves, and this firmware has never
//! flown. One such error is already known (see the ignored test at the
//! bottom), and it survived precisely because nothing exercised it.
//!
//! # The canonical convention
//!
//! World frame is **NED**: +X north, +Y east, +Z **down**. Attitude is
//! **3-2-1 Tait-Bryan (ZYX)**, which is what `attitude_mekf::quat_to_euler`
//! produces and what `nalgebra::Rotation3::from_euler_angles` builds.
//!
//! - **+roll**  = right wing down
//! - **+pitch** = nose **up**
//! - **+yaw**   = nose right (clockwise seen from above)
//!
//! The consequence people get wrong, and the reason this file exists:
//!
//! ```text
//!     a_north = -(T/m) * cos(roll) * sin(pitch)
//!     a_east  = +(T/m) * sin(roll)                (at zero yaw)
//! ```
//!
//! So flying **north requires NOSE-DOWN, i.e. NEGATIVE pitch**, and flying
//! east requires positive roll. Tilting nose-up moves you backwards. That
//! sign is the one that has already been got wrong once here.
//!
//! # What these tests do
//!
//! Each feeds one component data whose frame is unambiguous and asserts the
//! DIRECTION of its response. Directions only — never magnitudes, which are
//! a tuning question and would make these brittle. If two components
//! disagree about a convention, their tests contradict each other and the
//! suite says which stage is the outlier.

#![allow(dead_code)]

#[cfg(test)]
mod tests {
    use crate::attitude_mekf::{AttitudeMekf, MekfParams};
    use crate::control::mixer::{ControlDemand, QUAD_X};
    use crate::control::position::{PositionController, PositionGains};
    use crate::sim::{MotorForces, QuadParams, QuadSim, QuadState};

    const D2R: f32 = core::f32::consts::PI / 180.0;

    fn level_demand(roll: f32, pitch: f32, yaw: f32) -> ControlDemand {
        ControlDemand { thrust: 0.5, roll, pitch, yaw }
    }

    // ---- Stage 1: the mixer ----

    /// +roll demand must put MORE thrust on the LEFT (M3 RL, M4 FL), which
    /// rolls the aircraft right-wing-down.
    #[test]
    fn mixer_positive_roll_lifts_the_left_motors() {
        let m = QUAD_X.apply_no_airmode(&level_demand(0.2, 0.0, 0.0)).motors;
        let left = m[2] + m[3];
        let right = m[0] + m[1];
        assert!(left > right, "+roll must lift left (M3+M4)={left} over right={right}");
    }

    /// +pitch demand must put MORE thrust at the FRONT (M2 FR, M4 FL),
    /// which pitches the nose UP.
    #[test]
    fn mixer_positive_pitch_lifts_the_front_motors() {
        let m = QUAD_X.apply_no_airmode(&level_demand(0.0, 0.2, 0.0)).motors;
        let front = m[1] + m[3];
        let rear = m[0] + m[2];
        assert!(front > rear, "+pitch must lift front (M2+M4)={front} over rear={rear}");
    }

    // ---- Stage 2: mixer -> rigid body ----

    /// Closes the mixer/physics seam: the motor pattern the mixer emits for
    /// +roll must actually produce +roll in the rigid body.
    #[test]
    fn mixer_roll_pattern_produces_positive_roll_angle() {
        let p = QuadParams::default();
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        let motors = QUAD_X.apply_no_airmode(&level_demand(0.2, 0.0, 0.0)).motors;
        for _ in 0..200 {
            sim.step(&MotorForces { motors }, 1.0 / 1000.0);
        }
        assert!(sim.state.roll > 0.0, "roll angle {} should be positive", sim.state.roll);
    }

    #[test]
    fn mixer_pitch_pattern_produces_positive_pitch_angle() {
        let p = QuadParams::default();
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        let motors = QUAD_X.apply_no_airmode(&level_demand(0.0, 0.2, 0.0)).motors;
        for _ in 0..200 {
            sim.step(&MotorForces { motors }, 1.0 / 1000.0);
        }
        assert!(sim.state.pitch > 0.0, "pitch angle {} should be positive", sim.state.pitch);
    }

    // ---- Stage 3: attitude -> translation ----

    /// THE one that matters. Nose-up must move you BACKWARDS (south).
    #[test]
    fn nose_up_pitch_accelerates_south() {
        let p = QuadParams::default();
        let hover = (p.mass * 9.81) / p.max_thrust;
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        sim.state.pitch = 20.0; // nose up
        for _ in 0..400 {
            sim.step(&MotorForces { motors: [hover; 4] }, 1.0 / 400.0);
        }
        assert!(sim.state.vx < -0.5, "+pitch (nose up) must go SOUTH, vx={}", sim.state.vx);
    }

    #[test]
    fn right_wing_down_roll_accelerates_east() {
        let p = QuadParams::default();
        let hover = (p.mass * 9.81) / p.max_thrust;
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        sim.state.roll = 20.0; // right wing down
        for _ in 0..400 {
            sim.step(&MotorForces { motors: [hover; 4] }, 1.0 / 400.0);
        }
        assert!(sim.state.vy > 0.5, "+roll must go EAST, vy={}", sim.state.vy);
    }

    // ---- Stage 4: the estimator ----

    /// Feed the MEKF the specific force a genuinely nose-up aircraft would
    /// measure, and it must report POSITIVE pitch. This is what pins the
    /// convention every controller downstream is written against.
    ///
    /// At rest, an accelerometer measures specific force = -g in body axes.
    /// Nose-up by theta gives [ +sin(theta), 0, -cos(theta) ] g.
    #[test]
    fn mekf_reports_nose_up_as_positive_pitch() {
        let theta = 20.0 * D2R;
        let accel_g = [libm::sinf(theta), 0.0, -libm::cosf(theta)];
        let mut mekf = AttitudeMekf::new(MekfParams::default());
        for _ in 0..4000 {
            mekf.predict([0.0; 3], 1.0 / 400.0);
            mekf.update_accel(accel_g);
        }
        let pitch_deg = mekf.euler()[1] / D2R;
        assert!(pitch_deg > 10.0, "nose-up must read +pitch, got {pitch_deg}");
    }

    /// Right wing down by phi gives [ 0, -sin(phi), -cos(phi) ] g.
    #[test]
    fn mekf_reports_right_wing_down_as_positive_roll() {
        let phi = 20.0 * D2R;
        let accel_g = [0.0, -libm::sinf(phi), -libm::cosf(phi)];
        let mut mekf = AttitudeMekf::new(MekfParams::default());
        for _ in 0..4000 {
            mekf.predict([0.0; 3], 1.0 / 400.0);
            mekf.update_accel(accel_g);
        }
        let roll_deg = mekf.euler()[0] / D2R;
        assert!(roll_deg > 10.0, "right-wing-down must read +roll, got {roll_deg}");
    }

    // ---- Stage 5: the outer loop ----

    /// KNOWN FAILING — this is the bug, written as the test that will pass
    /// when it is fixed.
    ///
    /// To fly NORTH the aircraft must pitch NOSE-DOWN, i.e. the position
    /// controller must command NEGATIVE pitch for a northward error. It
    /// commands positive, because its implementation comment derives the
    /// sign from an old, incorrect QuadSim thrust projection
    /// ("positive pitch = north accel") rather than from the convention
    /// every other stage above uses.
    ///
    /// Ignored rather than deleted or inverted: inverting it would encode
    /// the bug as correct, and deleting it would lose the record. Run with
    /// `cargo test -- --ignored` to see it fail. Un-ignore when position.rs
    /// is fixed and verified.
    #[test]
    fn position_controller_commands_nose_down_to_fly_north() {
        let ctl = PositionController::new(PositionGains::default());
        // 10 m south of target, stationary, facing north: must go north.
        let out = ctl.update([-10.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(
            out.pitch_rad < 0.0,
            "to fly north the aircraft must pitch NOSE-DOWN (negative); got {} rad",
            out.pitch_rad
        );
    }

    /// The roll half of the same bug: to fly EAST, roll right (positive).
    #[test]
    fn position_controller_commands_right_roll_to_fly_east() {
        let ctl = PositionController::new(PositionGains::default());
        let out = ctl.update([0.0, -10.0], [0.0, 0.0], [0.0, 0.0], 0.0);
        assert!(
            out.roll_rad > 0.0,
            "to fly east the aircraft must roll RIGHT (positive); got {} rad",
            out.roll_rad
        );
    }
}
