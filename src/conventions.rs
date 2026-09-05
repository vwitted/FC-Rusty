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
        sim.set_attitude_deg(0.0, 20.0, 0.0); // nose up
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
        sim.set_attitude_deg(20.0, 0.0, 0.0); // right wing down
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

    // ---- Stage 0: sensor mounting ----

    /// A board-orientation sign vector must be a PROPER rotation
    /// (determinant +1). A determinant of -1 is a reflection: it mirrors
    /// the aircraft, looks entirely plausible as a sign triple, and would
    /// invert one axis of everything downstream. [1,1,-1] and [-1,-1,-1]
    /// are both reflections and both look like reasonable typos.
    #[test]
    fn board_orientations_are_rotations_not_reflections() {
        use crate::drivers::orientation::Orientation;
        for o in [Orientation::Roll180, Orientation::Pitch180, Orientation::Identity] {
            let x = o.apply([1.0, 0.0, 0.0]);
            let y = o.apply([0.0, 1.0, 0.0]);
            let z = o.apply([0.0, 0.0, 1.0]);
            // These are all diagonal, so det is just the product of the
            // diagonal, but compute it properly in case one stops being.
            let det = x[0] * (y[1] * z[2] - y[2] * z[1])
                - x[1] * (y[0] * z[2] - y[2] * z[0])
                + x[2] * (y[0] * z[1] - y[1] * z[0]);
            assert!((det - 1.0).abs() < 1e-6, "{o:?} has det {det}, must be +1");
        }
    }

    /// Both IMUs are mounted differently and their readings are AVERAGED.
    /// If their corrections disagreed, averaging would cancel signal rather
    /// than noise. Verifies the algebra round-trips; it cannot verify the
    /// physical mounting, which needs the bench.
    #[test]
    fn both_imu_orientations_agree_after_correction() {
        use crate::drivers::orientation::Orientation;
        let body = [0.3f32, -0.7, 1.1];
        // A 180-degree rotation is its own inverse, so the sensor-native
        // reading for a given body vector is the same map applied once.
        let imu1_native = Orientation::Roll180.apply(body);
        let imu2_native = Orientation::Pitch180.apply(body);
        let imu1_corrected = Orientation::Roll180.apply(imu1_native);
        let imu2_corrected = Orientation::Pitch180.apply(imu2_native);
        for k in 0..3 {
            assert!((imu1_corrected[k] - body[k]).abs() < 1e-6, "IMU1 axis {k}");
            assert!((imu2_corrected[k] - body[k]).abs() < 1e-6, "IMU2 axis {k}");
        }
    }

    // ---- Yaw, which nothing above exercises ----

    /// +yaw demand must spin the aircraft NOSE-RIGHT. Crosses the mixer and
    /// the rigid body: the mixer lifts the CCW props (M2 FR, M3 RL), whose
    /// reaction torque on the frame is clockwise seen from above.
    #[test]
    fn positive_yaw_demand_yaws_nose_right() {
        let p = QuadParams::default();
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        let motors = QUAD_X.apply_no_airmode(&level_demand(0.0, 0.0, 0.2)).motors;
        // CCW pair up, CW pair down.
        assert!(motors[1] + motors[2] > motors[0] + motors[3], "+yaw lifts the CCW pair");
        for _ in 0..400 {
            sim.step(&MotorForces { motors }, 1.0 / 1000.0);
        }
        assert!(sim.state.yaw > 0.0, "+yaw demand must increase yaw, got {}", sim.state.yaw);
    }

    /// The gyro-to-attitude sign inside the estimator: a positive body roll
    /// rate must integrate into increasing roll. Nothing else here tests
    /// the MEKF's propagation convention -- the accel tests only pin its
    /// static response.
    #[test]
    fn mekf_integrates_positive_body_rates_into_positive_angles() {
        let rate = 20.0 * D2R; // rad/s
        let mut roll_mekf = AttitudeMekf::new(MekfParams::default());
        let mut yaw_mekf = AttitudeMekf::new(MekfParams::default());
        for _ in 0..100 {
            roll_mekf.predict([rate, 0.0, 0.0], 1.0 / 100.0);
            yaw_mekf.predict([0.0, 0.0, rate], 1.0 / 100.0);
        }
        assert!(roll_mekf.euler()[0] > 0.0, "roll {}", roll_mekf.euler()[0]);
        assert!(yaw_mekf.euler()[2] > 0.0, "yaw {}", yaw_mekf.euler()[2]);
    }

    // ---- Stage 6: the RC stick convention ----

    /// The RC stick convention, stated by the aircraft's owner and now
    /// enforced:
    ///
    ///   Stick UP -> INCREASING channel value -> positive `pitch_input`
    ///   Stick UP -> aircraft pitches DOWN    -> negative commanded pitch
    ///
    /// Three places read that channel, and they used to disagree. The COG
    /// gate and PosHold both treated positive as "forward"; Acro and
    /// AltHold treated it as nose-UP, which is backwards. This test crosses
    /// the two readings so they can never drift apart again: the same stick
    /// deflection the COG gate calls "flying forward" must produce a
    /// nose-DOWN attitude command.
    #[test]
    fn cog_gate_and_attitude_path_agree_on_what_forward_means() {
        use crate::control::modes::{
            nav_step, should_fuse_cog, CogGate, FlightMode, NavInputs, NavState,
            FWD_STICK_MIN,
        };
        use crate::control::altitude::{AltitudeController, AltitudeGains};
        use crate::control::position::{PositionController, PositionGains};

        let fwd = FWD_STICK_MIN + 0.2;

        // The COG gate accepts this as deliberate forward flight.
        assert!(should_fuse_cog(&CogGate {
            armed: true,
            has_3d_fix: true,
            ground_speed_ms: 10.0,
            pitch_input: fwd,
        }));

        // The attitude path must agree: nose DOWN, which flies forward.
        let mut st = NavState::new(
            AltitudeController::new(AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 }, 0.294),
            PositionController::new(PositionGains::default()),
            0.294,
        );
        let out = nav_step(
            &NavInputs {
                mode: FlightMode::Acro,
                roll_input: 0.0,
                pitch_input: fwd,
                yaw_input: 0.0,
                throttle_raw: 0.5,
                max_angle_deg: 30.0,
                yaw_rad: 0.0,
                pos_est: None,
                dt: 0.01,
                hover_throttle: 0.294,
            },
            &mut st,
        );
        assert!(
            out.desired_pitch_rad < 0.0,
            "the COG gate calls {fwd} 'forward', so the attitude path must command nose-DOWN; got {} rad",
            out.desired_pitch_rad
        );
    }
}
