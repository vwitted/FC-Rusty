// sim.rs — Simple quadrotor physics simulation
//
// A 6DOF rigid body model of a quadcopter with first-order motor
// dynamics. Motor commands pass through a low-pass filter (τ ≈ 30ms)
// before producing thrust, modelling ESC + motor inertia lag.
//
// Quadratic aerodynamic drag on airspeed relative to the wind; no ground
// effect. Motor lag makes PID tuning transfer more realistically to real
// hardware — higher Kp and nonzero Kd are now stable and beneficial.
//
// Rotation is a full ZYX matrix via nalgebra, matching what read_imu
// already used. It did NOT match before: step() projected thrust with a
// hand-rolled small-angle expression that ignored yaw entirely and carried
// opposite lateral signs, so the dynamics and the sensor model disagreed
// about which way the aircraft was pointing. Invisible at hover, fatal for
// anything that translates.
//
// Good enough to validate the control loop before risking hardware.
//
// Coordinate system (NED — North-East-Down):
//   X = forward, Y = right, Z = down
//   Roll = rotation about X, Pitch = about Y, Yaw = about Z
//   Positive Z acceleration = downward (gravity is +9.81)

/// Physical properties of the quadcopter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadParams {
    /// Mass in kg
    pub mass: f32,
    /// Moments of inertia [Ixx, Iyy, Izz] in kg·m²
    /// (assuming symmetric quad, Ixx ≈ Iyy)
    pub inertia: [f32; 3],
    /// Arm length in metres (centre to motor)
    pub arm_length: f32,
    /// Maximum total thrust in Newtons (all 4 motors at 100%)
    pub max_thrust: f32,
    /// Torque-to-thrust ratio for yaw (motor reaction torque)
    /// Typical value: ~0.01-0.02
    pub yaw_torque_coeff: f32,
    /// Motor time constant in seconds (first-order lag).
    /// Models ESC response + motor/prop inertia.
    /// Typical: 20-50ms for racing quads, 50-100ms for heavy lifters.
    pub motor_tau: f32,
    /// Quadratic drag coefficient, kg/m: accel = -(drag_k/mass)*|v_rel|*v_rel.
    /// Lumps 0.5*rho*Cd*A into one isotropic number -- a real airframe
    /// presents very different area forwards and downwards, so treat this as
    /// an order-of-magnitude term, not a wind-tunnel figure. Sized so a
    /// 0.6 kg quad terminals at about 30 m/s.
    pub drag_k: f32,
    /// Steady wind in world NED, m/s. Environment rather than airframe, but
    /// it lives here so it rides along with the plant everywhere the plant
    /// already goes. Wind reaches the aircraft ONLY through drag_k.
    pub wind_ned: [f32; 3],
}

impl Default for QuadParams {
    /// Reasonable defaults for a 5" racing quad (~600g)
    fn default() -> Self {
        Self {
            mass: 0.6,
            inertia: [0.004, 0.004, 0.008],
            arm_length: 0.12,
            max_thrust: 20.0, // ~3:1 thrust-to-weight ratio
            yaw_torque_coeff: 0.015,
            motor_tau: 0.03, // 30ms — typical racing quad ESC+motor
            drag_k: 0.0065,  // ~30 m/s terminal velocity at 0.6 kg
            wind_ned: [0.0; 3],
        }
    }
}

/// Full state of the quadcopter.
///
/// 12 states: position (3), velocity (3), attitude (3), angular rate (3)
/// Attitude is Euler angles in degrees for readability.
#[derive(Debug, Clone, Default)]
pub struct QuadState {
    // ---- Position (metres, NED frame) ----
    pub x: f32,
    pub y: f32,
    pub z: f32,     // positive = down (NED)

    // ---- Velocity (m/s, NED frame) ----
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,

    // ---- Attitude (degrees) ----
    pub roll: f32,  // degrees
    pub pitch: f32, // degrees
    pub yaw: f32,   // degrees

    // ---- Angular rates (°/s, body frame) ----
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,
}

impl QuadState {
    /// Create a state representing the quad sitting level on the
    /// ground (Z=0 means ground in our NED frame, but we'll use
    /// Z negative = above ground for intuition).
    pub fn on_ground() -> Self {
        Self::default()
    }

    /// Create a state hovering at a given altitude.
    /// altitude is in metres above ground (positive up).
    pub fn hovering(altitude: f32) -> Self {
        Self {
            z: -altitude, // NED: negative Z = above ground
            ..Default::default()
        }
    }
}

/// Motor forces — the output of the mixer, input to physics.
///
/// Each motor produces a thrust (0.0 - 1.0 normalised).
/// The simulation converts these to forces and torques.
pub struct MotorForces {
    pub motors: [f32; 4],
}

/// The physics simulation.
pub struct QuadSim {
    pub params: QuadParams,
    pub state: QuadState,
    /// Actual motor output after ESC+motor lag (0.0–1.0 per motor).
    /// Commands are filtered through a first-order lag before producing thrust.
    pub motor_state: [f32; 4],
    /// Kinematic acceleration in world NED frame [ax, ay, az] (m/s²).
    /// Saved at the end of every `step()` so sensor simulators and the
    /// state estimator can consume it as ground truth.
    pub last_accel_world: [f32; 3],
}

impl QuadSim {
    pub fn new(params: QuadParams, initial_state: QuadState) -> Self {
        Self {
            params,
            state: initial_state,
            motor_state: [0.0; 4],
            last_accel_world: [0.0, 0.0, 0.0],
        }
    }

    /// Create a sim pre-initialized for hover (motor state at hover throttle).
    pub fn new_hovering(params: QuadParams, altitude: f32) -> Self {
        let hover = (params.mass * 9.81) / params.max_thrust;
        Self {
            state: QuadState::hovering(altitude),
            motor_state: [hover; 4],
            params,
            // In hover the net specific force in world NED is zero
            // (thrust exactly cancels gravity). Kinematic accel is 0,0,0.
            last_accel_world: [0.0, 0.0, 0.0],
        }
    }

    /// Step the simulation forward by dt seconds.
    ///
    /// Takes normalised motor outputs (0.0-1.0 each) and
    /// computes the resulting forces, torques, accelerations,
    /// and integrates the state.
    ///
    /// Uses simple Euler integration — not great for large dt
    /// but fine for 200 Hz (dt = 0.005s).
    pub fn step(&mut self, motors: &MotorForces, dt: f32) {
        let p = &self.params;

        // ---- Motor dynamics: first-order lag ----
        // motor_actual += (motor_cmd - motor_actual) * (dt / tau)
        // This models the combined ESC processing + motor/prop spin-up time.
        // alpha = dt/tau, clamped to 1.0 for stability if dt > tau.
        let alpha = (dt / p.motor_tau).min(1.0);
        for i in 0..4 {
            let cmd = motors.motors[i].clamp(0.0, 1.0);
            self.motor_state[i] += (cmd - self.motor_state[i]) * alpha;
        }

        // ---- Convert actual motor output to forces and torques ----

        // Each motor's thrust in Newtons (uses filtered motor state, not raw command)
        let thrust_per_motor: [f32; 4] = [
            self.motor_state[0] * p.max_thrust / 4.0,
            self.motor_state[1] * p.max_thrust / 4.0,
            self.motor_state[2] * p.max_thrust / 4.0,
            self.motor_state[3] * p.max_thrust / 4.0,
        ];

        // Total thrust (acts along body Z axis, which is "up" in body frame)
        let total_thrust: f32 = thrust_per_motor.iter().sum();

        // Torques from differential thrust (quad-X, props-in):
        //   M1 (rear-right, CW), M2 (front-right, CCW),
        //   M3 (rear-left, CCW), M4 (front-left, CW)
        //
        //   Roll torque:  (left motors - right motors) * arm_length
        //   Pitch torque: (front motors - rear motors) * arm_length
        //   Yaw torque:   (CCW reactions - CW reactions) * yaw_coeff
        let l = p.arm_length;

        // Left = M3+M4, Right = M1+M2
        let roll_torque = ((thrust_per_motor[2] + thrust_per_motor[3])
            - (thrust_per_motor[0] + thrust_per_motor[1]))
            * l;

        // Front = M2+M4, Rear = M1+M3
        let pitch_torque = ((thrust_per_motor[1] + thrust_per_motor[3])
            - (thrust_per_motor[0] + thrust_per_motor[2]))
            * l;

        // Yaw torque from motor reaction:
        //   CW motor → CCW reaction on frame (negative yaw)
        //   CCW motor → CW reaction on frame (positive yaw)
        // CCW motors = M2 (FR) + M3 (RL); CW motors = M1 (RR) + M4 (FL)
        let yaw_torque = ((thrust_per_motor[1] + thrust_per_motor[2])
            - (thrust_per_motor[0] + thrust_per_motor[3]))
            * p.yaw_torque_coeff;

        // ---- Angular acceleration (body frame) ----
        //
        // Euler's equation: I·omega_dot = tau - omega x (I·omega). The
        // gyroscopic term was missing, which is exactly the term that
        // matters when more than one axis is rotating at once -- i.e. in
        // every recovery or aggressive manoeuvre this sim is meant to test.
        const D2R: f32 = core::f32::consts::PI / 180.0;
        const R2D: f32 = 180.0 / core::f32::consts::PI;
        let w = [
            self.state.roll_rate * D2R,
            self.state.pitch_rate * D2R,
            self.state.yaw_rate * D2R,
        ];
        let iw = [w[0] * p.inertia[0], w[1] * p.inertia[1], w[2] * p.inertia[2]];
        // omega x (I omega)
        let gyro_c = [
            w[1] * iw[2] - w[2] * iw[1],
            w[2] * iw[0] - w[0] * iw[2],
            w[0] * iw[1] - w[1] * iw[0],
        ];
        let roll_accel = (roll_torque - gyro_c[0]) / p.inertia[0];
        let pitch_accel = (pitch_torque - gyro_c[1]) / p.inertia[1];
        let yaw_accel = (yaw_torque - gyro_c[2]) / p.inertia[2];

        // Convert torque-induced angular accel from rad/s² to °/s²
        let roll_accel_deg = roll_accel * R2D;
        let pitch_accel_deg = pitch_accel * R2D;
        let yaw_accel_deg = yaw_accel * R2D;

        // ---- Linear acceleration (world frame) ----
        //
        // Full ZYX rotation, the same one read_imu uses. Thrust acts along
        // -Z in body frame (up); rotating it into world NED is the whole
        // coupling between attitude and translation, and getting it right
        // is what makes yaw affect WHICH WAY the aircraft accelerates.
        use nalgebra::{Rotation3, Vector3};
        let roll_rad = self.state.roll * D2R;
        let pitch_rad = self.state.pitch * D2R;
        let yaw_rad = self.state.yaw * D2R;
        let rot = Rotation3::from_euler_angles(roll_rad, pitch_rad, yaw_rad);
        let a_thrust = rot * Vector3::new(0.0, 0.0, -total_thrust / p.mass);

        // Quadratic drag on airspeed RELATIVE TO THE WIND. Without this the
        // aircraft has no terminal velocity and wind has no effect at all --
        // wind acts on a quad only through this term.
        let rel = Vector3::new(
            self.state.vx - p.wind_ned[0],
            self.state.vy - p.wind_ned[1],
            self.state.vz - p.wind_ned[2],
        );
        let speed = libm::sqrtf(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
        let a_drag = rel * (-(p.drag_k / p.mass) * speed);

        // Gravity (NED: positive Z = down)
        let gravity = 9.81;

        let ax = a_thrust.x + a_drag.x;
        let ay = a_thrust.y + a_drag.y;
        let az = a_thrust.z + a_drag.z + gravity;

        // ---- Euler integration ----

        // Angular rates
        self.state.roll_rate += roll_accel_deg * dt;
        self.state.pitch_rate += pitch_accel_deg * dt;
        self.state.yaw_rate += yaw_accel_deg * dt;

        // Attitude. Body rates are NOT Euler rates -- the old code integrated
        // them as if they were, which is a small-angle assumption hiding in
        // the kinematics rather than in the forces. The transform below is
        // exact; it only breaks at |pitch| = 90 deg, where Euler angles
        // themselves are singular (see the cos_p guard).
        let (sr, cr) = (libm::sinf(roll_rad), libm::cosf(roll_rad));
        let cos_p = libm::cosf(pitch_rad);
        // Guard the gimbal-lock singularity. Euler STATE cannot represent
        // it; representing it needs a quaternion state, which is a larger
        // change. Past ~85 deg pitch, treat results here as unreliable.
        let cos_p = if cos_p.abs() < 1e-3 { 1e-3f32.copysign(cos_p) } else { cos_p };
        let tan_p = libm::sinf(pitch_rad) / cos_p;
        let (wx, wy, wz) = (w[0], w[1], w[2]); // body rates, rad/s
        let roll_dot = wx + sr * tan_p * wy + cr * tan_p * wz;
        let pitch_dot = cr * wy - sr * wz;
        let yaw_dot = (sr / cos_p) * wy + (cr / cos_p) * wz;

        self.state.roll += roll_dot * R2D * dt;
        self.state.pitch += pitch_dot * R2D * dt;
        self.state.yaw += yaw_dot * R2D * dt;

        // Velocity
        self.state.vx += ax * dt;
        self.state.vy += ay * dt;
        self.state.vz += az * dt;

        // Position
        self.state.x += self.state.vx * dt;
        self.state.y += self.state.vy * dt;
        self.state.z += self.state.vz * dt;

        // ---- Ground collision (crude) ----
        if self.state.z > 0.0 {
            self.state.z = 0.0;
            self.state.vz = 0.0_f32.min(self.state.vz); // can't go through floor
        }

        // Save kinematic world-frame acceleration for sensor simulators
        // and state estimators (ground truth for the body-frame accel
        // that an IMU would measure: sf_body = R^T * (a_world - g)).
        self.last_accel_world = [ax, ay, az];
    }

    /// What the IMU would read (for feeding back to the controller).
    ///
    /// Returns simulated sensor data matching the WT901B format.
    ///
    /// `accel` is the body-frame **specific force** — the thing a real
    /// accelerometer measures — in m/s². It equals
    /// `R^T · (a_world − g_world)` where `g_world = [0, 0, +9.81]` (NED).
    /// At stationary hover this evaluates to `[0, 0, −9.81]` (the Z
    /// accelerometer reads the reaction force propping the quad up).
    ///
    /// The world → body rotation uses `nalgebra::Rotation3::from_euler_angles`
    /// so that a KF consumer can use the *same* call to decode it
    /// round-trip exactly, regardless of the small-angle approximation
    /// used inside `step()`.
    pub fn read_imu(&self) -> SimImu {
        use nalgebra::{Rotation3, Vector3};
        let r = self.state.roll * core::f32::consts::PI / 180.0;
        let p = self.state.pitch * core::f32::consts::PI / 180.0;
        let y = self.state.yaw * core::f32::consts::PI / 180.0;
        let rot = Rotation3::from_euler_angles(r, p, y);

        let a_world = Vector3::new(
            self.last_accel_world[0],
            self.last_accel_world[1],
            self.last_accel_world[2],
        );
        let g_world = Vector3::new(0.0, 0.0, 9.81);
        let sf_world = a_world - g_world;
        let sf_body = rot.inverse() * sf_world;

        SimImu {
            gyro: [
                self.state.roll_rate,
                self.state.pitch_rate,
                self.state.yaw_rate,
            ],
            angle: [
                self.state.roll,
                self.state.pitch,
                self.state.yaw,
            ],
            accel: [sf_body.x, sf_body.y, sf_body.z],
        }
    }
}

/// Simulated IMU reading (matches what the WT901B would output).
pub struct SimImu {
    pub gyro: [f32; 3],   // °/s
    pub angle: [f32; 3],  // degrees
    pub accel: [f32; 3],  // m/s²
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_freefall() {
        // No motors, should fall under gravity
        let mut sim = QuadSim::new(
            QuadParams::default(),
            QuadState::hovering(10.0),
        );

        let no_thrust = MotorForces { motors: [0.0; 4] };

        // Run for 1 second at 200 Hz
        for _ in 0..200 {
            sim.step(&no_thrust, 0.005);
        }

        // After 1s of freefall: z should have increased (NED: falling = +Z)
        // v = g*t = 9.81 m/s, z = -10 + 0.5*g*t² = -10 + 4.9 = -5.1
        assert!(sim.state.vz > 9.0, "should be falling fast: vz={}", sim.state.vz);
        assert!(sim.state.z > -6.0, "should have fallen significantly: z={}", sim.state.z);
    }

    #[test]
    fn test_hover_thrust() {
        // At hover, total thrust = weight = mass * g
        // Per motor normalised = (mass * g) / max_thrust
        let params = QuadParams::default();
        let hover_throttle = (params.mass * 9.81) / params.max_thrust;

        // Use new_hovering so motor state starts at hover throttle
        // (otherwise motor lag causes initial altitude drop)
        let mut sim = QuadSim::new_hovering(params, 10.0);

        let hover = MotorForces {
            motors: [hover_throttle; 4],
        };

        // Run for 2 seconds
        let initial_z = sim.state.z;
        for _ in 0..400 {
            sim.step(&hover, 0.005);
        }

        // Should stay roughly at the same altitude
        let drift = (sim.state.z - initial_z).abs();
        assert!(drift < 0.5, "should hover roughly in place, drifted {} m", drift);
    }

    #[test]
    fn test_roll_from_differential_thrust() {
        let mut sim = QuadSim::new(
            QuadParams::default(),
            QuadState::hovering(10.0),
        );

        // More thrust on left motors (M3=RL, M4=FL) than right (M1=RR, M2=FR)
        let roll_right = MotorForces {
            motors: [0.2, 0.2, 0.4, 0.4],
        };

        // Run for 0.5 seconds
        for _ in 0..100 {
            sim.step(&roll_right, 0.005);
        }

        // Should have developed a positive roll rate and roll angle
        assert!(
            sim.state.roll > 1.0,
            "should have rolled: roll={}°",
            sim.state.roll
        );
    }

    #[test]
    fn test_motor_lag() {
        // Motor state should lag behind commands by ~tau
        let params = QuadParams::default(); // tau=0.03
        let mut sim = QuadSim::new(params, QuadState::hovering(10.0));

        // Command full throttle — motor state starts at 0
        let full = MotorForces { motors: [1.0; 4] };

        // After one time constant (30ms = 6 steps at 200Hz),
        // first-order response should reach ~63% of target
        for _ in 0..6 {
            sim.step(&full, 0.005);
        }
        let reached = sim.motor_state[0];
        assert!(
            reached > 0.55 && reached < 0.75,
            "after 1τ should be ~63%, got {:.1}%",
            reached * 100.0
        );

        // After 5τ (150ms = 30 steps), should be >99%
        for _ in 0..24 {
            sim.step(&full, 0.005);
        }
        assert!(
            sim.motor_state[0] > 0.99,
            "after 5τ should be ~100%, got {:.1}%",
            sim.motor_state[0] * 100.0
        );
    }

    #[test]
    fn test_ground_collision() {
        let mut sim = QuadSim::new(
            QuadParams::default(),
            QuadState::hovering(1.0), // only 1m up
        );

        let no_thrust = MotorForces { motors: [0.0; 4] };

        // Fall for 2 seconds — should hit ground
        for _ in 0..400 {
            sim.step(&no_thrust, 0.005);
        }

        // Should be on the ground, not below it
        assert!(
            sim.state.z >= -0.01,
            "should be on ground, z={}",
            sim.state.z
        );
    }
    // ---- Physics fidelity ----

    fn hover_params() -> QuadParams {
        QuadParams::default()
    }

    /// Hover must still be an exact equilibrium. Drag is zero at zero
    /// airspeed, so adding it must not disturb the trim every other test
    /// and every sweep baseline depends on.
    #[test]
    fn hover_is_still_an_exact_equilibrium() {
        let p = hover_params();
        let mut sim = QuadSim::new_hovering(p, 5.0);
        let hover = (p.mass * 9.81) / p.max_thrust;
        for _ in 0..8000 {
            sim.step(&MotorForces { motors: [hover; 4] }, 1.0 / 8000.0);
        }
        assert!((-sim.state.z - 5.0).abs() < 1e-3, "alt drifted to {}", -sim.state.z);
        assert!(sim.state.vz.abs() < 1e-3, "vz drifted to {}", sim.state.vz);
    }

    /// The bug this rewrite fixes: step() ignored yaw, so a yawed aircraft
    /// accelerated in the wrong world direction while read_imu -- which DID
    /// use yaw -- reported something inconsistent with it.
    #[test]
    fn yaw_rotates_the_thrust_vector_into_the_world_frame() {
        let p = hover_params();
        let hover = (p.mass * 9.81) / p.max_thrust;

        let mut level = QuadSim::new(p, QuadState::hovering(20.0));
        level.state.pitch = -15.0;
        let mut yawed = QuadSim::new(p, QuadState::hovering(20.0));
        yawed.state.pitch = -15.0;
        yawed.state.yaw = 90.0;

        for _ in 0..400 {
            level.step(&MotorForces { motors: [hover; 4] }, 1.0 / 400.0);
            yawed.step(&MotorForces { motors: [hover; 4] }, 1.0 / 400.0);
        }
        // Nose-down pitch drives it along +X when facing north...
        assert!(level.state.vx > 1.0, "level vx {}", level.state.vx);
        assert!(level.state.vy.abs() < 0.2, "level vy {}", level.state.vy);
        // ...and along +Y when yawed 90 deg east. Same body-frame attitude,
        // different world direction. The old code gave the same answer for
        // both, which is the defect.
        assert!(yawed.state.vy > 1.0, "yawed vy {}", yawed.state.vy);
        assert!(yawed.state.vx.abs() < 0.2, "yawed vx {}", yawed.state.vx);
    }

    /// Drag must produce a finite terminal velocity. Without it the aircraft
    /// accelerates forever and any wind or dive test is fiction.
    #[test]
    fn free_fall_reaches_the_expected_terminal_velocity() {
        let p = hover_params();
        let mut sim = QuadSim::new(p, QuadState::hovering(10_000.0));
        for _ in 0..200_000 {
            sim.step(&MotorForces { motors: [0.0; 4] }, 1.0 / 1000.0);
        }
        // mg = k v^2  =>  v = sqrt(mg/k)
        let want = ((p.mass * 9.81) / p.drag_k).sqrt();
        assert!(
            (sim.state.vz - want).abs() < want * 0.02,
            "terminal {} should be ~{}", sim.state.vz, want
        );
    }

    /// Wind reaches the aircraft only through drag, so a hovering quad in
    /// wind must be pushed downwind. Zero drag would make wind a no-op.
    #[test]
    fn wind_pushes_a_hovering_quad_downwind() {
        let mut p = hover_params();
        p.wind_ned = [8.0, 0.0, 0.0];
        let hover = (p.mass * 9.81) / p.max_thrust;
        let mut sim = QuadSim::new_hovering(p, 20.0);
        for _ in 0..2000 {
            sim.step(&MotorForces { motors: [hover; 4] }, 1.0 / 400.0);
        }
        assert!(sim.state.vx > 1.0, "should drift downwind, vx={}", sim.state.vx);
        assert!(sim.state.x > 1.0, "should have moved downwind, x={}", sim.state.x);
    }

    /// Body rates are not Euler rates. Rolled 90 deg, a pure body-YAW rate
    /// changes PITCH, not heading -- the coupling the old integration missed
    /// entirely, and the one that dominates any recovery manoeuvre.
    #[test]
    fn body_rates_are_transformed_into_euler_rates() {
        let p = hover_params();
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        sim.state.roll = 90.0;
        sim.state.yaw_rate = 20.0; // deg/s about body z
        let before = (sim.state.pitch, sim.state.yaw);
        for _ in 0..100 {
            sim.step(&MotorForces { motors: [0.0; 4] }, 1.0 / 1000.0);
        }
        let d_pitch = (sim.state.pitch - before.0).abs();
        let d_yaw = (sim.state.yaw - before.1).abs();
        assert!(d_pitch > d_yaw, "at 90 deg roll, body yaw must feed PITCH: d_pitch {d_pitch} vs d_yaw {d_yaw}");
    }

    /// Euler's equation, not just I*omega_dot = tau. With two axes spinning,
    /// the omega x I*omega term produces torque on the third.
    #[test]
    fn gyroscopic_coupling_transfers_rate_between_axes() {
        let mut p = hover_params();
        p.inertia = [0.004, 0.006, 0.009]; // asymmetric, or the term vanishes
        let mut sim = QuadSim::new(p, QuadState::hovering(50.0));
        sim.state.roll_rate = 200.0;
        sim.state.pitch_rate = 200.0;
        let before = sim.state.yaw_rate;
        for _ in 0..100 {
            sim.step(&MotorForces { motors: [0.0; 4] }, 1.0 / 1000.0);
        }
        assert!(
            (sim.state.yaw_rate - before).abs() > 1.0,
            "yaw rate should be driven by roll*pitch coupling, moved {}",
            sim.state.yaw_rate - before
        );
    }
}