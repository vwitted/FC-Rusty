// sim.rs — Simple quadrotor physics simulation
//
// A 6DOF rigid body model of a quadcopter with first-order motor
// dynamics. Motor commands pass through a low-pass filter (τ ≈ 30ms)
// before producing thrust, modelling ESC + motor inertia lag.
//
// No aerodynamic drag or ground effect, but the motor lag makes PID
// tuning transfer more realistically to real hardware — higher Kp
// and nonzero Kd are now stable and beneficial.
//
// Good enough to validate the control loop before risking hardware.
//
// Coordinate system (NED — North-East-Down):
//   X = forward, Y = right, Z = down
//   Roll = rotation about X, Pitch = about Y, Yaw = about Z
//   Positive Z acceleration = downward (gravity is +9.81)

/// Physical properties of the quadcopter.
#[derive(Debug, Clone)]
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
        let roll_accel = roll_torque / p.inertia[0];    // °/s² (we'll convert)
        let pitch_accel = pitch_torque / p.inertia[1];
        let yaw_accel = yaw_torque / p.inertia[2];

        // Convert torque-induced angular accel from rad/s² to °/s²
        let roll_accel_deg = roll_accel * 180.0 / core::f32::consts::PI;
        let pitch_accel_deg = pitch_accel * 180.0 / core::f32::consts::PI;
        let yaw_accel_deg = yaw_accel * 180.0 / core::f32::consts::PI;

        // ---- Linear acceleration (world frame) ----
        // Thrust acts along body Z (upward in body frame).
        // We need to rotate it into the world frame using attitude.
        let roll_rad = self.state.roll * core::f32::consts::PI / 180.0;
        let pitch_rad = self.state.pitch * core::f32::consts::PI / 180.0;

        // Simplified rotation (small angle approximation breaks down
        // at large angles, but works for initial testing):
        // For a proper sim you'd use a full rotation matrix or quaternion.
        // This captures the essential coupling: tilting produces lateral accel.
        let az_thrust = -total_thrust * libm::cosf(roll_rad) * libm::cosf(pitch_rad) / p.mass;
        let ax_thrust = total_thrust * libm::sinf(pitch_rad) / p.mass;
        let ay_thrust = -total_thrust * libm::sinf(roll_rad) * libm::cosf(pitch_rad) / p.mass;

        // Gravity (NED: positive Z = down)
        let gravity = 9.81;

        let ax = ax_thrust;
        let ay = ay_thrust;
        let az = az_thrust + gravity;

        // ---- Euler integration ----

        // Angular rates
        self.state.roll_rate += roll_accel_deg * dt;
        self.state.pitch_rate += pitch_accel_deg * dt;
        self.state.yaw_rate += yaw_accel_deg * dt;

        // Attitude (integrate rates)
        self.state.roll += self.state.roll_rate * dt;
        self.state.pitch += self.state.pitch_rate * dt;
        self.state.yaw += self.state.yaw_rate * dt;

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
}
