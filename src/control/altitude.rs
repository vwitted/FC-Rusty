// altitude.rs — Altitude hold controller
//
// PD controller on altitude error with velocity damping.
// Outputs a thrust value (0.0–1.0 normalised) that replaces
// the fixed hover_throttle in the control loop.
//
// Runs at 50 Hz (same rate as MPC). The output feeds directly
// into the mixer's thrust channel.
//
// Coordinate convention: altitude is positive-up (converted from
// NED Z which is positive-down). Positive thrust correction = climb.

/// Altitude controller gains.
#[derive(Debug, Clone)]
pub struct AltitudeGains {
    /// Proportional gain on altitude error (m → thrust fraction)
    pub kp: f32,
    /// Derivative gain on vertical velocity (m/s → thrust fraction)
    /// Acts as velocity damping — resists fast vertical motion.
    pub kd: f32,
    /// Integral gain for steady-state error (wind, weight changes)
    pub ki: f32,
}

/// Altitude hold controller.
pub struct AltitudeController {
    pub gains: AltitudeGains,
    /// Base hover throttle (thrust fraction to hold altitude with no error)
    pub hover_throttle: f32,
    /// Maximum thrust output (0.0–1.0)
    pub max_thrust: f32,
    /// Minimum thrust output (0.0–1.0, typically > 0 to prevent freefall)
    pub min_thrust: f32,
    /// Integral accumulator
    integral: f32,
    /// Maximum integral contribution (anti-windup)
    integral_max: f32,
}

impl AltitudeController {
    /// Create a new altitude controller.
    ///
    /// # Arguments
    /// * `gains` — PID gains
    /// * `hover_throttle` — base throttle to maintain altitude (mass*g / max_thrust)
    pub fn new(gains: AltitudeGains, hover_throttle: f32) -> Self {
        Self {
            gains,
            hover_throttle,
            max_thrust: 0.9,
            min_thrust: 0.1,
            integral: 0.0,
            integral_max: 0.2, // ±20% thrust from integral
        }
    }

    /// Compute thrust output for altitude hold.
    ///
    /// # Arguments
    /// * `target_alt` — desired altitude in metres (positive up)
    /// * `current_alt` — current altitude in metres (positive up)
    /// * `vz_up` — vertical velocity in m/s (positive up)
    /// * `dt` — time step in seconds
    ///
    /// # Returns
    /// Normalised thrust (0.0–1.0) for the mixer's thrust channel.
    pub fn update(&mut self, target_alt: f32, current_alt: f32, vz_up: f32, dt: f32) -> f32 {
        let error = target_alt - current_alt;

        // Integral with anti-windup
        self.integral += error * dt;
        if self.integral > self.integral_max {
            self.integral = self.integral_max;
        } else if self.integral < -self.integral_max {
            self.integral = -self.integral_max;
        }

        // PD + feedforward hover
        // Positive error (below target) → increase thrust
        // Positive vz_up (climbing) → decrease thrust (damping)
        let thrust = self.hover_throttle
            + self.gains.kp * error
            - self.gains.kd * vz_up
            + self.gains.ki * self.integral;

        thrust.clamp(self.min_thrust, self.max_thrust)
    }

    /// Reset integrator (call on mode switch or arming).
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hover_at_target() {
        let gains = AltitudeGains { kp: 0.5, kd: 0.3, ki: 0.1 };
        let mut ctrl = AltitudeController::new(gains, 0.3);

        // At target altitude, zero velocity → should output ~hover throttle
        let thrust = ctrl.update(10.0, 10.0, 0.0, 0.02);
        assert!((thrust - 0.3).abs() < 0.01, "expected ~0.3, got {}", thrust);
    }

    #[test]
    fn test_below_target_increases_thrust() {
        let gains = AltitudeGains { kp: 0.5, kd: 0.3, ki: 0.0 };
        let mut ctrl = AltitudeController::new(gains, 0.3);

        // 2m below target → should increase thrust
        let thrust = ctrl.update(10.0, 8.0, 0.0, 0.02);
        assert!(thrust > 0.3, "should increase thrust when below target, got {}", thrust);
    }

    #[test]
    fn test_climbing_fast_reduces_thrust() {
        let gains = AltitudeGains { kp: 0.5, kd: 0.3, ki: 0.0 };
        let mut ctrl = AltitudeController::new(gains, 0.3);

        // At target but climbing fast → should reduce thrust (damping)
        let thrust = ctrl.update(10.0, 10.0, 3.0, 0.02);
        assert!(thrust < 0.3, "should reduce thrust when climbing fast, got {}", thrust);
    }

    #[test]
    fn test_clamped_output() {
        let gains = AltitudeGains { kp: 0.5, kd: 0.3, ki: 0.0 };
        let mut ctrl = AltitudeController::new(gains, 0.3);

        // Way below target → should clamp to max
        let thrust = ctrl.update(100.0, 0.0, 0.0, 0.02);
        assert_eq!(thrust, 0.9);

        // Way above target → should clamp to min
        let thrust = ctrl.update(0.0, 100.0, 0.0, 0.02);
        assert_eq!(thrust, 0.1);
    }
}
