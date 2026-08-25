// pid.rs — Single-axis PID controller for flight control
//
// One instance per axis — you'd have three: roll, pitch, yaw.
// Each takes a setpoint and a measurement, returns a control output.
//
// Key differences from textbook PID:
//
// 1. Derivative on measurement, not error.
//    Prevents derivative kick when the setpoint changes suddenly
//    (e.g. pilot moves stick). The derivative term only responds
//    to actual changes in the physical state, not to step changes
//    in what we're asking for.
//
// 2. Integral windup clamping.
//    Without this, the integral term grows without bound when the
//    controller can't achieve the setpoint (e.g. on the ground,
//    or saturating motor output). Clamping prevents the buildup.
//
// 3. Actual dt measurement.
//    Rather than assuming a fixed loop rate, we take dt as a
//    parameter. This makes I and D terms correct even if the
//    loop has timing jitter.

/// PID gains for one axis.
///
/// Start with small values and increase:
///   - Kp first, until it oscillates, then back off ~30%
///   - Kd next, to dampen the oscillation
///   - Ki last, small, just enough to eliminate steady-state error
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    /// Proportional gain — response to current error
    pub kp: f32,
    /// Integral gain — response to accumulated error
    pub ki: f32,
    /// Derivative gain — response to rate of change of measurement
    pub kd: f32,
}

/// Configuration limits for the PID controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidLimits {
    /// Maximum absolute value of the integral term.
    /// Prevents windup. A reasonable starting value might be
    /// 0.3 (30% of max output).
    pub integral_max: f32,

    /// Maximum absolute value of the total output.
    /// Prevents the PID from demanding more than the mixer
    /// can deliver. Typically 1.0 or less.
    pub output_max: f32,

    /// First-order low-pass filter time constant (seconds) for the
    /// D term. Essential when the measurement (gyro) can change
    /// rapidly between samples — without this filter, any large
    /// rate-of-change in the measurement produces huge D-term spikes
    /// that flip the PID output sign each loop tick, driving the
    /// actuators into bang-bang oscillation. Betaflight-style flight
    /// controllers always filter the D term for exactly this reason.
    ///
    /// Typical values: 0.005–0.010 s (32–16 Hz cutoff). Set to 0.0
    /// to disable filtering (only safe when kd is very small or the
    /// measurement is already low-noise).
    pub d_lpf_tau_s: f32,
}

impl Default for PidLimits {
    fn default() -> Self {
        Self {
            integral_max: 0.3,
            output_max: 1.0,
            d_lpf_tau_s: 0.008, // ~20 Hz cutoff
        }
    }
}

/// A single-axis PID controller.
pub struct Pid {
    pub gains: PidGains,
    pub limits: PidLimits,

    /// Accumulated integral term
    integral: f32,

    /// Previous measurement (for derivative-on-measurement)
    prev_measurement: f32,

    /// Low-pass filtered derivative of measurement (state for the
    /// first-order IIR filter on the D term). See `PidLimits::d_lpf_tau_s`.
    d_filtered: f32,

    /// Whether we've had at least one update (to avoid a
    /// derivative spike on the very first call)
    initialised: bool,
}

impl Pid {
    /// Create a new PID controller with the given gains and limits.
    pub fn new(gains: PidGains, limits: PidLimits) -> Self {
        Self {
            gains,
            limits,
            integral: 0.0,
            prev_measurement: 0.0,
            d_filtered: 0.0,
            initialised: false,
        }
    }

    /// Compute the PID output for one time step.
    ///
    /// # Arguments
    /// * `setpoint` — what we want (e.g. desired roll rate in °/s)
    /// * `measurement` — what we have (e.g. actual roll rate from gyro)
    /// * `dt` — time since last call in seconds
    ///
    /// # Returns
    /// The control output, clamped to ±output_max.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }

        // ---- Proportional term ----
        // Simple: how far are we from where we want to be?
        let error = setpoint - measurement;
        let p_term = self.gains.kp * error;

        // ---- Integral term ----
        // Accumulate error over time, with windup clamping.
        self.integral += error * dt;
        self.integral = self.integral.clamp(
            -self.limits.integral_max / self.gains.ki.max(1e-6),
            self.limits.integral_max / self.gains.ki.max(1e-6),
        );
        let i_term = self.gains.ki * self.integral;

        // ---- Derivative term (on measurement, low-pass filtered) ----
        // Rate of change of the measurement, not the error.
        // Negative sign because if measurement is increasing,
        // we want to slow down (counteract), not speed up.
        //
        // The first-order IIR filter is critical: motor dynamics and
        // discrete-time rate changes can make the raw d_measurement
        // flip sign every loop tick, producing bang-bang output that
        // the actuators average to an unwanted bias. Filtering it
        // smooths the derivative estimate so kd contributes damping
        // rather than oscillation.
        let d_term = if self.initialised {
            let d_measurement = (measurement - self.prev_measurement) / dt;
            if self.limits.d_lpf_tau_s > 0.0 {
                let alpha = dt / (self.limits.d_lpf_tau_s + dt);
                self.d_filtered += alpha * (d_measurement - self.d_filtered);
            } else {
                self.d_filtered = d_measurement;
            }
            -self.gains.kd * self.d_filtered
        } else {
            self.initialised = true;
            0.0
        };

        self.prev_measurement = measurement;

        // ---- Sum and clamp ----
        let output = p_term + i_term + d_term;
        output.clamp(-self.limits.output_max, self.limits.output_max)
    }

    /// Reset the controller state.
    ///
    /// Call this when arming, switching flight modes, or any
    /// time there's a discontinuity in what the controller
    /// should be doing. Prevents integral carryover and
    /// derivative spikes.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
        self.d_filtered = 0.0;
        self.initialised = false;
    }

    /// Update the gains at runtime (e.g. from a tuning interface).
    pub fn set_gains(&mut self, gains: PidGains) {
        self.gains = gains;
    }
}

/// Convenience: a set of three PID controllers for roll, pitch, yaw.
///
/// This is what the control loop actually uses.
pub struct RatePidController {
    pub roll: Pid,
    pub pitch: Pid,
    pub yaw: Pid,
}

impl RatePidController {
    /// Create a new rate controller with the given gains.
    ///
    /// Roll and pitch often share the same gains (symmetric quad),
    /// while yaw is tuned separately (different inertia axis).
    pub fn new(
        roll_gains: PidGains,
        pitch_gains: PidGains,
        yaw_gains: PidGains,
        limits: PidLimits,
    ) -> Self {
        Self {
            roll: Pid::new(roll_gains, limits),
            pitch: Pid::new(pitch_gains, limits),
            yaw: Pid::new(yaw_gains, limits),
        }
    }

    /// Compute PID outputs for all three axes.
    ///
    /// # Arguments
    /// * `setpoint` — desired rates [roll, pitch, yaw] in °/s
    /// * `gyro` — measured rates [roll, pitch, yaw] in °/s
    /// * `dt` — time step in seconds
    ///
    /// # Returns
    /// Control outputs [roll, pitch, yaw], each clamped to ±output_max.
    pub fn update(
        &mut self,
        setpoint: [f32; 3],
        gyro: [f32; 3],
        dt: f32,
    ) -> [f32; 3] {
        [
            self.roll.update(setpoint[0], gyro[0], dt),
            self.pitch.update(setpoint[1], gyro[1], dt),
            self.yaw.update(setpoint[2], gyro[2], dt),
        ]
    }

    /// Reset all three axes.
    pub fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_gains() -> PidGains {
        PidGains {
            kp: 1.0,
            ki: 0.5,
            kd: 0.1,
        }
    }

    fn test_limits() -> PidLimits {
        PidLimits {
            integral_max: 0.5,
            output_max: 1.0,
            d_lpf_tau_s: 0.0, // disable D-LPF so unit tests see raw D-term math
        }
    }

    #[test]
    fn test_zero_error_zero_output() {
        let mut pid = Pid::new(test_gains(), test_limits());
        let output = pid.update(100.0, 100.0, 0.005);
        assert!((output).abs() < 0.001);
    }

    #[test]
    fn test_proportional_response() {
        let gains = PidGains {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
        };
        let mut pid = Pid::new(gains, test_limits());

        // Error of 10 with Kp=2 should give output of 20,
        // but clamped to output_max=1.0
        let output = pid.update(10.0, 0.0, 0.005);
        assert!((output - 1.0).abs() < 0.001);

        // Smaller error: 0.25 with Kp=2 = 0.5
        let mut pid2 = Pid::new(gains, test_limits());
        let output2 = pid2.update(0.25, 0.0, 0.005);
        assert!((output2 - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_integral_accumulates() {
        let gains = PidGains {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
        };
        let mut pid = Pid::new(gains, test_limits());

        // Constant error of 1.0 at dt=0.01
        // After 10 steps: integral = 10 * 1.0 * 0.01 = 0.1
        // Output = ki * integral = 1.0 * 0.1 = 0.1
        for _ in 0..10 {
            pid.update(1.0, 0.0, 0.01);
        }
        let output = pid.update(1.0, 0.0, 0.01);

        // 11 steps: integral = 0.11, output ≈ 0.11
        assert!((output - 0.11).abs() < 0.01);
    }

    #[test]
    fn test_integral_windup_clamped() {
        let gains = PidGains {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
        };
        let limits = PidLimits {
            integral_max: 0.3,
            output_max: 1.0,
            d_lpf_tau_s: 0.0,
        };
        let mut pid = Pid::new(gains, limits);

        // Run for ages with constant error — integral should clamp
        for _ in 0..10000 {
            pid.update(100.0, 0.0, 0.01);
        }

        let output = pid.update(100.0, 0.0, 0.01);
        assert!(output <= 0.3 + 0.01, "integral should be clamped");
    }

    #[test]
    fn test_derivative_on_measurement() {
        let gains = PidGains {
            kp: 0.0,
            ki: 0.0,
            kd: 1.0,
        };
        let mut pid = Pid::new(gains, test_limits());

        // First call: no derivative (not initialised)
        let out1 = pid.update(0.0, 0.0, 0.01);
        assert!((out1).abs() < 0.001);

        // Measurement jumps from 0 to 10 in 0.01s
        // d_measurement/dt = 10/0.01 = 1000
        // D term = -Kd * d_measurement/dt = -1.0 * 1000 = -1000
        // Clamped to -1.0
        let out2 = pid.update(0.0, 10.0, 0.01);
        assert!((out2 - (-1.0)).abs() < 0.001);
    }

    #[test]
    fn test_no_derivative_kick_on_setpoint_change() {
        let gains = PidGains {
            kp: 0.0,
            ki: 0.0,
            kd: 0.1,
        };
        let mut pid = Pid::new(gains, test_limits());

        // Initialise with measurement at 0
        pid.update(0.0, 50.0, 0.005);

        // Setpoint jumps from 0 to 100, but measurement stays at 50.
        // Derivative-on-measurement means: no derivative kick,
        // because measurement didn't change.
        let output = pid.update(100.0, 50.0, 0.005);
        assert!(
            output.abs() < 0.001,
            "D term should be ~0 when measurement unchanged, got {}",
            output
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let mut pid = Pid::new(test_gains(), test_limits());

        // Build up some integral
        for _ in 0..100 {
            pid.update(10.0, 0.0, 0.01);
        }

        pid.reset();

        // After reset, with zero error, output should be zero
        let output = pid.update(5.0, 5.0, 0.005);
        assert!((output).abs() < 0.001);
    }

    #[test]
    fn test_zero_dt_returns_zero() {
        let mut pid = Pid::new(test_gains(), test_limits());
        let output = pid.update(100.0, 0.0, 0.0);
        assert_eq!(output, 0.0);
    }

    #[test]
    fn test_rate_controller_three_axes() {
        let gains = PidGains {
            kp: 0.5,
            ki: 0.0,
            kd: 0.0,
        };
        let limits = PidLimits {
            integral_max: 0.3,
            output_max: 1.0,
            d_lpf_tau_s: 0.0,
        };
        let mut ctrl = RatePidController::new(gains, gains, gains, limits);

        let output = ctrl.update(
            [10.0, -5.0, 20.0],  // setpoints
            [0.0, 0.0, 0.0],     // measurements
            0.005,
        );

        // Roll: 0.5 * 10 = 5.0, clamped to 1.0
        assert!((output[0] - 1.0).abs() < 0.001);
        // Pitch: 0.5 * -5 = -2.5, clamped to -1.0
        assert!((output[1] - (-1.0)).abs() < 0.001);
        // Yaw: 0.5 * 20 = 10, clamped to 1.0
        assert!((output[2] - 1.0).abs() < 0.001);
    }
}
