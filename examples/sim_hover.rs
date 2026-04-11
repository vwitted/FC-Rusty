// examples/sim_hover.rs — Simulate a hover test (PID only, with altitude hold)
//
// Run with: cargo run --example sim_hover --no-default-features
//
// This wires together the PID controller, altitude controller, mixer,
// and physics simulation to test a hover + disturbance scenario
// entirely in software. No hardware needed.

use fc_rusty::control::altitude::{AltitudeController, AltitudeGains};
use fc_rusty::control::mixer::{ControlDemand, QUAD_X};
use fc_rusty::control::pid::{PidGains, PidLimits, RatePidController};
use fc_rusty::sim::{MotorForces, QuadParams, QuadSim};

fn main() {
    let params = QuadParams::default();
    let hover_throttle = (params.mass * 9.81) / params.max_thrust;

    println!("=== Quadcopter Hover Simulation (PID + Alt Hold) ===");
    println!("Mass: {}kg, Max thrust: {}N", params.mass, params.max_thrust);
    println!("Hover throttle: {:.1}%", hover_throttle * 100.0);
    println!();

    // ---- Physics sim (motor state pre-initialized to hover) ----
    let mut sim = QuadSim::new_hovering(params, 5.0);

    // ---- Altitude controller (50 Hz) ----
    let alt_gains = AltitudeGains {
        kp: 0.15,  // 1m error → 15% thrust change
        kd: 0.1,   // 1m/s velocity → 10% thrust damping
        ki: 0.05,  // slow integral for steady-state
    };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let target_alt = 5.0;
    let alt_divider = 4; // 50 Hz (every 4th step at 200 Hz)
    let mut current_thrust = hover_throttle;

    // ---- Rate PID (200 Hz) ----
    let rate_gains = PidGains {
        kp: 0.02,
        ki: 0.005,
        kd: 0.001,
    };
    let yaw_gains = PidGains {
        kp: 0.03,
        ki: 0.005,
        kd: 0.0,
    };
    let limits = PidLimits {
        integral_max: 0.3,
        output_max: 0.5,
        d_lpf_tau_s: 0.008, // ~20 Hz cutoff — smooths D term against motor-lag bang-bang
    };
    let mut rate_pid = RatePidController::new(rate_gains, rate_gains, yaw_gains, limits);

    // ---- Simple angle-to-rate outer loop (placeholder for MPC) ----
    let angle_kp: f32 = 4.0;

    let dt = 0.005; // 200 Hz
    let total_time = 10.0;
    let steps = (total_time / dt) as usize;

    let target_roll = 0.0f32;
    let target_pitch = 0.0f32;
    let target_yaw = 0.0f32;

    println!(
        "{:>6} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>10}",
        "time", "roll", "pitch", "yaw", "alt", "vz", "throttle", "event"
    );
    println!("{}", "-".repeat(74));

    for step in 0..steps {
        let t = step as f32 * dt;

        // ---- Simulate disturbances ----
        let event = if step == (2.0 / dt) as usize {
            sim.state.roll_rate += 10.0;
            "GUST"
        } else if step == (5.0 / dt) as usize {
            sim.state.vz += 2.0;
            "DROP"
        } else {
            ""
        };

        // ---- Read simulated sensors ----
        let imu = sim.read_imu();

        // ---- Altitude hold (50 Hz) ----
        if step % alt_divider == 0 {
            let alt = -sim.state.z;       // NED → positive-up
            let vz_up = -sim.state.vz;    // NED → positive-up
            current_thrust = alt_ctrl.update(target_alt, alt, vz_up, dt * alt_divider as f32);
        }

        // ---- Angle → rate setpoint (outer loop) ----
        let rate_setpoint = [
            angle_kp * (target_roll - imu.angle[0]),
            angle_kp * (target_pitch - imu.angle[1]),
            angle_kp * (target_yaw - imu.angle[2]),
        ];

        // ---- Rate PID (inner loop) ----
        let pid_output = rate_pid.update(rate_setpoint, imu.gyro, dt);

        // ---- Mixer ----
        let demand = ControlDemand {
            thrust: current_thrust,
            roll: pid_output[0],
            pitch: pid_output[1],
            yaw: pid_output[2],
        };
        let motor_out = QUAD_X.apply(&demand);

        // ---- Step physics ----
        sim.step(
            &MotorForces {
                motors: motor_out.motors,
            },
            dt,
        );

        // ---- Print state every 50ms ----
        if step % 10 == 0 {
            let alt = -sim.state.z;
            let mean_motor = (motor_out.motors[0] + motor_out.motors[1]
                + motor_out.motors[2] + motor_out.motors[3]) / 4.0;
            println!(
                "{:6.2} r={:>6.2} p={:>6.2} y={:>6.2} a={:>6.2} vz={:>6.2} thr={:>5.1}% rsp={:>6.1} gyro={:>6.1} pid={:>6.3} mean={:.2} {}",
                t,
                sim.state.roll,
                sim.state.pitch,
                sim.state.yaw,
                alt,
                sim.state.vz,
                current_thrust * 100.0,
                rate_setpoint[0],
                imu.gyro[0],
                pid_output[0],
                mean_motor,
                event,
            );
        }
    }

    println!();
    println!("=== Final state ===");
    println!("Position: ({:.2}, {:.2}, {:.2}m altitude)", sim.state.x, sim.state.y, -sim.state.z);
    println!("Velocity: ({:.2}, {:.2}, {:.2}) m/s", sim.state.vx, sim.state.vy, sim.state.vz);
    println!(
        "Attitude: roll={:.2}° pitch={:.2}° yaw={:.2}°",
        sim.state.roll, sim.state.pitch, sim.state.yaw
    );
    println!(
        "Rates: ({:.2}, {:.2}, {:.2}) °/s",
        sim.state.roll_rate, sim.state.pitch_rate, sim.state.yaw_rate
    );
}
