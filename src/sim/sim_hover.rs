// examples/sim_hover.rs — Simulate a hover test
//
// Run with: cargo run --example sim_hover
//
// This wires together the PID controller, mixer, and physics
// simulation to test a simple hover + disturbance scenario
// entirely in software. No hardware needed.

use fc_firmware::control::mixer::{ControlDemand, QUAD_X};
use fc_firmware::control::pid::{Pid, PidGains, PidLimits, RatePidController};
use fc_firmware::sim::{MotorForces, QuadParams, QuadSim, QuadState};

fn main() {
    let params = QuadParams::default();
    let hover_throttle = (params.mass * 9.81) / params.max_thrust;

    println!("=== Quadcopter Hover Simulation ===");
    println!("Mass: {}kg, Max thrust: {}N", params.mass, params.max_thrust);
    println!("Hover throttle: {:.1}%", hover_throttle * 100.0);
    println!();

    // ---- Set up the physics sim ----
    let mut sim = QuadSim::new(params, QuadState::hovering(5.0));

    // ---- Set up the PID controller ----
    let rate_gains = PidGains {
        kp: 0.15,
        ki: 0.05,
        kd: 0.003,
    };
    let yaw_gains = PidGains {
        kp: 0.2,
        ki: 0.1,
        kd: 0.0,
    };
    let limits = PidLimits {
        integral_max: 0.2,
        output_max: 0.4,
        d_lpf_tau_s: 0.008,
    };

    let mut rate_pid = RatePidController::new(rate_gains, rate_gains, yaw_gains, limits);

    // ---- Simple angle-to-rate outer loop (placeholder for MPC) ----
    let angle_kp: f32 = 4.0; // degrees error → degrees/sec rate setpoint

    let dt = 0.005; // 200 Hz
    let total_time = 10.0; // seconds
    let steps = (total_time / dt) as usize;

    // ---- Target: hover level at current position ----
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

        // ---- Simulate a disturbance ----
        // At t=2.0s, apply a gust that kicks the quad 10°/s in roll
        let event = if step == (2.0 / dt) as usize {
            sim.state.roll_rate += 10.0;
            "GUST"
        } else if step == (5.0 / dt) as usize {
            // At t=5.0s, apply a downward velocity disturbance
            sim.state.vz += 2.0;
            "DROP"
        } else {
            ""
        };

        // ---- Read simulated sensors ----
        let imu = sim.read_imu();

        // ---- Angle → rate setpoint (outer loop placeholder) ----
        let rate_setpoint = [
            angle_kp * (target_roll - imu.angle[0]),
            angle_kp * (target_pitch - imu.angle[1]),
            angle_kp * (target_yaw - imu.angle[2]),
        ];

        // ---- Rate PID (inner loop) ----
        let pid_output = rate_pid.update(rate_setpoint, imu.gyro, dt);

        // ---- Build control demand ----
        let demand = ControlDemand {
            thrust: hover_throttle,
            roll: pid_output[0],
            pitch: pid_output[1],
            yaw: pid_output[2],
        };

        // ---- Mixer ----
        let motor_out = QUAD_X.apply(&demand);

        // ---- Step physics ----
        sim.step(
            &MotorForces {
                motors: motor_out.motors,
            },
            dt,
        );

        // ---- Print state every 50ms (every 10 steps) ----
        if step % 10 == 0 {
            let alt = -sim.state.z; // NED to altitude
            println!(
                "{:6.2} {:>8.2} {:>8.2} {:>8.2} {:>8.2} {:>8.2} {:>8.1}% {:>10}",
                t,
                sim.state.roll,
                sim.state.pitch,
                sim.state.yaw,
                alt,
                sim.state.vz,
                hover_throttle * 100.0,
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
