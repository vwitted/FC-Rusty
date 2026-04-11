// examples/sim_mpc_hover.rs — MPC + PID + Alt Hold cascaded hover simulation
//
// Run with: cargo run --example sim_mpc_hover --no-default-features
//
// Demonstrates the full cascaded control stack:
//   - Altitude controller (50 Hz) → thrust
//   - MPC attitude outer loop (50 Hz) → angular rate setpoints
//   - PID rate inner loop (200 Hz) → torque demands
//   - Mixer → motor commands → physics
//
// Same disturbance scenario as sim_hover for direct comparison.

use fc_rusty::control::altitude::{AltitudeController, AltitudeGains};
use fc_rusty::control::mpc::AttitudeMpc;
use fc_rusty::control::mixer::{ControlDemand, QUAD_X};
use fc_rusty::control::pid::{PidGains, PidLimits, RatePidController};
use fc_rusty::sim::{MotorForces, QuadParams, QuadSim};

use core::f32::consts::PI;

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

fn main() {
    let params = QuadParams::default();
    let hover_throttle = (params.mass * 9.81) / params.max_thrust;

    println!("=== Quadcopter Hover Simulation (MPC + PID + Alt Hold) ===");
    println!("Mass: {}kg, Max thrust: {}N", params.mass, params.max_thrust);
    println!("Hover throttle: {:.1}%", hover_throttle * 100.0);
    println!("MPC: 50 Hz (6-state attitude model, horizon=10)");
    println!("Alt Hold: 50 Hz (PD + integral)");
    println!("PID: 200 Hz (rate tracking inner loop)");
    println!();

    // ---- Physics sim (motor state pre-initialized to hover) ----
    let mut sim = QuadSim::new_hovering(params, 5.0);

    // ---- Altitude controller (50 Hz) ----
    let alt_gains = AltitudeGains {
        kp: 0.15,
        kd: 0.1,
        ki: 0.05,
    };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let target_alt = 5.0;
    let mut current_thrust = hover_throttle;

    // ---- MPC outer loop (50 Hz) ----
    let mut mpc = AttitudeMpc::new();
    mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

    // ---- PID inner loop (200 Hz) ----
    let rate_gains = PidGains {
        kp: 0.02,
        ki: 0.005,
        kd: 0.0,
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

    let dt = 0.005; // 200 Hz inner loop
    let total_time = 10.0;
    let steps = (total_time / dt) as usize;
    let mpc_divider = 4; // 50 Hz for both MPC and altitude

    // Rate setpoints from MPC (persisted between MPC solves)
    let mut rate_sp_degs = [0.0f32; 3];
    let mut last_mpc_iters: usize = 0;
    let mut last_mpc_converged = true;

    println!(
        "{:>6} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}  {:>4} {:>10}",
        "time", "roll", "pitch", "yaw", "alt", "vz", "thr%", "p_cmd", "iter", "event"
    );
    println!("{}", "-".repeat(92));

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

        // ---- 50 Hz outer loops: altitude + MPC ----
        if step % mpc_divider == 0 {
            // Altitude hold
            let alt = -sim.state.z;
            let vz_up = -sim.state.vz;
            current_thrust = alt_ctrl.update(target_alt, alt, vz_up, dt * mpc_divider as f32);

            // MPC attitude
            let angles_rad = [
                imu.angle[0] * DEG2RAD,
                imu.angle[1] * DEG2RAD,
                imu.angle[2] * DEG2RAD,
            ];
            let rates_rad = [
                imu.gyro[0] * DEG2RAD,
                imu.gyro[1] * DEG2RAD,
                imu.gyro[2] * DEG2RAD,
            ];

            let mpc_out = mpc.solve(angles_rad, rates_rad);

            rate_sp_degs = [
                mpc_out.rate_setpoints_rads[0] * RAD2DEG,
                mpc_out.rate_setpoints_rads[1] * RAD2DEG,
                mpc_out.rate_setpoints_rads[2] * RAD2DEG,
            ];
            last_mpc_iters = mpc_out.iterations;
            last_mpc_converged = mpc_out.converged;
        }

        // ---- PID inner loop (200 Hz) ----
        let pid_output = rate_pid.update(rate_sp_degs, imu.gyro, dt);

        // ---- Mixer → physics ----
        let demand = ControlDemand {
            thrust: current_thrust,
            roll: pid_output[0],
            pitch: pid_output[1],
            yaw: pid_output[2],
        };
        let motor_out = QUAD_X.apply(&demand);
        sim.step(
            &MotorForces {
                motors: motor_out.motors,
            },
            dt,
        );

        // ---- Print state every 50ms ----
        if step % 10 == 0 {
            let alt = -sim.state.z;
            let conv_marker = if last_mpc_converged { " " } else { "!" };
            // Mean of clamped motor outputs — this is what the airframe
            // actually feels. If it diverges from `current_thrust`, it's
            // evidence of asymmetric mixer clipping adding phantom thrust.
            let mean_motor =
                (motor_out.motors[0] + motor_out.motors[1] + motor_out.motors[2] + motor_out.motors[3]) / 4.0;
            println!(
                "{:6.2} {:>8.2} {:>8.2} {:>8.2} {:>8.2} {:>8.2} {:>7.1}% {:>7.1}°  {:>3}{} m=[{:.2} {:.2} {:.2} {:.2}] mean={:.2} {:>10}",
                t,
                sim.state.roll,
                sim.state.pitch,
                sim.state.yaw,
                alt,
                sim.state.vz,
                current_thrust * 100.0,
                rate_sp_degs[0],
                last_mpc_iters,
                conv_marker,
                motor_out.motors[0],
                motor_out.motors[1],
                motor_out.motors[2],
                motor_out.motors[3],
                mean_motor,
                event,
            );
        }
    }

    println!();
    println!("=== Final state ===");
    println!(
        "Position: ({:.2}, {:.2}, {:.2}m altitude)",
        sim.state.x, sim.state.y, -sim.state.z
    );
    println!(
        "Velocity: ({:.2}, {:.2}, {:.2}) m/s",
        sim.state.vx, sim.state.vy, sim.state.vz
    );
    println!(
        "Attitude: roll={:.2}° pitch={:.2}° yaw={:.2}°",
        sim.state.roll, sim.state.pitch, sim.state.yaw
    );
    println!(
        "Rates: ({:.2}, {:.2}, {:.2}) °/s",
        sim.state.roll_rate, sim.state.pitch_rate, sim.state.yaw_rate
    );
}
