// examples/sim_gps_rescue.rs — GPS rescue / return-to-home simulation
//
// Run with:
//   cargo run --example sim_gps_rescue --no-default-features \
//             --target x86_64-unknown-linux-gnu
//
// Scenario: the quad is 20 m north and 10 m east of "home" at 5 m
// altitude. GPS rescue activates and the full cascade flies it back:
//
//   PosKf (6-state) ← GPS (10 Hz, noisy) + baro (50 Hz, noisy+drift)
//         │
//         ▼
//   Position PD (5 Hz) → desired roll/pitch
//         │
//         ▼
//   Attitude MPC (50 Hz) → rate setpoints
//         │
//         ▼
//   Rate PID (200 Hz) → torque demands → mixer → motors → physics
//
// The position controller only runs at 5 Hz (every 40th inner-loop
// step) to reflect the GPS fix rate — no point computing new tilt
// targets faster than the sensor that feeds them.
//
// Success criteria: the quad should arrive within ~3 m of home
// (limited by GPS noise floor) and loiter there with stable attitude.

use fc_rusty::control::altitude::{AltitudeController, AltitudeGains};
use fc_rusty::control::mixer::{ControlDemand, QUAD_X};
use fc_rusty::control::mpc::AttitudeMpc;
use fc_rusty::control::pid::{PidGains, PidLimits, RatePidController};
use fc_rusty::control::position::{PositionController, PositionGains};
use fc_rusty::estimation::PosKf;
use fc_rusty::sim::sensors::{BaroSim, GpsSim};
use fc_rusty::sim::{MotorForces, QuadParams, QuadState, QuadSim};

use core::f32::consts::PI;
use nalgebra::{Rotation3, Vector3};

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

/// Wrap angle to [-π, π].
fn wrap_angle(a: f32) -> f32 {
    let two_pi = 2.0 * PI;
    let mut a = a % two_pi;
    if a > PI { a -= two_pi; }
    if a < -PI { a += two_pi; }
    a
}

fn main() {
    let params = QuadParams::default();
    let hover_throttle = (params.mass * 9.81) / params.max_thrust;

    // ---- Start 20 m north, 10 m east of home, at 5 m altitude --------
    let start_x = 20.0;  // north
    let start_y = 10.0;  // east
    let start_alt = 5.0;
    let home = [0.0f32, 0.0]; // [north, east]

    let initial_state = QuadState {
        x: start_x,
        y: start_y,
        z: -start_alt, // NED
        ..QuadState::default()
    };

    println!("=== GPS Rescue Simulation ===");
    println!("Start: ({:.0}, {:.0}) m,  altitude {:.0} m", start_x, start_y, start_alt);
    println!("Home:  ({:.0}, {:.0}) m,  hold altitude {:.0} m", home[0], home[1], start_alt);
    println!("Distance: {:.1} m", libm::sqrtf(start_x * start_x + start_y * start_y));
    println!();
    println!("Stack: PosKf → PosPD(5Hz) → MPC(50Hz) → PID(200Hz) → mixer → physics");
    println!("Sensors: GPS 10Hz (σ_h=2m, σ_v=5m), baro 50Hz (σ=0.3m, drift τ=60s)");
    println!();

    // ---- Physics sim --------------------------------------------------
    // Pre-set motor state to hover throttle so it doesn't drop on frame 1.
    let mut sim = QuadSim {
        motor_state: [hover_throttle; 4],
        state: initial_state,
        params,
        last_accel_world: [0.0; 3],
    };

    // ---- Sensor simulators --------------------------------------------
    let mut gps = GpsSim::new(10.0, 2.0, 5.0, 0xBEEF_CAFE);
    let mut baro = BaroSim::new(50.0, 0.3, 0.5, 60.0, 0xDEAD_C0DE);

    // ---- Kalman filter ------------------------------------------------
    let mut kf = PosKf::new_at(
        [start_x, start_y, -start_alt],
        0.5,   // σ_a
        2.0,   // σ_gps_h
        5.0,   // σ_gps_v
        0.3,   // σ_baro
    );

    // ---- Position controller (5 Hz) -----------------------------------
    // Gentle gains to limit approach speed and keep the mixer in its
    // linear range. With max_tilt = 10° the horizontal accel is about
    // g·tan(10°) ≈ 1.7 m/s². kd is chosen for near-critical damping
    // so the quad doesn't overshoot home and oscillate.
    let pos_gains = PositionGains {
        kp: 0.5,
        kd: 1.0,
        max_tilt_rad: 10.0 * DEG2RAD,
    };
    let pos_ctrl = PositionController::new(pos_gains);
    let mut desired_roll_rad = 0.0f32;
    let mut desired_pitch_rad = 0.0f32;

    // ---- Altitude controller (50 Hz) -----------------------------------
    let alt_gains = AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let target_alt = start_alt;
    let mut current_thrust = hover_throttle;

    // ---- MPC outer loop (50 Hz) ----------------------------------------
    let mut mpc = AttitudeMpc::new();
    // Initial reference will be set by the position controller.
    mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

    // ---- PID inner loop (200 Hz) --------------------------------------
    let rate_gains = PidGains { kp: 0.02, ki: 0.005, kd: 0.001 };
    let yaw_gains = PidGains { kp: 0.03, ki: 0.005, kd: 0.0 };
    let limits = PidLimits {
        integral_max: 0.3,
        output_max: 0.5,
        d_lpf_tau_s: 0.008,
    };
    let mut rate_pid = RatePidController::new(rate_gains, rate_gains, yaw_gains, limits);

    // ---- Timing -------------------------------------------------------
    let dt = 0.005;       // 200 Hz inner loop
    let total_time = 30.0; // 30 seconds — enough to fly 22 m at ~15° tilt
    let steps = (total_time / dt) as usize;
    let mpc_divider = 4;  // 50 Hz
    let pos_divider = 40; // 5 Hz (position controller)

    let mut rate_sp_degs = [0.0f32; 3];
    let mut last_mpc_iters: usize = 0;
    let mut last_mpc_converged = true;

    println!(
        "{:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>6} {:>6} {:>6}  {:>5}",
        "time", "dist_t", "dist_kf", "alt_t", "alt_kf", "roll", "pitch", "vn", "ve", "thr%", "event"
    );
    println!("{}", "-".repeat(100));

    for step in 0..steps {
        let t = step as f32 * dt;

        // ---- Event schedule -------------------------------------------
        let event = if step == 0 {
            "START"
        } else {
            ""
        };

        // ---- Sensors: IMU ---------------------------------------------
        let imu = sim.read_imu();

        // Body-frame SF → world-frame kinematic accel
        let roll_rad = imu.angle[0] * DEG2RAD;
        let pitch_rad = imu.angle[1] * DEG2RAD;
        let yaw_rad = imu.angle[2] * DEG2RAD;
        let rot = Rotation3::from_euler_angles(roll_rad, pitch_rad, yaw_rad);
        let sf_body = Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
        let sf_world = rot * sf_body;
        let a_world = [sf_world.x, sf_world.y, sf_world.z + 9.81];

        // ---- KF predict (200 Hz) -------------------------------------
        kf.predict(a_world, dt);

        // ---- GPS / baro updates (sensor-driven) ----------------------
        let truth_ned = [sim.state.x, sim.state.y, sim.state.z];
        if let Some(fix) = gps.tick(dt, truth_ned) {
            kf.update_gps(fix);
        }
        let truth_alt_up = -sim.state.z;
        if let Some(sample) = baro.tick(dt, truth_alt_up) {
            kf.update_baro(sample);
        }

        // ---- Position controller (5 Hz) ------------------------------
        if step % pos_divider == 0 {
            let kf_state = kf.state();
            let pos_out = pos_ctrl.update(
                [kf_state[0], kf_state[1]], // estimated position
                [kf_state[3], kf_state[4]], // estimated velocity
                home,
                yaw_rad,
            );
            desired_roll_rad = pos_out.roll_rad;
            desired_pitch_rad = pos_out.pitch_rad;
        }

        // ---- 50 Hz outer loops: altitude + MPC -----------------------
        if step % mpc_divider == 0 {
            // Altitude hold from KF estimate
            let alt_est = kf.altitude_up();
            let vz_up_est = kf.vz_up();
            current_thrust =
                alt_ctrl.update(target_alt, alt_est, vz_up_est, dt * mpc_divider as f32);

            // Set MPC reference from position controller output.
            // Yaw reference tracks current heading — GPS rescue doesn't
            // need heading hold, just no spinning. Wrapping avoids the
            // MPC fighting accumulated yaw error.
            let yaw_wrapped = wrap_angle(yaw_rad);
            mpc.set_reference(
                [desired_roll_rad, desired_pitch_rad, yaw_wrapped],
                [0.0, 0.0, 0.0],
            );

            // MPC solve — feed wrapped yaw so it stays within the
            // MPC's [-π, π] state constraints.
            let angles_rad = [
                imu.angle[0] * DEG2RAD,
                imu.angle[1] * DEG2RAD,
                yaw_wrapped,
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

        // ---- PID inner loop (200 Hz) ---------------------------------
        let pid_output = rate_pid.update(rate_sp_degs, imu.gyro, dt);

        // ---- Mixer → physics -----------------------------------------
        // Tilt-compensated thrust: when tilted, only the vertical
        // component of thrust supports the weight, so we divide by
        // cos(total_tilt) to maintain altitude. Clamped to 1.0.
        let total_tilt_rad = libm::sqrtf(
            desired_roll_rad * desired_roll_rad + desired_pitch_rad * desired_pitch_rad,
        );
        let tilt_comp = if total_tilt_rad > 0.01 {
            1.0 / libm::cosf(total_tilt_rad)
        } else {
            1.0
        };
        // No thrust floor — the altitude controller manages throttle
        // freely (its own min is 0.1). PID output scaling below
        // prevents motor clipping, so phantom thrust can't occur.
        let thrust_final = (current_thrust * tilt_comp).clamp(0.05, 0.95);

        // ---- Scale PID outputs to fit within the thrust budget ----
        // For QUAD_X, the worst-case motor is:
        //   M_min = thrust - |roll| - |pitch| - |yaw|
        // If this goes negative, motor clipping creates phantom thrust
        // and breaks the yaw balance. Scale all PID outputs uniformly
        // so they always fit, preserving the axis ratios.
        let pid_sum = pid_output[0].abs() + pid_output[1].abs() + pid_output[2].abs();
        let headroom = thrust_final * 0.95; // 5% margin for numerical safety
        let scale = if pid_sum > headroom && pid_sum > 1e-6 {
            headroom / pid_sum
        } else {
            1.0
        };

        let demand = ControlDemand {
            thrust: thrust_final,
            roll: pid_output[0] * scale,
            pitch: pid_output[1] * scale,
            yaw: pid_output[2] * scale,
        };
        let motor_out = QUAD_X.apply_no_airmode(&demand);
        sim.step(
            &MotorForces {
                motors: motor_out.motors,
            },
            dt,
        );

        // ---- Telemetry every 100 ms ----------------------------------
        if step % 20 == 0 {
            let dist_truth = libm::sqrtf(sim.state.x * sim.state.x + sim.state.y * sim.state.y);
            let kf_s = kf.state();
            let dist_kf = libm::sqrtf(kf_s[0] * kf_s[0] + kf_s[1] * kf_s[1]);
            let _conv = if last_mpc_converged { ' ' } else { '!' };
            let _ = last_mpc_iters;
            println!(
                "{:6.2} {:>7.2} {:>7.2} {:>7.2} {:>7.2} {:>+7.2} {:>+7.2} {:>+6.2} {:>+6.2} {:>5.1}%  {:>5}",
                t,
                dist_truth,
                dist_kf,
                -sim.state.z,
                kf.altitude_up(),
                sim.state.roll,
                sim.state.pitch,
                sim.state.vx,
                sim.state.vy,
                current_thrust * 100.0,
                event,
            );
        }
    }

    // ---- Final report -------------------------------------------------
    let final_dist = libm::sqrtf(sim.state.x * sim.state.x + sim.state.y * sim.state.y);
    let kf_s = kf.state();
    let final_dist_kf = libm::sqrtf(kf_s[0] * kf_s[0] + kf_s[1] * kf_s[1]);

    println!();
    println!("=== Final state (t = {:.0}s) ===", total_time);
    println!(
        "Truth   pos  = ({:>+7.2}, {:>+7.2}) m,  alt = {:.2} m,  dist-to-home = {:.2} m",
        sim.state.x, sim.state.y, -sim.state.z, final_dist
    );
    println!(
        "KF est  pos  = ({:>+7.2}, {:>+7.2}) m,  alt = {:.2} m,  dist-to-home = {:.2} m",
        kf_s[0], kf_s[1], kf.altitude_up(), final_dist_kf
    );
    println!(
        "Truth   vel  = ({:>+6.3}, {:>+6.3}, {:>+6.3}) m/s",
        sim.state.vx, sim.state.vy, sim.state.vz
    );
    println!(
        "Attitude     = roll {:>+.2}°  pitch {:>+.2}°  yaw {:>+.2}°",
        sim.state.roll, sim.state.pitch, sim.state.yaw
    );
    println!(
        "Pos ref      = roll {:>+.2}°  pitch {:>+.2}°  (tilt command from pos ctrl)",
        desired_roll_rad * RAD2DEG, desired_pitch_rad * RAD2DEG
    );
    println!(
        "KF diag(P)   = [{:.3}, {:.3}, {:.3} | {:.3}, {:.3}, {:.3}]",
        kf.p_cov[(0, 0)], kf.p_cov[(1, 1)], kf.p_cov[(2, 2)],
        kf.p_cov[(3, 3)], kf.p_cov[(4, 4)], kf.p_cov[(5, 5)],
    );

    if final_dist < 5.0 {
        println!();
        println!("GPS rescue SUCCESS — within {:.1} m of home.", final_dist);
    } else {
        println!();
        println!("GPS rescue INCOMPLETE — {:.1} m from home (may need more time or tuning).", final_dist);
    }
}
