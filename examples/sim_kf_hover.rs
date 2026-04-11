// examples/sim_kf_hover.rs — Full cascaded stack with noisy sensors + KF
//
// Run with:
//   cargo run --example sim_kf_hover --no-default-features \
//             --target x86_64-unknown-linux-gnu
//
// This mirrors sim_mpc_hover but closes the altitude loop through a
// linear 6-state position/velocity Kalman filter fed by a noisy GPS
// (10 Hz, σ_h = 2 m, σ_v = 5 m) and a noisy barometer (50 Hz, σ = 0.3 m,
// plus slow OU drift). The flow matches what we expect on real hardware:
//
//   WT901B onboard EKF → attitude (quaternion / Euler) + body-frame accel
//          │
//          ▼
//   Rotate body-frame specific force by attitude → world-frame specific
//   force. Add gravity to get kinematic world acceleration.
//          │
//          ▼
//   PosKf::predict(a_world, dt)   every inner-loop tick (200 Hz)
//   PosKf::update_gps(fix_ned)    whenever a GPS fix arrives  (10 Hz)
//   PosKf::update_baro(alt_up)    whenever a baro sample arrives (50 Hz)
//          │
//          ▼
//   Altitude controller + MPC outer loop consume PosKf.state() — *not*
//   sim.state — so the outer loop is actually flying on sensor estimates.
//
// The rate PID inner loop still uses the (very clean) gyro, matching
// the real flight controller where rates come straight from the IMU.
//
// Same disturbance schedule as sim_mpc_hover (GUST @ t=2 s, DROP @ t=5 s)
// for direct comparison.

use fc_rusty::control::altitude::{AltitudeController, AltitudeGains};
use fc_rusty::control::mixer::{ControlDemand, QUAD_X};
use fc_rusty::control::mpc::AttitudeMpc;
use fc_rusty::control::pid::{PidGains, PidLimits, RatePidController};
use fc_rusty::estimation::PosKf;
use fc_rusty::sim::sensors::{BaroSim, GpsSim};
use fc_rusty::sim::{MotorForces, QuadParams, QuadSim};

use core::f32::consts::PI;
use nalgebra::{Rotation3, Vector3};

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

fn main() {
    let params = QuadParams::default();
    let hover_throttle = (params.mass * 9.81) / params.max_thrust;

    println!("=== Hover sim: MPC + PID + altitude + noisy sensors + KF ===");
    println!("Mass: {}kg   Max thrust: {}N", params.mass, params.max_thrust);
    println!("Hover throttle: {:.1}%", hover_throttle * 100.0);
    println!("Flow: body-sf → rotate(Euler) → +g → KF.predict → KF.update_{{gps,baro}}");
    println!("Altitude loop closes on KF estimate, not truth.");
    println!();

    // ---- Physics sim (ground truth) ------------------------------------
    let mut sim = QuadSim::new_hovering(params, 5.0);

    // ---- Sensor simulators --------------------------------------------
    // GPS: 10 Hz, ~2 m horizontal, ~5 m vertical
    let mut gps = GpsSim::new(10.0, 2.0, 5.0, 0xC0FFEE);
    // Baro: 50 Hz, 0.3 m white noise, 0.5 m drift with τ = 60 s
    let mut baro = BaroSim::new(50.0, 0.3, 0.5, 60.0, 0xFEEDFACE);

    // ---- Kalman filter ------------------------------------------------
    // σ_a = 0.5 m/s² — loose enough to handle gust transients without
    // treating GPS noise as ground truth, tight enough that baro pulls
    // altitude in quickly.
    let mut kf = PosKf::new_at(
        [0.0, 0.0, -5.0], // seeded at truth — pilot usually knows takeoff point
        0.5,              // σ_a
        2.0,              // σ_gps_h
        5.0,              // σ_gps_v
        0.3,              // σ_baro
    );

    // ---- Altitude controller (50 Hz) -----------------------------------
    let alt_gains = AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let target_alt = 5.0;
    let mut current_thrust = hover_throttle;

    // ---- MPC outer loop (50 Hz) ----------------------------------------
    let mut mpc = AttitudeMpc::new();
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

    let dt = 0.005;
    let total_time = 10.0;
    let steps = (total_time / dt) as usize;
    let mpc_divider = 4; // 50 Hz

    let mut rate_sp_degs = [0.0f32; 3];
    let mut last_mpc_iters: usize = 0;
    let mut last_mpc_converged = true;

    // Keep last received measurements for logging
    let mut last_gps_alt: f32 = 5.0;
    let mut last_baro_alt: f32 = 5.0;
    let mut gps_fixes = 0usize;
    let mut baro_samples = 0usize;

    println!(
        "{:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>6} {:>7} {:>7} {:>5}",
        "time", "alt_t", "alt_kf", "kf_err", "gps_z", "baro", "vz_kf", "thr%", "roll", "p_sp", "event"
    );
    println!("{}", "-".repeat(100));

    for step in 0..steps {
        let t = step as f32 * dt;

        // ---- Disturbances --------------------------------------------
        let event = if step == (2.0 / dt) as usize {
            sim.state.roll_rate += 10.0;
            "GUST"
        } else if step == (5.0 / dt) as usize {
            sim.state.vz += 2.0;
            "DROP"
        } else {
            ""
        };

        // ---- Sensors: IMU (simulated WT901B output) ------------------
        let imu = sim.read_imu();

        // Rotate body-frame specific force into the world frame using
        // the attitude solution (from the WT901B EKF on real hardware;
        // here we use the truth Euler angles the sim exposes, same
        // Rotation3::from_euler_angles convention used to *encode* the
        // body accel in sim.rs — so this is round-trip exact).
        let roll_rad = imu.angle[0] * DEG2RAD;
        let pitch_rad = imu.angle[1] * DEG2RAD;
        let yaw_rad = imu.angle[2] * DEG2RAD;
        let rot = Rotation3::from_euler_angles(roll_rad, pitch_rad, yaw_rad);

        let sf_body = Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
        let sf_world = rot * sf_body;
        // Kinematic accel = specific force + gravity (NED: +Z down).
        let a_world = [sf_world.x, sf_world.y, sf_world.z + 9.81];

        // ---- KF predict (every inner-loop tick) ----------------------
        kf.predict(a_world, dt);

        // ---- GPS / baro updates (sensor-driven, not controller-timed) --
        // Ground truth in NED for the sensor sims.
        let truth_ned = [sim.state.x, sim.state.y, sim.state.z];
        if let Some(fix) = gps.tick(dt, truth_ned) {
            kf.update_gps(fix);
            last_gps_alt = -fix[2];
            gps_fixes += 1;
        }
        let truth_alt_up = -sim.state.z;
        if let Some(sample) = baro.tick(dt, truth_alt_up) {
            kf.update_baro(sample);
            last_baro_alt = sample;
            baro_samples += 1;
        }

        // ---- 50 Hz outer loops ---------------------------------------
        if step % mpc_divider == 0 {
            // Altitude hold now closes on KF estimate, not truth.
            let alt_est = kf.altitude_up();
            let vz_up_est = kf.vz_up();
            current_thrust =
                alt_ctrl.update(target_alt, alt_est, vz_up_est, dt * mpc_divider as f32);

            // MPC attitude still uses gyro+angles directly (the WT901B
            // EKF handles that part in real life; it's clean here).
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

        // ---- PID inner loop ------------------------------------------
        let pid_output = rate_pid.update(rate_sp_degs, imu.gyro, dt);

        // ---- Mixer → physics -----------------------------------------
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

        // ---- Telemetry every 50 ms -----------------------------------
        if step % 10 == 0 {
            let alt_truth = -sim.state.z;
            let alt_kf = kf.altitude_up();
            let err = alt_kf - alt_truth;
            let conv_marker = if last_mpc_converged { ' ' } else { '!' };
            let _ = (last_mpc_iters, conv_marker); // available if you want them
            println!(
                "{:6.2} {:>7.3} {:>7.3} {:>+7.3} {:>7.3} {:>7.3} {:>7.3} {:>5.1}% {:>7.2} {:>7.1} {:>5}",
                t,
                alt_truth,
                alt_kf,
                err,
                last_gps_alt,
                last_baro_alt,
                kf.vz_up(),
                current_thrust * 100.0,
                sim.state.roll,
                rate_sp_degs[0],
                event,
            );
        }
    }

    println!();
    println!("=== Final state ===");
    println!(
        "Truth   pos (x, y, altitude) = ({:.3}, {:.3}, {:.3}) m",
        sim.state.x, sim.state.y, -sim.state.z
    );
    println!(
        "KF est  pos (x, y, altitude) = ({:.3}, {:.3}, {:.3}) m",
        kf.x[0], kf.x[1], kf.altitude_up()
    );
    let pos_err = libm::sqrtf(
        (kf.x[0] - sim.state.x).powi(2)
            + (kf.x[1] - sim.state.y).powi(2)
            + (kf.x[2] - sim.state.z).powi(2),
    );
    println!("KF position error magnitude: {:.3} m", pos_err);

    println!(
        "Truth   vel (vx, vy, vz_up)  = ({:.3}, {:.3}, {:.3}) m/s",
        sim.state.vx, sim.state.vy, -sim.state.vz
    );
    println!(
        "KF est  vel (vx, vy, vz_up)  = ({:.3}, {:.3}, {:.3}) m/s",
        kf.x[3], kf.x[4], kf.vz_up()
    );
    println!(
        "Attitude: roll={:.2}° pitch={:.2}° yaw={:.2}°",
        sim.state.roll, sim.state.pitch, sim.state.yaw
    );
    println!(
        "Sensor samples used: GPS={}  baro={}",
        gps_fixes, baro_samples
    );
    println!(
        "KF diag(P) = [{:.3}, {:.3}, {:.3} | {:.3}, {:.3}, {:.3}]",
        kf.p_cov[(0, 0)],
        kf.p_cov[(1, 1)],
        kf.p_cov[(2, 2)],
        kf.p_cov[(3, 3)],
        kf.p_cov[(4, 4)],
        kf.p_cov[(5, 5)],
    );
}
