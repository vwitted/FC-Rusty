// examples/sim_sweep.rs — parameter sweep over sensor degradation.
//
// Run with:
//   cargo run --release --example sim_sweep --no-default-features
//   cargo run --release --example sim_sweep --no-default-features -- --csv
//
// Answers "where does the control stack fall over, and how gracefully".
// Each case is a full 10 s cascaded flight (Alt hold + MPC 50 Hz, rate PID
// 200 Hz, mixer, physics) with one degradation knob turned up, repeated over
// several seeds. Thousands of cases run in seconds because nothing here is
// emulated -- this is the host sim, not Renode.
//
// Read the tables as: the first row of each block is the idealised case and
// must be identical to an undegraded run (Degradation::none() is a verified
// passthrough). Everything after is measured against it.
//
// What the metrics mean:
//   att_rms   RMS of sqrt(roll^2 + pitch^2), degrees. Tracking quality.
//   att_max   worst instantaneous attitude error. Peak excursion.
//   alt_rms   RMS altitude error, metres.
//   air%      fraction of steps where the mixer had no headroom, i.e. a
//             motor sits on a rail after airmode. QUAD_X.apply uses airmode:
//             when the mix would clip it shifts the whole set to preserve the
//             roll/pitch/yaw differentials AT THE COST OF COMMANDED THRUST.
//             So this is the authority-exhaustion signal, and it explains the
//             most striking result below -- attitude tracking stays tight
//             while altitude falls apart, because altitude is the axis the
//             mixer is deliberately sacrificing.
//   fail      diverged: attitude > 90 deg, crashed (alt <= 0), flew away
//             (alt > 50 m), or went non-finite. Reported with the time.

use fc_rusty::control::altitude::{AltitudeController, AltitudeGains};
use fc_rusty::control::mixer::{ControlDemand, QUAD_X};
use fc_rusty::control::mpc::AttitudeMpc;
use fc_rusty::control::pid::{PidGains, PidLimits, RatePidController};
use fc_rusty::sim::degrade::{ChannelFault, Degradation, Degrader};
use fc_rusty::sim::{MotorForces, QuadParams, QuadSim};

use core::f32::consts::PI;

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

const DT: f32 = 0.005; // 200 Hz inner loop
const OUTER_DIV: usize = 4; // 50 Hz outer
const TOTAL_S: f32 = 10.0;
const TARGET_ALT: f32 = 5.0;

#[derive(Debug, Clone, Copy)]
struct Metrics {
    att_rms: f32,
    att_max: f32,
    alt_rms: f32,
    air_frac: f32,
    failed_at: Option<f32>,
}

impl Metrics {
    fn failed(t: f32) -> Self {
        Self { att_rms: f32::NAN, att_max: f32::NAN, alt_rms: f32::NAN,
               air_frac: f32::NAN, failed_at: Some(t) }
    }
}

/// One flight. Same cascade as sim_mpc_hover, with degradation applied
/// between truth and the controllers, and the same two disturbances so
/// results are comparable to that example.
fn run_case(cfg: Degradation, seed: u64) -> Metrics {
    let params = QuadParams::default();
    let hover_throttle = (params.mass * 9.81) / params.max_thrust;
    let mut sim = QuadSim::new_hovering(params, TARGET_ALT);
    let mut deg = Degrader::new(cfg, seed);

    let mut alt_ctrl = AltitudeController::new(
        AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 },
        hover_throttle,
    );
    let mut current_thrust = hover_throttle;

    let mut mpc = AttitudeMpc::new();
    mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

    let rate_gains = PidGains { kp: 0.02, ki: 0.005, kd: 0.001 };
    let yaw_gains = PidGains { kp: 0.03, ki: 0.005, kd: 0.0 };
    let limits = PidLimits { integral_max: 0.3, output_max: 0.5, d_lpf_tau_s: 0.008 };
    let mut rate_pid = RatePidController::new(rate_gains, rate_gains, yaw_gains, limits);

    let steps = (TOTAL_S / DT) as usize;
    let mut rate_sp_degs = [0.0f32; 3];

    let mut att_sq = 0.0f64;
    let mut alt_sq = 0.0f64;
    let mut att_max = 0.0f32;
    let mut air_steps = 0usize;

    for step in 0..steps {
        let t = step as f32 * DT;

        // Same disturbances as sim_mpc_hover, for comparability.
        if step == (2.0 / DT) as usize {
            sim.state.roll_rate += 10.0;
        } else if step == (5.0 / DT) as usize {
            sim.state.vz += 2.0;
        }

        let truth = sim.read_imu();
        let (gyro, angle_err) = deg.imu(truth.gyro, truth.accel, DT);

        // The accel channel stands in for attitude-ESTIMATE error: this
        // harness feeds truth angles (there is no estimator in the loop), so
        // accel noise has nothing physical to propagate through. Treating it
        // as angle error is the honest interpretation of that knob here.
        let angle = [
            truth.angle[0] + (angle_err[0] - truth.accel[0]),
            truth.angle[1] + (angle_err[1] - truth.accel[1]),
            truth.angle[2] + (angle_err[2] - truth.accel[2]),
        ];

        if step % OUTER_DIV == 0 {
            let alt = -sim.state.z;
            let vz_up = -sim.state.vz;
            current_thrust = alt_ctrl.update(TARGET_ALT, alt, vz_up, DT * OUTER_DIV as f32);

            let angles_rad = [angle[0] * DEG2RAD, angle[1] * DEG2RAD, angle[2] * DEG2RAD];
            let rates_rad = [gyro[0] * DEG2RAD, gyro[1] * DEG2RAD, gyro[2] * DEG2RAD];
            let out = mpc.solve(angles_rad, rates_rad);
            rate_sp_degs = [
                out.rate_setpoints_rads[0] * RAD2DEG,
                out.rate_setpoints_rads[1] * RAD2DEG,
                out.rate_setpoints_rads[2] * RAD2DEG,
            ];
        }

        let pid_output = rate_pid.update(rate_sp_degs, gyro, DT);
        let demand = ControlDemand {
            thrust: current_thrust,
            roll: pid_output[0],
            pitch: pid_output[1],
            yaw: pid_output[2],
        };
        let mixed = QUAD_X.apply(&demand);
        let motors = deg.motors(mixed.motors);

        // Post-airmode: a motor on a rail means the mixer had nothing left
        // to give, and thrust has already been traded away to hold attitude.
        if motors.iter().any(|&m| m <= 0.001 || m >= 0.999) {
            air_steps += 1;
        }

        sim.step(&MotorForces { motors }, DT);

        let roll = sim.state.roll;
        let pitch = sim.state.pitch;
        let alt = -sim.state.z;

        if !roll.is_finite() || !pitch.is_finite() || !alt.is_finite()
            || roll.abs() > 90.0 || pitch.abs() > 90.0 || alt <= 0.0 || alt > 50.0
        {
            return Metrics::failed(t);
        }

        let att = (roll * roll + pitch * pitch).sqrt();
        att_max = att_max.max(att);
        att_sq += (att * att) as f64;
        let ae = alt - TARGET_ALT;
        alt_sq += (ae * ae) as f64;
    }

    let n = steps as f64;
    Metrics {
        att_rms: (att_sq / n).sqrt() as f32,
        att_max,
        alt_rms: (alt_sq / n).sqrt() as f32,
        air_frac: air_steps as f32 / steps as f32,
        failed_at: None,
    }
}

/// Aggregate over seeds. Worst case is reported alongside the mean because a
/// stack that survives on average and diverges one run in twenty is not one
/// you want to fly.
#[derive(Debug, Clone, Copy)]
struct Agg {
    att_rms: f32,
    att_max: f32,
    alt_rms: f32,
    air_frac: f32,
    failures: usize,
    n: usize,
    first_fail_t: Option<f32>,
}

fn aggregate(cfg: Degradation, seeds: u64) -> Agg {
    let mut a = Agg { att_rms: 0.0, att_max: 0.0, alt_rms: 0.0, air_frac: 0.0,
                      failures: 0, n: 0, first_fail_t: None };
    let mut ok = 0usize;
    for s in 0..seeds {
        let m = run_case(cfg, s * 7919 + 1);
        if let Some(t) = m.failed_at {
            a.failures += 1;
            a.first_fail_t = Some(a.first_fail_t.map_or(t, |p: f32| p.min(t)));
            continue;
        }
        a.att_rms += m.att_rms;
        a.alt_rms += m.alt_rms;
        a.air_frac += m.air_frac;
        a.att_max = a.att_max.max(m.att_max);
        ok += 1;
    }
    a.n = seeds as usize;
    if ok > 0 {
        a.att_rms /= ok as f32;
        a.alt_rms /= ok as f32;
        a.air_frac /= ok as f32;
    }
    a
}

fn header(title: &str, knob: &str) {
    println!();
    println!("== {}", title);
    println!("{:>10} {:>9} {:>9} {:>9} {:>7} {:>10}",
             knob, "att_rms", "att_max", "alt_rms", "air%", "fail");
    println!("{}", "-".repeat(60));
}

fn row(label: String, a: Agg) {
    let fail = if a.failures == 0 {
        "-".to_string()
    } else {
        format!("{}/{} @{:.1}s", a.failures, a.n,
                a.first_fail_t.unwrap_or(f32::NAN))
    };
    if a.failures == a.n {
        println!("{:>10} {:>9} {:>9} {:>9} {:>7} {:>10}", label, "-", "-", "-", "-", fail);
    } else {
        println!("{:>10} {:>9.3} {:>9.3} {:>9.3} {:>6.1}% {:>10}",
                 label, a.att_rms, a.att_max, a.alt_rms, a.air_frac * 100.0, fail);
    }
}

fn csv_row(axis: &str, value: f32, a: Agg) {
    println!("{},{},{:.4},{:.4},{:.4},{:.4},{},{}",
             axis, value, a.att_rms, a.att_max, a.alt_rms, a.air_frac,
             a.failures, a.n);
}

fn main() {
    let csv = std::env::args().any(|a| a == "--csv");
    let seeds: u64 = 8;

    let gyro_sigmas = [0.0f32, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0];
    let vib_freqs = [10.0f32, 50.0, 100.0, 200.0, 400.0];
    let vib_amps = [0.0f32, 2.0, 5.0, 10.0, 20.0];
    let p_onlines = [1.0f32, 0.99, 0.95, 0.9, 0.8, 0.6, 0.4];
    let motor_scales = [1.0f32, 0.95, 0.9, 0.8, 0.7, 0.5];
    let biases = [0.0f32, 0.5, 1.0, 2.0, 5.0, 10.0];

    if csv {
        println!("axis,value,att_rms,att_max,alt_rms,air_frac,failures,seeds");
    } else {
        println!("=== control degradation sweep ===");
        println!("{} s flights, {} seeds per case, 200 Hz inner / 50 Hz outer",
                 TOTAL_S, seeds);
        println!("baseline row of each block is the idealised (undegraded) case");
    }

    // --- gyro white noise ---
    if !csv { header("gyro white noise", "sigma"); }
    for &s in &gyro_sigmas {
        let cfg = Degradation {
            gyro: ChannelFault { sigma: s, ..ChannelFault::none() },
            ..Degradation::none()
        };
        let a = aggregate(cfg, seeds);
        if csv { csv_row("gyro_sigma_dps", s, a) } else { row(format!("{:.2}", s), a) }
    }

    // --- gyro bias: does not average out, so the estimator eats it ---
    if !csv { header("gyro bias (roll axis)", "bias"); }
    for &b in &biases {
        let cfg = Degradation {
            gyro: ChannelFault { bias: [b, 0.0, 0.0], ..ChannelFault::none() },
            ..Degradation::none()
        };
        let a = aggregate(cfg, seeds);
        if csv { csv_row("gyro_bias_dps", b, a) } else { row(format!("{:.2}", b), a) }
    }

    // --- vibration: the loose-motor / prop-imbalance case ---
    for &f in &vib_freqs {
        if !csv { header(&format!("gyro vibration @ {:.0} Hz", f), "amplitude"); }
        for &amp in &vib_amps {
            let cfg = Degradation {
                gyro: ChannelFault { vib_amplitude: amp, vib_hz: f, ..ChannelFault::none() },
                ..Degradation::none()
            };
            let a = aggregate(cfg, seeds);
            if csv {
                csv_row(&format!("vib_{:.0}hz_amp_dps", f), amp, a)
            } else {
                row(format!("{:.1}", amp), a)
            }
        }
    }

    // --- intermittent gyro: stale samples held, not gaps ---
    if !csv { header("gyro intermittency (p online)", "p"); }
    for &p in &p_onlines {
        let cfg = Degradation {
            gyro: ChannelFault { p_online: p, dropout_dwell_s: 0.02, ..ChannelFault::none() },
            ..Degradation::none()
        };
        let a = aggregate(cfg, seeds);
        if csv { csv_row("gyro_p_online", p, a) } else { row(format!("{:.2}", p), a) }
    }

    // --- asymmetric airframe: one motor down ---
    if !csv { header("motor 3 thrust scale", "scale"); }
    for &m in &motor_scales {
        let cfg = Degradation { motor_scale: [1.0, 1.0, m, 1.0], ..Degradation::none() };
        let a = aggregate(cfg, seeds);
        if csv { csv_row("motor3_scale", m, a) } else { row(format!("{:.2}", m), a) }
    }

    if !csv {
        println!();
        println!("note: att_rms stays small while alt_rms explodes because");
        println!("QUAD_X airmode protects attitude by sacrificing thrust. Read");
        println!("air% as authority exhaustion: once it is near 100% the stack");
        println!("is holding attitude by giving up altitude, and the next");
        println!("disturbance is the one that hurts.");
    }
}
