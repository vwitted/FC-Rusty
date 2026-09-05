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

// The cascade itself lives in src/sim/harness.rs so this sweep and the
// tuner in ga_tune.rs drive ONE implementation. Two copies would drift, and
// a tuner optimising a subtly different plant than the sweep reports on is
// worse than no tuner.
use fc_rusty::imu_filter::ImuFilterParams;
use fc_rusty::sim::degrade::{ChannelFault, Degradation};
use fc_rusty::sim::dual_imu::{DualImuConfig, ImuFault};
use fc_rusty::sim::harness::{
    run_case, FailCause, HarnessCfg, Rates, Tunables,
};
use fc_rusty::sim::QuadParams;

const TOTAL_S: f32 = 10.0;

const TARGET_ALT: f32 = 5.0;

/// Gyro noise floor used on the intermittency axis, deg/s RMS.
///
/// Sweeping dropout against ZERO noise measures nothing under --dual: the
/// surviving sensor carries perfect truth, so losing its partner costs
/// exactly zero and the column sits flat at baseline. The real cost of
/// dropping to one sensor is sqrt2 more noise, and that only shows if there
/// is noise to double.
///
/// 1.0 is not the datasheet figure. An ICM-42688P at 0.0028 dps/sqrt(Hz)
/// through the firmware's 150 Hz filter (ENB ~166 Hz) gives ~0.036 dps RMS,
/// two orders below this. In flight the gyro floor is vibration-dominated,
/// not electronic, and 1.0 dps is a realistic build. It also sits where the
/// noise axis shows visible-but-not-dominant degradation (att_rms 0.029 vs
/// 0.009 baseline), which is exactly where a sqrt2 change is legible.
const GYRO_FLOOR_DPS: f32 = 1.0;

/// Plant parameters, overridable for sensitivity checks.
///
/// QuadParams::default() is labelled "reasonable defaults for a 5in racing
/// quad" -- plausible textbook values, not measurements of this airframe.
/// Two of them bear directly on any stability result: motor_tau sets the
/// loop's dominant phase lag (30 ms puts its corner at 5.3 Hz), and
/// max_thrust sets loop gain. motor_tau's own comment gives a RANGE
/// (20-50 ms), not a value, so the honest thing is to sweep it rather than
/// trust it.
fn plant_params() -> QuadParams {
    let mut p = QuadParams::default();
    if let Some(v) = std::env::var("PLANT_TAU").ok().and_then(|v| v.parse().ok()) {
        p.motor_tau = v;
    }
    if let Some(v) = std::env::var("PLANT_THRUST").ok().and_then(|v| v.parse().ok()) {
        p.max_thrust = v;
    }
    if let Some(v) = std::env::var("PLANT_INERTIA").ok().and_then(|v| v.parse().ok()) {
        p.inertia = [v, v, v * 2.0];
    }
    // Trap worth guarding: AltitudeController clamps to min_thrust = 0.1, so
    // once max_thrust is high enough that hover sits below that floor, the
    // controller CANNOT command hover and the aircraft climbs no matter how
    // stable the attitude loop is. At 60 N hover is 0.098 -- every run reads
    // as a flyaway, and it means nothing. Fails loudly rather than quietly
    // producing a table of artefacts.
    let hover = p.mass * 9.81 / p.max_thrust;
    assert!(
        hover > 0.12,
        "max_thrust {} N puts hover throttle at {:.3}, at or under the          AltitudeController's 0.1 floor -- every run would be a false flyaway",
        p.max_thrust, hover
    );
    p
}

/// Spread the disturbances over this many ms instead of stepping the state.
/// 0 keeps the original instantaneous poke.
fn disturb_ms() -> f32 {
    std::env::var("DISTURB_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0)
}

/// Map a single-channel Degradation onto the two-sensor model.
///
/// The mapping is the whole point, so it is explicit rather than uniform:
///
///   sigma     -> BOTH sensors, independently. Electronic noise is per-part,
///                so the fused stream sees sigma/sqrt2.
///   vibration -> COMMON. One airframe resonance, same phase in both parts;
///                averaging cannot touch it.
///   bias      -> ONE sensor. A zero offset belongs to a part, not the
///                frame, so averaging halves it.
///   p_online  -> ONE sensor. Losing an IMU degrades to single-IMU
///                operation; it does not blind the rate loop, which is what
///                the single-channel model wrongly implied.
///
/// CAVEAT on that last one. Because the sweep varies p_online with sigma at
/// zero, the surviving sensor here carries no noise, so a dropout costs
/// exactly nothing and the dual column comes out flat at the baseline. That
/// is real in the sense that matters -- the loop keeps a gyro -- but the
/// benefit is inflated: with both sensors noisy, losing one costs sqrt2. Do
/// not read the flat column as "dual gyros are immune to a flaky IMU".
/// Sweeping p_online against a non-zero sigma would measure it properly.
fn to_dual(cfg: &Degradation) -> DualImuConfig {
    let g = cfg.gyro;
    let noise = ChannelFault { sigma: g.sigma, ..ChannelFault::none() };
    // Dropout is INDEPENDENT per sensor by default: separate buses, separate
    // CS lines, so both-down happens only by coincidence at (1-p)^2. Set
    // CORRELATED_DROPOUT=1 to route the same p to the shared-bus outage
    // instead -- one task awaits both reads, so starving it (or dipping 3V3)
    // takes both together. Same p, very different consequence.
    let correlated = std::env::var("CORRELATED_DROPOUT").is_ok();
    let drop = ChannelFault {
        p_online: g.p_online,
        dropout_dwell_s: g.dropout_dwell_s,
        ..ChannelFault::none()
    };
    let per_drop = if correlated { ChannelFault::none() } else { drop };
    DualImuConfig {
        per_sensor: [
            ImuFault {
                gyro: ChannelFault {
                    bias: g.bias,
                    p_online: per_drop.p_online,
                    dropout_dwell_s: per_drop.dropout_dwell_s,
                    ..noise
                },
                accel: cfg.accel,
            },
            ImuFault {
                gyro: ChannelFault {
                    p_online: per_drop.p_online,
                    dropout_dwell_s: per_drop.dropout_dwell_s,
                    ..noise
                },
                accel: cfg.accel,
            },
        ],
        common: ImuFault {
            gyro: ChannelFault {
                vib_amplitude: g.vib_amplitude,
                vib_hz: g.vib_hz,
                p_online: if correlated { drop.p_online } else { 1.0 },
                dropout_dwell_s: drop.dropout_dwell_s,
                ..ChannelFault::none()
            },
            accel: ChannelFault::none(),
        },
        skew_s: 0.0,
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
    /// Per-cause tally, indexed by FailCause::idx.
    causes: [usize; 4],
}

/// Assemble the exam (plant, rates, disturbance shaping, dual-IMU mapping)
/// and the answers (firmware tunables, with the cutoff overridable for the
/// diagnostic cutoff sweep).
fn harness_cfg(cfg: &Degradation, r: Rates, dual: bool) -> (HarnessCfg, Tunables) {
    let h = HarnessCfg {
        rates: r,
        plant: plant_params(),
        total_s: TOTAL_S,
        target_alt: TARGET_ALT,
        disturb_ms: disturb_ms(),
        dual,
        dual_cfg: to_dual(cfg),
        // The sweep measures regulation about level, as before.
        cmd: fc_rusty::sim::harness::AttitudeStep::NONE,
    };
    (h, tunables())
}

/// The firmware's own values, with GYRO_LPF_HZ overriding the cutoff for
/// diagnosis, and --nofilter removing the filter entirely.
fn tunables() -> Tunables {
    let mut t = Tunables::firmware();
    t.gyro_fc_hz = std::env::var("GYRO_LPF_HZ").ok().and_then(|v| v.parse().ok())
        .unwrap_or_else(|| ImuFilterParams::default().gyro_fc_hz);
    if std::env::args().any(|a| a == "--nofilter") {
        t.gyro_fc_hz = 0.0;
    }
    t
}

fn aggregate(cfg: Degradation, seeds: u64, r: Rates, dual: bool) -> Agg {
    let (h, tun) = harness_cfg(&cfg, r, dual);
    let mut a = Agg { att_rms: 0.0, att_max: 0.0, alt_rms: 0.0, air_frac: 0.0,
                      failures: 0, n: 0, first_fail_t: None, causes: [0; 4] };
    let mut ok = 0usize;
    for s in 0..seeds {
        let m = run_case(&h, &tun, cfg, s * 7919 + 1, None);
        if let Some((t, cause)) = m.failed_at {
            a.failures += 1;
            a.causes[cause.idx()] += 1;
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
    println!("{:>10} {:>9} {:>9} {:>9} {:>7} {:>22}",
             knob, "att_rms", "att_max", "alt_rms", "air%", "fail");
    println!("{}", "-".repeat(72));
}

fn row(label: String, a: Agg) {
    let fail = if a.failures == 0 {
        "-".to_string()
    } else {
        // Name every cause present, not just the majority: a column reading
        // "5 crash 3 div" is the signal that two different things are going
        // wrong across seeds, which a single label would hide.
        let mix: Vec<String> = FailCause::ALL.iter()
            .filter(|c| a.causes[c.idx()] > 0)
            .map(|c| format!("{}{}", a.causes[c.idx()], c.short()))
            .collect();
        format!("{}/{} {} @{:.1}s", a.failures, a.n, mix.join("+"),
                a.first_fail_t.unwrap_or(f32::NAN))
    };
    if a.failures == a.n {
        println!("{:>10} {:>9} {:>9} {:>9} {:>7} {:>22}", label, "-", "-", "-", "-", fail);
    } else {
        println!("{:>10} {:>9.3} {:>9.3} {:>9.3} {:>6.1}% {:>22}",
                 label, a.att_rms, a.att_max, a.alt_rms, a.air_frac * 100.0, fail);
    }
}

fn csv_row(axis: &str, value: f32, a: Agg) {
    println!("{},{},{:.4},{:.4},{:.4},{:.4},{},{},{},{},{},{}",
             axis, value, a.att_rms, a.att_max, a.alt_rms, a.air_frac,
             a.failures, a.n,
             a.causes[FailCause::Diverged.idx()], a.causes[FailCause::Crashed.idx()],
             a.causes[FailCause::Flyaway.idx()], a.causes[FailCause::NonFinite.idx()]);
}

fn main() {
    let csv = std::env::args().any(|a| a == "--csv");
    let legacy = std::env::args().any(|a| a == "--legacy");
    // --nofilter drops the firmware's gyro LPF, to show what it is buying.
    let filter = !std::env::args().any(|a| a == "--nofilter");
    // --dual: model the board's two gyros (see to_dual). Off by default so
    // the single-gyro results in f005c4a and ff359bc stay reproducible.
    let dual = std::env::args().any(|a| a == "--dual");
    let rates = if legacy { Rates::LEGACY } else { Rates::FIRMWARE };
    let seeds: u64 = 8;

    // --trace: one undegraded run, one row per outer tick. Diagnostic entry
    // point for "why did this case fail", which the summary cannot answer.
    if std::env::args().any(|a| a == "--trace") {
        let cfg = Degradation::none();
        let (h, tun) = harness_cfg(&cfg, rates, dual);
        let mut show = |p: fc_rusty::sim::harness::TracePoint| {
            println!(
                "{:6.3} alt={:8.3} vz_up={:8.3} thr_dmd={:6.3} msum/4={:6.3} roll={:7.3} rollrate={:8.2} pid_roll={:7.3}",
                p.t, p.alt, p.vz_up, p.thrust_demand, p.motor_mean,
                p.roll, p.roll_rate, p.pid_roll
            );
        };
        let m = run_case(&h, &tun, cfg, 1, Some(&mut show));
        println!("result: {:?}", m.failed_at);
        return;
    }

    let gyro_sigmas = [0.0f32, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0];
    let vib_freqs = [10.0f32, 50.0, 100.0, 200.0, 400.0];
    let vib_amps = [0.0f32, 2.0, 5.0, 10.0, 20.0];
    let p_onlines = [1.0f32, 0.99, 0.95, 0.9, 0.8, 0.6, 0.4];
    // Down to total loss. 0.0 is a thrown prop or a dead ESC -- worth having
    // in the sweep because a quad cannot hold attitude AND yaw on three
    // motors, so what it does here is a real design question, not a detail.
    let motor_scales = [1.0f32, 0.95, 0.9, 0.8, 0.7, 0.5, 0.25, 0.0];
    let biases = [0.0f32, 0.5, 1.0, 2.0, 5.0, 10.0];

    if csv {
        println!("axis,value,att_rms,att_max,alt_rms,air_frac,failures,seeds,diverged,crashed,flyaway,nonfinite");
    } else {
        println!("=== control degradation sweep ===");
        println!("{} s flights, {} seeds per case, {:.0} Hz inner / {:.0} Hz outer{}",
                 TOTAL_S, seeds, rates.inner_hz(), rates.outer_hz(),
                 if legacy { "  (LEGACY example rates)" } else { "  (firmware rates)" });
        println!("gyro LPF: {}",
                 if filter { "150 Hz Butterworth (as firmware)" } else { "DISABLED (--nofilter)" });
        println!("baseline row of each block is the idealised (undegraded) case");
    }

    // --- gyro white noise ---
    if !csv { header("gyro white noise", "sigma"); }
    for &s in &gyro_sigmas {
        let cfg = Degradation {
            gyro: ChannelFault { sigma: s, ..ChannelFault::none() },
            ..Degradation::none()
        };
        let a = aggregate(cfg, seeds, rates, dual);
        if csv { csv_row("gyro_sigma_dps", s, a) } else { row(format!("{:.2}", s), a) }
    }

    // --- gyro bias: does not average out, so the estimator eats it ---
    if !csv { header("gyro bias (roll axis)", "bias"); }
    for &b in &biases {
        let cfg = Degradation {
            gyro: ChannelFault { bias: [b, 0.0, 0.0], ..ChannelFault::none() },
            ..Degradation::none()
        };
        let a = aggregate(cfg, seeds, rates, dual);
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
            let a = aggregate(cfg, seeds, rates, dual);
            if csv {
                csv_row(&format!("vib_{:.0}hz_amp_dps", f), amp, a)
            } else {
                row(format!("{:.1}", amp), a)
            }
        }
    }

    // --- intermittent gyro: stale samples held, not gaps ---
    if !csv {
        header(&format!("gyro intermittency (p online), sigma {GYRO_FLOOR_DPS} dps"), "p");
    }
    for &p in &p_onlines {
        let cfg = Degradation {
            gyro: ChannelFault {
                p_online: p,
                dropout_dwell_s: 0.02,
                sigma: GYRO_FLOOR_DPS,
                ..ChannelFault::none()
            },
            ..Degradation::none()
        };
        let a = aggregate(cfg, seeds, rates, dual);
        if csv { csv_row("gyro_p_online", p, a) } else { row(format!("{:.2}", p), a) }
    }

    // --- asymmetric airframe: one motor down ---
    if !csv { header("motor 3 thrust scale", "scale"); }
    for &m in &motor_scales {
        let cfg = Degradation { motor_scale: [1.0, 1.0, m, 1.0], ..Degradation::none() };
        let a = aggregate(cfg, seeds, rates, dual);
        if csv { csv_row("motor3_scale", m, a) } else { row(format!("{:.2}", m), a) }
    }

    // --- resonance scan: does the cliff track the OUTER loop rate? ---
    // If aliasing is the mechanism, the danger frequency should move with the
    // outer loop (50 Hz legacy vs 100 Hz firmware) and not with the inner one.
    let scan_hz = [10.0f32, 25.0, 40.0, 50.0, 60.0, 75.0, 100.0, 125.0,
                   150.0, 200.0, 400.0, 800.0, 1600.0];
    for (name, r) in [("firmware 8kHz/100Hz", Rates::FIRMWARE),
                      ("legacy 200Hz/50Hz", Rates::LEGACY)] {
        if !csv {
            header(&format!("resonance scan, amp 5 deg/s -- {}", name), "vib_hz");
        }
        for &f in &scan_hz {
            let cfg = Degradation {
                gyro: ChannelFault { vib_amplitude: 5.0, vib_hz: f, ..ChannelFault::none() },
                ..Degradation::none()
            };
            let a = aggregate(cfg, seeds, r, dual);
            if csv {
                csv_row(&format!("resonance_{}_hz", name.replace(' ', "_")), f, a)
            } else {
                row(format!("{:.0}", f), a)
            }
        }
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
