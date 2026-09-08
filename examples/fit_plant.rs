// examples/fit_plant.rs — turn a bench capture into a motor_tau.
//
// Run with:
//   TRIPLE=$(rustc -vV | sed -n 's/^host: //p')
//   cargo run --release --example fit_plant --no-default-features \
//             --target $TRIPLE -- capture.log
//
// `capture.log` is whatever defmt-print emitted while the motor-test
// firmware dumped its capture:
//
//   PROFILE=1 BIDIR=1 cargo build --release --features motor-test
//   ...flash, then...
//   <serial reader> | defmt-print -e target/.../fc-firmware | tee capture.log
//
// Lines that are not samples are ignored, so piping a whole session in is
// fine.

use fc_rusty::plant_fit::{find_steps, fit_step, summarise, summarise_by, Dir, Step, StepFit};
use fc_rusty::plant_log::{parse_csv_line, PlantSample};

/// Residual above which a step's fit is not trusted. See
/// `plant_fit::summarise`.
const MAX_RESIDUAL: f32 = 0.05;

/// Longest step window considered, milliseconds. The profile holds for
/// 400 ms; anything past that is a different step.
const MAX_WINDOW_MS: u16 = 400;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args.iter().find(|a| !a.starts_with("--"));
    let pole_pairs: f32 = std::env::var("POLE_PAIRS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7.0);

    let text = match path {
        Some(p) => std::fs::read_to_string(p)
            .unwrap_or_else(|e| panic!("cannot read {p}: {e}")),
        None => {
            use std::io::Read;
            let mut s = String::new();
            std::io::stdin().read_to_string(&mut s).expect("stdin");
            s
        }
    };

    let samples: Vec<PlantSample> = text.lines().filter_map(parse_csv_line).collect();
    if samples.is_empty() {
        eprintln!(
            "No PLANT sample lines found.\n\
             Expected lines like: PLANT,t_ms,cmd0..3,per0..3,gx,gy,gz\n\
             Was the firmware built with PROFILE=1 and BIDIR=1?"
        );
        std::process::exit(1);
    }

    let span_ms = samples.last().unwrap().t_ms.saturating_sub(samples[0].t_ms);
    println!("=== plant capture ===");
    println!(
        "{} samples over {} ms ({:.0} Hz), {} pole pairs assumed",
        samples.len(),
        span_ms,
        samples.len() as f32 / (span_ms as f32 * 1e-3).max(1e-6),
        pole_pairs,
    );

    // Telemetry health per motor, before any fitting. A motor with no
    // telemetry produces no fit, and "no fit" should not be reported as
    // if the fit had failed -- it is a wiring or ESC answer.
    print!("telemetry present: ");
    let mut usable = [false; 4];
    for m in 0..4 {
        let n = samples.iter().filter(|s| s.erpm(m).is_some()).count();
        let pct = 100.0 * n as f32 / samples.len() as f32;
        usable[m] = pct > 50.0;
        print!("M{}={:.0}%  ", m + 1, pct);
    }
    println!();
    if !usable.iter().all(|&u| u) {
        println!(
            "  NOTE: a motor below 50% has a telemetry problem, not a fit problem.\n\
             \x20       Check that ESC before reading anything into its numbers."
        );
    }
    println!();

    let mut all_fits: Vec<StepFit> = Vec::new();
    let mut per_motor_tau = [f32::NAN; 4];

    for m in 0..4 {
        let mut buf = [Step { motor: m, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 32];
        let n = find_steps(&samples, m, MAX_WINDOW_MS, &mut buf);
        let fits: Vec<StepFit> =
            buf[..n].iter().filter_map(|&st| fit_step(&samples, st)).collect();

        println!("--- M{} : {} steps, {} fitted ---", m + 1, n, fits.len());
        if fits.is_empty() {
            println!("  no usable steps\n");
            continue;
        }
        println!(
            "  {:>5} {:>5}  {:>9} {:>9}  {:>9} {:>9}  {:>8}",
            "from", "to", "rpm_from", "rpm_to", "tau_thr", "tau_omega", "resid"
        );
        for f in &fits {
            let flag = if f.residual_frac > MAX_RESIDUAL { " <-- rejected" } else { "" };
            println!(
                "  {:>4.0}% {:>4.0}%  {:>9.0} {:>9.0}  {:>8.1}ms {:>8.1}ms  {:>7.3}{}",
                f.cmd_from * 100.0,
                f.cmd_to * 100.0,
                f.erpm_from / pole_pairs,
                f.erpm_to / pole_pairs,
                f.tau_thrust_s * 1e3,
                f.tau_omega_s * 1e3,
                f.residual_frac,
                flag,
            );
        }
        // Report the three separately. tau_omega is the motor's actual
        // constant; the two thrust figures differ by construction and the
        // up one is what the sim should get. See plant_fit::summarise_by.
        let om = summarise_by(&fits, MAX_RESIDUAL, Dir::Both, |f| f.tau_omega_s);
        let up = summarise_by(&fits, MAX_RESIDUAL, Dir::Up, |f| f.tau_thrust_s);
        let dn = summarise_by(&fits, MAX_RESIDUAL, Dir::Down, |f| f.tau_thrust_s);
        match om {
            Some(o) => {
                per_motor_tau[m] = o.mean_s;
                println!(
                    "  motor_tau = {:.1} ms  (spread {:.1}..{:.1} over {} steps, both directions)",
                    o.mean_s * 1e3, o.min_s * 1e3, o.max_s * 1e3, o.n
                );
            }
            None => println!("  every step rejected — response is not first-order"),
        }
        // Thrust constants are a sanity check now, not a parameter. They
        // SHOULD differ by direction; if they do not, the capture is
        // suspect. See plant_fit's module docs.
        if let (Some(u), Some(d)) = (up, dn) {
            println!(
                "    (thrust-domain check: {:.1} ms up / {:.1} ms down, ratio {:.2} — expect >1)",
                u.mean_s * 1e3,
                d.mean_s * 1e3,
                u.mean_s / d.mean_s,
            );
        }
        println!();
        all_fits.extend(fits);
    }

    println!("=== result ===");
    match summarise(&all_fits, MAX_RESIDUAL) {
        None => {
            println!("No step survived the residual check. Nothing to report.");
            std::process::exit(2);
        }
        Some(s) => {
            println!(
                "motor_tau = {:.4} s   ({:.1} ms, from {} steps across all motors)",
                s.mean_s,
                s.mean_s * 1e3,
                s.n
            );
            println!("  sim default is 0.03 s — see QuadParams::motor_tau");
            println!("  This is the rotor-speed constant, which is what QuadSim lags;");
            println!("  it squares speed for thrust, so this is direction-independent.");

            // Motor-to-motor spread is worth more than the mean here. The
            // mean is what the sim wants; the spread is what tells you a
            // particular ESC or motor is different from its neighbours,
            // which is the failure that was just repaired on M4.
            let known: Vec<(usize, f32)> = per_motor_tau
                .iter()
                .enumerate()
                .filter(|(_, t)| t.is_finite())
                .map(|(i, &t)| (i, t))
                .collect();
            if known.len() >= 2 {
                let lo = known.iter().cloned().fold((0, f32::INFINITY), |a, b| if b.1 < a.1 { b } else { a });
                let hi = known.iter().cloned().fold((0, f32::NEG_INFINITY), |a, b| if b.1 > a.1 { b } else { a });
                let ratio = hi.1 / lo.1;
                println!(
                    "  per-motor spread: M{} fastest at {:.1} ms, M{} slowest at {:.1} ms ({:.2}x)",
                    lo.0 + 1,
                    lo.1 * 1e3,
                    hi.0 + 1,
                    hi.1 * 1e3,
                    ratio,
                );
                if ratio > 1.3 {
                    println!(
                        "  WARNING: >1.3x spread. The mixer assumes four identical motors,\n\
                         \x20         so this is a hardware finding, not a modelling one."
                    );
                }
            }

            println!();
            println!("What this does NOT give you:");
            println!("  max_thrust  — needs a load cell, or a measured hover throttle");
            println!("  inertia     — needs the airframe free to rotate");
            println!("  drag_k      — needs airspeed, so a forward-flight log");
            println!("Those are flight measurements and want the flash log, not the wire.");
        }
    }
}
