// ga_tune.rs — fit controller gains against the degradation harness.
//
// Rationale: you tune in sim first and refine the sim from reality later.
// That works even with an uncalibrated plant, because a search only needs
// the sim to RANK two gain sets consistently -- it does not need absolute
// fidelity. What it does need is to be stopped from cheating, and most of
// the design here is about that.
//
// Three guards, because an unguarded GA reliably produces a number that
// looks wonderful and means nothing:
//
//   1. The search may only touch `Tunables` (gains, cutoff, D filter). The
//      plant, disturbances and degradation live in `HarnessCfg` and are
//      fixed. A search allowed to touch those tunes the exam, not the
//      controller.
//   2. Fitness is averaged over a SPREAD of degradation cases, not the
//      clean one. Optimising the nominal case alone reliably yields gains
//      that are excellent at hover and fall over on the first gust.
//   3. A HOLDOUT set -- vibration frequencies, amplitudes and seeds the
//      search never sees -- is scored separately every generation. If
//      holdout diverges from training, the result is overfitted and the
//      run should be discarded. That number is printed, not hidden.
//
// Altitude gains are deliberately NOT tuned. The characteristic failure
// here is a rate-loop limit cycle that airmode converts into a climb; a
// search allowed to retune the altitude loop could paper over that instead
// of fixing it, and would report success while the instability remained.
//
// Usage:
//   cargo run --release --example ga_tune --no-default-features \
//        --target $(rustc -vV | sed -n 's/^host: //p')
//   GA_POP=48 GA_GENS=40 GA_SEED=7 GA_THREADS=8 ... (all optional)

use fc_rusty::control::altitude::AltitudeGains;
use fc_rusty::control::pid::{PidGains, PidLimits};
use fc_rusty::sim::degrade::{ChannelFault, Degradation};
use fc_rusty::sim::dual_imu::DualImuConfig;
use fc_rusty::sim::harness::{
    run_case, AttitudeStep, HarnessCfg, Metrics, Tunables,
};
use fc_rusty::sim::sensors::Rng;
use fc_rusty::sim::QuadParams;
use core::derive;

/// Flights are shorter than the sweep's 10 s: both disturbances land by 5 s,
/// and 8 s leaves 3 s of settling to score. The GA runs thousands of these.
const FLIGHT_S: f32 = 8.0;

/// One gene, held in [0,1] and mapped onto its real range exponentially.
/// Gains span orders of magnitude, so a linear genome would spend nearly all
/// its resolution in the top decade and barely explore the bottom.
#[derive(Debug, Clone, Copy)]
struct Gene {
    lo: f32,
    hi: f32,
}

impl Gene {
    const fn new(lo: f32, hi: f32) -> Self {
        Self { lo, hi }
    }
    fn decode(&self, x: f32) -> f32 {
        let x = x.clamp(0.0, 1.0);
        self.lo * (self.hi / self.lo).powf(x)
    }
}

const N_GENES: usize = 7;

/// Bounds bracket the firmware's current values so the incumbent sits
/// inside and the search can go both up and down. kd's floor is
/// effectively "off".
///
/// The rate floors are TWO decades below the firmware, not one. On the
/// measured 7in plant the best gains found by hand were kp 0.001 and
/// kd 5e-5 — kp sat below the old 0.002 floor, so the search could not
/// represent its own answer and would have reported the floor as optimal.
/// A bound that clips the result is worse than no search: it returns a
/// confident number with no warning that it is pinned.
const GENES: [Gene; N_GENES] = [
    Gene::new(0.0002, 0.2),     // rate kp   (firmware 0.02)
    Gene::new(2e-5, 0.05),      // rate ki   (firmware 0.005)
    Gene::new(1e-6, 0.01),      // rate kd   (firmware 0.001)
    Gene::new(0.003, 0.3),      // yaw kp    (firmware 0.03)
    Gene::new(0.0002, 0.05),    // yaw ki    (firmware 0.005)
    Gene::new(40.0, 2000.0),    // gyro fc   (firmware 150)
    Gene::new(0.0005, 0.05),    // d_lpf tau (firmware 0.008)
];

const GENE_NAMES: [&str; N_GENES] =
    ["rate_kp", "rate_ki", "rate_kd", "yaw_kp", "yaw_ki", "gyro_fc", "d_tau"];

fn to_tunables(g: &[f32; N_GENES]) -> Tunables {
    Tunables {
        rate: PidGains {
            kp: GENES[0].decode(g[0]),
            ki: GENES[1].decode(g[1]),
            kd: GENES[2].decode(g[2]),
        },
        yaw: PidGains {
            kp: GENES[3].decode(g[3]),
            ki: GENES[4].decode(g[4]),
            kd: 0.0,
        },
        limits: PidLimits {
            integral_max: 0.3,
            output_max: 0.5,
            d_lpf_tau_s: GENES[6].decode(g[6]),
        },
        gyro_fc_hz: GENES[5].decode(g[5]),
        // Fixed on purpose -- see the header.
        alt: AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 },
        // Not a tuned gene: it is a statement about the airframe, not a
        // gain. Set from the environment so a run can ask what the gains
        // look like with the collective trim removed.
        geometry_mixer: std::env::var("GEOMETRY_MIXER").is_ok(),
        // Everything the search does not tune comes from the firmware
        // baseline, so a field added later joins the fixed set rather than
        // silently becoming a zero.
        ..Tunables::firmware()
    }
}

/// Invert the mapping, so the firmware's own values can be injected into the
/// starting population. A search that cannot even represent the incumbent
/// cannot be said to have beaten it.
fn from_tunables(t: &Tunables) -> [f32; N_GENES] {
    let enc = |gene: Gene, v: f32| {
        ((v.max(gene.lo) / gene.lo).ln() / (gene.hi / gene.lo).ln()).clamp(0.0, 1.0)
    };
    [
        enc(GENES[0], t.rate.kp),
        enc(GENES[1], t.rate.ki),
        enc(GENES[2], t.rate.kd),
        enc(GENES[3], t.yaw.kp),
        enc(GENES[4], t.yaw.ki),
        enc(GENES[5], t.gyro_fc_hz),
        enc(GENES[6], t.limits.d_lpf_tau_s),
    ]
}

// ---- Fitness ----------------------------------------------------------

/// One scored condition: a degradation and the seed to fly it with.
#[derive(Clone, Copy)]
struct Case {
    deg: Degradation,
    seed: u64,
    /// Commanded attitude step. Cases with a command measure TRACKING;
    /// cases without measure disturbance rejection. Both are needed: score
    /// only the second and the search filters as hard as it can, because
    /// lag costs nothing when nothing ever asks the aircraft to move.
    cmd: AttitudeStep,
    /// Initial roll, degrees, for recovery-from-upset cases. 0 starts
    /// level. Non-zero also needs `alt_m` raised, or the aircraft simply
    /// runs out of height before it can finish recovering.
    roll0_deg: f32,
    /// Target altitude, metres. 0 keeps the shared config's.
    alt_m: f32,
}

fn case(deg: Degradation, seed: u64) -> Case {
    Case { deg, seed, cmd: AttitudeStep::NONE, roll0_deg: 0.0, alt_m: 0.0 }
}

/// Start rolled past level and require recovery, with height to do it in.
///
/// Added 2026-09-20 because the fitness was missing the cost that keeps
/// integral gain honest. Every case above starts level and stays near it,
/// so the integrator never saturates for long, and rate_ki simply drifted
/// to its ceiling with nothing to push back. The sweep disagreed sharply:
/// past ki 0.1 its upset and motor-failure rows collapse (744 of 768
/// failures at ki 0.25). A wound-up integrator is only expensive when the
/// error has been large for a while, and nothing here asked for that.
fn upset(roll0_deg: f32, deg: Degradation, seed: u64) -> Case {
    Case { deg, seed, cmd: AttitudeStep::NONE, roll0_deg, alt_m: 100.0 }
}

impl Case {
    /// This case's harness config: the shared one with the per-case
    /// command, upset and altitude applied.
    fn cfg(&self, h: &HarnessCfg) -> HarnessCfg {
        HarnessCfg {
            cmd: self.cmd,
            initial_attitude_deg: [self.roll0_deg, 0.0, 0.0],
            target_alt: if self.alt_m > 0.0 { self.alt_m } else { h.target_alt },
            ..*h
        }
    }
}

/// Roll out at 5.5 s and back to level at 7.0 s, after both disturbances
/// have landed. The return edge is the part that charges an over-wound
/// integrator for its overshoot.
fn tracked(deg: Degradation, seed: u64, roll: f32) -> Case {
    Case {
        deg,
        seed,
        cmd: AttitudeStep {
            at_s: 5.5,
            roll_deg: roll,
            pitch_deg: 0.0,
            return_at_s: 7.0,
        },
        roll0_deg: 0.0,
        alt_m: 0.0,
    }
}

fn gyro(f: ChannelFault) -> Degradation {
    Degradation { gyro: f, ..Degradation::none() }
}

fn noise(sigma: f32) -> ChannelFault {
    ChannelFault { sigma, ..ChannelFault::none() }
}

fn vib(amp: f32, hz: f32) -> ChannelFault {
    ChannelFault { vib_amplitude: amp, vib_hz: hz, ..ChannelFault::none() }
}

/// What the search is scored on. Spread deliberately: clean, noise,
/// vibration at two frequencies, a weak motor. Optimising only the clean
/// case yields gains that hover beautifully and fall over on a gust.
fn training_set() -> Vec<Case> {
    let mut v = Vec::new();
    for seed in [1u64, 2] {
        v.push(case(Degradation::none(), seed));
        v.push(case(gyro(noise(1.0)), seed));
        v.push(case(gyro(noise(4.0)), seed));
        v.push(case(gyro(vib(5.0, 80.0)), seed));
        v.push(case(gyro(vib(5.0, 300.0)), seed));
        // LOW-frequency, high-amplitude vibration. Added 2026-09-20 to
        // close a measured gap: with only the 80 and 300 Hz cases above,
        // the search settled on a 58 Hz gyro cutoff across three
        // independent runs, which passes everything below it straight
        // through. That genome scored best on this fitness and yet lost
        // on the sweep -- 90 failures against 74 for a 300 Hz cutoff --
        // because the sweep spends 51 of its 96 rows on vibration, up to
        // 20 dps and down to 10 Hz, and this set asked for none of it.
        // A search cannot trade off a cost it is never shown.
        v.push(case(gyro(vib(15.0, 30.0)), seed));
        v.push(case(gyro(vib(10.0, 50.0)), seed));
        // Sustained large error, the cost that prices integral gain. A
        // 90 deg upset and a half-dead motor both hold the integrator
        // against its clamp for seconds at a time.
        v.push(upset(90.0, Degradation::none(), seed));
        v.push(case(
            Degradation { motor_scale: [1.0, 1.0, 0.5, 1.0], ..Degradation::none() },
            seed,
        ));
        v.push(case(
            Degradation { motor_scale: [1.0, 1.0, 0.8, 1.0], ..Degradation::none() },
            seed,
        ));
        // Tracking: these are what stop the search buying quiet with lag.
        v.push(tracked(Degradation::none(), seed, 20.0));
        v.push(tracked(gyro(noise(2.0)), seed, 30.0));
    }
    v
}

/// Never seen by the search. Different frequencies, amplitudes and seeds.
/// If holdout tracks training the result generalises; if it does not, the
/// run is overfitted and the answer should be thrown away.
fn holdout_set() -> Vec<Case> {
    let mut v = Vec::new();
    for seed in [101u64, 102] {
        v.push(case(gyro(noise(2.0)), seed));
        v.push(case(gyro(noise(8.0)), seed));
        v.push(case(gyro(vib(3.0, 45.0)), seed));
        v.push(case(gyro(vib(8.0, 160.0)), seed));
        v.push(case(gyro(vib(5.0, 600.0)), seed));
        // Low-frequency holdout, at amplitudes and frequencies the
        // training set does not use, so the widened spread above is
        // checked rather than merely memorised.
        v.push(case(gyro(vib(12.0, 20.0)), seed));
        v.push(case(gyro(vib(15.0, 70.0)), seed));
        v.push(upset(120.0, Degradation::none(), seed));
        v.push(case(
            Degradation { motor_scale: [0.6, 1.0, 1.0, 1.0], ..Degradation::none() },
            seed,
        ));
        v.push(case(
            Degradation { motor_scale: [0.85, 1.0, 1.0, 1.0], ..Degradation::none() },
            seed,
        ));
        v.push(tracked(Degradation::none(), seed, 15.0));
        v.push(tracked(gyro(vib(4.0, 120.0)), seed, 25.0));
    }
    v
}

/// Cost for one flight. Lower is better.
///
/// The scale factors are a PREFERENCE, not a truth: they say a degree of
/// attitude RMS matters about as much as half a metre of altitude RMS. They
/// are chosen so an undegraded firmware-quality flight scores near 1, which
/// makes the printed numbers readable rather than because the ratio is
/// derived from anything.
fn cost(m: &Metrics, total_s: f32) -> f64 {
    match m.failed_at {
        // Failure dominates, and failing EARLY is worse than failing late --
        // otherwise every failing genome scores identically and the search
        // gets no gradient out of the dead region it starts in.
        Some((t, _)) => 1000.0 + 100.0 * (total_s - t) as f64,
        None => {
            let att = (m.att_rms / 0.05) as f64;
            let peak = (m.att_max / 0.30) as f64;
            let alt = (m.alt_rms / 0.50) as f64;
            let air = (m.air_frac / 0.20) as f64;
            att + peak + alt + air
        }
    }
}

fn evaluate(h: &HarnessCfg, tun: &Tunables, cases: &[Case]) -> f64 {
    let mut total = 0.0;
    for c in cases {
        let hc = c.cfg(h);
        total += cost(&run_case(&hc, tun, c.deg, c.seed, None), h.total_s);
    }
    total / cases.len() as f64
}

/// How many of `cases` this genome actually completes. Reported alongside
/// cost because a mean can hide "survives 11 of 12".
fn survived(h: &HarnessCfg, tun: &Tunables, cases: &[Case]) -> usize {
    cases
        .iter()
        .filter(|c| {
            let hc = c.cfg(h);
            run_case(&hc, tun, c.deg, c.seed, None).failed_at.is_none()
        })
        .count()
}

// ---- The search -------------------------------------------------------

fn env_f32(k: &str, d: f32) -> f32 {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}
fn env_usize(k: &str, d: usize) -> usize {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}

fn tournament(pop: &[([f32; N_GENES], f64)], rng: &mut Rng, k: usize) -> [f32; N_GENES] {
    let mut best = (rng.uniform() * pop.len() as f32) as usize % pop.len();
    for _ in 1..k {
        let c = (rng.uniform() * pop.len() as f32) as usize % pop.len();
        if pop[c].1 < pop[best].1 {
            best = c;
        }
    }
    pop[best].0
}

/// Score one explicit candidate instead of searching.
///
/// The point of the tool is not only to produce gains but to grade them, and
/// a hand-built compromise (take the genes a search agrees on, keep the
/// firmware's where it does not) is exactly the sort of thing that must be
/// measured rather than assumed -- it is a set the GA itself never evaluated.
fn eval_mode(h: &HarnessCfg, train: &[Case], hold: &[Case]) {
    let mut t = Tunables::firmware();
    let g = |k: &str, d: f32| env_f32(k, d);
    t.rate.kp = g("EVAL_KP", t.rate.kp);
    t.rate.ki = g("EVAL_KI", t.rate.ki);
    t.rate.kd = g("EVAL_KD", t.rate.kd);
    t.yaw.kp = g("EVAL_YAW_KP", t.yaw.kp);
    t.yaw.ki = g("EVAL_YAW_KI", t.yaw.ki);
    t.gyro_fc_hz = g("EVAL_FC", t.gyro_fc_hz);
    t.limits.d_lpf_tau_s = g("EVAL_DTAU", t.limits.d_lpf_tau_s);

    println!("candidate: kp {:.6} ki {:.6} kd {:.6} yaw_kp {:.6} yaw_ki {:.6} fc {:.1} d_tau {:.5}",
             t.rate.kp, t.rate.ki, t.rate.kd, t.yaw.kp, t.yaw.ki,
             t.gyro_fc_hz, t.limits.d_lpf_tau_s);
    println!("  train   {:8.2}  ({}/{} survive)",
             evaluate(h, &t, train), survived(h, &t, train), train.len());
    println!("  holdout {:8.2}  ({}/{} survive)",
             evaluate(h, &t, hold), survived(h, &t, hold), hold.len());
}

fn main() {
    let pop_size = env_usize("GA_POP", 32);
    let generations = env_usize("GA_GENS", 25);
    let threads = env_usize("GA_THREADS", std::thread::available_parallelism()
        .map(|n| n.get()).unwrap_or(4));
    let mut rng = Rng::new(env_usize("GA_SEED", 12345) as u64);
    let mut_sigma = env_f32("GA_MUT", 0.08);

    // PLANT=deadcat selects the measured 7in airframe; the default stays
    // the 5in figures the baseline is blessed against. PLANT_THRUST and
    // PLANT_INERTIA then override either, to check a candidate against a
    // plant wrong in the direction the real one might be. Loop gain goes
    // as the SQUARE ROOT of max_thrust at the hover point, so a 4x thrust
    // error is a 2x gain error — gains fitted at one thrust do partly
    // transfer to another, but not exactly, and that is the whole risk of
    // tuning in sim.
    let mut plant = match std::env::var("PLANT").as_deref() {
        Ok("deadcat") => QuadParams::deadcat_7in(),
        Ok("default") | Err(_) => QuadParams::default(),
        Ok(other) => panic!("PLANT={other}: expected 'deadcat' or 'default'"),
    };
    if let Some(v) = std::env::var("PLANT_THRUST").ok().and_then(|v| v.parse().ok()) {
        plant.max_thrust = v;
    }
    if let Some(v) = std::env::var("PLANT_INERTIA").ok().and_then(|v| v.parse().ok()) {
        plant.inertia = [v, v, v * 2.0];
    }
    let h = HarnessCfg {
        total_s: FLIGHT_S,
        target_alt: 5.0,
        // A raised-cosine gust rather than a state poke: the search should
        // not be rewarded for handling an unphysical input.
        disturb_ms: 20.0,
        dual: false,
        dual_cfg: DualImuConfig::none(),
        cmd: AttitudeStep::NONE, // per-case; see evaluate()
        // Same reason as to_tunables: inherit the rest, so the scenario the
        // search optimises against does not quietly change shape whenever
        // the harness grows a knob.
        ..HarnessCfg::firmware_rates(plant)
    };

    // GA_EVAL="kp,ki,kd,yaw_kp,yaw_ki,gyro_fc,d_tau" scores one genome
    // against train and holdout and exits, so two rival answers can be
    // compared on the SAME fitness the search uses. Without this the only
    // way to compare was the sweep, which asks a different question and
    // therefore cannot settle whether a search under-performed or was
    // merely asked the wrong thing.
    if let Ok(spec) = std::env::var("GA_EVAL") {
        let vals: Vec<f32> = spec
            .split(',')
            .map(|t| t.trim().parse().expect("GA_EVAL: expected 7 comma-separated numbers"))
            .collect();
        assert_eq!(vals.len(), N_GENES, "GA_EVAL needs {N_GENES} values: {}", GENE_NAMES.join(","));
        let t = Tunables {
            rate: PidGains { kp: vals[0], ki: vals[1], kd: vals[2] },
            yaw: PidGains { kp: vals[3], ki: vals[4], kd: 0.0 },
            limits: PidLimits { integral_max: 0.3, output_max: 0.5, d_lpf_tau_s: vals[6] },
            gyro_fc_hz: vals[5],
            alt: AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 },
            geometry_mixer: std::env::var("GEOMETRY_MIXER").is_ok(),
            ..Tunables::firmware()
        };
        let (tr, ho) = (training_set(), holdout_set());
        println!("GA_EVAL  train {:8.2} ({}/{})   holdout {:8.2} ({}/{})",
                 evaluate(&h, &t, &tr), survived(&h, &t, &tr), tr.len(),
                 evaluate(&h, &t, &ho), survived(&h, &t, &ho), ho.len());
        return;
    }

    let train = training_set();
    let hold = holdout_set();

    if std::env::args().any(|a| a == "--eval") {
        eval_mode(&h, &train, &hold);
        return;
    }

    let base = Tunables::firmware();
    let base_train = evaluate(&h, &base, &train);
    let base_hold = evaluate(&h, &base, &hold);

    println!("=== GA gain tuning ===");
    println!("pop {pop_size}, {generations} generations, {threads} threads, \
{} training / {} holdout cases, {FLIGHT_S} s flights",
             train.len(), hold.len());
    println!();
    println!("firmware baseline:  train {:8.2} ({}/{} survive)   holdout {:8.2} ({}/{})",
             base_train, survived(&h, &base, &train), train.len(),
             base_hold, survived(&h, &base, &hold), hold.len());
    println!();
    println!("{:>4} {:>10} {:>10} {:>9} {:>9}", "gen", "best train", "holdout", "surv_tr", "surv_ho");
    println!("{}", "-".repeat(48));

    // Seed the population with the incumbent plus random genomes. Including
    // the firmware's own values means "the GA beat it" is a real comparison
    // rather than an artefact of the incumbent being unrepresentable.
    let mut genomes: Vec<[f32; N_GENES]> = Vec::with_capacity(pop_size);
    genomes.push(from_tunables(&base));
    while genomes.len() < pop_size {
        let mut g = [0.0f32; N_GENES];
        for x in g.iter_mut() {
            *x = rng.uniform();
        }
        genomes.push(g);
    }

    let mut best_overall = (from_tunables(&base), base_train);
    // Generation at which the incumbent was last beaten by more than a
    // rounding error, for the stall warning at the end.
    let mut last_gain_gen: usize = 0;

    for generation in 0..generations {
        // Fitness in parallel: evaluations are independent and this is the
        // whole cost of the run.
        let scores: Vec<f64> = {
            let chunk = genomes.len().div_ceil(threads.max(1));
            let mut out = vec![0.0f64; genomes.len()];
            std::thread::scope(|s| {
                let mut handles = Vec::new();
                for (ci, gs) in genomes.chunks(chunk).enumerate() {
                    let h = &h;
                    let train = &train;
                    handles.push(s.spawn(move || {
                        let v: Vec<f64> = gs
                            .iter()
                            .map(|g| evaluate(h, &to_tunables(g), train))
                            .collect();
                        (ci * chunk, v)
                    }));
                }
                for hd in handles {
                    let (off, v) = hd.join().unwrap();
                    out[off..off + v.len()].copy_from_slice(&v);
                }
            });
            out
        };

        let mut pop: Vec<([f32; N_GENES], f64)> =
            genomes.iter().copied().zip(scores.iter().copied()).collect();
        pop.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        if pop[0].1 < best_overall.1 {
            // 0.1% counts as progress; anything smaller is drift and
            // should not keep resetting the stall counter.
            if pop[0].1 < best_overall.1 * 0.999 {
                last_gain_gen = generation;
            }
            best_overall = pop[0];
        }

        let bt = to_tunables(&pop[0].0);
        println!("{:>4} {:>10.2} {:>10.2} {:>7}/{:<2} {:>7}/{:<2}",
                 generation, pop[0].1, evaluate(&h, &bt, &hold),
                 survived(&h, &bt, &train), train.len(),
                 survived(&h, &bt, &hold), hold.len());

        // Next generation: elitism + tournament + uniform crossover +
        // gaussian mutation in genome space.
        let mut next: Vec<[f32; N_GENES]> = Vec::with_capacity(pop_size);
        next.push(pop[0].0);
        next.push(pop[1].0);
        while next.len() < pop_size {
            let a = tournament(&pop, &mut rng, 3);
            let b = tournament(&pop, &mut rng, 3);
            let mut c = [0.0f32; N_GENES];
            for i in 0..N_GENES {
                c[i] = if rng.uniform() < 0.5 { a[i] } else { b[i] };
                c[i] = (c[i] + rng.normal() * mut_sigma).clamp(0.0, 1.0);
            }
            next.push(c);
        }
        genomes = next;
    }

    let best = to_tunables(&best_overall.0);
    println!();
    println!("=== best genome ===");
    for (i, name) in GENE_NAMES.iter().enumerate() {
        println!("  {:<8} {:>12.6}   (firmware {:>10.6})",
                 name, GENES[i].decode(best_overall.0[i]),
                 GENES[i].decode(from_tunables(&base)[i]));
    }
    println!();
    println!("             train {:8.2} ({}/{})   holdout {:8.2} ({}/{})",
             best_overall.1, survived(&h, &best, &train), train.len(),
             evaluate(&h, &best, &hold), survived(&h, &best, &hold), hold.len());
    println!("  firmware:  train {:8.2} ({}/{})   holdout {:8.2} ({}/{})",
             base_train, survived(&h, &base, &train), train.len(),
             base_hold, survived(&h, &base, &hold), hold.len());

    // A gene resting on its bound is not an optimum, it is the search
    // pressing against a wall -- and usually it means the fitness function
    // is missing a cost, not that the bound is wrong. The first version of
    // this tool pinned gyro_fc, d_tau and ki because nothing in the score
    // penalised lag or windup. Say so rather than leaving it to be noticed.
    let pinned: Vec<&str> = GENE_NAMES
        .iter()
        .enumerate()
        .filter(|(i, _)| best_overall.0[*i] < 0.01 || best_overall.0[*i] > 0.99)
        .map(|(_, n)| *n)
        .collect();
    println!();
    if pinned.is_empty() {
        println!("No gene is resting on its bound: this is an interior optimum.");
    } else {
        println!("WARNING: pinned at bounds: {}", pinned.join(", "));
        println!("  A pinned gene means the search wanted to go further. Ask what");
        println!("  cost is MISSING from the fitness function before widening the");
        println!("  bound -- widening it usually just moves the wall.");
    }
    // An early stall is worth reporting, but do NOT call it a local
    // optimum without checking, because the same symptom has a second and
    // more likely cause: a training set that does not ask for the thing
    // the genome is bad at.
    //
    // 2026-09-20 is the cautionary case. A 64x60 run settled by generation
    // 19 on gyro_fc 57.9 Hz / d_tau 0.77 ms. A hand scan found gyro_fc
    // 300 Hz with d_tau 8 ms strictly better ON THE SWEEP -- 74 failures
    // against 90, vibration failures halved -- which looked exactly like
    // premature convergence. It was not. Two further runs at 96x140 with
    // higher mutation and different seeds landed in the same place
    // (fitness 57.25, 57.31, 57.32), so the search was finding a robust
    // optimum OF THIS FITNESS FUNCTION.
    //
    // The gap was the training set. It carries two vibration cases per
    // seed, both at 5 dps and at 80 and 300 Hz, while the sweep devotes
    // 51 of its 96 rows to vibration, up to 20 dps and down to 10 Hz --
    // right where a 58 Hz cutoff passes everything straight through. The
    // search was answering a question about vibration that nobody asked
    // it. Widening the training spread is the fix; re-seeding is not.
    //
    // Use GA_EVAL to score a specific genome against train and holdout
    // before concluding anything about which of two answers is better.
    let stalled = generations.saturating_sub(last_gain_gen);
    if generations >= 10 && stalled * 2 >= generations {
        println!();
        println!("NOTE: no real improvement for the last {stalled} of {generations} generations.");
        println!("  That is usually convergence rather than a problem. Before");
        println!("  treating it as a LOCAL optimum, re-run with another GA_SEED");
        println!("  and a larger GA_MUT: if they agree, the search is fine and any");
        println!("  disagreement with the sweep is a TRAINING SET gap, not a search");
        println!("  failure. Score the rival genome with GA_EVAL to tell which.");
    }
    println!();
    println!("Read the HOLDOUT column. If it did not improve alongside train,");
    println!("the run overfitted and this genome is not a result.");
}
