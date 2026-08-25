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
    run_case, AttitudeStep, HarnessCfg, Metrics, Rates, Tunables,
};
use fc_rusty::sim::sensors::Rng;
use fc_rusty::sim::QuadParams;

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

/// Bounds bracket the firmware's current values by roughly a decade either
/// way, so the incumbent sits mid-genome and the search can go both up and
/// down. kd's floor is effectively "off".
const GENES: [Gene; N_GENES] = [
    Gene::new(0.002, 0.2),      // rate kp   (firmware 0.02)
    Gene::new(0.0002, 0.05),    // rate ki   (firmware 0.005)
    Gene::new(1e-5, 0.01),      // rate kd   (firmware 0.001)
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
}

fn case(deg: Degradation, seed: u64) -> Case {
    Case { deg, seed, cmd: AttitudeStep::NONE }
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
        let hc = HarnessCfg { cmd: c.cmd, ..*h };
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
            let hc = HarnessCfg { cmd: c.cmd, ..*h };
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

fn main() {
    let pop_size = env_usize("GA_POP", 32);
    let generations = env_usize("GA_GENS", 25);
    let threads = env_usize("GA_THREADS", std::thread::available_parallelism()
        .map(|n| n.get()).unwrap_or(4));
    let mut rng = Rng::new(env_usize("GA_SEED", 12345) as u64);
    let mut_sigma = env_f32("GA_MUT", 0.08);

    let h = HarnessCfg {
        rates: Rates::FIRMWARE,
        plant: QuadParams::default(),
        total_s: FLIGHT_S,
        target_alt: 5.0,
        // A raised-cosine gust rather than a state poke: the search should
        // not be rewarded for handling an unphysical input.
        disturb_ms: 20.0,
        dual: false,
        dual_cfg: DualImuConfig::none(),
        cmd: AttitudeStep::NONE, // per-case; see evaluate()
    };

    let train = training_set();
    let hold = holdout_set();

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
    println!();
    println!("Read the HOLDOUT column. If it did not improve alongside train,");
    println!("the run overfitted and this genome is not a result.");
}
