// sim_eval — headless estimator evaluation (see src/sim/est_eval.rs).
//
//   cargo sim-eval sim/scenarios/nominal_hover.toml
//       -> {"rmse_attitude":..,"rmse_altitude":..,"max_overshoot":..,
//           "divergence_count":..,"pass":..}
//   cargo sim-eval --suite sim/scenarios            all *.toml, one object
//   cargo sim-eval --suite sim/scenarios --params cand.toml
//   cargo sim-eval --compare old.json new.json [--baseline base.json]
//   add --detail to print fault-evidence counters to stderr
//
// `cargo sim-eval` is an alias in .cargo/config.toml for
// `run --release --no-default-features --target host-tuple --example
// sim_eval --`, so the same command works in PowerShell and bash.
//
// Metric definitions (pooled over the scenario's seeds):
//   rmse_attitude    deg. RMS of the MEKF TILT error: the angle between the
//                    estimated and true body z-axes. Yaw is excluded; the
//                    sim has no magnetometer, so yaw is unobservable.
//                    Seeds whose aircraft failed are excluded.
//   rmse_altitude    m. RMS of PosKF altitude minus true altitude.
//                    Failed seeds excluded.
//   max_overshoot    Unit-free. Largest excursion of the TRUE response past
//                    a commanded step (altitude target or attitude command),
//                    as a fraction of that step. 0 when nothing is stepped.
//   divergence_count Seeds that failed (crash, tumble, flyaway, drift, NaN)
//                    or whose estimator error exceeded est_eval::DIVERGED_*.
//   pass             divergence_count <= pass.max_divergence and each metric
//                    within its pass.* threshold. A null metric fails.
// A metric is null when every seed failed.

use std::collections::BTreeMap;
use std::process::ExitCode;

use fc_rusty::sim::degrade::{ChannelFault, Degradation};
use fc_rusty::sim::est_eval::{self, EstParams, EstRun, EstScenario};
use fc_rusty::sim::harness::Tunables;

// ---------------------------------------------------------------- config

#[derive(Debug, Clone)]
enum Val {
    Num(f64),
    Bool(bool),
    Str(String),
    List(Vec<Val>),
}

impl Val {
    fn num(&self, k: &str) -> f32 {
        match self {
            Val::Num(n) => *n as f32,
            _ => die(&format!("{k}: expected a number")),
        }
    }
    fn boolean(&self, k: &str) -> bool {
        match self {
            Val::Bool(b) => *b,
            _ => die(&format!("{k}: expected true/false")),
        }
    }
    fn nums(&self, k: &str) -> Vec<f32> {
        match self {
            Val::List(v) => v.iter().map(|x| x.num(k)).collect(),
            _ => die(&format!("{k}: expected a list of numbers")),
        }
    }
    fn vec3(&self, k: &str) -> [f32; 3] {
        let v = self.nums(k);
        if v.len() != 3 {
            die(&format!("{k}: expected 3 numbers"));
        }
        [v[0], v[1], v[2]]
    }
    fn rows(&self, k: &str, width: usize) -> Vec<Vec<f32>> {
        match self {
            Val::List(v) => v
                .iter()
                .map(|r| {
                    let r = r.nums(k);
                    if r.len() != width {
                        die(&format!("{k}: each row needs {width} numbers"));
                    }
                    r
                })
                .collect(),
            _ => die(&format!("{k}: expected a list of rows")),
        }
    }
}

fn die(msg: &str) -> ! {
    eprintln!("sim_eval: {msg}");
    std::process::exit(2);
}

fn parse_val(s: &str) -> (Val, &str) {
    let s = s.trim_start();
    if let Some(rest) = s.strip_prefix('[') {
        let mut items = Vec::new();
        let mut rest = rest.trim_start();
        loop {
            if let Some(r) = rest.strip_prefix(']') {
                return (Val::List(items), r);
            }
            let (v, r) = parse_val(rest);
            items.push(v);
            rest = r.trim_start();
            if let Some(r) = rest.strip_prefix(',') {
                rest = r.trim_start();
            }
        }
    }
    if let Some(rest) = s.strip_prefix('"') {
        let end = rest.find('"').unwrap_or_else(|| die("unterminated string"));
        return (Val::Str(rest[..end].to_string()), &rest[end + 1..]);
    }
    let end = s.find(|c: char| c == ',' || c == ']' || c.is_whitespace()).unwrap_or(s.len());
    let tok = &s[..end];
    let v = match tok {
        "true" => Val::Bool(true),
        "false" => Val::Bool(false),
        _ => Val::Num(tok.parse().unwrap_or_else(|_| die(&format!("bad value '{tok}'")))),
    };
    (v, &s[end..])
}

/// `key = value` lines, `#` comments. A TOML subset: numbers, booleans,
/// strings, and (nested) lists of numbers.
fn parse_file(path: &str) -> BTreeMap<String, Val> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| die(&format!("{path}: {e}")));
    let mut m = BTreeMap::new();
    for (n, line) in text.lines().enumerate() {
        let line = match line.find('#') {
            Some(i) if !line[..i].contains('"') => &line[..i],
            _ => line,
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let (k, v) = line
            .split_once('=')
            .unwrap_or_else(|| die(&format!("{path}:{}: expected key = value", n + 1)));
        let (val, rest) = parse_val(v);
        if !rest.trim().is_empty() {
            die(&format!("{path}:{}: trailing text '{}'", n + 1, rest.trim()));
        }
        m.insert(k.trim().to_string(), val);
    }
    m
}

/// Estimator overrides. Shared by scenario files and `--params` files.
fn apply_param(p: &mut EstParams, k: &str, v: &Val) -> bool {
    let x = || v.num(k);
    match k {
        "mekf.sigma_g" => p.mekf.sigma_g = x(),
        "mekf.sigma_bw" => p.mekf.sigma_bw = x(),
        "mekf.sigma_a" => p.mekf.sigma_a = x(),
        "mekf.accel_gate" => p.mekf.accel_gate = x(),
        "mekf.init_sigma_bias" => p.mekf.init_sigma_bias = x(),
        "mekf.accel_nis_gate" => p.mekf.accel_nis_gate = x(),
        "mekf.accel_nis_release" => p.mekf.accel_nis_release = x() as u32,
        "poskf.sigma_a" => p.poskf.sigma_a = x(),
        "poskf.sigma_gps_h" => p.poskf.sigma_gps_h = x(),
        "poskf.sigma_gps_v" => p.poskf.sigma_gps_v = x(),
        "poskf.sigma_baro" => p.poskf.sigma_baro = x(),
        "gps_accel.tau_s" => p.gps_accel_tau_s = x(),
        "imu.gyro_fc_hz" => p.imu_filter.gyro_fc_hz = x(),
        "imu.accel_fc_hz" => p.imu_filter.accel_fc_hz = x(),
        "est.accel_window_mean" => p.accel_window_mean = v.boolean(k),
        "est.accel_window_mean_kf" => p.accel_window_mean_kf = v.boolean(k),
        "est.clip_skip_update" => p.clip_skip_update = v.boolean(k),
        "est.clip_kf_coast" => p.clip_kf_coast = v.boolean(k),
        "poskf.coast_sigma_a" => p.poskf.coast_sigma_a = x(),
        "est.sigma_a_uncompensated" => p.sigma_a_uncompensated = x(),
        "est.lost_zero_vel_sigma" => p.lost_zero_vel_sigma = x(),
        "est.sf_invariant_check" => p.sf_invariant_check = v.boolean(k),
        "est.sf_invariant_persist" => p.sf_invariant_persist = x() as u32,
        "est.clip_holdoff_windows" => p.clip_holdoff_windows = x() as u32,
        "est.accel_comp_delay_windows" => p.accel_comp_delay_windows = x() as u32,
        "poskf.gps_vel_sigma" => p.gps_vel_sigma = x(),
        "est.bias_watchdog_dps" => p.bias_watchdog_dps = x(),
        "est.r_k_norm" => p.r_k_norm = x(),
        "est.r_k_rate" => p.r_k_rate = x(),
        "est.r_k_gacc" => p.r_k_gacc = x(),
        "est.thrust_aid_sigma" => p.thrust_aid_sigma = x(),
        "est.thrust_aid_always_sigma" => p.thrust_aid_always_sigma = x(),
        _ => return false,
    }
    true
}

fn channel(f: &mut ChannelFault, field: &str, k: &str, v: &Val) -> bool {
    match field {
        "sigma" => f.sigma = v.num(k),
        "bias" => f.bias = v.vec3(k),
        "vib_amplitude" => f.vib_amplitude = v.num(k),
        "vib_hz" => f.vib_hz = v.num(k),
        "p_online" => f.p_online = v.num(k),
        "dropout_dwell_s" => f.dropout_dwell_s = v.num(k),
        _ => return false,
    }
    true
}

#[derive(Debug, Clone, Copy)]
struct PassRule {
    max_rmse_attitude: f32,
    max_rmse_altitude: f32,
    max_overshoot: f32,
    max_divergence: u32,
}

struct Scenario {
    name: String,
    seeds: Vec<u64>,
    sc: EstScenario,
    params: EstParams,
    tun: Tunables,
    pass: PassRule,
}

fn load_scenario(path: &str, overrides: &BTreeMap<String, Val>) -> Scenario {
    let m = parse_file(path);
    let stem = std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut s = Scenario {
        name: stem,
        seeds: vec![1, 2, 3, 4],
        sc: EstScenario::nominal(),
        params: EstParams::firmware(),
        tun: Tunables::firmware(),
        pass: PassRule {
            max_rmse_attitude: f32::INFINITY,
            max_rmse_altitude: f32::INFINITY,
            max_overshoot: f32::INFINITY,
            max_divergence: 0,
        },
    };
    let mut deg = Degradation::none();
    for (k, v) in &m {
        let k = k.as_str();
        let sc = &mut s.sc;
        match k {
            "name" => match v {
                Val::Str(n) => s.name = n.clone(),
                _ => die("name: expected a string"),
            },
            "seeds" => s.seeds = v.nums(k).iter().map(|&x| x as u64).collect(),
            "total_s" => sc.total_s = v.num(k),
            "start_alt" => sc.start_alt = v.num(k),
            "wind_ned" => sc.plant.wind_ned = v.vec3(k),
            "pos_hold" => sc.pos_hold = v.boolean(k),
            "gps_outage_s" => {
                let o = v.nums(k);
                sc.gps_outage_s = (o[0], o[1]);
            }
            "accel_full_scale_g" => sc.accel_full_scale_g = v.num(k),
            "est_accel_sign" => sc.est_accel_sign = v.vec3(k),
            "est_gyro_sign" => sc.est_gyro_sign = v.vec3(k),
            "motor_scale" => {
                let x = v.nums(k);
                deg.motor_scale = [x[0], x[1], x[2], x[3]];
            }
            "alt_steps" => {
                let r = v.rows(k, 2);
                if r.len() > est_eval::MAX_ALT_STEPS {
                    die("alt_steps: too many");
                }
                for (i, row) in r.iter().enumerate() {
                    sc.alt_steps[i] = (row[0], row[1]);
                }
                sc.n_alt_steps = r.len();
            }
            "att_steps" => {
                let r = v.rows(k, 3);
                if r.len() > est_eval::MAX_ATT_STEPS {
                    die("att_steps: too many");
                }
                for (i, row) in r.iter().enumerate() {
                    sc.att_steps[i] = (row[0], row[1], row[2]);
                }
                sc.n_att_steps = r.len();
            }
            "gps.sigma_h" => sc.gps_sigma_h = v.num(k),
            "gps.sigma_v" => sc.gps_sigma_v = v.num(k),
            "gps.sigma_vel_h" => sc.gps_sigma_vel_h = v.num(k),
            "gps.sigma_vel_v" => sc.gps_sigma_vel_v = v.num(k),
            "baro.sigma" => sc.baro_sigma = v.num(k),
            "baro.drift_sigma" => sc.baro_drift_sigma = v.num(k),
            "baro.drift_tau_s" => sc.baro_drift_tau_s = v.num(k),
            // The controller is part of the exam, not a tunable here.
            "ctrl.rate_kp" => s.tun.rate.kp = v.num(k),
            "ctrl.rate_ki" => s.tun.rate.ki = v.num(k),
            "ctrl.rate_kd" => s.tun.rate.kd = v.num(k),
            "pass.max_rmse_attitude" => s.pass.max_rmse_attitude = v.num(k),
            "pass.max_rmse_altitude" => s.pass.max_rmse_altitude = v.num(k),
            "pass.max_overshoot" => s.pass.max_overshoot = v.num(k),
            "pass.max_divergence" => s.pass.max_divergence = v.num(k) as u32,
            _ => {
                let handled = if let Some(f) = k.strip_prefix("gyro.") {
                    channel(&mut deg.gyro, f, k, v)
                } else if let Some(f) = k.strip_prefix("accel.") {
                    channel(&mut deg.accel, f, k, v)
                } else {
                    apply_param(&mut s.params, k, v)
                };
                if !handled {
                    die(&format!("{path}: unknown key '{k}'"));
                }
            }
        }
    }
    s.sc.deg = deg;
    for (k, v) in overrides {
        if !apply_param(&mut s.params, k, v) {
            die(&format!("--params: unknown estimator key '{k}'"));
        }
    }
    s
}

// ---------------------------------------------------------------- scoring

#[derive(Debug, Clone, Copy)]
struct Score {
    rmse_attitude: Option<f64>,
    rmse_altitude: Option<f64>,
    max_overshoot: Option<f64>,
    divergence_count: u32,
    pass: bool,
}

fn score(s: &Scenario, runs: &[EstRun]) -> Score {
    let (mut tsq, mut asq, mut n) = (0.0f64, 0.0f64, 0u64);
    let mut ovs: Option<f64> = None;
    let mut div = 0u32;
    for r in runs {
        if r.failed.is_some() || r.est_diverged {
            div += 1;
        }
        if r.failed.is_none() {
            tsq += r.tilt_sq;
            asq += r.alt_sq;
            n += r.samples;
            ovs = Some(ovs.unwrap_or(0.0).max(r.max_overshoot as f64));
        }
    }
    let rm = |sq: f64| (n > 0).then(|| (sq / n as f64).sqrt());
    let (ra, rl) = (rm(tsq), rm(asq));
    let within = |v: Option<f64>, lim: f32| v.is_some_and(|x| x <= lim as f64);
    let pass = div <= s.pass.max_divergence
        && within(ra, s.pass.max_rmse_attitude)
        && within(rl, s.pass.max_rmse_altitude)
        && within(ovs, s.pass.max_overshoot);
    Score { rmse_attitude: ra, rmse_altitude: rl, max_overshoot: ovs, divergence_count: div, pass }
}

fn run_scenario(s: &Scenario, detail: bool) -> Score {
    let tun = s.tun;
    let runs: Vec<EstRun> = std::thread::scope(|scope| {
        let hs: Vec<_> = s
            .seeds
            .iter()
            .map(|&seed| scope.spawn(move || est_eval::run(&s.sc, &s.params, &tun, seed)))
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    if detail {
        for (seed, r) in s.seeds.iter().zip(&runs) {
            eprintln!(
                "{:<22} seed {seed}: fail={:?} est_div={} tilt_max={:.2}deg alt_max={:.2}m \
                 horiz_max={:.2}m clipped={} implausible={}/{} gps_dropped={} acc_upd={}/{} rail={:.1}% bias_err={:.3}dps resets={}",
                s.name,
                r.failed,
                r.est_diverged,
                r.tilt_err_max_deg,
                r.alt_err_max_m,
                r.horiz_err_max_m,
                r.accel_clipped,
                r.accel_implausible,
                r.accel_implausible_unclipped,
                r.gps_fixes_dropped,
                r.accel_updates_applied,
                r.accel_updates_applied + r.accel_updates_rejected,
                100.0 * r.rail_steps as f64 / r.steps.max(1) as f64,
                r.bias_err_dps,
                r.mekf_resets,
            );
        }
    }
    score(s, &runs)
}

fn fmt_opt(v: Option<f64>) -> String {
    match v {
        Some(x) if x.is_finite() => format!("{x:.6}"),
        _ => "null".to_string(),
    }
}

fn score_json(sc: &Score) -> String {
    format!(
        "{{\"rmse_attitude\": {}, \"rmse_altitude\": {}, \"max_overshoot\": {}, \
         \"divergence_count\": {}, \"pass\": {}}}",
        fmt_opt(sc.rmse_attitude),
        fmt_opt(sc.rmse_altitude),
        fmt_opt(sc.max_overshoot),
        sc.divergence_count,
        sc.pass
    )
}

// ---------------------------------------------------------------- compare
//
// Acceptance rule, fixed before the baseline was taken and not changed after:
// a candidate is ACCEPTED iff
//   - at least one (scenario, metric) IMPROVES on the current best, and
//   - no (scenario, metric) REGRESSES against the current best, and
//   - no (scenario, metric) REGRESSES against the frozen baseline (so
//     sub-threshold drift cannot accumulate across iterations).
// For the three continuous metrics, a change counts only beyond a noise
// floor of max(REL_EPS * |old|, ABS_EPS[metric]). divergence_count is exact;
// pass going true -> false is a regression, false -> true an improvement.
// null (every seed failed) is worse than any number.

const REL_EPS: f64 = 0.01;
const ABS_EPS: [(&str, f64); 3] =
    [("rmse_attitude", 0.002), ("rmse_altitude", 0.002), ("max_overshoot", 0.002)];

fn rules_json() -> String {
    format!(
        "{{\"rel_eps\": {REL_EPS}, \"abs_eps\": {{\"rmse_attitude\": {}, \"rmse_altitude\": {}, \
         \"max_overshoot\": {}}}, \"lower_is_better\": true}}",
        ABS_EPS[0].1, ABS_EPS[1].1, ABS_EPS[2].1
    )
}

/// Minimal reader for the files this program writes.
fn read_suite(path: &str) -> BTreeMap<String, BTreeMap<String, Option<f64>>> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| die(&format!("{path}: {e}")));
    let body = text
        .find("\"scenarios\"")
        .map(|i| &text[i..])
        .unwrap_or_else(|| die(&format!("{path}: no \"scenarios\" object")));
    let mut out = BTreeMap::new();
    let mut rest = &body[body.find('{').unwrap() + 1..];
    while let Some(q) = rest.find('"') {
        let r = &rest[q + 1..];
        let e = r.find('"').unwrap();
        let name = r[..e].to_string();
        let obj_start = r.find('{').unwrap();
        let obj_end = r.find('}').unwrap();
        let obj = &r[obj_start + 1..obj_end];
        let mut fields = BTreeMap::new();
        for kv in obj.split(',') {
            let (k, v) = kv.split_once(':').unwrap();
            let k = k.trim().trim_matches('"').to_string();
            let v = v.trim();
            let x = match v {
                "null" => None,
                "true" => Some(1.0),
                "false" => Some(0.0),
                _ => Some(v.parse().unwrap()),
            };
            fields.insert(k, x);
        }
        out.insert(name, fields);
        rest = &r[obj_end + 1..];
        if rest.trim_start().starts_with('}') {
            break;
        }
    }
    out
}

#[derive(PartialEq)]
enum Delta {
    Better,
    Same,
    Worse,
}

fn delta(metric: &str, old: Option<f64>, new: Option<f64>) -> Delta {
    match metric {
        "pass" => match (old, new) {
            (Some(o), Some(n)) if n > o => Delta::Better,
            (Some(o), Some(n)) if n < o => Delta::Worse,
            _ => Delta::Same,
        },
        "divergence_count" => match (old, new) {
            (Some(o), Some(n)) if n < o => Delta::Better,
            (Some(o), Some(n)) if n > o => Delta::Worse,
            _ => Delta::Same,
        },
        _ => match (old, new) {
            (None, None) => Delta::Same,
            (None, Some(_)) => Delta::Better,
            (Some(_), None) => Delta::Worse,
            (Some(o), Some(n)) => {
                let abs = ABS_EPS.iter().find(|(m, _)| *m == metric).map_or(0.0, |x| x.1);
                let eps = (REL_EPS * o.abs()).max(abs);
                if n < o - eps {
                    Delta::Better
                } else if n > o + eps {
                    Delta::Worse
                } else {
                    Delta::Same
                }
            }
        },
    }
}

fn compare(old: &str, new: &str, base: Option<&str>) -> ExitCode {
    let o = read_suite(old);
    let n = read_suite(new);
    let b = base.map(read_suite);
    let mut improved = Vec::new();
    let mut regressed = Vec::new();
    for (name, nf) in &n {
        let Some(of) = o.get(name) else { die(&format!("{old}: no scenario {name}")) };
        for (metric, nv) in nf {
            let ov = of.get(metric).copied().flatten();
            let show = |v: Option<f64>| v.map_or("null".into(), |x| format!("{x:.4}"));
            match delta(metric, ov, *nv) {
                Delta::Better => improved.push(format!("{name}.{metric} {} -> {}", show(ov), show(*nv))),
                Delta::Worse => regressed.push(format!("{name}.{metric} {} -> {}", show(ov), show(*nv))),
                Delta::Same => {}
            }
            if let Some(bf) = b.as_ref().and_then(|b| b.get(name)) {
                let bv = bf.get(metric).copied().flatten();
                if delta(metric, bv, *nv) == Delta::Worse {
                    regressed.push(format!("{name}.{metric} vs BASELINE {} -> {}", show(bv), show(*nv)));
                }
            }
        }
    }
    for i in &improved {
        println!("  improved   {i}");
    }
    for r in &regressed {
        println!("  REGRESSED  {r}");
    }
    let ok = !improved.is_empty() && regressed.is_empty();
    println!("{}", if ok { "ACCEPT" } else { "REJECT" });
    if ok { ExitCode::SUCCESS } else { ExitCode::from(1) }
}

// ---------------------------------------------------------------- main

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut detail = false;
    let mut trace = false;
    let mut suite: Option<String> = None;
    let mut params: Option<String> = None;
    let mut cmp: Option<(String, String)> = None;
    let mut base: Option<String> = None;
    let mut config: Option<String> = None;
    let mut i = 0;
    let next = |i: &mut usize| -> String {
        *i += 1;
        args.get(*i).cloned().unwrap_or_else(|| die("missing argument"))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--detail" => detail = true,
            "--trace" => trace = true,
            "--suite" => suite = Some(next(&mut i)),
            "--params" => params = Some(next(&mut i)),
            "--baseline" => base = Some(next(&mut i)),
            "--compare" => {
                let a = next(&mut i);
                let b = next(&mut i);
                cmp = Some((a, b));
            }
            "-h" | "--help" => {
                println!("usage: sim_eval <config.toml> | --suite <dir> [--params <file>] [--detail]");
                println!("       sim_eval --compare <old.json> <new.json> [--baseline <base.json>]");
                return ExitCode::SUCCESS;
            }
            a if !a.starts_with("--") && config.is_none() => config = Some(a.to_string()),
            a => die(&format!("unexpected argument '{a}'")),
        }
        i += 1;
    }
    if let Some((a, b)) = cmp {
        return compare(&a, &b, base.as_deref());
    }
    let overrides = params.as_deref().map(parse_file).unwrap_or_default();
    if let Some(dir) = suite {
        let mut paths: Vec<_> = std::fs::read_dir(&dir)
            .unwrap_or_else(|e| die(&format!("{dir}: {e}")))
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|x| x == "toml"))
            .collect();
        paths.sort();
        let scenarios: Vec<Scenario> =
            paths.iter().map(|p| load_scenario(&p.to_string_lossy(), &overrides)).collect();
        let scores: Vec<Score> = std::thread::scope(|scope| {
            let hs: Vec<_> =
                scenarios.iter().map(|s| scope.spawn(move || run_scenario(s, detail))).collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        println!("{{\n  \"rules\": {},\n  \"scenarios\": {{", rules_json());
        for (k, (s, sc)) in scenarios.iter().zip(&scores).enumerate() {
            let comma = if k + 1 < scenarios.len() { "," } else { "" };
            println!("    \"{}\": {}{comma}", s.name, score_json(sc));
        }
        println!("  }}\n}}");
        let all = scores.iter().all(|s| s.pass);
        return if all { ExitCode::SUCCESS } else { ExitCode::from(1) };
    }
    let Some(cfg) = config else { die("no config file (see --help)") };
    let s = load_scenario(&cfg, &overrides);
    if trace {
        // CSV of the first seed to stderr, every 10th outer tick (10 Hz).
        eprintln!("t,tilt_err,roll,pitch,roll_est,pitch_est,n,e,n_est,e_est,vn,ve,vn_est,ve_est,gacc_n,gacc_e,acc_n,acc_e,alt_err,upd");
        let mut k = 0u32;
        est_eval::run_traced(&s.sc, &s.params, &s.tun, s.seeds[0], &mut |r| {
            k += 1;
            if k % 10 == 1 {
                eprintln!(
                    "{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{}",
                    r.t, r.tilt_err_deg, r.roll_true, r.pitch_true, r.roll_est, r.pitch_est,
                    r.pos_true[0], r.pos_true[1], r.pos_est[0], r.pos_est[1],
                    r.vel_true[0], r.vel_true[1], r.vel_est[0], r.vel_est[1],
                    r.gps_accel[0], r.gps_accel[1], r.accel_true[0], r.accel_true[1],
                    r.alt_err, r.accel_update_applied as u8
                );
            }
        });
    }
    let sc = run_scenario(&s, detail);
    println!("{}", score_json(&sc));
    if sc.pass { ExitCode::SUCCESS } else { ExitCode::from(1) }
}
