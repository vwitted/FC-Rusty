//! Fit plant parameters from a captured step response.
//!
//! What this identifies, and what it deliberately does not.
//!
//! `motor_tau` is the one parameter a BENCH capture can give you, and it
//! is the one the sim is most uncertain about. Command a throttle step,
//! watch what the motors actually do through bidirectional DShot
//! telemetry, fit a first-order lag. No flight, no scale, no rig.
//!
//! `max_thrust`, `inertia` and `drag_k` cannot come from this. Thrust
//! needs a load cell or a hover throttle; inertia needs the airframe free
//! to rotate; drag needs airspeed. Those are flight measurements and want
//! the flash log, not the wire.
//!
//! # The domain the fit runs in
//!
//! `QuadSim` lags ROTOR SPEED with `motor_tau` and squares it for thrust
//! (`QuadParams::thrust_frac`). So `motor_tau` is the motor's own
//! mechanical time constant and `tau_omega_s` below is exactly it: one
//! number, direction-independent, straight into the sim.
//!
//! It was not always that simple, and the history is the reason both time
//! constants are still reported.
//!
//! The sim used to lag THRUST and map it linearly. Thrust goes as the
//! square of rotor speed, so a first-order model of thrust has no single
//! time constant -- fitting one gives a different answer depending on
//! which way the step went. Measured on synthetic data with a known
//! tau_omega of 30 ms:
//!
//!     step up    tau_thrust ~= 33 ms   (omega 63% complete at t=tau,
//!     step down  tau_thrust ~= 21 ms    thrust only 55%)
//!
//! while tau_omega came back within 0.2 ms in every single row. That
//! asymmetry was the model showing through as a number that would not sit
//! still, and it forced a conservative choice: report the accelerating
//! figure, because a rate loop arrests a rotation by ADDING thrust.
//!
//! Squaring in the sim removed the need for that choice. `tau_thrust_s`
//! is kept because the RATIO between the two is a cheap check that the
//! capture is sane -- it should be near 1.1-1.2 on a step up and below 1
//! on a step down, and if it is not, something about the data is wrong.

use crate::plant_log::PlantSample;

/// Result of fitting one motor across one commanded step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepFit {
    /// Time constant of the THRUST proxy, seconds. A diagnostic, not a
    /// parameter: it is not the same number on a step up as on a step
    /// down, because thrust is not first-order. See the module docs.
    pub tau_thrust_s: f32,
    /// Time constant of rotational speed, seconds. THIS is
    /// `QuadParams::motor_tau` -- the sim lags rotor speed and squares
    /// it, so this is the physical constant it wants.
    pub tau_omega_s: f32,
    /// Steady eRPM before and after the step.
    pub erpm_from: f32,
    pub erpm_to: f32,
    /// Normalised command before and after.
    pub cmd_from: f32,
    pub cmd_to: f32,
    /// Root-mean-square fit residual, as a fraction of the step size.
    /// Large means the response is not first-order (or the capture is
    /// noisy), and the tau should not be trusted.
    pub residual_frac: f32,
    /// Samples used.
    pub n: usize,
}

/// A detected commanded step for one motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    pub motor: usize,
    /// Index of the first sample at the new command.
    pub start: usize,
    /// One past the last sample considered part of the response.
    pub end: usize,
    pub cmd_from: f32,
    pub cmd_to: f32,
}

/// Smallest command change treated as a step, normalised.
///
/// Below this the response is buried in telemetry quantisation: the
/// eRPM period is an integer number of microseconds, so at ~500 us
/// periods one count is already 0.2% of speed.
pub const MIN_STEP: f32 = 0.02;

/// Find commanded steps for one motor.
///
/// A step ends when the command changes again, so a profile of holds
/// yields one window per hold. `max_window_ms` bounds it further, because
/// only the first few time constants carry information about tau and a
/// long tail lets slow drift (heating, battery sag) dominate the fit.
/// Writes into `out` and returns how many were found. Caller-provided
/// buffer because this crate is `no_std` on the host as well as on the
/// target, so there is no allocator to return a Vec from.
pub fn find_steps(
    samples: &[PlantSample],
    motor: usize,
    max_window_ms: u16,
    out: &mut [Step],
) -> usize {
    let mut n = 0usize;
    let mut i = 1;
    while i < samples.len() && n < out.len() {
        let prev = samples[i - 1].cmd_f32(motor);
        let now = samples[i].cmd_f32(motor);
        if (now - prev).abs() >= MIN_STEP {
            let t0 = samples[i].t_ms;
            let mut j = i + 1;
            while j < samples.len()
                && (samples[j].cmd_f32(motor) - now).abs() < MIN_STEP
                && samples[j].t_ms.wrapping_sub(t0) <= max_window_ms as u32
            {
                j += 1;
            }
            out[n] = Step { motor, start: i, end: j, cmd_from: prev, cmd_to: now };
            n += 1;
            i = j;
        } else {
            i += 1;
        }
    }
    n
}

/// Fit a first-order lag to one step.
///
/// Returns `None` if the window is too short, carries too little
/// telemetry, or the motors did not actually move.
pub fn fit_step(samples: &[PlantSample], step: Step) -> Option<StepFit> {
    let win = &samples[step.start..step.end];
    if win.len() < 8 {
        return None;
    }

    // Collect (t seconds since step, omega) for samples that have
    // telemetry. Dropped frames are simply absent rather than
    // interpolated: a fit over irregular samples is fine, an invented
    // point is not.
    let t0 = win[0].t_ms;
    let mut ts = [0.0f32; MAX_WINDOW];
    let mut om = [0.0f32; MAX_WINDOW];
    let mut n = 0usize;
    for s in win.iter() {
        if n == MAX_WINDOW {
            break;
        }
        if let Some(e) = s.erpm(step.motor) {
            ts[n] = s.t_ms.wrapping_sub(t0) as f32 * 1e-3;
            om[n] = e;
            n += 1;
        }
    }
    if n < 8 {
        return None;
    }

    // Endpoints. The start value is the LAST steady sample before the
    // step, taken from before the window, because the first in-window
    // sample has already begun to move.
    let om_from = if step.start > 0 {
        samples[step.start - 1].erpm(step.motor).unwrap_or(om[0])
    } else {
        om[0]
    };
    // The end value is the mean of the last eighth of the window rather
    // than its final sample, so one noisy point cannot set the asymptote
    // that every residual is measured against.
    let tail = (n / 8).max(1);
    let om_to = om[n - tail..n].iter().sum::<f32>() / tail as f32;

    if (om_to - om_from).abs() < 1.0 {
        return None; // motors did not move; nothing to fit
    }

    let tau_omega = fit_first_order(&ts[..n], &om[..n], om_from, om_to)?;

    // Thrust proxy. omega^2 up to a constant, and the constant divides
    // out of a normalised fit, so no thrust coefficient is needed.
    let mut th = [0.0f32; MAX_WINDOW];
    for i in 0..n {
        th[i] = om[i] * om[i];
    }
    let th_from = om_from * om_from;
    let th_to = th[n - tail..n].iter().sum::<f32>() / tail as f32;
    let tau_thrust = fit_first_order(&ts[..n], &th[..n], th_from, th_to)?;

    let residual = rms_residual(&ts[..n], &th[..n], th_from, th_to, tau_thrust)
        / (th_to - th_from).abs();

    Some(StepFit {
        tau_thrust_s: tau_thrust,
        tau_omega_s: tau_omega,
        erpm_from: om_from,
        erpm_to: om_to,
        cmd_from: step.cmd_from,
        cmd_to: step.cmd_to,
        residual_frac: residual,
        n,
    })
}

/// Longest step window the fitter will consider, in samples.
pub const MAX_WINDOW: usize = 2048;

/// Search bounds on tau, seconds. A quad motor lag outside 1 ms..1 s is
/// not a lag, it is a broken capture.
const TAU_MIN: f32 = 0.001;
const TAU_MAX: f32 = 1.0;

/// Fit tau in `y(t) = y_inf + (y_0 - y_inf) exp(-t/tau)` by minimising
/// squared error.
///
/// Golden-section search rather than the usual log-linearisation.
/// Linearising -- taking log of the normalised residual and doing least
/// squares on it -- is the standard trick and it is wrong here: it
/// weights points by 1/residual, so the noisiest samples, the ones near
/// the asymptote where the residual is small and the telemetry
/// quantisation is the same absolute size, dominate the answer. It also
/// cannot use any sample that overshoots the asymptote, which near the
/// end is half of them. Direct search costs a few hundred evaluations of
/// a handful of points and has neither problem.
///
/// The objective is unimodal in tau for a genuine first-order response,
/// which is what makes golden-section valid; `residual_frac` is reported
/// so a response that is NOT first-order shows up rather than silently
/// returning the least-bad tau.
fn fit_first_order(ts: &[f32], ys: &[f32], y0: f32, y_inf: f32) -> Option<f32> {
    if ts.len() < 2 {
        return None;
    }
    let sse = |tau: f32| sse_for(ts, ys, y0, y_inf, tau);

    // Golden-section on [TAU_MIN, TAU_MAX].
    let phi_inv = 0.618_034_f32;
    let (mut a, mut b) = (TAU_MIN, TAU_MAX);
    let mut c = b - phi_inv * (b - a);
    let mut d = a + phi_inv * (b - a);
    let (mut fc, mut fd) = (sse(c), sse(d));
    // 60 iterations shrinks the bracket by 0.618^60, far below any
    // meaningful resolution in tau.
    for _ in 0..60 {
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - phi_inv * (b - a);
            fc = sse(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + phi_inv * (b - a);
            fd = sse(d);
        }
    }
    let tau = 0.5 * (a + b);
    if tau.is_finite() { Some(tau) } else { None }
}

fn sse_for(ts: &[f32], ys: &[f32], y0: f32, y_inf: f32, tau: f32) -> f32 {
    let mut acc = 0.0;
    for (t, y) in ts.iter().zip(ys.iter()) {
        let pred = y_inf + (y0 - y_inf) * libm::expf(-t / tau);
        let e = y - pred;
        acc += e * e;
    }
    acc
}

fn rms_residual(ts: &[f32], ys: &[f32], y0: f32, y_inf: f32, tau: f32) -> f32 {
    libm::sqrtf(sse_for(ts, ys, y0, y_inf, tau) / ts.len() as f32)
}

/// Mean and spread of a set of per-step tau estimates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TauSummary {
    pub mean_s: f32,
    pub min_s: f32,
    pub max_s: f32,
    pub n: usize,
}

/// Which steps to include in a summary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    /// Accelerating. The direction that limits how fast the controller
    /// can ADD thrust to arrest a rotation, so the conservative one.
    Up,
    /// Decelerating.
    Down,
    Both,
}

impl Dir {
    fn matches(self, f: &StepFit) -> bool {
        match self {
            Self::Up => f.cmd_to > f.cmd_from,
            Self::Down => f.cmd_to < f.cmd_from,
            Self::Both => true,
        }
    }
}

/// Summarise a chosen time constant across steps, ignoring fits whose
/// residual says the response was not first-order.
///
/// `max_residual_frac` of 0.05 means "the first-order curve explains the
/// step to within 5% of its size". A capture with a bad ESC, a stuck
/// motor or a mistimed profile fails this rather than quietly widening
/// the mean.
///
/// `sel` picks the field, and `dir` the step direction. Both exist
/// because of something the synthetic-data end-to-end run showed and I
/// had not predicted: `tau_thrust_s` is systematically LONGER on steps up
/// than on steps down (30 ms vs 21 ms for the same motor), while
/// `tau_omega_s` is the same to a tenth of a millisecond in every row.
///
/// That is not noise, it is the model. Rotor speed genuinely is
/// first-order, so its time constant is a property of the motor. Thrust
/// is speed squared, which is NOT first-order, so "the first-order time
/// constant of thrust" is not a constant at all -- it depends on where
/// the step starts and which way it goes. Averaging the two directions
/// gives a number close to `tau_omega_s` purely because this profile is
/// symmetric, which is an artefact of the profile and not a measurement.
pub fn summarise_by(
    fits: &[StepFit],
    max_residual_frac: f32,
    dir: Dir,
    sel: fn(&StepFit) -> f32,
) -> Option<TauSummary> {
    let mut sum = 0.0;
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    let mut n = 0usize;
    for f in fits
        .iter()
        .filter(|f| f.residual_frac <= max_residual_frac && dir.matches(f))
    {
        let v = sel(f);
        sum += v;
        min = min.min(v);
        max = max.max(v);
        n += 1;
    }
    if n == 0 {
        return None;
    }
    Some(TauSummary { mean_s: sum / n as f32, min_s: min, max_s: max, n })
}

/// The value to put in `QuadParams::motor_tau`: the rotor-speed time
/// constant, over steps in both directions.
///
/// Both directions because it is the same number in both -- that is what
/// makes it the right one. This used to return the accelerating THRUST
/// constant, which was the conservative pick among several answers that
/// should have been one answer. The sim now squares rotor speed rather
/// than lagging thrust, so the ambiguity is gone and the honest thing is
/// the physical constant.
pub fn summarise(fits: &[StepFit], max_residual_frac: f32) -> Option<TauSummary> {
    summarise_by(fits, max_residual_frac, Dir::Both, |f| f.tau_omega_s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plant_log::CMD_SCALE;

    /// Synthesise a capture: omega follows a first-order lag with
    /// `tau_omega`, sampled at `hz`, with a step at `step_ms`.
    fn synth(
        tau_omega: f32,
        hz: f32,
        hold_ms: u16,
        e_from: f32,
        e_to: f32,
        quantise: bool,
    ) -> std::vec::Vec<PlantSample> {
        let dt_ms = (1000.0 / hz) as u32;
        let mut out = std::vec::Vec::new();
        let mut t: u32 = 0;
        // Steady before the step.
        while t < hold_ms as u32 {
            out.push(mk(t, 0.10, e_from, quantise));
            t += dt_ms;
        }
        let t_step = t;
        while t < hold_ms as u32 * 2 {
            let dt = (t - t_step) as f32 * 1e-3;
            let e = e_to + (e_from - e_to) * (-dt / tau_omega).exp();
            out.push(mk(t, 0.20, e, quantise));
            t += dt_ms;
        }
        out
    }

    fn mk(t_ms: u32, cmd: f32, erpm: f32, quantise: bool) -> PlantSample {
        let period = 60_000_000.0 / erpm;
        let p = if quantise { period as u16 } else { period.round() as u16 };
        PlantSample {
            t_ms,
            cmd: [(cmd * CMD_SCALE) as u16; 4],
            period_us: [p; 4],
            gyro_dps10: [0; 3],
        }
    }

    #[test]
    fn recovers_a_known_omega_time_constant() {
        let s = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, false);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n = find_steps(&s, 0, 400, &mut buf);
        assert_eq!(n, 1, "expected exactly one step");
        let steps = &buf[..n];
        let f = fit_step(&s, steps[0]).unwrap();
        assert!(
            (f.tau_omega_s - 0.030).abs() < 0.003,
            "tau_omega {} should be ~0.030",
            f.tau_omega_s
        );
    }

    /// The load-bearing one, and the assertion this test originally made
    /// was backwards. On a step UP, thrust LAGS speed: at t = tau_omega
    /// omega is 63% complete and omega^2 only 55%, because omega is at
    /// its smallest exactly where the response is being established.
    ///
    /// So reporting tau_omega as `motor_tau` gives the sim motors faster
    /// than the real ones -- an optimistic stability margin, which is the
    /// direction that gets an aircraft broken rather than merely
    /// mis-modelled.
    #[test]
    fn thrust_lags_speed_so_motor_tau_is_the_longer_constant() {
        let s = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, false);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n_steps];
        let f = fit_step(&s, steps[0]).unwrap();
        assert!(
            f.tau_thrust_s > f.tau_omega_s,
            "thrust tau {} must EXCEED omega tau {} on a step up",
            f.tau_thrust_s,
            f.tau_omega_s
        );
        // And by enough to matter: if they were within a few percent the
        // distinction would be pedantry rather than a correction.
        let ratio = f.tau_thrust_s / f.tau_omega_s;
        assert!(
            (1.05..1.40).contains(&ratio),
            "ratio {ratio} outside the range a squared first-order response gives"
        );
    }

    /// A step DOWN reverses it, which is why the fit must run on the
    /// thrust proxy rather than applying a fixed correction factor to a
    /// speed fit. Coming down, omega is at its largest where the response
    /// starts, so omega^2 moves sooner.
    #[test]
    fn on_a_step_down_the_relationship_inverts() {
        let s = synth(0.030, 500.0, 400, 40_000.0, 20_000.0, false);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let f = fit_step(&s, buf[..n_steps][0]).unwrap();
        assert!(
            f.tau_thrust_s < f.tau_omega_s,
            "on a step down thrust tau {} should be shorter than omega tau {}",
            f.tau_thrust_s,
            f.tau_omega_s
        );
    }

    #[test]
    fn survives_telemetry_quantisation() {
        // Real periods are whole microseconds. At ~1500 us that is coarse
        // enough to matter, and a fit that only works on exact data is a
        // fit that will not work at all.
        let s = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, true);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n_steps];
        let f = fit_step(&s, steps[0]).unwrap();
        assert!((f.tau_omega_s - 0.030).abs() < 0.004, "tau {}", f.tau_omega_s);
        assert!(f.residual_frac < 0.05, "residual {}", f.residual_frac);
    }

    #[test]
    fn a_fast_motor_and_a_slow_motor_are_told_apart() {
        // The whole point of the measurement: a real number, not a
        // plausible one. 15 ms and 60 ms must not both come back as 30.
        for tau in [0.015f32, 0.060] {
            let s = synth(tau, 500.0, 400, 20_000.0, 40_000.0, true);
            let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n_steps];
            let f = fit_step(&s, steps[0]).unwrap();
            assert!(
                (f.tau_omega_s - tau).abs() < 0.15 * tau,
                "tau {} should be within 15% of {}",
                f.tau_omega_s,
                tau
            );
        }
    }

    #[test]
    fn dropped_telemetry_frames_are_skipped_not_interpolated() {
        let mut s = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, false);
        // Punch out every third frame, as a marginal ESC link would.
        for (i, x) in s.iter_mut().enumerate() {
            if i % 3 == 0 {
                x.period_us = [0; 4];
            }
        }
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n_steps];
        let f = fit_step(&s, steps[0]).unwrap();
        assert!((f.tau_omega_s - 0.030).abs() < 0.004, "tau {}", f.tau_omega_s);
    }

    #[test]
    fn a_motor_that_never_moved_yields_no_fit() {
        // A dead ESC gives a flat line, which a fitter that does not check
        // will happily describe with some tau or other.
        let s = synth(0.030, 500.0, 400, 20_000.0, 20_000.0, false);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n_steps];
        assert!(fit_step(&s, steps[0]).is_none());
    }

    #[test]
    fn a_non_first_order_response_reports_a_large_residual() {
        // A linear ramp is the obvious way for this to be wrong quietly:
        // it has a rise time, so it produces a tau, and only the residual
        // says the model does not fit.
        let mut s = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, false);
        let n = s.len();
        for (i, x) in s.iter_mut().enumerate().skip(n / 2) {
            let frac = (i - n / 2) as f32 / (n / 2) as f32;
            let e = 20_000.0 + 20_000.0 * frac.min(1.0);
            x.period_us = [(60_000_000.0 / e) as u16; 4];
        }
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n_steps = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n_steps];
        let f = fit_step(&s, steps[0]).unwrap();
        assert!(f.residual_frac > 0.03, "residual {} too small", f.residual_frac);
    }

    #[test]
    fn steps_are_found_at_command_changes_only() {
        let s = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, false);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];
        let n = find_steps(&s, 0, 400, &mut buf);
        let steps = &buf[..n];
        assert_eq!(n, 1);
        assert!((steps[0].cmd_from - 0.10).abs() < 1e-3);
        assert!((steps[0].cmd_to - 0.20).abs() < 1e-3);
    }

    /// The invariant the reporting now rests on: omega's time constant
    /// is a property of the motor and does not care which way the step
    /// went, while thrust's does. Found by running the end-to-end tool on
    /// synthetic data, not by reasoning about it.
    #[test]
    fn omega_tau_is_direction_invariant_but_thrust_tau_is_not() {
        let up = synth(0.030, 500.0, 400, 20_000.0, 40_000.0, false);
        let down = synth(0.030, 500.0, 400, 40_000.0, 20_000.0, false);
        let mut buf = [Step { motor: 0, start: 0, end: 0, cmd_from: 0.0, cmd_to: 0.0 }; 8];

        let n = find_steps(&up, 0, 400, &mut buf);
        let fu = fit_step(&up, buf[..n][0]).unwrap();
        let n = find_steps(&down, 0, 400, &mut buf);
        let fd = fit_step(&down, buf[..n][0]).unwrap();

        // Same motor, same tau: omega agrees to within a few percent.
        let d_omega = (fu.tau_omega_s - fd.tau_omega_s).abs() / fu.tau_omega_s;
        assert!(d_omega < 0.05, "omega tau differs by {d_omega} between directions");

        // Thrust does not, and by a lot more than that.
        let d_thrust = (fu.tau_thrust_s - fd.tau_thrust_s).abs() / fu.tau_thrust_s;
        assert!(
            d_thrust > 0.15,
            "thrust tau differs by only {d_thrust}; the asymmetry the reporting \
             is built around would not be real"
        );
    }

    #[test]
    fn the_recommended_tau_is_the_rotor_speed_one_over_both_directions() {
        // summarise() feeds QuadParams::motor_tau, and the sim lags rotor
        // speed. Returning a thrust constant here would put a
        // direction-dependent number into a field that has to be one
        // value.
        let base = StepFit {
            tau_thrust_s: 0.033,
            tau_omega_s: 0.030,
            erpm_from: 1.0,
            erpm_to: 2.0,
            cmd_from: 0.08,
            cmd_to: 0.20,
            residual_frac: 0.01,
            n: 100,
        };
        let down = StepFit { tau_thrust_s: 0.021, cmd_from: 0.20, cmd_to: 0.08, ..base };
        let s = summarise(&[base, down], 0.05).unwrap();
        assert_eq!(s.n, 2, "both directions count for the speed constant");
        assert!((s.mean_s - 0.030).abs() < 1e-6, "got {}", s.mean_s);
        // ...and it must not have picked up the thrust figures, which
        // differ from each other by 50%.
        assert!((s.min_s - s.max_s).abs() < 1e-6, "speed tau should not vary by direction");

        // The direction filters still work, for the diagnostic view.
        let u = summarise_by(&[base, down], 0.05, Dir::Up, |f| f.tau_thrust_s).unwrap();
        let d = summarise_by(&[base, down], 0.05, Dir::Down, |f| f.tau_thrust_s).unwrap();
        assert!((u.mean_s - 0.033).abs() < 1e-6);
        assert!((d.mean_s - 0.021).abs() < 1e-6);
    }

    #[test]
    fn summarise_rejects_bad_fits() {
        let good = StepFit {
            tau_thrust_s: 0.02,
            tau_omega_s: 0.04,
            erpm_from: 1.0,
            erpm_to: 2.0,
            cmd_from: 0.1,
            cmd_to: 0.2,
            residual_frac: 0.01,
            n: 100,
        };
        let bad = StepFit { tau_omega_s: 0.5, residual_frac: 0.4, ..good };
        // Only the residual separates them; summarise reads tau_omega_s.
        let sum = summarise(&[good, bad], 0.05).unwrap();
        assert_eq!(sum.n, 1);
        assert!((sum.mean_s - 0.04).abs() < 1e-6, "got {}", sum.mean_s);
        // And if everything is rejected, say so rather than returning a
        // confident average of nothing.
        assert!(summarise(&[bad], 0.05).is_none());
    }
}
