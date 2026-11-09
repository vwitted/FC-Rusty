// est_eval.rs — closed-loop ESTIMATOR evaluation.
//
// `harness::run_case` exercises the control cascade; with `use_estimator` it
// runs the MEKF, but it closes altitude and position on TRUTH and its
// GPS-acceleration path is a separate implementation rotated by the TRUE
// attitude. Neither is what the firmware flies, so neither can score an
// estimator change. This loop is the firmware's estimator wiring, end to end:
//
//   ICM-42688P model   clip at +/-16 g / +/-2000 dps (i16 full scale), then
//                      the firmware's ImuFilter (gyro 150 Hz, accel 25 Hz)
//   mekf_task          predict every 8 kHz sample on filtered gyro; every
//                      ACCEL_DECIMATION-th sample an accel update on filtered
//                      accel -- skipped if any raw sample in the window
//                      clipped or the window mean violates the multirotor
//                      specific-force invariant for SF_INVARIANT_PERSIST
//                      windows, in which case the PosKF coasts -- compensated by gps_accel::GpsAccelEstimator
//                      through gps_accel::compensate_specific_force_g, which
//                      rotates with the MEKF's OWN attitude, as main.rs does
//   pos_kf_task        100 Hz: gps_accel.tick, predict on R(q_mekf)*f + g,
//                      GPS position + velocity (10 Hz; velocity through the
//                      same low-speed deadband), baro (25 Hz)
//   control            altitude and position loops close on the PosKF; the
//                      MPC and the rate PID close on the MEKF and on
//                      bias-corrected filtered gyro
//
// Kept separate from run_case deliberately: run_case's numbers are the
// committed sim-baseline, and changing its loop would move them.
//
// no_std, like the rest of `sim`: the caller parses configs and prints.

use crate::attitude_mekf::{
    specific_force_implausible, AccelDecimator, AttitudeMekf, MekfParams, ACCEL_DECIMATION, G_MPS2,
};
use crate::control::altitude::AltitudeController;
use crate::control::mixer::{ControlDemand, Mixer, QUAD_X};
use crate::control::mpc::{AttitudeMpc, MpcModel};
use crate::control::pid::RatePidController;
use crate::control::position::{PositionController, PositionGains};
use crate::estimation::{gps_velocity_ned, PosKf, PosKfParams};
use crate::gps_accel::{compensate_specific_force_g, GpsAccelEstimator};
use crate::imu_filter::{Biquad, ImuFilterParams};
use crate::sim::degrade::{Degradation, Degrader};
use crate::sim::harness::{FailCause, Rates, Tunables, FLYAWAY_MARGIN_M};
use crate::sim::sensors::{BaroSim, GpsSim};
use crate::sim::{MotorForces, QuadParams, QuadSim};

use core::f32::consts::PI;

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

/// ICM-42688P full scale as configured in drivers/icm42688.rs: +/-16 g at
/// 2048 LSB/g, +/-2000 dps. An i16 count saturates at 32767.
pub const ACCEL_FULL_SCALE_G: f32 = 32767.0 / 2048.0;
pub const GYRO_FULL_SCALE_DPS: f32 = 2000.0;

/// Estimator-error bounds past which a run counts as an estimator
/// divergence even if the aircraft survived it.
pub const DIVERGED_TILT_DEG: f32 = 30.0;
pub const DIVERGED_ALT_M: f32 = 10.0;
pub const DIVERGED_HORIZ_M: f32 = 50.0;

pub const MAX_ALT_STEPS: usize = 8;
pub const MAX_ATT_STEPS: usize = 32;

/// The estimator's tunables: everything a candidate may change.
#[derive(Debug, Clone, Copy)]
pub struct EstParams {
    pub mekf: MekfParams,
    pub poskf: PosKfParams,
    pub gps_accel_tau_s: f32,
    /// Firmware IMU front-end low-pass cutoffs, Hz (0 = bypass).
    pub imu_filter: ImuFilterParams,
    /// Feed the accel update and the PosKF the window mean
    /// (attitude_mekf::AccelDecimator) rather than the tick's sample.
    pub accel_window_mean: bool,
    /// Skip the MEKF accel update when the window held a clipped sample.
    pub clip_skip_update: bool,
    /// Predict the PosKF with zero acceleration when the window held a
    /// clipped sample: a clipped mean is biased, baro and GPS are not.
    pub clip_kf_coast: bool,
    /// Accel-update σ once the GPS acceleration reference has been fresh
    /// and then gone stale (GPS lost in flight). 0 = mekf.sigma_a always.
    pub sigma_a_uncompensated: f32,
    /// While GPS is lost in flight, fuse a zero-velocity pseudo-measurement
    /// into the PosKF at 10 Hz with this σ, m/s. 0 = off.
    pub lost_zero_vel_sigma: f32,
    /// Treat a window whose mean violates the multirotor specific-force
    /// invariant (attitude_mekf::specific_force_implausible) like a clipped
    /// one: skip the accel update and coast the PosKF.
    pub sf_invariant_check: bool,
    /// Consecutive violating windows before the invariant acts.
    pub sf_invariant_persist: u32,
    /// Windows after one containing a clipped raw sample that are still
    /// treated as clipped: the front-end low-pass carries the clipped bias
    /// forward. 0 = only the window itself.
    pub clip_holdoff_windows: u32,
}

impl EstParams {
    /// What main.rs constructs today.
    pub fn firmware() -> Self {
        Self {
            mekf: MekfParams::default(),
            poskf: PosKfParams::FIRMWARE,
            gps_accel_tau_s: crate::gps_accel::DEFAULT_TAU_S,
            imu_filter: ImuFilterParams::default(),
            accel_window_mean: false,
            clip_skip_update: true,
            clip_kf_coast: true,
            sigma_a_uncompensated: 0.0,
            lost_zero_vel_sigma: 0.0,
            sf_invariant_check: true,
            sf_invariant_persist: crate::attitude_mekf::SF_INVARIANT_PERSIST,
            clip_holdoff_windows: 0,
        }
    }
}

/// One scenario: the exam. Nothing here is a tunable.
#[derive(Debug, Clone, Copy)]
pub struct EstScenario {
    pub total_s: f32,
    pub plant: QuadParams,
    pub start_alt: f32,
    /// Altitude target schedule: (time s, target m). Before the first edge
    /// the target is `start_alt`.
    pub alt_steps: [(f32, f32); MAX_ALT_STEPS],
    pub n_alt_steps: usize,
    /// Attitude command schedule: (time s, roll deg, pitch deg). Ignored
    /// under `pos_hold`, which owns the reference.
    pub att_steps: [(f32, f32, f32); MAX_ATT_STEPS],
    pub n_att_steps: usize,
    /// Hold the origin with the firmware position controller, closed on the
    /// PosKF estimate.
    pub pos_hold: bool,
    /// GPS produces no fixes in [start, end).
    pub gps_outage_s: (f32, f32),
    /// Per-axis clip, g. The firmware's is ACCEL_FULL_SCALE_G.
    pub accel_full_scale_g: f32,
    /// Sign applied to the ESTIMATOR's copy of each sensor axis only. The
    /// rate PID keeps the true sign: a rate-loop inversion is positive
    /// feedback that no estimator can survive, so it is not an estimator
    /// test. This models a mounting/orientation error seen by the MEKF.
    pub est_accel_sign: [f32; 3],
    pub est_gyro_sign: [f32; 3],
    pub deg: Degradation,
    pub gps_sigma_h: f32,
    pub gps_sigma_v: f32,
    pub gps_sigma_vel_h: f32,
    pub gps_sigma_vel_v: f32,
    pub baro_sigma: f32,
    pub baro_drift_sigma: f32,
    pub baro_drift_tau_s: f32,
}

impl EstScenario {
    pub fn nominal() -> Self {
        Self {
            total_s: 20.0,
            plant: QuadParams::default(),
            start_alt: 5.0,
            alt_steps: [(f32::INFINITY, 0.0); MAX_ALT_STEPS],
            n_alt_steps: 0,
            att_steps: [(f32::INFINITY, 0.0, 0.0); MAX_ATT_STEPS],
            n_att_steps: 0,
            pos_hold: true,
            gps_outage_s: (f32::INFINITY, f32::INFINITY),
            accel_full_scale_g: ACCEL_FULL_SCALE_G,
            est_accel_sign: [1.0; 3],
            est_gyro_sign: [1.0; 3],
            deg: Degradation::none(),
            gps_sigma_h: 2.0,
            gps_sigma_v: 5.0,
            gps_sigma_vel_h: 0.05,
            gps_sigma_vel_v: 0.15,
            baro_sigma: 0.2,
            baro_drift_sigma: 0.3,
            baro_drift_tau_s: 60.0,
        }
    }
}

/// One seed's outcome. Sums rather than means, so the caller pools.
#[derive(Debug, Clone, Copy, Default)]
pub struct EstRun {
    pub failed: Option<(f32, FailCause)>,
    /// An estimator error exceeded a DIVERGED_* bound at some point.
    pub est_diverged: bool,
    pub tilt_sq: f64,
    pub alt_sq: f64,
    pub samples: u64,
    pub tilt_err_max_deg: f32,
    pub alt_err_max_m: f32,
    pub horiz_err_max_m: f32,
    /// Largest overshoot past a commanded step, as a fraction of the step.
    pub max_overshoot: f32,
    /// Evidence that the scenario's fault actually fired.
    pub accel_clipped: u64,
    pub gps_fixes_dropped: u64,
    /// Windows failing the specific-force invariant / of those, not clipped.
    pub accel_implausible: u64,
    pub accel_implausible_unclipped: u64,
    pub accel_updates_applied: u64,
    pub accel_updates_rejected: u64,
    pub rail_steps: u64,
    pub steps: u64,
    /// Final gyro-bias estimate error magnitude, deg/s.
    pub bias_err_dps: f32,
}

/// Overshoot bookkeeping for one commanded edge.
struct Edge {
    dir: [f32; 2],
    mag: f32,
    target: [f32; 2],
}

impl Edge {
    fn new(from: [f32; 2], to: [f32; 2]) -> Option<Self> {
        let d = [to[0] - from[0], to[1] - from[1]];
        let mag = libm::sqrtf(d[0] * d[0] + d[1] * d[1]);
        (mag > 1e-6).then(|| Edge { dir: [d[0] / mag, d[1] / mag], mag, target: to })
    }
    fn overshoot(&self, y: [f32; 2]) -> f32 {
        let past = (y[0] - self.target[0]) * self.dir[0] + (y[1] - self.target[1]) * self.dir[1];
        (past / self.mag).max(0.0)
    }
}

fn quat_rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let r = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q[0], q[1], q[2], q[3],
    ));
    let o = r * nalgebra::Vector3::new(v[0], v[1], v[2]);
    [o.x, o.y, o.z]
}

/// One outer-tick sample for diagnosis. Observability only: tracing does
/// not change the run.
#[derive(Debug, Clone, Copy)]
pub struct TraceRow {
    pub t: f32,
    pub tilt_err_deg: f32,
    pub roll_true: f32,
    pub pitch_true: f32,
    pub roll_est: f32,
    pub pitch_est: f32,
    pub pos_true: [f32; 2],
    pub pos_est: [f32; 2],
    pub vel_true: [f32; 2],
    pub vel_est: [f32; 2],
    pub gps_accel: [f32; 2],
    pub accel_true: [f32; 2],
    pub alt_err: f32,
    pub accel_update_applied: bool,
}

/// Fly one seed of one scenario with the given estimator parameters.
pub fn run(sc: &EstScenario, p: &EstParams, tun: &Tunables, seed: u64) -> EstRun {
    run_traced(sc, p, tun, seed, &mut |_| {})
}

/// `run`, calling `trace` once per outer tick.
pub fn run_traced(
    sc: &EstScenario,
    p: &EstParams,
    tun: &Tunables,
    seed: u64,
    trace: &mut dyn FnMut(TraceRow),
) -> EstRun {
    let r = Rates::FIRMWARE;
    let dt = r.dt;
    let dt_outer = dt * ACCEL_DECIMATION as f32;
    let hover = sc.plant.hover_throttle();
    let mixer: Mixer<4> = Mixer { mix: QUAD_X.mix };

    let mut sim = QuadSim::new_hovering(sc.plant, sc.start_alt);
    let mut deg = Degrader::new(sc.deg, seed);
    let mut gps = GpsSim::with_velocity_noise(
        10.0,
        sc.gps_sigma_h,
        sc.gps_sigma_v,
        sc.gps_sigma_vel_h,
        sc.gps_sigma_vel_v,
        seed ^ 0x6750_5300,
    );
    let mut baro = BaroSim::new(
        25.0,
        sc.baro_sigma,
        sc.baro_drift_sigma,
        sc.baro_drift_tau_s,
        seed ^ 0xBA50_0000,
    );

    // Front-end filters, as ImuFilter in the IMU read task.
    let fs = 1.0 / dt;
    let lp = |fc: f32| {
        if fc > 0.0 { Biquad::new_lowpass_butterworth(fc, fs) } else { Biquad::identity() }
    };
    let mut gyro_f = [lp(p.imu_filter.gyro_fc_hz); 3];
    let mut accel_f = [lp(p.imu_filter.accel_fc_hz); 3];
    let mut primed = false;

    // MEKF converged level before the flight, as after boot.
    let mut mekf = AttitudeMekf::new(p.mekf);
    for _ in 0..3000 {
        mekf.predict([0.0; 3], dt);
        mekf.update_accel([0.0, 0.0, -1.0]);
    }
    let mut kf = PosKf::with_params([0.0, 0.0, -sc.start_alt], &p.poskf);
    let mut gps_accel = GpsAccelEstimator::new(p.gps_accel_tau_s);
    let mut pending_fix = None;
    let mut pending_baro = None;
    let mut last_fix_t = 0.0f32;
    let mut accel_dec = AccelDecimator::new();
    let mut a_upd = [0.0f32; 3];
    let mut clipped_in_window = false;
    let mut kf_coast = false;
    let mut gps_was_fresh = false;
    let mut last_applied = false;
    let mut holdoff = 0u32;
    let mut sf_run = 0u32;

    let mut alt_ctrl = AltitudeController::new(tun.alt, hover);
    let mut pos_ctrl = PositionController::new(PositionGains {
        max_tilt_rad: tun.pos_max_tilt_deg * DEG2RAD,
        max_tilt_rad_degraded: PositionGains::default().max_tilt_rad_degraded,
        ..PositionGains::default()
    });
    let mut mpc: AttitudeMpc = AttitudeMpc::with_model(MpcModel { max_iter: tun.mpc_max_iter, ..r.mpc_model() });
    mpc.set_reference([0.0; 3], [0.0; 3]);
    let mut rate_pid = RatePidController::new(tun.rate, tun.rate, tun.yaw, tun.limits);
    let mut thrust = hover;
    let mut rate_sp = [0.0f32; 3];

    let mut alt_target = sc.start_alt;
    let mut att_ref = [0.0f32; 2];
    let mut alt_i = 0usize;
    let mut att_i = 0usize;
    let mut alt_edge: Option<Edge> = None;
    let mut att_edge: Option<Edge> = None;

    let mut out = EstRun::default();
    let steps = (sc.total_s / dt) as usize;
    out.steps = steps as u64;

    for step in 0..steps {
        let t = step as f32 * dt;
        let outer = step % ACCEL_DECIMATION as usize == 0;

        // ---- Sensors ----
        if let Some(fix) = gps.tick_full(
            dt,
            [sim.state.x, sim.state.y, sim.state.z],
            [sim.state.vx, sim.state.vy, sim.state.vz],
        ) {
            if t >= sc.gps_outage_s.0 && t < sc.gps_outage_s.1 {
                out.gps_fixes_dropped += 1;
            } else {
                pending_fix = Some(fix);
            }
        }
        if let Some(b) = baro.tick(dt, -sim.state.z) {
            pending_baro = Some(b);
        }

        let truth = sim.read_imu();
        let (g_raw, a_raw) = deg.imu(truth.gyro, truth.accel, dt);
        let full_a = sc.accel_full_scale_g * G_MPS2;
        let mut a_clip = [0.0f32; 3];
        let mut g_clip = [0.0f32; 3];
        for i in 0..3 {
            if a_raw[i].abs() >= full_a {
                out.accel_clipped += 1;
                clipped_in_window = true;
            }
            a_clip[i] = a_raw[i].clamp(-full_a, full_a);
            g_clip[i] = g_raw[i].clamp(-GYRO_FULL_SCALE_DPS, GYRO_FULL_SCALE_DPS);
        }
        if !primed {
            for i in 0..3 {
                gyro_f[i].prime(g_clip[i]);
                accel_f[i].prime(a_clip[i]);
            }
            primed = true;
        }
        let mut gyro = [0.0f32; 3];
        let mut accel = [0.0f32; 3];
        for i in 0..3 {
            gyro[i] = gyro_f[i].apply(g_clip[i]);
            accel[i] = accel_f[i].apply(a_clip[i]);
        }
        let mut g_est = [0.0f32; 3];
        let mut a_est = [0.0f32; 3];
        for i in 0..3 {
            g_est[i] = gyro[i] * sc.est_gyro_sign[i] * DEG2RAD;
            a_est[i] = accel[i] * sc.est_accel_sign[i];
        }

        // ---- mekf_task ----
        mekf.predict(g_est, dt);
        accel_dec.push(a_est);
        if outer {
            let m = accel_dec.take(a_est);
            a_upd = if p.accel_window_mean { m } else { a_est };
            let a_g = [a_upd[0] / G_MPS2, a_upd[1] / G_MPS2, a_upd[2] / G_MPS2];
            let a_c = compensate_specific_force_g(a_g, gps_accel.accel_ned(), mekf.euler());
            if specific_force_implausible([m[0] / G_MPS2, m[1] / G_MPS2, m[2] / G_MPS2]) {
                sf_run = sf_run.saturating_add(1);
            } else {
                sf_run = 0;
            }
            let implausible = p.sf_invariant_check && sf_run >= p.sf_invariant_persist.max(1);
            if implausible {
                out.accel_implausible += 1;
                if !clipped_in_window {
                    out.accel_implausible_unclipped += 1;
                }
            }
            if clipped_in_window {
                holdoff = p.clip_holdoff_windows;
            } else if holdoff > 0 {
                holdoff -= 1;
                clipped_in_window = true;
            }
            let window_clipped = clipped_in_window || implausible;
            clipped_in_window = false;
            let skip = p.clip_skip_update && window_clipped;
            kf_coast = p.clip_kf_coast && window_clipped;
            gps_was_fresh |= gps_accel.is_fresh();
            let gps_lost = gps_was_fresh && !gps_accel.is_fresh();
            let sig = if p.sigma_a_uncompensated > 0.0 && gps_lost {
                p.sigma_a_uncompensated
            } else {
                p.mekf.sigma_a
            };
            last_applied = !skip && mekf.update_accel_sigma(a_c, sig);
            if last_applied {
                out.accel_updates_applied += 1;
            } else {
                out.accel_updates_rejected += 1;
            }
        }
        let bias = mekf.bias();
        let gyro_corr = [
            gyro[0] - sc.est_gyro_sign[0] * bias[0] * RAD2DEG,
            gyro[1] - sc.est_gyro_sign[1] * bias[1] * RAD2DEG,
            gyro[2] - sc.est_gyro_sign[2] * bias[2] * RAD2DEG,
        ];

        if outer {
            // ---- pos_kf_task (100 Hz) ----
            gps_accel.tick(dt_outer);
            let q = mekf.quaternion();
            let sf_w = quat_rotate(q, a_upd);
            let a_w = if kf_coast { [0.0; 3] } else { [sf_w[0], sf_w[1], sf_w[2] + G_MPS2] };
            if kf_coast {
                kf.predict_with_sigma(a_w, dt_outer, p.poskf.coast_sigma_a);
            } else {
                kf.predict(a_w, dt_outer);
            }
            if let Some(fix) = pending_fix.take() {
                kf.update_gps_scaled(fix.position_ned, p.poskf.sigma_gps_h, p.poskf.sigma_gps_v);
                // As main.rs: the receiver reports ground speed and course,
                // and below the deadband the velocity is zeroed.
                let (fvn, fve) = (fix.velocity_ned[0], fix.velocity_ned[1]);
                let (vn, ve) = gps_velocity_ned(
                    libm::sqrtf(fvn * fvn + fve * fve),
                    libm::atan2f(fve, fvn) * RAD2DEG,
                );
                kf.update_gps_velocity(vn, ve);
                gps_accel.update(vn, ve, t - last_fix_t);
                last_fix_t = t;
            }
            if let Some(b) = pending_baro.take() {
                kf.update_baro(b);
            }
            if p.lost_zero_vel_sigma > 0.0
                && gps_was_fresh
                && !gps_accel.is_fresh()
                && (step / ACCEL_DECIMATION as usize) % 10 == 0
            {
                kf.update_gps_velocity_scaled(0.0, 0.0, p.lost_zero_vel_sigma);
            }

            // ---- Schedules ----
            while alt_i < sc.n_alt_steps && t >= sc.alt_steps[alt_i].0 {
                let to = sc.alt_steps[alt_i].1;
                alt_edge = Edge::new([alt_target, 0.0], [to, 0.0]).or(alt_edge);
                alt_target = to;
                alt_i += 1;
            }
            if !sc.pos_hold {
                while att_i < sc.n_att_steps && t >= sc.att_steps[att_i].0 {
                    let to = [sc.att_steps[att_i].1, sc.att_steps[att_i].2];
                    att_edge = Edge::new(att_ref, to).or(att_edge);
                    att_ref = to;
                    att_i += 1;
                    mpc.set_reference([att_ref[0] * DEG2RAD, att_ref[1] * DEG2RAD, 0.0], [0.0; 3]);
                }
            }

            // ---- Control, closed on the estimates ----
            let e = mekf.euler();
            thrust = alt_ctrl.update(alt_target, kf.altitude_up(), kf.vz_up(), dt_outer);
            if sc.pos_hold {
                pos_ctrl.set_compensation(gps_accel.is_fresh(), dt_outer);
                let po = pos_ctrl.update([kf.x[0], kf.x[1]], [kf.x[3], kf.x[4]], [0.0, 0.0], e[2]);
                att_ref = [po.roll_rad * RAD2DEG, po.pitch_rad * RAD2DEG];
                mpc.set_reference([po.roll_rad, po.pitch_rad, 0.0], [0.0; 3]);
            }
            let mo = mpc.solve(
                e,
                [gyro_corr[0] * DEG2RAD, gyro_corr[1] * DEG2RAD, gyro_corr[2] * DEG2RAD],
            );
            rate_sp = [
                mo.rate_setpoints_rads[0] * RAD2DEG,
                mo.rate_setpoints_rads[1] * RAD2DEG,
                mo.rate_setpoints_rads[2] * RAD2DEG,
            ];
        }

        let pid = rate_pid.update(rate_sp, gyro_corr, dt);
        let mixed = mixer.apply(&ControlDemand { thrust, roll: pid[0], pitch: pid[1], yaw: pid[2] });
        let motors = deg.motors(mixed.motors);
        if motors.iter().any(|&m| m <= 0.001 || m >= 0.999) {
            out.rail_steps += 1;
        }
        sim.step(&MotorForces { motors }, dt);

        // ---- Failure, as run_case ----
        let (roll, pitch, alt) = (sim.state.roll, sim.state.pitch, -sim.state.z);
        let horiz = libm::sqrtf(sim.state.x * sim.state.x + sim.state.y * sim.state.y);
        let cause = if !roll.is_finite() || !pitch.is_finite() || !alt.is_finite()
            || !kf.x[2].is_finite() || !mekf.quaternion()[0].is_finite()
        {
            Some(FailCause::NonFinite)
        } else if roll.abs() > 90.0 || pitch.abs() > 90.0 {
            Some(FailCause::Diverged)
        } else if alt <= 0.0 {
            Some(FailCause::Crashed)
        } else if alt > alt_target.max(sc.start_alt) + FLYAWAY_MARGIN_M {
            Some(FailCause::Flyaway)
        } else if sc.pos_hold && horiz > 100.0 {
            Some(FailCause::Drifted)
        } else {
            None
        };
        if let Some(c) = cause {
            out.failed = Some((t, c));
            break;
        }

        // ---- Scoring, at the outer rate ----
        if outer {
            let z_true = quat_rotate(sim.state.q, [0.0, 0.0, 1.0]);
            let z_est = quat_rotate(mekf.quaternion(), [0.0, 0.0, 1.0]);
            let c = z_true[0] * z_est[0] + z_true[1] * z_est[1] + z_true[2] * z_est[2];
            let tilt = libm::acosf(c.clamp(-1.0, 1.0)) * RAD2DEG;
            let ae = kf.altitude_up() - alt;
            let he = libm::sqrtf(
                (kf.x[0] - sim.state.x) * (kf.x[0] - sim.state.x)
                    + (kf.x[1] - sim.state.y) * (kf.x[1] - sim.state.y),
            );
            let e = mekf.euler();
            trace(TraceRow {
                t,
                tilt_err_deg: tilt,
                roll_true: roll,
                pitch_true: pitch,
                roll_est: e[0] * RAD2DEG,
                pitch_est: e[1] * RAD2DEG,
                pos_true: [sim.state.x, sim.state.y],
                pos_est: [kf.x[0], kf.x[1]],
                vel_true: [sim.state.vx, sim.state.vy],
                vel_est: [kf.x[3], kf.x[4]],
                gps_accel: gps_accel.accel_ned(),
                accel_true: [sim.last_accel_world[0], sim.last_accel_world[1]],
                alt_err: ae,
                accel_update_applied: last_applied,
            });
            out.tilt_sq += (tilt * tilt) as f64;
            out.alt_sq += (ae * ae) as f64;
            out.samples += 1;
            out.tilt_err_max_deg = out.tilt_err_max_deg.max(tilt);
            out.alt_err_max_m = out.alt_err_max_m.max(ae.abs());
            out.horiz_err_max_m = out.horiz_err_max_m.max(he);
            if tilt > DIVERGED_TILT_DEG || ae.abs() > DIVERGED_ALT_M || he > DIVERGED_HORIZ_M {
                out.est_diverged = true;
            }
            if let Some(ed) = &alt_edge {
                out.max_overshoot = out.max_overshoot.max(ed.overshoot([alt, 0.0]));
            }
            if let Some(ed) = &att_edge {
                out.max_overshoot = out.max_overshoot.max(ed.overshoot([roll, pitch]));
            }
        }
    }

    let b = mekf.bias();
    let tb = sc.deg.gyro.bias;
    let be = [
        b[0] * RAD2DEG * sc.est_gyro_sign[0] - tb[0],
        b[1] * RAD2DEG * sc.est_gyro_sign[1] - tb[1],
        b[2] * RAD2DEG * sc.est_gyro_sign[2] - tb[2],
    ];
    out.bias_err_dps = libm::sqrtf(be[0] * be[0] + be[1] * be[1] + be[2] * be[2]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A clean, calm hover with perfect sensors must be estimated almost
    /// exactly and must not fail. If this breaks, the loop is wrong, not
    /// the estimator.
    #[test]
    fn clean_hover_is_estimated_and_survives() {
        let mut sc = EstScenario::nominal();
        sc.total_s = 3.0;
        sc.gps_sigma_h = 0.0;
        sc.gps_sigma_v = 0.0;
        sc.gps_sigma_vel_h = 0.0;
        sc.gps_sigma_vel_v = 0.0;
        sc.baro_sigma = 0.0;
        sc.baro_drift_sigma = 0.0;
        let r = run(&sc, &EstParams::firmware(), &Tunables::firmware(), 1);
        assert!(r.failed.is_none(), "{:?}", r.failed);
        let tilt = libm::sqrt(r.tilt_sq / r.samples as f64);
        let alt = libm::sqrt(r.alt_sq / r.samples as f64);
        assert!(tilt < 0.5, "tilt rmse {tilt}");
        assert!(alt < 0.1, "alt rmse {alt}");
    }

    /// The overshoot measure: past the target along the step, as a fraction.
    #[test]
    fn edge_overshoot_is_a_fraction_of_the_step() {
        let e = Edge::new([5.0, 0.0], [10.0, 0.0]).unwrap();
        assert_eq!(e.overshoot([9.0, 0.0]), 0.0);
        assert!((e.overshoot([11.0, 0.0]) - 0.2).abs() < 1e-6);
        let d = Edge::new([10.0, 0.0], [5.0, 0.0]).unwrap();
        assert!((d.overshoot([4.0, 0.0]) - 0.2).abs() < 1e-6);
    }
}
