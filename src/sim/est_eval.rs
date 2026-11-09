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
//                      windows, in which case the PosKF coasts and, with fresh
//                      GPS, tilt is aided by the GPS-implied thrust axis --
//                      compensated by gps_accel::GpsAccelEstimator
//                      through gps_accel::compensate_specific_force_g, which
//                      rotates with the MEKF's OWN attitude, as main.rs does
//   pos_kf_task        100 Hz: gps_accel.tick, predict on R(q_mekf)*f + g,
//                      GPS position + velocity (10 Hz; velocity through the
//                      same low-speed deadband), baro (25 Hz)
//   control            altitude and position loops close on the PosKF; the
//                      MPC and the rate PID close on the MEKF and on
//                      bias-corrected filtered gyro
//
// Optional, default off so the v1 exam is unchanged:
//   magnetometer       sim::mag_sim at 100 Hz, fused as mekf_task does: seed
//                      the reference from the first sample, anchor to true
//                      north once level-and-still when a stored cal is
//                      loaded, then update_mag on every sample
//   control_on_truth   the controllers close on the TRUE state, so an
//                      estimator is scored on a trajectory it cannot perturb
//
// Kept separate from run_case deliberately: run_case's numbers are the
// committed sim-baseline, and changing its loop would move them.
//
// no_std, like the rest of `sim`: the caller parses configs and prints.

use crate::attitude_mekf::{
    accel_window_excluded, specific_force_implausible, thrust_aid_sigma, thrust_axis_nav,
    AccelDecimator, AttitudeMekf, MekfParams,
    ACCEL_DECIMATION, G_MPS2,
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
use crate::sim::mag_sim::{MagSim, MagSimParams};
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
    /// Window mean for the PosKF predict input only.
    pub accel_window_mean_kf: bool,
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
    /// 1σ GPS horizontal-velocity noise for the PosKF update, m/s.
    pub gps_vel_sigma: f32,
    /// Adaptive accel-update noise: σ = mekf.sigma_a * (1 + k_norm*| |a|-1 |
    /// + accel_r_rate_gain*|ω| (rad/s) + k_gacc*|a_gps|/g).
    /// (The rate term is MekfParams::accel_r_rate_gain.)
    pub r_k_norm: f32,
    pub r_k_gacc: f32,
    /// While the accelerometer window is excluded and the GPS acceleration is
    /// fresh, aid tilt with the thrust axis implied by GPS acceleration
    /// (attitude_mekf::thrust_axis_nav), at this σ. 0 = off (gyro only).
    pub thrust_aid_sigma: f32,
    /// Also fuse thrust-axis aiding on VALID windows, at this σ. 0 = off.
    pub thrust_aid_always_sigma: f32,
}

impl EstParams {
    /// What main.rs constructs today.
    pub fn firmware() -> Self {
        Self {
            mekf: MekfParams::default(),
            poskf: PosKfParams::FIRMWARE,
            gps_accel_tau_s: crate::gps_accel::DEFAULT_TAU_S,
            imu_filter: ImuFilterParams::default(),
            accel_window_mean: true,
            accel_window_mean_kf: false,
            clip_skip_update: true,
            clip_kf_coast: true,
            sigma_a_uncompensated: 0.0,
            lost_zero_vel_sigma: 0.0,
            sf_invariant_check: true,
            sf_invariant_persist: crate::attitude_mekf::SF_INVARIANT_PERSIST,
            clip_holdoff_windows: 0,
            gps_vel_sigma: PosKf::SIGMA_GPS_VEL_DEFAULT,
            r_k_norm: 0.0,
            r_k_gacc: 0.0,
            thrust_aid_sigma: crate::attitude_mekf::THRUST_AID_SIGMA,
            thrust_aid_always_sigma: crate::attitude_mekf::THRUST_AID_SIGMA_VALID,
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
    /// Magnetometer. `MagSimParams::OFF` (the default) produces no samples.
    pub mag: MagSimParams,
    /// A stored mag calibration is loaded at boot (`mag_cal_ut` as the
    /// hard-iron estimate) and the heading is anchored to true north once
    /// level and still. False: no cal, relative boot heading, as mekf_task
    /// without a stored config.
    pub mag_calibrated: bool,
    /// The stored hard-iron estimate, uT. Its difference from
    /// `mag.hard_iron_ut` is the calibration residual.
    pub mag_cal_ut: [f32; 3],
    /// Yaw reference schedule: (time s, yaw deg). Before the first edge 0.
    pub yaw_steps: [(f32, f32); MAX_ALT_STEPS],
    pub n_yaw_steps: usize,
    /// Close every controller on the true state instead of the estimates.
    /// The estimator still runs on the same sensor stream and is scored.
    pub control_on_truth: bool,
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
            mag: MagSimParams::OFF,
            mag_calibrated: true,
            mag_cal_ut: [0.0; 3],
            yaw_steps: [(f32::INFINITY, 0.0); MAX_ALT_STEPS],
            n_yaw_steps: 0,
            control_on_truth: false,
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
    pub thrust_aid_updates: u64,
    pub accel_updates_rejected: u64,
    pub rail_steps: u64,
    pub steps: u64,
    /// Final gyro-bias estimate error magnitude, deg/s.
    pub bias_err_dps: f32,
    /// Heading error (estimated minus true yaw, wrapped), deg.
    pub yaw_sq: f64,
    pub yaw_err_max_deg: f32,
    pub mag_updates_applied: u64,
    pub mag_updates_rejected: u64,
    /// Consistency (NEES) sums at the outer rate. Tilt: the nav-frame
    /// horizontal components of the attitude error against the matching
    /// 2x2 block of the MEKF covariance, 2 dof. Attitude: the full body
    /// error, 3 dof. PosKF: altitude, 1 dof, and position+velocity, 6 dof.
    /// `*_in95`: samples under the chi-square 95 % bound for that dof.
    pub nees_tilt: f64,
    pub nees_tilt_in95: u64,
    pub nees_att: f64,
    pub nees_att_in95: u64,
    pub nees_alt: f64,
    pub nees_alt_in95: u64,
    pub nees_pv: f64,
    pub nees_pv_in95: u64,
    pub nees_samples: u64,
    /// Tilt error per outer sample, deg, held at DIVERGED_TILT_DEG from a
    /// failure to the end of the run. Sum of squares over ALL scheduled
    /// samples, so a failed seed is scored rather than dropped.
    pub tilt_sq_penalised: f64,
    pub samples_scheduled: u64,
}

/// Chi-square 95 % quantiles for 1, 2, 3 and 6 degrees of freedom.
pub const CHI2_95: [(u32, f64); 4] = [(1, 3.841), (2, 5.991), (3, 7.815), (6, 12.592)];

fn chi2_95(dof: u32) -> f64 {
    CHI2_95.iter().find(|(d, _)| *d == dof).map(|(_, v)| *v).unwrap_or(f64::INFINITY)
}

/// x^T A^-1 x for a symmetric positive-definite A; None when singular.
fn nees<const N: usize>(x: &[f64; N], a: &[[f64; N]; N]) -> Option<f64> {
    let m = nalgebra::SMatrix::<f64, N, N>::from_fn(|i, j| a[i][j]);
    let v = nalgebra::SVector::<f64, N>::from_fn(|i, _| x[i]);
    let chol = m.cholesky()?;
    Some(v.dot(&chol.solve(&v)))
}

/// Yaw of a body -> nav quaternion, rad (heading of body x).
fn quat_yaw(q: [f32; 4]) -> f32 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    libm::atan2f(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z)
}

fn wrap_deg(d: f32) -> f32 {
    let mut d = d % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d < -180.0 {
        d += 360.0;
    }
    d
}

/// Small-angle attitude error, body frame, rad: q_true = q_est * dq.
fn attitude_error_body(q_est: [f32; 4], q_true: [f32; 4]) -> [f32; 3] {
    let e = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_est[0], q_est[1], q_est[2], q_est[3],
    ));
    let t = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q_true[0], q_true[1], q_true[2], q_true[3],
    ));
    let d = e.inverse() * t;
    let s = if d.w < 0.0 { -2.0 } else { 2.0 };
    [d.i * s, d.j * s, d.k * s]
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
    /// Estimated minus true yaw, wrapped, deg.
    pub yaw_err_deg: f32,
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
    let mut a_kf = [0.0f32; 3];
    let mut clipped_in_window = false;
    let mut kf_coast = false;
    let mut gps_was_fresh = false;
    let mut last_applied = false;
    let mut holdoff = 0u32;
    let mut sf_run = 0u32;
    let mut mag = MagSim::new(sc.mag, seed ^ 0x3A65_0000);
    let mut anchor_pending = false;
    if sc.mag.enabled() && sc.mag_calibrated {
        mekf.set_hard_iron(sc.mag_cal_ut);
        anchor_pending = true;
    }
    let mut motor_mean = hover;
    let mut yaw_ref = 0.0f32;
    let mut yaw_i = 0usize;

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
    out.samples_scheduled = steps.div_ceil(ACCEL_DECIMATION as usize) as u64;

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
            let a_upd = if p.accel_window_mean { m } else { a_est };
            a_kf = if p.accel_window_mean || p.accel_window_mean_kf { m } else { a_est };
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
            // The sim always flies: armed throughout, as the firmware's
            // ARMED_FLAG would be. The invariant and the thrust aid are
            // gated on it through the same helpers main.rs calls.
            let armed = true;
            let window_clipped = clipped_in_window
                || (implausible && accel_window_excluded(false, sf_run, armed));
            clipped_in_window = false;
            let skip = p.clip_skip_update && window_clipped;
            kf_coast = p.clip_kf_coast && window_clipped;
            gps_was_fresh |= gps_accel.is_fresh();
            let gps_lost = gps_was_fresh && !gps_accel.is_fresh();
            let sig0 = if p.sigma_a_uncompensated > 0.0 && gps_lost {
                p.sigma_a_uncompensated
            } else {
                p.mekf.sigma_a
            };
            let an = libm::sqrtf(a_c[0] * a_c[0] + a_c[1] * a_c[1] + a_c[2] * a_c[2]);
            let w = libm::sqrtf(g_est[0] * g_est[0] + g_est[1] * g_est[1] + g_est[2] * g_est[2]);
            let ga = gps_accel.accel_ned();
            let gm = libm::sqrtf(ga[0] * ga[0] + ga[1] * ga[1]) / G_MPS2;
            let k_rate = p.mekf.accel_r_rate_gain;
            let sig = sig0 * (1.0 + p.r_k_norm * (an - 1.0).abs() + k_rate * w + p.r_k_gacc * gm);
            last_applied = !skip && mekf.update_accel_sigma(a_c, sig);
            // Firmware: attitude_mekf::thrust_aid_sigma(excluded, armed,
            // fresh). The σ values are overridable here for scans.
            if let Some(fw) = thrust_aid_sigma(skip, armed, gps_accel.is_fresh()) {
                let sigma = if skip { p.thrust_aid_sigma } else { p.thrust_aid_always_sigma };
                debug_assert!(fw > 0.0);
                if sigma > 0.0 {
                    mekf.update_vector(thrust_axis_nav(gps_accel.accel_ned()), [0.0, 0.0, 1.0], sigma);
                    if skip {
                        out.thrust_aid_updates += 1;
                    }
                }
            }
            if last_applied {
                out.accel_updates_applied += 1;
            } else {
                out.accel_updates_rejected += 1;
            }
        }
        // ---- mekf_task magnetometer branch ----
        if let Some(ut) = mag.tick(dt, sim.state.q, motor_mean) {
            if !mekf.mag_initialized() {
                mekf.initialize_mag_from_first(ut);
            } else {
                // As main.rs mag_anchor_ready, on the estimator's copy of
                // the filtered sensors.
                let an = libm::sqrtf(a_est[0] * a_est[0] + a_est[1] * a_est[1] + a_est[2] * a_est[2]) / G_MPS2;
                let gn = libm::sqrtf(g_est[0] * g_est[0] + g_est[1] * g_est[1] + g_est[2] * g_est[2]) * RAD2DEG;
                if anchor_pending && (an - 1.0).abs() < 0.1 && gn < 5.0 {
                    mekf.anchor_heading(ut, crate::control::mag_cal::DECLINATION_DEG.to_radians());
                    anchor_pending = false;
                }
                if mekf.update_mag(ut) {
                    out.mag_updates_applied += 1;
                } else {
                    out.mag_updates_rejected += 1;
                }
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
            let sf_w = quat_rotate(q, a_kf);
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
                kf.update_gps_velocity_scaled(vn, ve, p.gps_vel_sigma);
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

            while yaw_i < sc.n_yaw_steps && t >= sc.yaw_steps[yaw_i].0 {
                yaw_ref = sc.yaw_steps[yaw_i].1 * DEG2RAD;
                yaw_i += 1;
                if !sc.pos_hold {
                    mpc.set_reference([att_ref[0] * DEG2RAD, att_ref[1] * DEG2RAD, yaw_ref], [0.0; 3]);
                }
            }

            // ---- Control, closed on the estimates (or on truth) ----
            let (e, pos, vel, alt_up, vz_up, rates) = if sc.control_on_truth {
                (
                    [sim.state.roll * DEG2RAD, sim.state.pitch * DEG2RAD, sim.state.yaw * DEG2RAD],
                    [sim.state.x, sim.state.y],
                    [sim.state.vx, sim.state.vy],
                    -sim.state.z,
                    -sim.state.vz,
                    [truth.gyro[0] * DEG2RAD, truth.gyro[1] * DEG2RAD, truth.gyro[2] * DEG2RAD],
                )
            } else {
                (
                    mekf.euler(),
                    [kf.x[0], kf.x[1]],
                    [kf.x[3], kf.x[4]],
                    kf.altitude_up(),
                    kf.vz_up(),
                    [gyro_corr[0] * DEG2RAD, gyro_corr[1] * DEG2RAD, gyro_corr[2] * DEG2RAD],
                )
            };
            thrust = alt_ctrl.update(alt_target, alt_up, vz_up, dt_outer);
            if sc.pos_hold {
                pos_ctrl.set_compensation(gps_accel.is_fresh(), dt_outer);
                let po = pos_ctrl.update(pos, vel, [0.0, 0.0], e[2]);
                att_ref = [po.roll_rad * RAD2DEG, po.pitch_rad * RAD2DEG];
                mpc.set_reference([po.roll_rad, po.pitch_rad, yaw_ref], [0.0; 3]);
            }
            let mo = mpc.solve(e, rates);
            rate_sp = [
                mo.rate_setpoints_rads[0] * RAD2DEG,
                mo.rate_setpoints_rads[1] * RAD2DEG,
                mo.rate_setpoints_rads[2] * RAD2DEG,
            ];
        }

        let pid_gyro = if sc.control_on_truth { truth.gyro } else { gyro_corr };
        let pid = rate_pid.update(rate_sp, pid_gyro, dt);
        let mixed = mixer.apply(&ControlDemand { thrust, roll: pid[0], pitch: pid[1], yaw: pid[2] });
        let motors = deg.motors(mixed.motors);
        motor_mean = (motors[0] + motors[1] + motors[2] + motors[3]) * 0.25;
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
            let remaining = out.samples_scheduled.saturating_sub(out.samples);
            out.tilt_sq_penalised += remaining as f64 * (DIVERGED_TILT_DEG as f64) * (DIVERGED_TILT_DEG as f64);
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
            let ye = wrap_deg((quat_yaw(mekf.quaternion()) - quat_yaw(sim.state.q)) * RAD2DEG);
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
                yaw_err_deg: ye,
            });
            out.tilt_sq += (tilt * tilt) as f64;
            out.alt_sq += (ae * ae) as f64;
            out.samples += 1;
            let tp = tilt.min(DIVERGED_TILT_DEG);
            out.tilt_sq_penalised += (tp * tp) as f64;
            out.yaw_sq += (ye * ye) as f64;
            out.yaw_err_max_deg = out.yaw_err_max_deg.max(ye.abs());

            // ---- Consistency (NEES) ----
            let qe = mekf.quaternion();
            let eb = attitude_error_body(qe, sim.state.q);
            let pb = mekf.attitude_cov();
            let eb64 = [eb[0] as f64, eb[1] as f64, eb[2] as f64];
            let pb64 = [
                [pb[0][0] as f64, pb[0][1] as f64, pb[0][2] as f64],
                [pb[1][0] as f64, pb[1][1] as f64, pb[1][2] as f64],
                [pb[2][0] as f64, pb[2][1] as f64, pb[2][2] as f64],
            ];
            // Tilt: rotate error and covariance to nav, keep N and E.
            let r = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                qe[0], qe[1], qe[2], qe[3],
            ))
            .to_rotation_matrix();
            let rm = nalgebra::Matrix3::<f64>::from_fn(|i, j| r[(i, j)] as f64);
            let en = rm * nalgebra::Vector3::new(eb64[0], eb64[1], eb64[2]);
            let pn = rm * nalgebra::Matrix3::from_fn(|i, j| pb64[i][j]) * rm.transpose();
            let et = [en[0], en[1]];
            let pt = [[pn[(0, 0)], pn[(0, 1)]], [pn[(1, 0)], pn[(1, 1)]]];
            let pk = &kf.p_cov;
            let ealt = [(kf.x[2] - sim.state.z) as f64];
            let palt = [[pk[(2, 2)] as f64]];
            let truth_pv = [sim.state.x, sim.state.y, sim.state.z, sim.state.vx, sim.state.vy, sim.state.vz];
            let epv: [f64; 6] = core::array::from_fn(|i| (kf.x[i] - truth_pv[i]) as f64);
            let ppv: [[f64; 6]; 6] = core::array::from_fn(|i| core::array::from_fn(|j| pk[(i, j)] as f64));
            if let (Some(nt), Some(na), Some(nh), Some(npv)) =
                (nees(&et, &pt), nees(&eb64, &pb64), nees(&ealt, &palt), nees(&epv, &ppv))
            {
                out.nees_tilt += nt;
                out.nees_tilt_in95 += (nt <= chi2_95(2)) as u64;
                out.nees_att += na;
                out.nees_att_in95 += (na <= chi2_95(3)) as u64;
                out.nees_alt += nh;
                out.nees_alt_in95 += (nh <= chi2_95(1)) as u64;
                out.nees_pv += npv;
                out.nees_pv_in95 += (npv <= chi2_95(6)) as u64;
                out.nees_samples += 1;
            }
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

    /// The GA-fitted rate gains every scenario file declares.
    fn ga() -> Tunables {
        let f = Tunables::firmware();
        Tunables { rate: crate::control::pid::PidGains { kp: 0.009, kd: 0.00015, ..f.rate }, ..f }
    }

    fn yaw_rmse(r: &EstRun) -> f64 {
        libm::sqrt(r.yaw_sq / r.samples as f64)
    }

    /// A gyro z bias is unobservable without a heading reference, so yaw
    /// drifts; with the magnetometer fused it does not. Attitude hold, not
    /// position hold: in position hold the accelerations the loop produces
    /// make yaw observable through the GPS acceleration compensation, and
    /// the drift stays bounded without a magnetometer.
    #[test]
    fn magnetometer_bounds_the_yaw_drift_of_a_gyro_z_bias() {
        let mut sc = EstScenario::nominal();
        sc.total_s = 10.0;
        sc.pos_hold = false;
        sc.deg.gyro.bias = [0.0, 0.0, 3.0];
        let p = EstParams::firmware();
        let t = ga();
        let off = run(&sc, &p, &t, 1);
        sc.mag = MagSimParams::PLACEHOLDER;
        sc.mag_cal_ut = sc.mag.hard_iron_ut;
        let on = run(&sc, &p, &t, 1);
        assert!(off.failed.is_none() && on.failed.is_none());
        assert_eq!(off.mag_updates_applied, 0);
        assert!(on.mag_updates_applied > 900, "{}", on.mag_updates_applied);
        assert!(off.yaw_err_max_deg > 20.0, "no-mag yaw max {}", off.yaw_err_max_deg);
        assert!(yaw_rmse(&on) < 3.0 && on.yaw_err_max_deg < 8.0, "mag yaw rmse {} max {}", yaw_rmse(&on), on.yaw_err_max_deg);
        assert!(on.bias_err_dps < 1.0, "mag bias err {}", on.bias_err_dps);
    }

    /// With the controllers closed on truth, a broken estimator cannot take
    /// the aircraft down: an inverted gyro axis in the estimator's copy
    /// destroys the closed-loop flight, while on truth the flight completes
    /// and the estimator's error is measured on it.
    #[test]
    fn control_on_truth_decouples_the_trajectory_from_the_estimator() {
        let mut sc = EstScenario::nominal();
        sc.total_s = 8.0;
        sc.plant.wind_ned = [6.0, 2.0, 0.0];
        sc.deg.gyro.sigma = 1.0;
        sc.deg.accel.sigma = 0.5;
        sc.est_gyro_sign = [-1.0, 1.0, 1.0];
        let p = EstParams::firmware();
        let t = ga();
        let closed = run(&sc, &p, &t, 1);
        sc.control_on_truth = true;
        let truth = run(&sc, &p, &t, 1);
        assert!(closed.failed.is_some() || closed.est_diverged, "the fault must bite in closed loop");
        assert!(truth.failed.is_none(), "{:?}", truth.failed);
        assert!(truth.samples == truth.samples_scheduled);
        // Open loop the same fault costs only a few degrees: the v1
        // gyro_sign divergences are the estimator error fed back through
        // the controller, not the estimator alone.
        assert!(truth.tilt_err_max_deg < DIVERGED_TILT_DEG, "{}", truth.tilt_err_max_deg);
    }

    /// NEES is accumulated and finite on a nominal run, and a failed run's
    /// penalised tilt covers every scheduled sample.
    #[test]
    fn consistency_and_penalised_scores_are_accumulated() {
        let mut sc = EstScenario::nominal();
        sc.total_s = 3.0;
        let r = run(&sc, &EstParams::firmware(), &ga(), 1);
        assert!(r.nees_samples > 250, "{}", r.nees_samples);
        assert!((r.nees_tilt / r.nees_samples as f64).is_finite());
        assert!(r.samples_scheduled >= r.samples);
        assert!(r.tilt_sq_penalised >= 0.0);
    }

    #[test]
    fn nees_and_attitude_error_helpers() {
        // Identity covariance: NEES is the squared norm.
        let v = nees(&[1.0, 2.0], &[[1.0, 0.0], [0.0, 1.0]]).unwrap();
        assert!((v - 5.0).abs() < 1e-12);
        let v = nees(&[2.0], &[[4.0]]).unwrap();
        assert!((v - 1.0).abs() < 1e-12);
        assert!(nees(&[1.0], &[[0.0]]).is_none());
        // A 2 deg rotation about body x is a 2 deg body error on x.
        let h = 1f32.to_radians();
        let e = attitude_error_body([1.0, 0.0, 0.0, 0.0], [libm::cosf(h), libm::sinf(h), 0.0, 0.0]);
        assert!((e[0] - 2f32.to_radians()).abs() < 1e-5 && e[1].abs() < 1e-6 && e[2].abs() < 1e-6);
        assert!((wrap_deg(350.0) + 10.0).abs() < 1e-4 && (wrap_deg(-190.0) - 170.0).abs() < 1e-4);
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
