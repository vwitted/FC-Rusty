// harness.rs — one flight, scored.
//
// Extracted from examples/sim_sweep.rs so the sweep and the tuner drive the
// SAME cascade. Two copies would drift, and a tuner optimising against a
// slightly different plant than the sweep reports on is worse than useless.
//
// no_std: this module is compiled into the firmware build too, so it must not
// read the environment or print. Callers do both. Tracing is a callback.

use crate::control::altitude::{AltitudeController, AltitudeGains};
use crate::control::mixer::{ControlDemand, QUAD_X};
use crate::control::mpc::AttitudeMpc;
use crate::control::pid::{PidGains, PidLimits, RatePidController};
use crate::imu_filter::Biquad;
use crate::sim::degrade::{Degradation, Degrader};
use crate::sim::dual_imu::{DualImu, DualImuConfig};
use crate::sim::{MotorForces, QuadParams, QuadSim};

use core::f32::consts::PI;

const DEG2RAD: f32 = PI / 180.0;
const RAD2DEG: f32 = 180.0 / PI;

/// Inner (rate/IMU) and outer (MPC/altitude) loop rates.
#[derive(Debug, Clone, Copy)]
pub struct Rates {
    pub dt: f32,
    pub outer_div: usize,
}

impl Rates {
    /// What the firmware runs: 8 kHz IMU/rate, 100 Hz MPC.
    pub const FIRMWARE: Rates = Rates { dt: 125e-6, outer_div: 80 };
    /// What sim_mpc_hover uses -- 200 Hz / 50 Hz, inherited from the F405
    /// days. Kept so old results remain reproducible.
    pub const LEGACY: Rates = Rates { dt: 0.005, outer_div: 4 };

    pub fn inner_hz(&self) -> f32 { 1.0 / self.dt }
    pub fn outer_hz(&self) -> f32 { 1.0 / (self.dt * self.outer_div as f32) }
}

/// Why a run ended early. Not interchangeable: a sink with attitude level
/// says the mixer traded thrust away, a divergence says the loop lost the
/// aircraft. Collapsing them into one bool hid a broken baseline for a whole
/// session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailCause {
    Diverged,
    Crashed,
    Flyaway,
    /// NaN/inf. Always a harness or solver defect, never a flight outcome.
    NonFinite,
}

impl FailCause {
    pub const ALL: [FailCause; 4] = [
        FailCause::Diverged, FailCause::Crashed,
        FailCause::Flyaway, FailCause::NonFinite,
    ];
    pub fn idx(self) -> usize {
        match self {
            FailCause::Diverged => 0,
            FailCause::Crashed => 1,
            FailCause::Flyaway => 2,
            FailCause::NonFinite => 3,
        }
    }
    pub fn short(self) -> &'static str {
        match self {
            FailCause::Diverged => "div",
            FailCause::Crashed => "crash",
            FailCause::Flyaway => "away",
            FailCause::NonFinite => "nan",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Metrics {
    pub att_rms: f32,
    pub att_max: f32,
    pub alt_rms: f32,
    pub air_frac: f32,
    pub failed_at: Option<(f32, FailCause)>,
}

impl Metrics {
    pub fn failed(t: f32, cause: FailCause) -> Self {
        Self {
            att_rms: f32::NAN, att_max: f32::NAN, alt_rms: f32::NAN,
            air_frac: f32::NAN, failed_at: Some((t, cause)),
        }
    }
}

/// Everything a tuner is allowed to change.
///
/// Deliberately narrow. The plant, the disturbances and the degradation are
/// the EXAM; these are the answers. Letting a search touch the first three
/// would let it tune the test rather than the controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tunables {
    pub rate: PidGains,
    pub yaw: PidGains,
    pub limits: PidLimits,
    /// Gyro low-pass cutoff, Hz. 0 disables the filter.
    pub gyro_fc_hz: f32,
    pub alt: AltitudeGains,
}

impl Tunables {
    /// Exactly what main.rs flies today. The baseline any search must beat.
    pub fn firmware() -> Self {
        Self {
            rate: PidGains { kp: 0.02, ki: 0.005, kd: 0.001 },
            yaw: PidGains { kp: 0.03, ki: 0.005, kd: 0.0 },
            limits: PidLimits { integral_max: 0.3, output_max: 0.5, d_lpf_tau_s: 0.008 },
            gyro_fc_hz: 150.0,
            alt: AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 },
        }
    }
}

/// The exam: everything the tuner may NOT change.
#[derive(Debug, Clone, Copy)]
pub struct HarnessCfg {
    pub rates: Rates,
    pub plant: QuadParams,
    pub total_s: f32,
    pub target_alt: f32,
    /// Spread the disturbances over this many ms; 0 = instantaneous poke.
    pub disturb_ms: f32,
    /// Model both gyros rather than one.
    pub dual: bool,
    /// Dual-IMU fault mapping. Ignored unless `dual`.
    pub dual_cfg: DualImuConfig,
}

impl HarnessCfg {
    pub fn firmware_rates(plant: QuadParams) -> Self {
        Self {
            rates: Rates::FIRMWARE,
            plant,
            total_s: 10.0,
            target_alt: 5.0,
            disturb_ms: 0.0,
            dual: false,
            dual_cfg: DualImuConfig::none(),
        }
    }
}

/// One sample handed to a trace callback, once per outer tick.
#[derive(Debug, Clone, Copy)]
pub struct TracePoint {
    pub t: f32,
    pub alt: f32,
    pub vz_up: f32,
    pub thrust_demand: f32,
    pub motor_mean: f32,
    pub roll: f32,
    pub roll_rate: f32,
    pub pid_roll: f32,
}

/// Fly one case and score it.
///
/// `trace` is called once per outer tick when present. The sim never prints.
pub fn run_case(
    h: &HarnessCfg,
    tun: &Tunables,
    cfg: Degradation,
    seed: u64,
    mut trace: Option<&mut dyn FnMut(TracePoint)>,
) -> Metrics {
    let r = h.rates;
    let hover_throttle = (h.plant.mass * 9.81) / h.plant.max_thrust;
    let mut sim = QuadSim::new_hovering(h.plant, h.target_alt);
    let mut deg = Degrader::new(cfg, seed);
    // Motors always come from `deg`; only the IMU path forks.
    let mut dual_imu = DualImu::new(h.dual_cfg, seed);

    let mut alt_ctrl = AltitudeController::new(tun.alt, hover_throttle);
    let mut current_thrust = hover_throttle;

    let mut mpc = AttitudeMpc::new();
    mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

    let mut rate_pid =
        RatePidController::new(tun.rate, tun.rate, tun.yaw, tun.limits);

    let mut gyro_lpf = if tun.gyro_fc_hz > 0.0 {
        [Biquad::new_lowpass_butterworth(tun.gyro_fc_hz, r.inner_hz()); 3]
    } else {
        [Biquad::identity(); 3]
    };
    let mut primed = false;

    let steps = (h.total_s / r.dt) as usize;
    let mut rate_sp_degs = [0.0f32; 3];

    let mut att_sq = 0.0f64;
    let mut alt_sq = 0.0f64;
    let mut att_max = 0.0f32;
    let mut air_steps = 0usize;
    let mut trace_due = false;

    for step in 0..steps {
        let t = step as f32 * r.dt;

        // A direct state poke is an instantaneous step in angular rate, i.e.
        // infinite angular acceleration, flat to Nyquist. Nothing physical
        // does that. disturb_ms spreads the same momentum over a raised
        // cosine, which is what a gust looks like.
        let s2 = (2.0 / r.dt) as usize;
        let s5 = (5.0 / r.dt) as usize;
        if h.disturb_ms <= 0.0 {
            if step == s2 {
                sim.state.roll_rate += 10.0;
            } else if step == s5 {
                sim.state.vz += 2.0;
            }
        } else {
            let win = (h.disturb_ms * 1e-3 / r.dt).max(1.0) as usize;
            let shape = |k: usize| {
                let x = (k as f32 + 0.5) / win as f32;
                (1.0 - libm::cosf(2.0 * PI * x)) / win as f32
            };
            if step >= s2 && step < s2 + win {
                sim.state.roll_rate += 10.0 * shape(step - s2);
            }
            if step >= s5 && step < s5 + win {
                sim.state.vz += 2.0 * shape(step - s5);
            }
        }

        let truth = sim.read_imu();
        let (gyro_raw, angle_err) = if h.dual {
            dual_imu.read(truth.gyro, truth.accel, r.dt).0
        } else {
            deg.imu(truth.gyro, truth.accel, r.dt)
        };

        if !primed {
            for i in 0..3 {
                gyro_lpf[i].prime(gyro_raw[i]);
            }
            primed = true;
        }
        let gyro = [
            gyro_lpf[0].apply(gyro_raw[0]),
            gyro_lpf[1].apply(gyro_raw[1]),
            gyro_lpf[2].apply(gyro_raw[2]),
        ];

        // The accel channel stands in for attitude-ESTIMATE error: this
        // harness feeds truth angles (there is no estimator in the loop), so
        // accel noise has nothing physical to propagate through. Treating it
        // as angle error is the honest interpretation of that knob here.
        let angle = [
            truth.angle[0] + (angle_err[0] - truth.accel[0]),
            truth.angle[1] + (angle_err[1] - truth.accel[1]),
            truth.angle[2] + (angle_err[2] - truth.accel[2]),
        ];

        if step % r.outer_div == 0 {
            let alt = -sim.state.z;
            let vz_up = -sim.state.vz;
            current_thrust =
                alt_ctrl.update(h.target_alt, alt, vz_up, r.dt * r.outer_div as f32);
            trace_due = trace.is_some();

            let angles_rad = [angle[0] * DEG2RAD, angle[1] * DEG2RAD, angle[2] * DEG2RAD];
            let rates_rad = [gyro[0] * DEG2RAD, gyro[1] * DEG2RAD, gyro[2] * DEG2RAD];
            let out = mpc.solve(angles_rad, rates_rad);
            rate_sp_degs = [
                out.rate_setpoints_rads[0] * RAD2DEG,
                out.rate_setpoints_rads[1] * RAD2DEG,
                out.rate_setpoints_rads[2] * RAD2DEG,
            ];
        }

        let pid_output = rate_pid.update(rate_sp_degs, gyro, r.dt);
        let demand = ControlDemand {
            thrust: current_thrust,
            roll: pid_output[0],
            pitch: pid_output[1],
            yaw: pid_output[2],
        };
        let mixed = QUAD_X.apply(&demand);
        let motors = deg.motors(mixed.motors);

        if trace_due {
            trace_due = false;
            if let Some(f) = trace.as_mut() {
                let msum: f32 = motors.iter().sum();
                f(TracePoint {
                    t,
                    alt: -sim.state.z,
                    vz_up: -sim.state.vz,
                    thrust_demand: current_thrust,
                    motor_mean: msum / 4.0,
                    roll: sim.state.roll,
                    roll_rate: sim.state.roll_rate,
                    pid_roll: pid_output[0],
                });
            }
        }

        // Post-airmode: a motor on a rail means the mixer had nothing left
        // to give, and thrust has already been traded away to hold attitude.
        if motors.iter().any(|&m| m <= 0.001 || m >= 0.999) {
            air_steps += 1;
        }

        sim.step(&MotorForces { motors }, r.dt);

        let roll = sim.state.roll;
        let pitch = sim.state.pitch;
        let alt = -sim.state.z;

        // Order matters. NonFinite first: once a value is NaN every
        // comparison below is false, so a later arm would absorb it silently.
        // Divergence before Crashed because a tumble that then hits the
        // ground is a divergence; the ground contact is its consequence.
        let cause = if !roll.is_finite() || !pitch.is_finite() || !alt.is_finite() {
            Some(FailCause::NonFinite)
        } else if roll.abs() > 90.0 || pitch.abs() > 90.0 {
            Some(FailCause::Diverged)
        } else if alt <= 0.0 {
            Some(FailCause::Crashed)
        } else if alt > 50.0 {
            Some(FailCause::Flyaway)
        } else {
            None
        };
        if let Some(c) = cause {
            return Metrics::failed(t, c);
        }

        let att = libm::sqrtf(roll * roll + pitch * pitch);
        att_max = att_max.max(att);
        att_sq += (att * att) as f64;
        let ae = alt - h.target_alt;
        alt_sq += (ae * ae) as f64;
    }

    let n = steps as f64;
    Metrics {
        att_rms: libm::sqrt(att_sq / n) as f32,
        att_max,
        alt_rms: libm::sqrt(alt_sq / n) as f32,
        air_frac: air_steps as f32 / steps as f32,
        failed_at: None,
    }
}
