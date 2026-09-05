// harness.rs — one flight, scored.
//
// Extracted from examples/sim_sweep.rs so the sweep and the tuner drive the
// SAME cascade. Two copies would drift, and a tuner optimising against a
// slightly different plant than the sweep reports on is worse than useless.
//
// no_std: this module is compiled into the firmware build too, so it must not
// read the environment or print. Callers do both. Tracing is a callback.

use crate::attitude_mekf::{AttitudeMekf, MekfParams};
use crate::control::altitude::{AltitudeController, AltitudeGains};
use crate::control::mixer::{ControlDemand, QUAD_X};
use crate::control::mpc::AttitudeMpc;
use crate::control::modes::{
    nav_step, FlightMode, NavInputs, NavState, PosEstimate,
};
use crate::control::position::{PositionController, PositionGains};
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
    /// Horizontal position lost. Only reachable with position hold on --
    /// without it the aircraft is SUPPOSED to translate freely, and calling
    /// that a failure would be scoring it against a job it was not given.
    Drifted,
    /// NaN/inf. Always a harness or solver defect, never a flight outcome.
    NonFinite,
}

impl FailCause {
    pub const ALL: [FailCause; 5] = [
        FailCause::Diverged, FailCause::Crashed,
        FailCause::Flyaway, FailCause::Drifted, FailCause::NonFinite,
    ];
    pub fn idx(self) -> usize {
        match self {
            FailCause::Diverged => 0,
            FailCause::Crashed => 1,
            FailCause::Flyaway => 2,
            FailCause::Drifted => 3,
            FailCause::NonFinite => 4,
        }
    }
    pub fn short(self) -> &'static str {
        match self {
            FailCause::Diverged => "div",
            FailCause::Crashed => "crash",
            FailCause::Flyaway => "away",
            FailCause::Drifted => "drift",
            FailCause::NonFinite => "nan",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Metrics {
    pub att_rms: f32,
    pub att_max: f32,
    pub alt_rms: f32,
    /// Horizontal distance from the position target, RMS and worst-case.
    /// Meaningless unless HarnessCfg::pos_hold is set -- with no position
    /// loop the aircraft simply translates, which is correct behaviour and
    /// not an error.
    pub pos_rms: f32,
    pub pos_max: f32,
    pub air_frac: f32,
    /// Time at which attitude first came back inside RECOVERY_LEVEL_DEG.
    /// Only meaningful when `initial_attitude_deg` is non-zero.
    pub recovered_at: Option<f32>,
    /// Lowest altitude reached, metres. For an upset run this is the number
    /// that matters: recovery TIME is academic, height LOST tells you the
    /// altitude below which the upset is unsurvivable.
    pub alt_min: f32,
    pub failed_at: Option<(f32, FailCause)>,
}

/// How far above the altitude target counts as a flyaway, metres.
///
/// Relative, not absolute: it used to be a hard 50 m, which silently made
/// every run starting above that altitude an instant failure. At the
/// default 5 m target this is still exactly 50 m, so existing results are
/// unchanged.
pub const FLYAWAY_MARGIN_M: f32 = 45.0;

/// Attitude below which an upset counts as recovered, degrees.
pub const RECOVERY_LEVEL_DEG: f32 = 15.0;

impl Metrics {
    pub fn failed(t: f32, cause: FailCause) -> Self {
        Self {
            att_rms: f32::NAN, att_max: f32::NAN, alt_rms: f32::NAN,
            pos_rms: f32::NAN, pos_max: f32::NAN,
            air_frac: f32::NAN, recovered_at: None, alt_min: f32::NAN,
            failed_at: Some((t, cause)),
        }
    }
}

/// Which controller turns attitude error into rate setpoints.
///
/// The point of having two is to ask whether the MPC earns its compute.
/// It is the expensive part of the stack -- an ADMM solve every 10 ms on
/// an H7 -- and nothing here has ever compared it against the thing every
/// other flight controller does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AttitudeMode {
    /// The firmware's cascaded MPC (control::mpc).
    Mpc,
    /// Classic angle mode: rate_sp = kp * angle_error, clamped. Pure
    /// proportional, which is what Betaflight and friends actually run --
    /// damping comes from the rate loop underneath, not from a D term
    /// here. Deliberately the standard version rather than a souped-up one,
    /// because the question is whether MPC beats the ORDINARY alternative.
    AnglePid { kp: f32, max_rate_dps: f32 },
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
    /// Which attitude controller to run.
    pub attitude: AttitudeMode,
    /// Position controller's max tilt, degrees. Bounds horizontal
    /// acceleration to g*tan(tilt), and hence cruise and wind-holding
    /// speed to sqrt(m*a/drag_k).
    pub pos_max_tilt_deg: f32,
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
            attitude: AttitudeMode::Mpc,
            pos_max_tilt_deg: 15.0,
            alt: AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 },
        }
    }
}

/// A commanded attitude step, for measuring TRACKING rather than only
/// disturbance rejection.
///
/// Without this the harness only ever asks for level flight, and a search
/// scored on it will filter as hard as its bounds allow and wind the
/// integral up as far as it can -- both are free when nothing ever asks the
/// aircraft to move. That is not a hypothetical: the first GA run pinned
/// gyro_fc, d_lpf_tau and ki against their bounds for exactly this reason.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AttitudeStep {
    pub at_s: f32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    /// Return to level at this time. A step that is HELD to the end of the
    /// flight never charges an integrator for its windup -- the offset it
    /// accumulated is exactly what the held command wants. Commanding the
    /// way back is what makes overshoot cost something, and without it a
    /// search drives ki to whatever ceiling it is given.
    pub return_at_s: f32,
}

impl AttitudeStep {
    pub const NONE: AttitudeStep = AttitudeStep {
        at_s: f32::INFINITY,
        roll_deg: 0.0,
        pitch_deg: 0.0,
        return_at_s: f32::INFINITY,
    };
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
    /// Commanded attitude step. `AttitudeStep::NONE` keeps the old
    /// regulate-about-level behaviour exactly.
    pub cmd: AttitudeStep,
    /// Start the aircraft at this attitude, degrees [roll, pitch, yaw].
    ///
    /// For recovery-from-upset testing. Non-zero values disarm the
    /// "attitude > 90 deg means diverged" check UNTIL the aircraft has
    /// recovered once -- a run that starts inverted is diverged by
    /// construction, and scoring that as failure would make the test
    /// vacuous. After the first recovery the check re-arms, so losing it
    /// again still counts.
    pub initial_attitude_deg: [f32; 3],
    /// Close the position loop: the firmware's own PositionController holds
    /// the origin, and its tilt output becomes the MPC reference -- exactly
    /// the cascade main.rs runs. Without this the harness cannot ask any
    /// question about position, which is why its wind column was flat.
    /// Mutually exclusive with `cmd`: both drive the MPC reference.
    pub pos_hold: bool,
    /// Run the FIRMWARE's mode logic (control::modes::nav_step) instead of
    /// the harness's own altitude/position handling.
    ///
    /// Until this existed the sweep reimplemented a simplified cascade, so
    /// it was testing a control path the firmware does not have: no mode
    /// logic, no rescue staging, no heading hold. Two implementations of
    /// one thing, drifting apart -- which is the exact failure this session
    /// kept finding elsewhere. It is also why the harness never caught the
    /// pitch-stick or yaw-reference bugs: it never ran the code containing
    /// them.
    ///
    /// `None` keeps the original behaviour so committed results stay
    /// reproducible.
    pub firmware_mode: Option<FlightMode>,
    /// Start a rescue already in the Navigate stage, i.e. WITHOUT levelling
    /// first. Exists to A/B the staging rather than argue about it: the
    /// position controller is tilt-clamped to 15 deg, so the claim that
    /// levelling protects the recovery needs evidence.
    pub skip_rescue_levelling: bool,
    /// Run the firmware's MEKF and feed the controller ITS attitude
    /// estimate, instead of handing it truth.
    ///
    /// Without this the harness has no estimator in the loop, which has two
    /// consequences it is easy to forget. First, the `accel` degradation
    /// channel has nothing physical to propagate through, so it is
    /// currently a stand-in for attitude error rather than the real thing.
    /// Second, and the reason this exists: the MEKF's accel update treats
    /// the accelerometer as reading gravity, and sustained lateral
    /// acceleration violates that. A quad holding 15 deg of tilt carries
    /// 2.63 m/s2 of unmodelled specific force. Nothing about that is a
    /// sensor imperfection -- a PERFECT accelerometer misleads the
    /// estimator identically -- and the sim already computes the correct
    /// specific force, so this needs plumbing rather than sensor models.
    pub use_estimator: bool,
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
            cmd: AttitudeStep::NONE,
            initial_attitude_deg: [0.0; 3],
            pos_hold: false,
            firmware_mode: None,
            skip_rescue_levelling: false,
            use_estimator: false,
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
    let upset = h.initial_attitude_deg != [0.0; 3];
    if upset {
        sim.set_attitude_deg(
            h.initial_attitude_deg[0],
            h.initial_attitude_deg[1],
            h.initial_attitude_deg[2],
        );
    }
    // Starts armed unless the run begins upset, in which case it arms on
    // the first recovery.
    let mut divergence_armed = !upset;
    let mut recovered_at = None;
    let mut alt_min = f32::INFINITY;
    let mut deg = Degrader::new(cfg, seed);
    // Motors always come from `deg`; only the IMU path forks.
    let mut dual_imu = DualImu::new(h.dual_cfg, seed);

    let mut alt_ctrl = AltitudeController::new(tun.alt, hover_throttle);
    let mut current_thrust = hover_throttle;

    let mut mpc = AttitudeMpc::new();
    mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let mut ref_deg = [0.0f32; 2];
    let mut commanded = false;
    // Runs at the outer rate, which is what main.rs does: pos_ctrl.update
    // sits inside the MPC_PERIOD_US ticker, i.e. 100 Hz.
    let pos_ctrl = PositionController::new(PositionGains {
            max_tilt_rad: tun.pos_max_tilt_deg * DEG2RAD,
            ..PositionGains::default()
        });
    // Firmware mode logic, when driving it. Its own alt/pos controllers
    // live inside, so the harness's are unused in that path.
    let mut nav_state = h.firmware_mode.map(|_| {
        let mut ns = NavState::new(
            AltitudeController::new(tun.alt, hover_throttle),
            PositionController::new(PositionGains {
            max_tilt_rad: tun.pos_max_tilt_deg * DEG2RAD,
            ..PositionGains::default()
        }),
            hover_throttle,
        );
        ns.alt_target = h.target_alt;
        if h.skip_rescue_levelling {
            ns.rescue_stage = crate::control::modes::RescueStage::Navigate;
        }
        ns
    });
    let mut pos_sq = 0.0f64;
    let mut pos_max = 0.0f32;

    let mut rate_pid =
        RatePidController::new(tun.rate, tun.rate, tun.yaw, tun.limits);

    let mut gyro_lpf = if tun.gyro_fc_hz > 0.0 {
        [Biquad::new_lowpass_butterworth(tun.gyro_fc_hz, r.inner_hz()); 3]
    } else {
        [Biquad::identity(); 3]
    };
    let mut primed = false;

    // The firmware's attitude estimator, when we are running it. Primed the
    // way a real boot converges: repeated accel updates against a level
    // sample before the flight starts.
    let mut mekf = h.use_estimator.then(|| {
        let mut m = AttitudeMekf::new(MekfParams::default());
        let level_g = [0.0, 0.0, -1.0];
        for _ in 0..3000 {
            m.predict([0.0; 3], r.dt);
            m.update_accel(level_g);
        }
        m
    });

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

        let angle = if let Some(m) = mekf.as_mut() {
            // Real estimator: drive it exactly as main.rs does -- predict on
            // every IMU tick from the filtered gyro, accel update every
            // ACCEL_DECIMATION-th sample. `angle_err` here is the DEGRADED
            // specific force in m/s2; the MEKF wants g.
            m.predict(
                [gyro[0] * DEG2RAD, gyro[1] * DEG2RAD, gyro[2] * DEG2RAD],
                r.dt,
            );
            if step % r.outer_div == 0 {
                let g = 9.81;
                m.update_accel([
                    angle_err[0] / g,
                    angle_err[1] / g,
                    angle_err[2] / g,
                ]);
            }
            let e = m.euler();
            [e[0] * RAD2DEG, e[1] * RAD2DEG, e[2] * RAD2DEG]
        } else {
            // No estimator: the accel channel stands in for attitude-ESTIMATE
            // error, because it has nothing physical to propagate through.
            // The honest interpretation of that knob in this mode.
            [
                truth.angle[0] + (angle_err[0] - truth.accel[0]),
                truth.angle[1] + (angle_err[1] - truth.accel[1]),
                truth.angle[2] + (angle_err[2] - truth.accel[2]),
            ]
        };

        // Two edges: out to the commanded attitude, then back to level.
        // Skipped under position hold or firmware mode, which own the
        // reference.
        let want = if h.firmware_mode.is_some() {
            ref_deg
        } else if h.pos_hold {
            ref_deg
        } else if t >= h.cmd.return_at_s {
            [0.0, 0.0]
        } else if t >= h.cmd.at_s {
            [h.cmd.roll_deg, h.cmd.pitch_deg]
        } else {
            [0.0, 0.0]
        };
        if want != ref_deg || !commanded {
            commanded = true;
            ref_deg = want;
            mpc.set_reference(
                [ref_deg[0] * DEG2RAD, ref_deg[1] * DEG2RAD, 0.0],
                [0.0, 0.0, 0.0],
            );
        }

        if step % r.outer_div == 0 {
            let alt = -sim.state.z;
            let vz_up = -sim.state.vz;
            let dt_outer = r.dt * r.outer_div as f32;

            if let Some(mode) = h.firmware_mode {
                // Drive the FIRMWARE's mode logic. The estimate is built
                // from truth: this exercises the control path, not the
                // estimator, and a degraded estimator is a separate axis.
                let est = PosEstimate {
                    position_ned: [sim.state.x, sim.state.y, sim.state.z],
                    velocity_ned: [sim.state.vx, sim.state.vy, sim.state.vz],
                    altitude_up: alt,
                    vz_up,
                    altitude_ready: true,
                    home_latched: true,
                    ..PosEstimate::default()
                };
                let out = nav_step(
                    &NavInputs {
                        mode,
                        roll_input: 0.0,
                        pitch_input: 0.0,
                        yaw_input: 0.0,
                        throttle_raw: 0.5,
                        max_angle_deg: 30.0,
                        yaw_rad: sim.state.yaw * DEG2RAD,
                        roll_rad: angle[0] * DEG2RAD,
                        pitch_rad: angle[1] * DEG2RAD,
                        pos_est: Some(est),
                        dt: dt_outer,
                        hover_throttle,
                    },
                    nav_state.as_mut().unwrap(),
                );
                current_thrust = out.thrust;
                ref_deg = [
                    out.desired_roll_rad * RAD2DEG,
                    out.desired_pitch_rad * RAD2DEG,
                ];
                mpc.set_reference(
                    [out.desired_roll_rad, out.desired_pitch_rad, out.desired_yaw_rad],
                    [0.0, 0.0, out.yaw_rate_dps * DEG2RAD],
                );
            } else {
            current_thrust = alt_ctrl.update(h.target_alt, alt, vz_up, dt_outer);

            // Position PD -> tilt reference -> MPC. The firmware's own
            // cascade, with the origin as the target.
            if h.pos_hold {
                let out = pos_ctrl.update(
                    [sim.state.x, sim.state.y],
                    [sim.state.vx, sim.state.vy],
                    [0.0, 0.0],
                    sim.state.yaw * DEG2RAD,
                );
                ref_deg = [out.roll_rad * RAD2DEG, out.pitch_rad * RAD2DEG];
                mpc.set_reference([out.roll_rad, out.pitch_rad, 0.0], [0.0, 0.0, 0.0]);
            }
            }
            trace_due = trace.is_some();

            rate_sp_degs = match tun.attitude {
                AttitudeMode::Mpc => {
                    let angles_rad =
                        [angle[0] * DEG2RAD, angle[1] * DEG2RAD, angle[2] * DEG2RAD];
                    let rates_rad =
                        [gyro[0] * DEG2RAD, gyro[1] * DEG2RAD, gyro[2] * DEG2RAD];
                    let out = mpc.solve(angles_rad, rates_rad);
                    [
                        out.rate_setpoints_rads[0] * RAD2DEG,
                        out.rate_setpoints_rads[1] * RAD2DEG,
                        out.rate_setpoints_rads[2] * RAD2DEG,
                    ]
                }
                AttitudeMode::AnglePid { kp, max_rate_dps } => {
                    // Same reference and same feedback as the MPC gets, so
                    // the only difference is the control law itself.
                    let e = [
                        ref_deg[0] - angle[0],
                        ref_deg[1] - angle[1],
                        0.0 - angle[2],
                    ];
                    [
                        (kp * e[0]).clamp(-max_rate_dps, max_rate_dps),
                        (kp * e[1]).clamp(-max_rate_dps, max_rate_dps),
                        (kp * e[2]).clamp(-max_rate_dps, max_rate_dps),
                    ]
                }
            };
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
        } else if divergence_armed && (roll.abs() > 90.0 || pitch.abs() > 90.0) {
            Some(FailCause::Diverged)
        } else if alt <= 0.0 {
            Some(FailCause::Crashed)
        } else if alt > h.target_alt + FLYAWAY_MARGIN_M {
            Some(FailCause::Flyaway)
        } else if h.pos_hold
            && libm::sqrtf(sim.state.x * sim.state.x + sim.state.y * sim.state.y) > 100.0
        {
            Some(FailCause::Drifted)
        } else {
            None
        };
        if let Some(c) = cause {
            return Metrics::failed(t, c);
        }

        let (re, pe) = (roll - ref_deg[0], pitch - ref_deg[1]);
        let att = libm::sqrtf(re * re + pe * pe);
        alt_min = alt_min.min(alt);
        if recovered_at.is_none() && att < RECOVERY_LEVEL_DEG {
            recovered_at = Some(t);
            divergence_armed = true; // losing it again now counts
        }
        att_max = att_max.max(att);
        att_sq += (att * att) as f64;
        let ae = alt - h.target_alt;
        alt_sq += (ae * ae) as f64;
        let pe = libm::sqrtf(sim.state.x * sim.state.x + sim.state.y * sim.state.y);
        pos_sq += (pe * pe) as f64;
        pos_max = pos_max.max(pe);
    }

    let n = steps as f64;
    Metrics {
        att_rms: libm::sqrt(att_sq / n) as f32,
        att_max,
        alt_rms: libm::sqrt(alt_sq / n) as f32,
        pos_rms: libm::sqrt(pos_sq / n) as f32,
        pos_max,
        air_frac: air_steps as f32 / steps as f32,
        recovered_at,
        alt_min,
        failed_at: None,
    }
}
