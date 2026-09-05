// modes.rs — flight-mode logic, extracted from the navigation task.
//
// This is the block that decides, once per outer tick, what attitude and
// thrust the aircraft should be asking for: sticks in Acro/AltHold, the
// position controller in PosHold/GpsRescue/GpsHome, the descent ladder in
// the two failsafe modes.
//
// It lived inline inside an Embassy task in main.rs, which made it
// unreachable from any test — not because it touched hardware (it does not;
// there is no await, no channel, no peripheral in it) but purely because of
// where it sat. That mattered: the sign error fixed in d1793c6 was in a
// component this block CONSUMES, and nothing could check that the two were
// wired together correctly.
//
// So: pure computation, `&NavInputs` in, `&mut NavState` threaded, an owned
// `NavOutputs` out. No I/O, no clock, no logging. The four defmt lines that
// were here return as `NavEvent`s for the caller to log, which also makes
// them assertable — previously you could not test that "arrived at home"
// fired.
//
// STICK CONVENTION, stated once because three places depend on it:
//
//   Stick UP  ->  INCREASING channel value  ->  positive `pitch_input`
//   Stick UP  ->  aircraft pitches DOWN     ->  NEGATIVE `desired_pitch_rad`
//
// "UP" means up the face of the transmitter, i.e. AWAY from the pilot --
// the same gesture as pushing an aircraft stick forward, just on a control
// surface rotated 90 deg because you are holding a box rather than sitting
// behind a column. Spelt out because "up" is exactly the sort of
// orientation-relative word this whole class of bug hides in.
//
// (nose-down is negative pitch in 3-2-1 Tait-Bryan; see src/conventions.rs.)
//
// So positive pitch_input means "fly forward", and the attitude path must
// NEGATE it while PosHold and the COG gate must not. Acro and AltHold did
// not negate, which inverted the pitch stick in the two modes you take off
// in.
//
// WHY ONLY PITCH. Not because RC and aviation disagree -- a yoke, a
// sidestick and an RC gimbal all move the way you want the nose to move.
// The clash is between the stick DISPLACEMENT and the ANGLE's sign, and it
// falls out of body axes being FRD (X fwd, Y RIGHT, Z DOWN):
//
//   +roll  about X: Y(right) -> Z(down)  =>  right wing DOWN
//   +pitch about Y: Z(down)  -> X(fwd)   =>  nose UP
//
// Positive roll moves a wing down; positive pitch moves the nose up. That
// asymmetry is the whole reason exactly one axis needs the negation, and
// stick-right happens to land on the agreeing one.
//
// It is a consequence of Z-down, not something inherent: in FLU (X fwd,
// Y left, Z up) both axes agree and neither needs negating.
//
// One deliberate behaviour change, called out rather than hidden: the GPS
// loiter timer used `Instant::now()` / `.elapsed()`. It now accumulates
// `dt`. On the fixed 100 Hz ticker these agree, and it removes a hidden
// dependency on the wall clock that would otherwise make the timeout
// untestable.

use crate::control::altitude::AltitudeController;
use crate::control::position::PositionController;

// ---- GPS rescue / failsafe parameters ----
// Moved here with the logic that uses them; every one of these was used
// only by this block.

/// Radius (m) within which GPS-home counts as arrived.
pub const RESCUE_ARRIVAL_RADIUS_M: f32 = 5.0;
/// Descent rate (m/s) during the GPS-home auto-land.
pub const RESCUE_LAND_RATE_MPS: f32 = 0.5;
/// Loiter time (s) at home before auto-landing.
pub const RESCUE_LAND_TIMEOUT_S: f32 = 30.0;
/// Altitude (m) below which the auto-land disarms.
pub const RESCUE_DISARM_ALT_M: f32 = 1.0;
/// Stick deadband, normalised, for altitude and position commands.
pub const ALT_HOLD_DEADBAND: f32 = 0.05;
/// Max commanded climb/descent rate (m/s) at full throttle deflection.
pub const ALT_HOLD_MAX_RATE_MPS: f32 = 2.0;
/// Max commanded horizontal velocity (m/s) at full stick in PosHold.
pub const POS_HOLD_MAX_VEL_MPS: f32 = 5.0;
/// Closed-loop failsafe descent rate (m/s).
pub const FAILSAFE_DESCENT_RATE_MPS: f32 = 0.7;
/// Altitude (m) below which the failsafe descent disarms.
pub const FAILSAFE_LAND_DISARM_ALT_M: f32 = 0.3;
/// Open-loop blind-descent throttle, as a fraction of hover.
pub const FAILSAFE_BLIND_THROTTLE_FRAC: f32 = 0.9;

/// Active flight mode — determines how RC sticks, the position
/// controller, and the altitude controller interact in the control
/// loop. Selected by RC channel 5 (3-position mode switch), channel 6
/// (GPS rescue override), or the arming FSM's failsafe flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum FlightMode {
    /// Sticks → angle (MPC) → rate (PID). Direct throttle.
    Acro,
    /// Sticks → angle (MPC) → rate (PID). Altitude held by
    /// controller; throttle stick commands climb/descend rate.
    AltHold,
    /// Altitude + horizontal position held. Sticks command velocity.
    /// Without GPS home, horizontal hold is best-effort dead-reckoning
    /// damped by NMEA velocity fusion — drift accumulates linearly
    /// rather than quadratically and is acceptable for tens of seconds.
    PosHold,
    /// Autonomous return to GPS home at a safe altitude. Triggered by switch.
    GpsHome,
    /// Failsafe mode (RC lost, GPS home available): hover at home.
    GpsRescue,
    /// Failsafe mode (RC lost, no GPS home, baro alive): closed-loop
    /// controlled descent. Level attitude, alt-hold target ramps down,
    /// auto-disarm at low altitude.
    FailsafeLand,
    /// Failsafe mode (RC lost, no altitude reference at all): open-loop
    /// blind descent. Level attitude, fixed throttle slightly below hover.
    FailsafeBlind,
}

/// Fused position / velocity estimate published by `pos_kf_task`.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub struct PosEstimate {
    /// Position in NED world frame (m), relative to the GPS home point
    /// once the GPS home latch completes. Before home latch the
    /// horizontal components are dead-reckoned and only the altitude
    /// channel is meaningful.
    pub position_ned: [f32; 3],
    /// Velocity in NED world frame (m/s).
    pub velocity_ned: [f32; 3],
    /// Altitude (m, positive up).
    pub altitude_up: f32,
    /// Vertical velocity (m/s, positive up).
    pub vz_up: f32,
    /// Reference pressure latched at arm time (Pa). 0.0 if baro is
    /// absent or arm has not yet fired.
    pub p_ref_pa: f32,
    /// Milliseconds since the last baro update was applied.
    pub baro_age_ms: u32,
    /// True once at least one altitude sensor is anchored. Altitude-hold
    /// and any throttle controller consuming `altitude_up` / `vz_up` must
    /// gate on this.
    pub altitude_ready: bool,
    /// True once a sufficiently good GPS fix has been captured as the home
    /// origin. Horizontal `position_ned` is meaningful only when true.
    pub home_latched: bool,
    /// Monotonic counter incremented each time the PosKF consumes the arm
    /// latch (re-anchors origins + zeros state). Used via `ArmOriginSync` to
    /// withhold target capture until the arm-time re-origin has actually
    /// landed — otherwise it would capture stale pre-zero targets and lurch
    /// on arm.
    pub arm_origin_seq: u32,
}

/// Altitude to climb to during GPS rescue (metres, positive-up).
pub const RESCUE_ALT_M: f32 = 50.0;

/// Switch thresholds, microseconds. Named because an unlabelled 1500 in a
/// failsafe ladder is the kind of thing that gets "tidied".
pub const RESCUE_SWITCH_ON_US: u16 = 1500;
pub const MODE_SWITCH_POSHOLD_US: u16 = 1600;
pub const MODE_SWITCH_ALTHOLD_US: u16 = 1200;

/// Minimum ground speed (m/s) before GPS course is trusted as heading.
pub const V_MIN_COG: f32 = 2.0;
/// Minimum forward pitch-stick deflection, normalised, before GPS course is
/// trusted as heading.
pub const FWD_STICK_MIN: f32 = 0.3;

/// Tilt below which a rescue skips levelling entirely and navigates
/// straight away.
///
/// MEASURED, not assumed. Levelling disables the position loop, and the
/// A/B in sim_sweep says that trade is bad at moderate upsets and free at
/// extreme ones:
///
///     initial roll   staged drift/recov   unstaged drift/recov
///        30 deg        7.8 m / 0.55 s       3.6 m / 0.93 s
///        70 deg       52.1 m / 1.52 s      44.3 m / 1.90 s
///       120 deg      147.5 m / 2.38 s     147.4 m / 2.74 s
///       170 deg      105.6 m / 3.05 s     105.5 m / 3.35 s
///
/// Levelling is ~0.3 s faster to recover throughout, but costs up to 2x
/// the drift below 90 deg -- drifting further from home, during a
/// procedure whose whole point is getting home. Past 90 deg the drift
/// penalty vanishes (the aircraft is ballistic either way) and the 0.3 s
/// is free, and that is also where the position controller's small-angle
/// model stops meaning anything.
///
/// So: level first only when past 90 deg. The original always-level design
/// was wrong, and wrong for an instructive reason -- it assumed the two
/// loops compete for authority, when PositionController is tilt-clamped to
/// 15 deg and cannot meaningfully contest a 70 deg recovery.
pub const RESCUE_LEVEL_MIN_TILT_DEG: f32 = 90.0;

/// Tilt below which the levelling stage of a GPS rescue exits early.
/// Tight on purpose: the stage AIMS at 0 deg, and this is only "close
/// enough that waiting longer buys nothing".
pub const RESCUE_LEVEL_EXIT_DEG: f32 = 5.0;
/// Floor on the levelling stage's give-up timeout, seconds.
pub const RESCUE_LEVEL_FLOOR_S: f32 = 0.5;
/// Give-up timeout per degree of initial tilt, seconds.
///
/// Sized from measured recovery times (sim_sweep's upset axis, MPC at its
/// 40 deg/s command limit): 0.0175-0.0215 s/deg across 30-179 deg, so
/// 0.0215 worst case with a 1.6x margin. That gives ~1.5 s from 30 deg and
/// ~6.7 s from inverted.
///
/// A timeout rather than "wait until level" on purpose. If wind holds the
/// aircraft at 18 deg it will never satisfy a tilt gate, and hanging in the
/// levelling stage forever is worse than flying home at 18 deg of bank.
/// Best-effort beats waiting for a condition that may never arrive.
pub const RESCUE_LEVEL_S_PER_DEG: f32 = 0.0344;

/// Stage of a GPS rescue.
///
/// Rescue used to go straight to commanding tilt toward home, with no
/// attitude precondition. That is backwards for the situation that
/// triggers it: link loss during an aggressive manoeuvre can enter rescue
/// banked hard or inverted, and the first thing the old code did was
/// command MORE tilt. Recover first, then navigate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum RescueStage {
    /// Levelling. Attitude reference is level, the position loop is OFF, so
    /// the aircraft drifts rather than splitting authority between
    /// recovering and chasing a waypoint. Altitude control stays on.
    Level,
    /// Attitude recovered, or the give-up timeout expired: fly home.
    Navigate,
}

/// Inputs to the GPS course-over-ground yaw gate.
#[derive(Clone, Copy, Debug)]
pub struct CogGate {
    pub armed: bool,
    pub has_3d_fix: bool,
    pub ground_speed_ms: f32,
    /// Pitch stick, normalised, SAME channel and sign as `NavInputs::pitch_input`.
    pub pitch_input: f32,
}

/// Should GPS course-over-ground be fused as a yaw reference this tick?
///
/// COG equals heading only in deliberate forward flight, so this requires
/// armed + a good 3D fix + above V_MIN_COG + forward pitch stick. Get the
/// last condition backwards and the gate fires while flying BACKWARDS,
/// where course-over-ground is ~180 deg from heading -- i.e. it injects a
/// half-turn of yaw error straight into the MEKF.
///
/// Positive `pitch_input` is stick-up, which is forward flight -- see
/// STICK CONVENTION in this module's header. This gate was always right
/// about that; it was the attitude path in Acro/AltHold that disagreed,
/// which is what the contradiction test caught.
pub fn should_fuse_cog(g: &CogGate) -> bool {
    g.armed
        && g.has_3d_fix
        && g.ground_speed_ms > V_MIN_COG
        && g.pitch_input > FWD_STICK_MIN
}

/// Inputs to flight-mode selection.
#[derive(Clone, Copy, Debug)]
pub struct ModeSelect {
    pub armed: bool,
    /// Arming FSM's failsafe flag (RC link lost).
    pub failsafe_active: bool,
    /// Baro OR GPS anchored — gates altitude-aware modes.
    pub altitude_ready: bool,
    /// GPS home captured — gates NED-frame modes.
    pub home_latched: bool,
    /// AUX: return-to-home switch, microseconds.
    pub rescue_switch_us: u16,
    /// AUX: three-position mode switch, microseconds.
    pub mode_switch_us: u16,
}

/// Choose the flight mode. Pure, and the failsafe half of it is the ladder
/// that decides what happens when the link drops -- which is precisely the
/// code you cannot afford to test by flying.
pub fn select_mode(s: &ModeSelect) -> FlightMode {
    if !s.armed {
        FlightMode::Acro
    } else if s.failsafe_active {
        // Pick the best descent we can run with what's available.
        if s.home_latched {
            FlightMode::GpsRescue
        } else if s.altitude_ready {
            FlightMode::FailsafeLand
        } else {
            FlightMode::FailsafeBlind
        }
    } else if s.rescue_switch_us > RESCUE_SWITCH_ON_US && s.home_latched {
        FlightMode::GpsHome
    } else if s.mode_switch_us > MODE_SWITCH_POSHOLD_US && s.altitude_ready {
        // PosHold: GPS home gives true position hold; without it, PosKF
        // velocity-fusion damps DR drift and the controller does
        // best-effort horizontal hold for tens of seconds.
        FlightMode::PosHold
    } else if s.mode_switch_us > MODE_SWITCH_ALTHOLD_US && s.altitude_ready {
        FlightMode::AltHold
    } else {
        FlightMode::Acro
    }
}

/// State for mode-entry target capture.
#[derive(Clone, Copy, Debug)]
pub struct EntryState {
    pub prev_mode: FlightMode,
    pub targets_captured: bool,
}

impl EntryState {
    pub fn new() -> Self {
        Self { prev_mode: FlightMode::Acro, targets_captured: false }
    }
}

impl Default for EntryState {
    fn default() -> Self { Self::new() }
}

/// Note a mode change. Returns the previous mode when it changed, so the
/// caller can log the transition; resets the capture flag.
pub fn note_mode_change(mode: FlightMode, es: &mut EntryState) -> Option<FlightMode> {
    if mode != es.prev_mode {
        let was = es.prev_mode;
        es.prev_mode = mode;
        es.targets_captured = false;
        Some(was)
    } else {
        None
    }
}

/// Capture altitude/position targets on entering a mode.
///
/// Gated on `reoriginated`: sampling the estimate before the arm-time
/// re-origin has landed would latch stale pre-zero values and lurch the
/// instant the KF zeroes. Mid-flight switches see it already true.
pub fn capture_targets(
    mode: FlightMode,
    pos_est: Option<PosEstimate>,
    reoriginated: bool,
    es: &mut EntryState,
    nav: &mut NavState,
) {
    if es.targets_captured {
        return;
    }
    let altitude_ready = pos_est.map(|e| e.altitude_ready).unwrap_or(false);
    let home_latched = pos_est.map(|e| e.home_latched).unwrap_or(false);
    // AltHold + FailsafeLand need altitude; PosHold + GPS modes need NED.
    // FailsafeBlind needs nothing (open loop).
    let target_gate = match mode {
        FlightMode::AltHold | FlightMode::FailsafeLand => altitude_ready,
        FlightMode::PosHold => altitude_ready, // best-effort horizontal w/o home
        FlightMode::GpsRescue | FlightMode::GpsHome => home_latched,
        FlightMode::Acro | FlightMode::FailsafeBlind => false,
    };
    if let Some(est) = pos_est.filter(|_| target_gate && reoriginated) {
        match mode {
            FlightMode::AltHold => {
                nav.alt_target = est.altitude_up;
                nav.alt_ctrl.reset();
            }
            FlightMode::PosHold => {
                nav.alt_target = est.altitude_up;
                nav.pos_target = [est.position_ned[0], est.position_ned[1]];
                nav.alt_ctrl.reset();
            }
            FlightMode::GpsRescue => {
                // Hover in place (lock current position and altitude)
                nav.alt_target = est.altitude_up;
                nav.pos_target = [est.position_ned[0], est.position_ned[1]];
                nav.alt_ctrl.reset();
                nav.rescue_stage = RescueStage::Level;
                nav.rescue_level_s = 0.0;
                nav.rescue_level_deadline_s = None;
            }
            FlightMode::GpsHome => {
                // Climb to rescue alt or hold current if already higher.
                nav.alt_target = if est.altitude_up > RESCUE_ALT_M {
                    est.altitude_up
                } else {
                    RESCUE_ALT_M
                };
                nav.pos_target = [0.0, 0.0]; // home is NED origin
                nav.rescue_loiter_s = None;
                nav.rescue_landing = false;
                nav.alt_ctrl.reset();
                nav.rescue_stage = RescueStage::Level;
                nav.rescue_level_s = 0.0;
                nav.rescue_level_deadline_s = None;
            }
            FlightMode::FailsafeLand => {
                // Start descent from current altitude; nav_step ramps
                // alt_target down at FAILSAFE_DESCENT_RATE_MPS.
                nav.alt_target = est.altitude_up;
                nav.alt_ctrl.reset();
            }
            FlightMode::Acro | FlightMode::FailsafeBlind => {}
        }
        es.targets_captured = true;
    }
}

/// Everything the mode logic reads. All borrowed or Copy; nothing here is
/// mutated.
#[derive(Clone, Copy, Debug)]
pub struct NavInputs {
    pub mode: FlightMode,
    /// Normalised stick inputs, -1..=1 (throttle 0..=1).
    pub roll_input: f32,
    pub pitch_input: f32,
    pub yaw_input: f32,
    pub throttle_raw: f32,
    /// Full-stick attitude command, degrees.
    pub max_angle_deg: f32,
    /// Current heading, radians. Used to rotate stick and position demands
    /// into the body frame.
    pub yaw_rad: f32,
    /// Current roll and pitch, radians. Needed to know whether a rescue has
    /// recovered attitude yet.
    pub roll_rad: f32,
    pub pitch_rad: f32,
    /// Latest fused estimate, if the KF has published one.
    pub pos_est: Option<PosEstimate>,
    /// Outer-loop period, seconds.
    pub dt: f32,
    pub hover_throttle: f32,
}

/// State carried between ticks. Borrowed mutably by `nav_step`.
pub struct NavState {
    /// Commanded altitude, metres positive-up.
    pub alt_target: f32,
    /// Commanded horizontal position, [north, east] metres.
    pub pos_target: [f32; 2],
    /// Seconds spent loitering at home; `None` until arrival.
    pub rescue_loiter_s: Option<f32>,
    /// True once the GPS-home auto-land has begun.
    pub rescue_landing: bool,
    /// Latched heading to hold, radians, when the pilot is not commanding
    /// yaw. `None` until the first tick latches the current heading.
    ///
    /// Yaw is the one axis with no natural "return to level": there is no
    /// privileged heading. Holding a fixed 0 means holding NORTH, which is
    /// what this used to do.
    pub yaw_target_rad: Option<f32>,
    /// Which stage of a GPS rescue we are in.
    pub rescue_stage: RescueStage,
    /// Seconds spent in the levelling stage.
    pub rescue_level_s: f32,
    /// Give-up deadline for levelling, computed from the tilt observed on
    /// the first tick of the stage. `None` until then.
    pub rescue_level_deadline_s: Option<f32>,
    /// Last commanded thrust. Genuinely state, not just an output: AltHold
    /// leaves it untouched when no altitude estimate is available, so the
    /// previous value carries.
    pub current_thrust: f32,
    pub alt_ctrl: AltitudeController,
    pub pos_ctrl: PositionController,
}

impl NavState {
    pub fn new(alt_ctrl: AltitudeController, pos_ctrl: PositionController,
               hover_throttle: f32) -> Self {
        Self {
            alt_target: 0.0,
            pos_target: [0.0, 0.0],
            rescue_loiter_s: None,
            rescue_landing: false,
            yaw_target_rad: None,
            rescue_stage: RescueStage::Level,
            rescue_level_s: 0.0,
            rescue_level_deadline_s: None,
            current_thrust: hover_throttle,
            alt_ctrl,
            pos_ctrl,
        }
    }
}

/// Something the caller should log or act on. These were `defmt::info!`
/// calls inline; returning them keeps the logging in main.rs and makes the
/// transitions assertable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum NavEvent {
    /// Rescue levelling finished: attitude recovered.
    RescueLevelled,
    /// Rescue levelling gave up on time and is proceeding best-effort.
    RescueLevelTimeout,
    /// GPS-home auto-land reached the disarm floor.
    AutoLandComplete,
    /// Entered the arrival radius; loiter timer started.
    ArrivedAtHome,
    /// Loiter expired; auto-land beginning.
    LoiterTimeout,
    /// Failsafe descent reached the disarm floor.
    FailsafeFloorReached,
}

/// What the mode logic produced this tick.
#[derive(Clone, Copy, Debug)]
pub struct NavOutputs {
    pub desired_roll_rad: f32,
    pub desired_pitch_rad: f32,
    pub desired_yaw_rad: f32,
    pub yaw_rate_dps: f32,
    pub thrust: f32,
    /// The logic wants the arming FSM disarmed. Previously a direct
    /// `arming.force_disarm()` call from inside the block.
    pub disarm: bool,
    pub event: Option<NavEvent>,
}

/// One outer-loop tick of mode logic.
///
/// Transcribed from the match block that lived in `navigation_task`. The
/// only intended behaviour change is the loiter timer (see the module
/// header); everything else is the same arithmetic in the same order.
/// Shortest signed difference between two angles, radians.
fn wrap_pi(a: f32) -> f32 {
    const TAU: f32 = 2.0 * core::f32::consts::PI;
    let mut x = a;
    while x > core::f32::consts::PI {
        x -= TAU;
    }
    while x < -core::f32::consts::PI {
        x += TAU;
    }
    x
}

/// Current tilt from level, degrees.
fn tilt_deg(inp: &NavInputs) -> f32 {
    const R2D: f32 = 180.0 / core::f32::consts::PI;
    libm::sqrtf(inp.roll_rad * inp.roll_rad + inp.pitch_rad * inp.pitch_rad) * R2D
}

/// Advance the rescue levelling stage. Returns true while still levelling,
/// i.e. while the caller must NOT engage the position loop.
fn rescue_levelling(inp: &NavInputs, st: &mut NavState) -> (bool, Option<NavEvent>) {
    if st.rescue_stage == RescueStage::Navigate {
        return (false, None);
    }
    let tilt = tilt_deg(inp);
    if st.rescue_level_deadline_s.is_none() && tilt < RESCUE_LEVEL_MIN_TILT_DEG {
        // Not upset enough to be worth surrendering position control.
        st.rescue_stage = RescueStage::Navigate;
        return (false, None);
    }
    let deadline = *st.rescue_level_deadline_s.get_or_insert_with(|| {
        RESCUE_LEVEL_FLOOR_S + RESCUE_LEVEL_S_PER_DEG * tilt
    });
    st.rescue_level_s += inp.dt;

    if tilt < RESCUE_LEVEL_EXIT_DEG {
        st.rescue_stage = RescueStage::Navigate;
        (false, Some(NavEvent::RescueLevelled))
    } else if st.rescue_level_s >= deadline {
        // Best effort: flying home at 18 deg of bank beats hanging here
        // waiting for a condition the wind may never allow.
        st.rescue_stage = RescueStage::Navigate;
        (false, Some(NavEvent::RescueLevelTimeout))
    } else {
        (true, None)
    }
}

pub fn nav_step(inp: &NavInputs, st: &mut NavState) -> NavOutputs {
    const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
    let dt = inp.dt;

    // ---- Heading hold ----
    //
    // While the pilot commands yaw the target tracks the current heading,
    // so releasing the stick holds wherever they stopped. Every mode used
    // to pass desired_yaw_rad = 0, which is not "hold heading" -- it is
    // "point NORTH", and the MPC weights yaw at 2.0, so it really did steer
    // back (about -42 deg/s from a heading of east).
    //
    // The reference is expressed as current heading plus the SHORTEST arc
    // to the target. Handing the MPC an absolute angle would give it a ~2pi
    // error at the wrap point and send it the long way round.
    let piloting_yaw = libm::fabsf(inp.yaw_input) > ALT_HOLD_DEADBAND;
    if piloting_yaw || st.yaw_target_rad.is_none() {
        st.yaw_target_rad = Some(inp.yaw_rad);
    }
    let yaw_ref = inp.yaw_rad
        + wrap_pi(st.yaw_target_rad.unwrap_or(inp.yaw_rad) - inp.yaw_rad);

    // Declared without initialisers: every arm assigns all four, and a
    // placeholder would let a future arm silently forget one.
    let desired_roll_rad;
    let desired_pitch_rad;
    let desired_yaw_rad;
    let yaw_rate_dps;
    let mut disarm = false;
    let mut event = None;

    match inp.mode {
        FlightMode::Acro => {
            desired_roll_rad = inp.roll_input * inp.max_angle_deg * DEG2RAD;
            // Negated: see STICK CONVENTION in the module header. Stick up
            // is a positive channel value and must pitch the aircraft DOWN,
            // and nose-down is NEGATIVE pitch in 3-2-1 Tait-Bryan.
            desired_pitch_rad = -inp.pitch_input * inp.max_angle_deg * DEG2RAD;
            desired_yaw_rad = yaw_ref;
            yaw_rate_dps = inp.yaw_input * 200.0;
            // Direct throttle pass-through
            st.current_thrust = inp.throttle_raw.clamp(0.0, 1.0);
        }
        FlightMode::AltHold => {
            desired_roll_rad = inp.roll_input * inp.max_angle_deg * DEG2RAD;
            // Negated, as in Acro -- see STICK CONVENTION in the header.
            desired_pitch_rad = -inp.pitch_input * inp.max_angle_deg * DEG2RAD;
            desired_yaw_rad = yaw_ref;
            yaw_rate_dps = inp.yaw_input * 200.0;
            // Throttle stick -> climb/descend rate -> alt target adjustment
            let thr_centered = inp.throttle_raw - 0.5; // -0.5..+0.5
            if libm::fabsf(thr_centered) > ALT_HOLD_DEADBAND {
                let rate = thr_centered * 2.0 * ALT_HOLD_MAX_RATE_MPS;
                st.alt_target += rate * dt;
            }
            if let Some(est) = inp.pos_est.filter(|e| e.altitude_ready) {
                st.current_thrust =
                    st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
            }
        }
        FlightMode::PosHold => {
            yaw_rate_dps = inp.yaw_input * 200.0;
            // Sticks -> velocity -> position target offset
            if libm::fabsf(inp.roll_input) > ALT_HOLD_DEADBAND
                || libm::fabsf(inp.pitch_input) > ALT_HOLD_DEADBAND
            {
                // Body -> world is R(yaw); this used to be R(yaw) TRANSPOSED,
                // i.e. the world->body form that position.rs uses correctly
                // for its own rotation, copied into a place needing the
                // inverse. The effect was that yawing reversed which way the
                // sticks moved the position target.
                let cos_yaw = libm::cosf(inp.yaw_rad);
                let sin_yaw = libm::sinf(inp.yaw_rad);
                let vn = (cos_yaw * inp.pitch_input - sin_yaw * inp.roll_input)
                    * POS_HOLD_MAX_VEL_MPS;
                let ve = (sin_yaw * inp.pitch_input + cos_yaw * inp.roll_input)
                    * POS_HOLD_MAX_VEL_MPS;
                st.pos_target[0] += vn * dt;
                st.pos_target[1] += ve * dt;
            }
            // Throttle -> altitude target
            let thr_centered = inp.throttle_raw - 0.5;
            if libm::fabsf(thr_centered) > ALT_HOLD_DEADBAND {
                st.alt_target += thr_centered * 2.0 * ALT_HOLD_MAX_RATE_MPS * dt;
            }
            if let Some(est) = inp.pos_est.filter(|e| e.home_latched) {
                let pos_out = st.pos_ctrl.update(
                    [est.position_ned[0], est.position_ned[1]],
                    [est.velocity_ned[0], est.velocity_ned[1]],
                    st.pos_target,
                    inp.yaw_rad,
                );
                desired_roll_rad = pos_out.roll_rad;
                desired_pitch_rad = pos_out.pitch_rad;
                st.current_thrust =
                    st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
            } else {
                desired_roll_rad = 0.0;
                desired_pitch_rad = 0.0;
                st.current_thrust = inp.hover_throttle;
            }
            desired_yaw_rad = yaw_ref;
        }
        FlightMode::GpsRescue => {
            // Failsafe: level first, then hover in place.
            yaw_rate_dps = 0.0;
            desired_yaw_rad = yaw_ref;
            let (levelling, ev) = rescue_levelling(inp, st);
            if ev.is_some() {
                event = ev;
            }
            if levelling {
                // Position loop OFF: all attitude authority goes to
                // recovery. The aircraft drifts meanwhile, which is the
                // deliberate trade.
                desired_roll_rad = 0.0;
                desired_pitch_rad = 0.0;
                if let Some(est) = inp.pos_est.filter(|e| e.altitude_ready) {
                    st.current_thrust =
                        st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
                }
            } else if let Some(est) = inp.pos_est.filter(|e| e.home_latched) {
                let pos_out = st.pos_ctrl.update(
                    [est.position_ned[0], est.position_ned[1]],
                    [est.velocity_ned[0], est.velocity_ned[1]],
                    st.pos_target,
                    inp.yaw_rad,
                );
                desired_roll_rad = pos_out.roll_rad;
                desired_pitch_rad = pos_out.pitch_rad;
                st.current_thrust =
                    st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
            } else {
                desired_roll_rad = 0.0;
                desired_pitch_rad = 0.0;
                st.current_thrust = inp.hover_throttle;
            }
        }
        FlightMode::GpsHome => {
            // Level first, then return to home.
            yaw_rate_dps = 0.0;
            desired_yaw_rad = yaw_ref;
            let (levelling, ev) = rescue_levelling(inp, st);
            if ev.is_some() {
                event = ev;
            }
            if levelling {
                desired_roll_rad = 0.0;
                desired_pitch_rad = 0.0;
                if let Some(est) = inp.pos_est.filter(|e| e.altitude_ready) {
                    st.current_thrust =
                        st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
                }
            } else if let Some(est) = inp.pos_est.filter(|e| e.home_latched) {
                let dist_home = libm::sqrtf(
                    est.position_ned[0] * est.position_ned[0]
                        + est.position_ned[1] * est.position_ned[1],
                );

                // Auto-land sequence
                if st.rescue_landing {
                    st.alt_target -= RESCUE_LAND_RATE_MPS * dt;
                    if est.altitude_up < RESCUE_DISARM_ALT_M {
                        event = Some(NavEvent::AutoLandComplete);
                        disarm = true;
                    }
                } else if dist_home < RESCUE_ARRIVAL_RADIUS_M {
                    // Arrived -- start loiter timer
                    if st.rescue_loiter_s.is_none() {
                        event = Some(NavEvent::ArrivedAtHome);
                        st.rescue_loiter_s = Some(0.0);
                    }
                    if let Some(loiter_s) = st.rescue_loiter_s.as_mut() {
                        *loiter_s += dt;
                        if *loiter_s > RESCUE_LAND_TIMEOUT_S {
                            event = Some(NavEvent::LoiterTimeout);
                            st.rescue_landing = true;
                        }
                    }
                }

                let pos_out = st.pos_ctrl.update(
                    [est.position_ned[0], est.position_ned[1]],
                    [est.velocity_ned[0], est.velocity_ned[1]],
                    st.pos_target,
                    inp.yaw_rad,
                );
                desired_roll_rad = pos_out.roll_rad;
                desired_pitch_rad = pos_out.pitch_rad;
                st.current_thrust =
                    st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
            } else {
                desired_roll_rad = 0.0;
                desired_pitch_rad = 0.0;
                st.current_thrust = inp.hover_throttle;
            }
        }
        FlightMode::FailsafeLand => {
            // RC lost, no GPS home, baro alive: closed-loop controlled
            // descent. Disarm when altitude crosses the floor. No timeout --
            // altitude-floor is the sole stop condition.
            yaw_rate_dps = 0.0;
            desired_yaw_rad = yaw_ref;
            desired_roll_rad = 0.0;
            desired_pitch_rad = 0.0;
            st.alt_target -= FAILSAFE_DESCENT_RATE_MPS * dt;
            if let Some(est) = inp.pos_est.filter(|e| e.altitude_ready) {
                st.current_thrust =
                    st.alt_ctrl.update(st.alt_target, est.altitude_up, est.vz_up, dt);
                if est.altitude_up < FAILSAFE_LAND_DISARM_ALT_M {
                    event = Some(NavEvent::FailsafeFloorReached);
                    disarm = true;
                }
            } else {
                // Altitude went stale mid-descent -- blind throttle for this
                // tick. Mode selection switches to FailsafeBlind next pass.
                st.current_thrust = inp.hover_throttle * FAILSAFE_BLIND_THROTTLE_FRAC;
            }
        }
        FlightMode::FailsafeBlind => {
            // RC lost AND no altitude reference. Open-loop throttle slightly
            // below hover, level attitude. No auto-disarm -- without altitude
            // we cannot tell when to stop.
            yaw_rate_dps = 0.0;
            desired_yaw_rad = yaw_ref;
            desired_roll_rad = 0.0;
            desired_pitch_rad = 0.0;
            st.current_thrust = inp.hover_throttle * FAILSAFE_BLIND_THROTTLE_FRAC;
        }
    }

    NavOutputs {
        desired_roll_rad,
        desired_pitch_rad,
        desired_yaw_rad,
        yaw_rate_dps,
        thrust: st.current_thrust,
        disarm,
        event,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::altitude::AltitudeGains;
    use crate::control::position::PositionGains;

    const DT: f32 = 0.01; // 100 Hz outer loop
    const D2R_T: f32 = core::f32::consts::PI / 180.0;
    const HOVER: f32 = 0.294;

    fn state() -> NavState {
        NavState::new(
            AltitudeController::new(
                AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 },
                HOVER,
            ),
            PositionController::new(PositionGains::default()),
            HOVER,
        )
    }

    fn inputs(mode: FlightMode) -> NavInputs {
        NavInputs {
            mode,
            roll_input: 0.0,
            pitch_input: 0.0,
            yaw_input: 0.0,
            throttle_raw: 0.5,
            max_angle_deg: 30.0,
            yaw_rad: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            pos_est: None,
            dt: DT,
            hover_throttle: HOVER,
        }
    }

    fn est() -> PosEstimate {
        PosEstimate {
            altitude_ready: true,
            home_latched: true,
            altitude_up: 10.0,
            ..PosEstimate::default()
        }
    }

    // ---- Acro / AltHold ----

    #[test]
    fn acro_passes_sticks_through_and_throttle_direct() {
        let mut st = state();
        let mut inp = inputs(FlightMode::Acro);
        inp.roll_input = 0.5;   // right
        inp.pitch_input = 0.5;  // stick UP
        inp.throttle_raw = 0.7;
        let out = nav_step(&inp, &mut st);
        // Stick right -> roll right -> positive roll.
        assert!((out.desired_roll_rad - 15.0_f32.to_radians()).abs() < 1e-5);
        // Stick UP -> nose DOWN -> NEGATIVE pitch. This is the assertion
        // that was inverted, in the test as well as in the code.
        assert!((out.desired_pitch_rad + 15.0_f32.to_radians()).abs() < 1e-5,
                "stick up must pitch nose-down, got {}", out.desired_pitch_rad);
        assert!((out.thrust - 0.7).abs() < 1e-6, "direct throttle");
    }

    #[test]
    fn althold_throttle_inside_deadband_does_not_move_the_target() {
        let mut st = state();
        let mut inp = inputs(FlightMode::AltHold);
        inp.pos_est = Some(est());
        inp.throttle_raw = 0.5 + ALT_HOLD_DEADBAND * 0.5; // inside
        let before = st.alt_target;
        nav_step(&inp, &mut st);
        assert_eq!(st.alt_target, before);
    }

    #[test]
    fn althold_throttle_outside_deadband_climbs() {
        let mut st = state();
        let mut inp = inputs(FlightMode::AltHold);
        inp.pos_est = Some(est());
        inp.throttle_raw = 1.0; // full up
        nav_step(&inp, &mut st);
        let expect = 0.5 * 2.0 * ALT_HOLD_MAX_RATE_MPS * DT;
        assert!((st.alt_target - expect).abs() < 1e-6, "alt_target {}", st.alt_target);
    }

    /// Thrust is state, not just an output: with no altitude reference
    /// AltHold leaves it alone and the previous command carries. Worth
    /// pinning, because it is the one place the distinction is visible.
    #[test]
    fn althold_without_altitude_reference_holds_the_previous_thrust() {
        let mut st = state();
        st.current_thrust = 0.61;
        let inp = inputs(FlightMode::AltHold); // pos_est None
        let out = nav_step(&inp, &mut st);
        assert!((out.thrust - 0.61).abs() < 1e-6, "thrust {}", out.thrust);
    }

    // ---- PosHold ----

    /// Stick demands are body-frame and must be rotated into the world by
    /// R(yaw). This caught the transpose fixed alongside it.
    ///
    /// Deliberately stated without reference to what a positive pitch stick
    /// MEANS, so it holds whatever that convention turns out to be: whatever
    /// direction a given stick moves the target at yaw=0, at yaw=+90 deg it
    /// must move it 90 deg clockwise (north -> east). Transposed, it went
    /// counter-clockwise.
    #[test]
    fn poshold_stick_rotation_follows_yaw() {
        let mut north = state();
        let mut inp = inputs(FlightMode::PosHold);
        inp.pos_est = Some(est());
        inp.pitch_input = 1.0;
        nav_step(&inp, &mut north);
        let at_zero = north.pos_target; // some direction in the N/E plane

        let mut east = state();
        inp.yaw_rad = core::f32::consts::FRAC_PI_2;
        nav_step(&inp, &mut east);
        let at_ninety = east.pos_target;

        // Rotating the yaw=0 result by +90 deg (n,e) -> (-e, n) must give
        // the yaw=90 result.
        let expect = [-at_zero[1], at_zero[0]];
        assert!(
            (at_ninety[0] - expect[0]).abs() < 1e-6
                && (at_ninety[1] - expect[1]).abs() < 1e-6,
            "yaw+90 gave {at_ninety:?}, expected {expect:?} (rotation is transposed)"
        );
    }

    #[test]
    fn poshold_without_home_latch_levels_and_hovers() {
        let mut st = state();
        let mut inp = inputs(FlightMode::PosHold);
        inp.pos_est = Some(PosEstimate { home_latched: false, ..est() });
        let out = nav_step(&inp, &mut st);
        assert_eq!(out.desired_roll_rad, 0.0);
        assert_eq!(out.desired_pitch_rad, 0.0);
        assert!((out.thrust - HOVER).abs() < 1e-6);
    }

    // ---- The wiring test ----

    /// THE test this whole extraction was for. Nothing could previously
    /// check that the mode logic and the position controller were wired
    /// together with a consistent sign: position.rs was unit-testable and
    /// the MEKF was unit-testable, but the code joining them lived inside
    /// an Embassy task.
    ///
    /// South of home, GPS-home must command NOSE-DOWN to fly north.
    #[test]
    fn gps_home_south_of_target_commands_nose_down() {
        let mut st = state();
        let mut inp = inputs(FlightMode::GpsHome);
        inp.pos_est = Some(PosEstimate {
            position_ned: [-30.0, 0.0, 0.0], // 30 m south of home
            ..est()
        });
        let out = nav_step(&inp, &mut st);
        assert!(
            out.desired_pitch_rad < 0.0,
            "must pitch nose-down to fly north; got {}",
            out.desired_pitch_rad
        );
    }

    /// And west of home, roll RIGHT to fly east.
    #[test]
    fn gps_home_west_of_target_commands_right_roll() {
        let mut st = state();
        let mut inp = inputs(FlightMode::GpsHome);
        inp.pos_est = Some(PosEstimate {
            position_ned: [0.0, -30.0, 0.0], // 30 m west of home
            ..est()
        });
        let out = nav_step(&inp, &mut st);
        assert!(out.desired_roll_rad > 0.0, "roll right to fly east; got {}", out.desired_roll_rad);
    }

    // ---- The rescue ladder ----

    #[test]
    fn gps_home_arrival_fires_once_and_starts_the_loiter_timer() {
        let mut st = state();
        let mut inp = inputs(FlightMode::GpsHome);
        inp.pos_est = Some(PosEstimate { position_ned: [1.0, 0.0, 0.0], ..est() });
        let first = nav_step(&inp, &mut st);
        assert_eq!(first.event, Some(NavEvent::ArrivedAtHome));
        let second = nav_step(&inp, &mut st);
        assert_eq!(second.event, None, "arrival must not re-fire");
        assert!(st.rescue_loiter_s.unwrap() > 0.0);
    }

    #[test]
    fn gps_home_loiter_timeout_starts_the_auto_land() {
        let mut st = state();
        let mut inp = inputs(FlightMode::GpsHome);
        inp.pos_est = Some(PosEstimate { position_ned: [1.0, 0.0, 0.0], ..est() });
        let ticks = (RESCUE_LAND_TIMEOUT_S / DT) as usize + 5;
        let mut saw_timeout = false;
        for _ in 0..ticks {
            if nav_step(&inp, &mut st).event == Some(NavEvent::LoiterTimeout) {
                saw_timeout = true;
                break;
            }
        }
        assert!(saw_timeout, "loiter must time out after {RESCUE_LAND_TIMEOUT_S}s");
        assert!(st.rescue_landing);
    }

    #[test]
    fn gps_home_auto_land_ramps_down_and_disarms_at_the_floor() {
        let mut st = state();
        st.rescue_landing = true;
        st.alt_target = 20.0;
        let mut inp = inputs(FlightMode::GpsHome);
        inp.pos_est = Some(PosEstimate { position_ned: [1.0, 0.0, 0.0], ..est() });
        let out = nav_step(&inp, &mut st);
        assert!(st.alt_target < 20.0, "target must ramp down");
        assert!(!out.disarm, "still at 10 m, must not disarm");

        inp.pos_est = Some(PosEstimate {
            position_ned: [1.0, 0.0, 0.0],
            altitude_up: RESCUE_DISARM_ALT_M - 0.1,
            ..est()
        });
        let out = nav_step(&inp, &mut st);
        assert!(out.disarm, "below the floor it must disarm");
        assert_eq!(out.event, Some(NavEvent::AutoLandComplete));
    }

    // ---- Failsafe ladder ----

    #[test]
    fn failsafe_land_descends_level_and_disarms_at_the_floor() {
        let mut st = state();
        st.alt_target = 5.0;
        let mut inp = inputs(FlightMode::FailsafeLand);
        inp.pos_est = Some(est());
        let out = nav_step(&inp, &mut st);
        assert_eq!(out.desired_roll_rad, 0.0);
        assert_eq!(out.desired_pitch_rad, 0.0);
        assert!((st.alt_target - (5.0 - FAILSAFE_DESCENT_RATE_MPS * DT)).abs() < 1e-6);
        assert!(!out.disarm);

        inp.pos_est = Some(PosEstimate {
            altitude_up: FAILSAFE_LAND_DISARM_ALT_M - 0.05,
            ..est()
        });
        let out = nav_step(&inp, &mut st);
        assert!(out.disarm);
        assert_eq!(out.event, Some(NavEvent::FailsafeFloorReached));
    }

    /// Altitude going stale mid-descent must fall back to blind throttle
    /// rather than holding a stale closed-loop command.
    #[test]
    fn failsafe_land_without_altitude_falls_back_to_blind_throttle() {
        let mut st = state();
        let mut inp = inputs(FlightMode::FailsafeLand);
        inp.pos_est = Some(PosEstimate { altitude_ready: false, ..est() });
        let out = nav_step(&inp, &mut st);
        assert!((out.thrust - HOVER * FAILSAFE_BLIND_THROTTLE_FRAC).abs() < 1e-6);
        assert!(!out.disarm, "cannot disarm without an altitude reference");
    }

    #[test]
    fn failsafe_blind_is_level_and_never_disarms() {
        let mut st = state();
        let inp = inputs(FlightMode::FailsafeBlind);
        let out = nav_step(&inp, &mut st);
        assert_eq!(out.desired_roll_rad, 0.0);
        assert_eq!(out.desired_pitch_rad, 0.0);
        assert_eq!(out.yaw_rate_dps, 0.0);
        assert!((out.thrust - HOVER * FAILSAFE_BLIND_THROTTLE_FRAC).abs() < 1e-6);
        assert!(!out.disarm, "no altitude reference means no stop condition");
    }

    // ---- Mode selection: the failsafe ladder ----

    fn sel() -> ModeSelect {
        ModeSelect {
            armed: true,
            failsafe_active: false,
            altitude_ready: true,
            home_latched: true,
            rescue_switch_us: 1000,
            mode_switch_us: 1000,
        }
    }

    #[test]
    fn disarmed_is_always_acro_whatever_the_switches_say() {
        let s = ModeSelect {
            armed: false,
            failsafe_active: true,
            rescue_switch_us: 2000,
            mode_switch_us: 2000,
            ..sel()
        };
        assert_eq!(select_mode(&s), FlightMode::Acro);
    }

    /// The ladder that runs when the link drops. Each rung degrades to the
    /// best descent the remaining sensors allow.
    #[test]
    fn failsafe_degrades_through_the_ladder_as_sensors_are_lost() {
        let base = ModeSelect { failsafe_active: true, ..sel() };
        assert_eq!(
            select_mode(&base),
            FlightMode::GpsRescue,
            "home latched: hover at home"
        );
        assert_eq!(
            select_mode(&ModeSelect { home_latched: false, ..base }),
            FlightMode::FailsafeLand,
            "no home but altitude: closed-loop descent"
        );
        assert_eq!(
            select_mode(&ModeSelect { home_latched: false, altitude_ready: false, ..base }),
            FlightMode::FailsafeBlind,
            "nothing left: open-loop blind descent"
        );
    }

    /// Failsafe outranks every pilot switch. If this ever stopped holding,
    /// a stuck switch could keep the aircraft out of its descent.
    #[test]
    fn failsafe_overrides_the_pilot_switches() {
        let s = ModeSelect {
            failsafe_active: true,
            rescue_switch_us: 2000,
            mode_switch_us: 2000,
            ..sel()
        };
        assert_eq!(select_mode(&s), FlightMode::GpsRescue);
    }

    #[test]
    fn switches_select_gps_home_poshold_althold_acro_in_priority_order() {
        assert_eq!(
            select_mode(&ModeSelect { rescue_switch_us: 1900, mode_switch_us: 1900, ..sel() }),
            FlightMode::GpsHome,
            "rescue switch outranks the mode switch"
        );
        assert_eq!(
            select_mode(&ModeSelect { mode_switch_us: 1700, ..sel() }),
            FlightMode::PosHold
        );
        assert_eq!(
            select_mode(&ModeSelect { mode_switch_us: 1300, ..sel() }),
            FlightMode::AltHold
        );
        assert_eq!(select_mode(&sel()), FlightMode::Acro);
    }

    /// Modes that need a sensor must not be selectable without it. This is
    /// the guard that stops AltHold engaging with no altitude reference.
    #[test]
    fn modes_requiring_sensors_fall_back_when_those_sensors_are_absent() {
        let no_alt = ModeSelect { altitude_ready: false, ..sel() };
        assert_eq!(
            select_mode(&ModeSelect { mode_switch_us: 1700, ..no_alt }),
            FlightMode::Acro,
            "PosHold needs altitude"
        );
        assert_eq!(
            select_mode(&ModeSelect { mode_switch_us: 1300, ..no_alt }),
            FlightMode::Acro,
            "AltHold needs altitude"
        );
        assert_eq!(
            select_mode(&ModeSelect { rescue_switch_us: 1900, home_latched: false, ..sel() }),
            FlightMode::Acro,
            "GpsHome needs home"
        );
    }

    // ---- Mode entry / target capture ----

    #[test]
    fn entering_althold_captures_the_current_altitude_as_target() {
        let (mut es, mut nav) = (EntryState::new(), state());
        let e = PosEstimate { altitude_up: 12.5, ..est() };
        note_mode_change(FlightMode::AltHold, &mut es);
        capture_targets(FlightMode::AltHold, Some(e), true, &mut es, &mut nav);
        assert!((nav.alt_target - 12.5).abs() < 1e-6);
        assert!(es.targets_captured);
    }

    /// The arm-into-altitude-mode lurch: capturing before the PosKF has
    /// re-origined latches a stale pre-zero altitude, and the aircraft
    /// jumps the instant the KF zeroes.
    #[test]
    fn capture_waits_for_the_arm_time_reorigin() {
        let (mut es, mut nav) = (EntryState::new(), state());
        let e = PosEstimate { altitude_up: 12.5, ..est() };
        note_mode_change(FlightMode::AltHold, &mut es);
        capture_targets(FlightMode::AltHold, Some(e), false, &mut es, &mut nav);
        assert!(!es.targets_captured, "must not capture before re-origin");
        assert_eq!(nav.alt_target, 0.0);

        capture_targets(FlightMode::AltHold, Some(e), true, &mut es, &mut nav);
        assert!(es.targets_captured, "captures once the zero lands");
        assert!((nav.alt_target - 12.5).abs() < 1e-6);
    }

    #[test]
    fn capture_happens_once_per_mode_entry() {
        let (mut es, mut nav) = (EntryState::new(), state());
        note_mode_change(FlightMode::AltHold, &mut es);
        capture_targets(FlightMode::AltHold, Some(PosEstimate { altitude_up: 10.0, ..est() }),
                        true, &mut es, &mut nav);
        capture_targets(FlightMode::AltHold, Some(PosEstimate { altitude_up: 99.0, ..est() }),
                        true, &mut es, &mut nav);
        assert!((nav.alt_target - 10.0).abs() < 1e-6, "second call must not re-capture");
    }

    /// GpsHome climbs to the rescue altitude, or holds current if already
    /// higher -- never descends to reach it.
    #[test]
    fn entering_gps_home_climbs_to_rescue_altitude_but_never_descends() {
        let (mut es, mut nav) = (EntryState::new(), state());
        note_mode_change(FlightMode::GpsHome, &mut es);
        capture_targets(FlightMode::GpsHome, Some(PosEstimate { altitude_up: 10.0, ..est() }),
                        true, &mut es, &mut nav);
        assert!((nav.alt_target - RESCUE_ALT_M).abs() < 1e-6, "below: climb to rescue alt");
        assert_eq!(nav.pos_target, [0.0, 0.0], "home is the NED origin");

        let (mut es, mut nav) = (EntryState::new(), state());
        note_mode_change(FlightMode::GpsHome, &mut es);
        capture_targets(FlightMode::GpsHome,
                        Some(PosEstimate { altitude_up: RESCUE_ALT_M + 20.0, ..est() }),
                        true, &mut es, &mut nav);
        assert!((nav.alt_target - (RESCUE_ALT_M + 20.0)).abs() < 1e-6, "above: hold current");
    }

    /// Re-entering GpsHome must clear a previous rescue's landing state,
    /// or the second rescue starts already descending.
    #[test]
    fn entering_gps_home_clears_stale_rescue_state() {
        let (mut es, mut nav) = (EntryState::new(), state());
        nav.rescue_landing = true;
        nav.rescue_loiter_s = Some(12.0);
        note_mode_change(FlightMode::GpsHome, &mut es);
        capture_targets(FlightMode::GpsHome, Some(est()), true, &mut es, &mut nav);
        assert!(!nav.rescue_landing);
        assert_eq!(nav.rescue_loiter_s, None);
    }

    #[test]
    fn mode_change_is_reported_once_and_resets_the_capture_flag() {
        let mut es = EntryState::new();
        es.targets_captured = true;
        assert_eq!(note_mode_change(FlightMode::AltHold, &mut es), Some(FlightMode::Acro));
        assert!(!es.targets_captured);
        assert_eq!(note_mode_change(FlightMode::AltHold, &mut es), None, "no re-fire");
    }

    // ---- Heading hold ----

    /// The bug this replaced: every mode passed desired_yaw_rad = 0, which
    /// is "point north", not "hold heading". With centred sticks at 90 deg
    /// the MPC commanded about -42 deg/s back toward north.
    #[test]
    fn centred_yaw_stick_holds_the_current_heading_not_north() {
        let mut st = state();
        let mut inp = inputs(FlightMode::Acro);
        inp.yaw_rad = 90.0 * D2R_T; // pointing east
        let out = nav_step(&inp, &mut st);
        assert!(
            (out.desired_yaw_rad - inp.yaw_rad).abs() < 1e-5,
            "reference should be the current heading, got {} for heading {}",
            out.desired_yaw_rad, inp.yaw_rad
        );
        assert_eq!(out.yaw_rate_dps, 0.0, "no stick, no commanded rate");
    }

    /// While the stick is deflected the target follows, so releasing it
    /// holds wherever the pilot stopped rather than snapping back.
    #[test]
    fn yaw_target_follows_the_stick_then_latches_on_release() {
        let mut st = state();
        let mut inp = inputs(FlightMode::Acro);
        inp.yaw_input = 0.8;
        inp.yaw_rad = 0.0;
        nav_step(&inp, &mut st);
        // Pilot yaws round to 120 deg, still holding the stick.
        inp.yaw_rad = 120.0 * D2R_T;
        nav_step(&inp, &mut st);
        assert!((st.yaw_target_rad.unwrap() - 120.0 * D2R_T).abs() < 1e-5);
        // Stick released: the target stays where they stopped.
        inp.yaw_input = 0.0;
        inp.yaw_rad = 121.0 * D2R_T;
        let out = nav_step(&inp, &mut st);
        assert!((st.yaw_target_rad.unwrap() - 120.0 * D2R_T).abs() < 1e-5, "latched");
        assert!(out.desired_yaw_rad < inp.yaw_rad, "should turn back the 1 deg");
    }

    /// The wrap case. Holding 179 deg while drifting to -179 deg is a 2 deg
    /// error, not 358 deg. An absolute reference would send the MPC the
    /// long way round at full commanded rate.
    #[test]
    fn heading_hold_takes_the_short_way_round_at_the_wrap_point() {
        let mut st = state();
        let mut inp = inputs(FlightMode::Acro);
        st.yaw_target_rad = Some(179.0 * D2R_T);
        inp.yaw_rad = -179.0 * D2R_T; // just across the wrap
        let out = nav_step(&inp, &mut st);
        // Short arc from -179 to +179 is -2 deg (left, back through 180),
        // NOT +358. Getting the sign right matters as much as the size:
        // an absolute reference would command a full turn the other way.
        let err_deg = (out.desired_yaw_rad - inp.yaw_rad) / D2R_T;
        assert!(
            (err_deg + 2.0).abs() < 0.01,
            "expected -2 deg (short way through 180), got {err_deg} deg"
        );
    }

    /// Failsafe modes have no pilot input, so they must hold heading too --
    /// not spin to north while descending.
    #[test]
    fn failsafe_descent_holds_heading() {
        let mut st = state();
        let mut inp = inputs(FlightMode::FailsafeLand);
        inp.pos_est = Some(est());
        inp.yaw_rad = -60.0 * D2R_T;
        let out = nav_step(&inp, &mut st);
        assert!((out.desired_yaw_rad - inp.yaw_rad).abs() < 1e-5);
        assert_eq!(out.yaw_rate_dps, 0.0);
    }

    // ---- Staged GPS rescue ----

    fn upset_home(roll_deg: f32) -> NavInputs {
        let mut inp = inputs(FlightMode::GpsHome);
        inp.pos_est = Some(PosEstimate { position_ned: [-50.0, 0.0, 0.0], ..est() });
        inp.roll_rad = roll_deg * D2R_T;
        inp
    }

    /// THE point of the staging. Entering rescue banked hard, the first
    /// thing the aircraft must do is level -- NOT command more tilt toward
    /// home, which is what it used to do.
    #[test]
    fn rescue_levels_before_it_navigates() {
        let mut st = state();
        let out = nav_step(&upset_home(120.0), &mut st);
        assert_eq!(st.rescue_stage, RescueStage::Level);
        assert_eq!(out.desired_roll_rad, 0.0, "must command LEVEL, not tilt home");
        assert_eq!(out.desired_pitch_rad, 0.0);
    }

    /// Once level, the position loop engages and it flies home. 50 m north
    /// of the target means a nose-down command.
    #[test]
    fn rescue_navigates_once_attitude_is_recovered() {
        let mut st = state();
        nav_step(&upset_home(120.0), &mut st);
        assert_eq!(st.rescue_stage, RescueStage::Level);

        // Now level.
        let mut inp = upset_home(1.0);
        let out = nav_step(&inp, &mut st);
        assert_eq!(st.rescue_stage, RescueStage::Navigate);
        assert_eq!(out.event, Some(NavEvent::RescueLevelled));

        inp.roll_rad = 0.0;
        let out = nav_step(&inp, &mut st);
        assert!(out.desired_pitch_rad < 0.0, "50 m south of home: nose down to fly north");
    }

    /// The give-up path. Wind holding it at 18 deg must not hang the
    /// rescue: it proceeds best-effort once the deadline passes.
    #[test]
    fn rescue_gives_up_levelling_on_time_and_proceeds() {
        let mut st = state();
        let inp = upset_home(100.0); // past the gate, never reaches 5 deg
        let mut timed_out = false;
        // Deadline for 18 deg is well under a second; run a generous window.
        for _ in 0..500 {
            if nav_step(&inp, &mut st).event == Some(NavEvent::RescueLevelTimeout) {
                timed_out = true;
                break;
            }
        }
        assert!(timed_out, "must give up rather than hang at 100 deg");
        assert_eq!(st.rescue_stage, RescueStage::Navigate);
        let out = nav_step(&inp, &mut st);
        assert!(out.desired_pitch_rad != 0.0, "and then actually navigate");
    }

    /// The deadline scales with how far over it started -- inverted gets
    /// far longer than a gentle bank.
    #[test]
    fn levelling_deadline_scales_with_initial_tilt() {
        let mut gentle = state();
        nav_step(&upset_home(100.0), &mut gentle);
        let mut inverted = state();
        nav_step(&upset_home(170.0), &mut inverted);
        let (g, i) = (
            gentle.rescue_level_deadline_s.unwrap(),
            inverted.rescue_level_deadline_s.unwrap(),
        );
        assert!(i > g * 1.4, "inverted deadline {i}s should exceed 100 deg's {g}s");
        assert!(g > RESCUE_LEVEL_FLOOR_S, "and never below the floor");
    }

    /// Re-entering rescue restarts the stage, or a second rescue would skip
    /// levelling entirely because the first one left it in Navigate.
    #[test]
    fn re_entering_rescue_restarts_the_levelling_stage() {
        let (mut es, mut nav) = (EntryState::new(), state());
        nav.rescue_stage = RescueStage::Navigate;
        nav.rescue_level_s = 9.0;
        note_mode_change(FlightMode::GpsHome, &mut es);
        capture_targets(FlightMode::GpsHome, Some(est()), true, &mut es, &mut nav);
        assert_eq!(nav.rescue_stage, RescueStage::Level);
        assert_eq!(nav.rescue_level_s, 0.0);
        assert_eq!(nav.rescue_level_deadline_s, None);
    }

    /// Altitude control keeps running while levelling -- authority is taken
    /// from the POSITION loop, not from holding height.
    #[test]
    fn altitude_is_still_controlled_while_levelling() {
        let mut st = state();
        st.alt_target = 40.0;
        let mut inp = upset_home(120.0);
        inp.pos_est = Some(PosEstimate {
            position_ned: [-50.0, 0.0, 0.0],
            altitude_up: 10.0,
            ..est()
        });
        let out = nav_step(&inp, &mut st);
        assert_eq!(st.rescue_stage, RescueStage::Level);
        assert!(out.thrust > HOVER, "30 m below target: must climb, got {}", out.thrust);
    }

    /// The gate. Below 90 deg a rescue navigates immediately rather than
    /// surrendering the position loop -- measured to cost up to 2x the
    /// drift for ~0.3 s of recovery time that a moderate upset does not
    /// need. See RESCUE_LEVEL_MIN_TILT_DEG for the numbers.
    #[test]
    fn moderate_upsets_navigate_immediately_without_levelling() {
        for roll in [10.0f32, 45.0, 85.0] {
            let mut st = state();
            let out = nav_step(&upset_home(roll), &mut st);
            assert_eq!(
                st.rescue_stage, RescueStage::Navigate,
                "{roll} deg is under the gate: navigate, do not level"
            );
            assert!(
                out.desired_pitch_rad < 0.0,
                "and actually fly home (nose down, 50 m south): {roll} deg gave {}",
                out.desired_pitch_rad
            );
        }
    }

    /// Past the gate it still levels first.
    #[test]
    fn severe_upsets_still_level_first() {
        for roll in [95.0f32, 140.0, 175.0] {
            let mut st = state();
            let out = nav_step(&upset_home(roll), &mut st);
            assert_eq!(st.rescue_stage, RescueStage::Level, "{roll} deg must level");
            assert_eq!(out.desired_roll_rad, 0.0);
        }
    }
}
