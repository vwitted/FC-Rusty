// mpc.rs — MPC attitude controller wrapping tinympc-rs
//
// Provides a 6-state attitude model for cascaded MPC + PID control.
//
// Model:
//   State x = [roll, pitch, yaw, p, q, r]   (radians, rad/s)
//   Input u = [p_cmd, q_cmd, r_cmd]          (rad/s — rate commands for PID)
//
//   Angles integrate rates:  angle[k+1] = angle[k] + dt * rate[k]
//   Rates track with lag:    rate[k+1]  = α * rate[k] + (1-α) * u[k]
//
// The first-order lag (α ≈ 0.6) models the PID inner loop's finite
// bandwidth — rates don't jump instantly to the commanded value.
// This prevents the MPC from commanding unrealistically aggressive
// rate changes that the PID can't deliver.
//
// MPC runs at 100 Hz (see `MPC_PERIOD_US`). Its output
// [p_cmd, q_cmd, r_cmd] becomes the rate setpoint for the 8 kHz PID
// inner loop.
//
// Usage:
//   let mut mpc = AttitudeMpc::new();
//   mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
//   let out = mpc.solve([roll, pitch, yaw], [p, q, r]);
//   // out.rate_setpoints_rads → PID setpoints

use core::f32::consts::PI;
use nalgebra::{SMatrix, SVector};
use tinympc_rs::{
    Solver, TerminationReason,
    constraint::Constraint,
    policy::FixedPolicy,
    project::{time::Fixed},
};
use tinympc_rs::project;

// ---- Dimensions ----
pub const NX: usize = 6;   // [roll, pitch, yaw, p, q, r]
pub const NU: usize = 3;   // [p_cmd, q_cmd, r_cmd]
pub const HX: usize = 10;  // state prediction horizon
pub const HU: usize = 9;   // control horizon

/// Outer control-loop period in microseconds — the single source of
/// truth for the rate at which the navigation task runs the MPC and the
/// altitude/position controllers. `main.rs` builds its ticker from this,
/// and the MPC's A/B model below is discretised for exactly this step, so
/// the two can never drift apart. 100 Hz = 10_000 µs.
pub const MPC_PERIOD_US: u64 = 10_000;

/// MPC sample period (seconds) = `MPC_PERIOD_US` · 1e-6. 100 Hz.
pub const MPC_DT: f32 = 0.01;

// ---- Rate tracking model ----
// The PID inner loop doesn't track rate commands instantly. We model it
// as a first-order lag: rate[k+1] = α·rate[k] + (1-α)·u[k], with
//   α = exp(-MPC_DT / TAU_MOTOR) + RATE_ALPHA_MARGIN
// TAU_MOTOR is the dominant rate-loop pole (motor/ESC lag ≈ 30 ms). The
// margin makes the MPC see a slightly *slower* plant than reality, so it
// commands a little less aggressively (the 50 Hz model carried the same
// ~0.035 margin: 0.55 vs an exact 0.51). Deriving α from MPC_DT rather
// than hand-coding it means the lag model can never silently fall out of
// sync with the loop rate again.
const TAU_MOTOR: f32 = 0.030;
const RATE_ALPHA_MARGIN: f32 = 0.035;

/// First-order rate-lag coefficient α for the current model timestep.
/// Derived from `MPC_DT` so it tracks any change to the loop rate.
fn rate_alpha() -> f32 {
    libm::expf(-MPC_DT / TAU_MOTOR) + RATE_ALPHA_MARGIN
}

// ---- Constraint bounds ----
const MAX_ANGLE_RAD: f32 = 45.0 * PI / 180.0;  // ±45°
const MAX_RATE_RAD: f32 = 400.0 * PI / 180.0;   // ±400°/s
// A SECOND reason this bound matters, found after the fact and unrelated to
// the derivation below: the control loop must not outrun the ESTIMATOR.
//
// At large attitude error the MEKF's estimate is badly wrong (its accel
// update is corrupted by the manoeuvre's own acceleration), and the command
// bound decides how hard the controller acts on that wrong information.
// Measured, recovering from an upset with the estimator and GPS accel
// compensation in the loop, on a 6.8:1 airframe:
//
//     rate bound    90 deg upset    120 deg upset
//        40 deg/s      4.28 s          5.25 s
//        80 deg/s      4.33 s          4.84 s
//       150 deg/s      3.63 s          NEVER
//       300 deg/s      3.54 s          NEVER
//       400 deg/s      3.54 s          NEVER
//
// More authority improves the moderate case monotonically and BREAKS the
// large one above ~80-150 deg/s. Below the threshold the estimator keeps
// up; above it, extra authority becomes faster divergence. So an FPV
// airframe's real 800-2000 deg/s of capability is not something to unlock
// here without the estimator to match it.
//
// 40 deg/s therefore sits in the safe region, though the derivation below
// arrived there for an unrelated reason.
//
// BUT NOT BY BEING THE BINDING CONSTRAINT. Sweeping this bound with the MPC
// changes nothing at all -- 40, 60, 80, 100 and 150 deg/s all give
// identical upset recoveries (90 deg: 3.28 s; 120 deg: 3.54 s; 150 deg:
// never). What limits the MPC is Q/R, not the clamp: the weights are tuned
// to command <=15 deg/s for typical disturbances, so the bound only bites
// briefly at the start of a large recovery.
//
// The threshold above was measured with the angle PID, where kp = 6 against
// a 120 deg error demands 720 deg/s and the bound is the only thing holding
// it back. So the principle holds -- control bandwidth must not exceed
// estimator bandwidth -- but for the MPC the knob that would violate it is
// Q/R, not this. Relaxing MAX_CMD_RAD alone would be a no-op.
//
// MPC rate-command constraint. The inner PID (kp=0.02, output_max=0.5)
// saturates at a rate error of ~25 °/s. To keep PID in its linear
// regime we want |u_mpc - rate_actual| < 25 °/s. With Q/R tuned to
// command ≤15 °/s for typical disturbances and rates damped aggressively,
// 40 °/s is enough hard-limit headroom to absorb transient overshoot
// without the constraint itself becoming the operating point.
const MAX_CMD_RAD: f32 = 40.0 * PI / 180.0;     // ±40°/s

// ---- Type aliases ----
type MpcPolicy = FixedPolicy<f32, NX, NU>;
type MpcSolver = Solver<f32, MpcPolicy, NX, NU, HX, HU>;
type XConstraint = Constraint<f32, Fixed<project::Box<f32, NX>>, NX, HX>;
type UConstraint = Constraint<f32, Fixed<project::Box<f32, NU>>, NU, HU>;

/// Result of one MPC solve step.
pub struct MpcOutput {
    /// Rate setpoints [p, q, r] in rad/s for the PID inner loop
    pub rate_setpoints_rads: [f32; 3],
    /// Whether the solver converged within the iteration limit
    pub converged: bool,
    /// Number of ADMM iterations used
    pub iterations: usize,
}

/// MPC attitude controller.
///
/// Wraps a tinympc-rs solver with a 6-state attitude model.
/// Constraints and dual variables are persisted between solves
/// for warm-starting.
pub struct AttitudeMpc {
    solver: MpcSolver,
    x_ref: SMatrix<f32, NX, HX>,
    x_con: XConstraint,
    u_con: UConstraint,
    /// Output clamp, rad/s. Defaults to MAX_CMD_RAD; settable so the
    /// harness can sweep it, since the right value depends on the
    /// ESTIMATOR's bandwidth as much as the inner PID's linear range.
    cmd_bound: f32,
}

impl AttitudeMpc {
    /// Create a new MPC attitude controller with default weights.
    ///
    /// # Panics
    ///
    /// If the Riccati solve fails (should not happen with valid parameters).
    pub fn new() -> Self {
        // ---- System matrices ----
        // angle[k+1] = angle[k] + dt * rate[k]
        // rate[k+1]  = α * rate[k] + (1-α) * u[k]
        //
        // The first-order lag models the PID's finite bandwidth:
        // rates don't jump instantly to the commanded value.
        let alpha = rate_alpha();
        let beta = 1.0 - alpha;

        let a: SMatrix<f32, NX, NX> = SMatrix::from_row_slice(&[
            1.0, 0.0, 0.0, MPC_DT, 0.0,    0.0,
            0.0, 1.0, 0.0, 0.0,    MPC_DT,  0.0,
            0.0, 0.0, 1.0, 0.0,    0.0,     MPC_DT,
            0.0, 0.0, 0.0, alpha,  0.0,     0.0,
            0.0, 0.0, 0.0, 0.0,    alpha,   0.0,
            0.0, 0.0, 0.0, 0.0,    0.0,     alpha,
        ]);

        let b: SMatrix<f32, NX, NU> = SMatrix::from_row_slice(&[
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,
            beta, 0.0,  0.0,
            0.0,  beta, 0.0,
            0.0,  0.0,  beta,
        ]);

        // ---- Cost matrices ----
        // Tuned against the nonlinear failure mode: if the MPC commands
        // |u - rate| > 25 °/s, the inner PID saturates, the mixer
        // airmode-shifts to preserve torque at the cost of thrust, and
        // altitude control is lost. So the unconstrained LQR gain must
        // keep normal-state commands well inside that envelope.
        //
        // Reduction to a 1-axis double integrator + first-order lag:
        //   K_angle ≈ sqrt(Q_angle / R)   [rad/s per rad of angle]
        //   K_rate  ≈ sqrt(Q_rate  / R)   [rad/s per rad/s of rate]
        //
        // With Q_angle=5, Q_rate=2, R=1:
        //   K_angle ≈ 2.24  → 1° angle error    → ~2.2 °/s command
        //   K_rate  ≈ 1.41  → 10 °/s rate error → ~14 °/s command
        //
        // Both branches keep the PID error inside ±25 °/s for realistic
        // disturbances, and the rate-dominant weighting (Q_rate > Q_angle/2
        // once you account for scaling) means the controller kills body
        // rates first and only then drifts the angle back to zero — the
        // same philosophy as a well-damped cascaded PD outer loop.
        let q = SMatrix::<f32, NX, NX>::from_diagonal(&SVector::from_row_slice(&[
            5.0, 5.0, 2.0, 2.0, 2.0, 2.0,
        ]));

        let r = SMatrix::<f32, NU, NU>::from_diagonal(&SVector::from_row_slice(&[
            1.0, 1.0, 2.0,
        ]));

        let s = SMatrix::<f32, NX, NU>::zeros();

        // ---- Policy (precomputed LQR gain + Riccati) ----
        // rho is the ADMM penalty weight. Too high over-regularises the
        // problem and biases the solution; 1.0 is the tinympc-rs default
        // and behaves well for the magnitudes we use here.
        let rho = 1.0;
        let riccati_iters = 100;
        let policy = FixedPolicy::new(rho, riccati_iters, &a, &b, &q, &r, &s)
            .expect("MPC policy computation failed — check A, B, Q, R");

        let mut solver = MpcSolver::new(a, b, policy);
        // Cap ADMM iterations so the solve fits inside the 10 ms / 100 Hz
        // navigation budget (the H743's 480 MHz Cortex-M7 with a
        // double-precision FPU has ample headroom, but bounding the
        // worst case keeps the outer loop deterministic alongside the
        // altitude/position controllers). Attitude tracking is well below
        // the loop bandwidth, so partial convergence is fine — the next
        // solve 10 ms later continues from a warm start. The navigation
        // task records `mpc_time_us_max`; check it on hardware if you
        // change this cap.
        solver.config.max_iter = 10;
        solver.config.do_check = 1;

        // ---- Constraints ----
        let x_box = project::Box {
            lower: SVector::<f32, NX>::from_row_slice(&[
                -MAX_ANGLE_RAD, -MAX_ANGLE_RAD, -PI,
                -MAX_RATE_RAD, -MAX_RATE_RAD, -MAX_RATE_RAD,
            ]),
            upper: SVector::<f32, NX>::from_row_slice(&[
                MAX_ANGLE_RAD, MAX_ANGLE_RAD, PI,
                MAX_RATE_RAD, MAX_RATE_RAD, MAX_RATE_RAD,
            ]),
        };
        let x_con = Constraint::new(Fixed::new(x_box));

        let u_box = project::Box {
            lower: SVector::<f32, NU>::from_element(-MAX_CMD_RAD),
            upper: SVector::<f32, NU>::from_element(MAX_CMD_RAD),
        };
        let u_con = Constraint::new(Fixed::new(u_box));

        Self {
            solver,
            x_ref: SMatrix::zeros(),
            x_con,
            u_con,
            cmd_bound: MAX_CMD_RAD,
        }
    }

    /// Set the reference attitude for all horizon steps.
    ///
    /// # Arguments
    /// * `angles_rad` — target [roll, pitch, yaw] in radians
    /// * `rates_rad` — target [p, q, r] in rad/s (typically zero for hover)
    pub fn set_reference(&mut self, angles_rad: [f32; 3], rates_rad: [f32; 3]) {
        let ref_state = SVector::<f32, NX>::from_row_slice(&[
            angles_rad[0], angles_rad[1], angles_rad[2],
            rates_rad[0], rates_rad[1], rates_rad[2],
        ]);
        for i in 0..HX {
            self.x_ref.set_column(i, &ref_state);
        }
    }

    /// Run one MPC solve.
    ///
    /// # Arguments
    /// * `angles_rad` — current [roll, pitch, yaw] in radians
    /// * `rates_rad` — current [p, q, r] in rad/s
    ///
    /// # Returns
    /// Rate setpoints for the PID inner loop, plus convergence info.
    pub fn solve(
        &mut self,
        angles_rad: [f32; 3],
        rates_rad: [f32; 3],
    ) -> MpcOutput {
        let x_now = SVector::<f32, NX>::from_row_slice(&[
            angles_rad[0], angles_rad[1], angles_rad[2],
            rates_rad[0], rates_rad[1], rates_rad[2],
        ]);

        // Use direct solve() to satisfy the borrow checker:
        // solver, x_ref, x_con, u_con are all separate fields.
        let solution = self.solver.solve(
            x_now,
            Some(&self.x_ref),
            None,
            Some(core::slice::from_mut(&mut self.x_con)),
            Some(core::slice::from_mut(&mut self.u_con)),
        );

        let u = solution.u_now();

        // Clamp to the command bound.
        //
        // The solver's u_now() is the ADMM PRIMAL variable, not its
        // constraint-projected copy, so it satisfies MAX_CMD_RAD only to
        // tolerance -- and measurably does not. At a 20 deg attitude error
        // it returns 40.39 deg/s against a 40 deg/s bound WHILE REPORTING
        // CONVERGED; at 120 deg it settles at 43.89 and never converges.
        // More iterations do not help: 10, 20, 40, 80 and 200 all land on
        // the same value, so this is not a budget problem.
        //
        // That matters because this bound is not a preference. It is sized
        // so the inner rate PID stays linear (see MAX_CMD_RAD), and a
        // controller quietly exceeding the limit its downstream stage
        // depends on shows up only as unexplained saturation in flight.
        let clamp = |v: f32| v.clamp(-self.cmd_bound, self.cmd_bound);

        MpcOutput {
            rate_setpoints_rads: [clamp(u[0]), clamp(u[1]), clamp(u[2])],
            converged: solution.reason == TerminationReason::Converged,
            iterations: solution.iterations,
        }
    }

    /// Override the output command bound, rad/s.
    ///
    /// Two independent constraints set this and they are easy to confuse.
    /// The documented one keeps the inner rate PID linear. The other, found
    /// later, is that the control loop must not outrun the ESTIMATOR --
    /// past a threshold, extra authority becomes faster divergence from a
    /// wrong attitude rather than faster recovery. See the note by
    /// MAX_CMD_RAD.
    pub fn set_cmd_bound(&mut self, rad_per_s: f32) {
        self.cmd_bound = rad_per_s;
    }

    /// Reset the solver state (constraints, warm-start).
    ///
    /// Call when arming or switching flight modes.
    pub fn reset(&mut self) {
        self.x_ref = SMatrix::zeros();
        self.x_con.reset();
        self.u_con.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The MPC's A/B model is discretised for exactly the outer-loop
    // period. If MPC_DT drifts from the rate at which the navigation task
    // actually calls solve(), the internal model runs at the wrong speed
    // and the solver over-/under-shoots its rate commands. Lock the model
    // timestep to the single-source-of-truth period so they can't desync.
    #[test]
    fn mpc_dt_matches_outer_loop_period() {
        let expected = MPC_PERIOD_US as f32 * 1.0e-6;
        assert!(
            (MPC_DT - expected).abs() < 1.0e-9,
            "MPC_DT {} s must equal MPC_PERIOD_US ({} us = {} s)",
            MPC_DT,
            MPC_PERIOD_US,
            expected,
        );
    }

    // The first-order rate-lag coefficient is a function of the model
    // timestep: α = exp(-MPC_DT / TAU_MOTOR). If MPC_DT changes, α must be
    // retuned or the lag model no longer matches the plant. Allow a small
    // upward margin (model sees a slightly slower plant) but never let α
    // fall below the exact figure or drift far above it.
    #[test]
    fn rate_alpha_consistent_with_timestep() {
        let exact = libm::expf(-MPC_DT / TAU_MOTOR);
        let alpha = rate_alpha();
        assert!(
            alpha >= exact - 1.0e-3 && alpha <= exact + 0.06,
            "rate_alpha() {} inconsistent with exp(-MPC_DT/TAU_MOTOR) = {} \
             (expected within [{}, {}])",
            alpha,
            exact,
            exact - 1.0e-3,
            exact + 0.06,
        );
    }

    /// DOCUMENTS A DESIGN QUESTION, does not endorse it.
    ///
    /// Q weights yaw ANGLE at 2.0, and every flight mode in
    /// control::modes sets `desired_yaw_rad = 0`, which main.rs passes
    /// straight to set_reference. So the yaw reference is not "hold your
    /// current heading" -- it is "point NORTH", and the MPC commands rate
    /// to get there whenever the pilot's yaw stick is centred:
    ///
    ///     heading  30 deg -> -24.1 deg/s
    ///     heading  90 deg -> -41.9 deg/s
    ///     heading 180 deg -> -44.7 deg/s
    ///
    /// A quad is normally either heading-hold (reference latched to
    /// wherever you stopped) or rate-only. North-seeking is neither, and
    /// on a real aircraft it would read as the machine slowly weathervaning
    /// whenever you release the stick.
    ///
    /// Pinned rather than fixed because the fix is a decision about
    /// intended behaviour: latch the yaw reference on stick release, or
    /// zero Q[2] and drive yaw purely on rate. Both are defensible; picking
    /// one is not the test's job.
    #[test]
    fn heading_offset_alone_commands_yaw_toward_north() {
        let mut mpc = AttitudeMpc::new();
        mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let deg = core::f32::consts::PI / 180.0;

        let level = mpc.solve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        assert!(
            level.rate_setpoints_rads[2].abs() < 1e-3,
            "pointing north, no yaw command"
        );

        let east = mpc.solve([0.0, 0.0, 90.0 * deg], [0.0, 0.0, 0.0]);
        assert!(
            east.rate_setpoints_rads[2] < -10.0 * deg,
            "heading east with centred sticks commands yaw back toward north: {} deg/s",
            east.rate_setpoints_rads[2] / deg
        );
    }

    /// The command bound is enforced on the way out, because the solver
    /// does not enforce it itself.
    ///
    /// Measured before the clamp existed: 40.39 deg/s at a 20 deg attitude
    /// error WHILE REPORTING CONVERGED, and 43.89 at 120 deg regardless of
    /// iteration budget (10 through 200 all agree). u_now() is the ADMM
    /// primal, not its projected copy, so the constraint holds only to
    /// tolerance.
    #[test]
    fn output_never_exceeds_the_command_bound() {
        let mut mpc = AttitudeMpc::new();
        mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let deg = core::f32::consts::PI / 180.0;
        for err in [1.0f32, 20.0, 120.0, 180.0] {
            let out = mpc.solve([err * deg, err * deg, err * deg], [0.0; 3]);
            for (i, u) in out.rate_setpoints_rads.iter().enumerate() {
                assert!(
                    u.abs() <= MAX_CMD_RAD + 1e-6,
                    "axis {i} at {err} deg gave {} deg/s, bound is {}",
                    u / deg,
                    MAX_CMD_RAD / deg
                );
            }
        }
    }

    /// Normal errors converge well inside the iteration cap, so max_iter =
    /// 10 is not the binding constraint it looks like: 3 iterations at
    /// 1 deg, 5 at 5 deg, 6 at 20 deg.
    #[test]
    fn normal_attitude_errors_converge_well_inside_the_iteration_cap() {
        let mut mpc = AttitudeMpc::new();
        mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let deg = core::f32::consts::PI / 180.0;
        for err in [1.0f32, 5.0, 20.0] {
            let out = mpc.solve([err * deg, 0.0, 0.0], [0.0, 0.0, 0.0]);
            assert!(out.converged, "{err} deg should converge");
            assert!(out.iterations <= 8, "{err} deg used {} iters", out.iterations);
        }
    }
}
