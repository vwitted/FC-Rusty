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
// MPC runs at 50 Hz. Its output [p_cmd, q_cmd, r_cmd] becomes the
// rate setpoint for the 200 Hz PID inner loop.
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
    project::{self, time::Fixed},
};

// ---- Dimensions ----
pub const NX: usize = 6;   // [roll, pitch, yaw, p, q, r]
pub const NU: usize = 3;   // [p_cmd, q_cmd, r_cmd]
pub const HX: usize = 10;  // state prediction horizon
pub const HU: usize = 9;   // control horizon

/// MPC sample period (seconds). 50 Hz.
pub const MPC_DT: f32 = 0.02;

// ---- Rate tracking model ----
// The PID inner loop doesn't track rate commands instantly.
// We model it as a first-order lag: rate[k+1] = α·rate[k] + (1-α)·u[k]
// α = exp(-dt_mpc / τ_cl) where τ_cl is the rate-loop closed-loop time
// constant. Dominant pole is the motor/ESC lag τ_motor ≈ 30 ms, so
// α ≈ exp(-20/30) ≈ 0.51. Slightly higher (0.55) builds in a bit of
// margin against model error — the MPC sees a marginally *slower*
// plant than reality and so commands a little less aggressively.
const RATE_ALPHA: f32 = 0.55;

// ---- Constraint bounds ----
const MAX_ANGLE_RAD: f32 = 45.0 * PI / 180.0;  // ±45°
const MAX_RATE_RAD: f32 = 400.0 * PI / 180.0;   // ±400°/s
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
        let alpha = RATE_ALPHA;
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
        // max_iter 50 matches the tinympc-rs library default.
        // We have plenty of timing headroom (control loop avg ~61us
        // out of 5000us, and MPC only runs every 4th cycle at 50 Hz),
        // so there's no reason to undercut the library default here.
        solver.config.max_iter = 50;
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

        MpcOutput {
            rate_setpoints_rads: [u[0], u[1], u[2]],
            converged: solution.reason == TerminationReason::Converged,
            iterations: solution.iterations,
        }
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
