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
// α = exp(-dt_mpc / τ_pid) where τ_pid is the PID closed-loop time constant.
// With τ_pid ≈ 0.04s (well-tuned PID at 200 Hz): α ≈ 0.6
const RATE_ALPHA: f32 = 0.6;

// ---- Constraint bounds ----
const MAX_ANGLE_RAD: f32 = 45.0 * PI / 180.0;  // ±45°
const MAX_RATE_RAD: f32 = 800.0 * PI / 180.0;   // ±800°/s
const MAX_CMD_RAD: f32 = 800.0 * PI / 180.0;    // ±800°/s

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
        // Q: penalise attitude errors heavily to get fast correction.
        // Rate states penalised lightly — they're transient.
        let q = SMatrix::<f32, NX, NX>::from_diagonal(&SVector::from_row_slice(&[
            100.0, 100.0, 50.0, 1.0, 1.0, 1.0,
        ]));

        // R: light penalty on rate commands — allow aggressive corrections.
        // The PID output limits (±0.4) and constraint bounds provide the
        // real limit on how aggressive the system can be.
        let r = SMatrix::<f32, NU, NU>::from_diagonal(&SVector::from_row_slice(&[
            0.1, 0.1, 0.2,
        ]));

        let s = SMatrix::<f32, NX, NU>::zeros();

        // ---- Policy (precomputed LQR gain + Riccati) ----
        let rho = 10.0;
        let riccati_iters = 100;
        let policy = FixedPolicy::new(rho, riccati_iters, &a, &b, &q, &r, &s)
            .expect("MPC policy computation failed — check A, B, Q, R");

        let mut solver = MpcSolver::new(a, b, policy);
        solver.config.max_iter = 20;
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
