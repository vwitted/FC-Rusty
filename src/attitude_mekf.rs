// attitude_mekf.rs — Multiplicative Extended Kalman Filter for attitude
//
// Fuses the ICM-42688P gyro (high-rate, low-bias at short timescales
// but drifts) and accelerometer (noisy, but has an absolute gravity
// reference) into a quaternion attitude estimate plus a running gyro
// bias estimate.
//
// Conventions (match the rest of the codebase — see `drivers/icm42688.rs`):
//   - Body frame: X-forward, Y-right, Z-down (aerospace NED).
//   - Nav  frame: North-East-Down. Gravity is +g on the nav Z axis.
//   - Quaternion q rotates body → nav:  v_nav = R(q) · v_body.
//   - At rest, body accel reads ≈ (0, 0, −1) g  (specific force = −gravity).
//
// State (6-DOF error-state MEKF):
//   - Nominal quaternion q  (4-vector, always renormalised; error is tracked
//     in the 3-vector δθ representing a small body-frame rotation).
//   - Nominal gyro bias b (rad/s, body frame).
//   Error vector is [δθ; δb] ∈ R⁶, with covariance P ∈ R⁶ˣ⁶.
//
// Predict: runs every IMU sample (8 kHz).
//   ω  = ω_meas − b
//   q ← q ⊗ [1, ω·dt/2]  (first-order, renormalise each step)
//   F  = I + dt · [[ −[ω]×,  −I₃ ],
//                  [    0,     0  ]]
//   P ← F P Fᵀ + Q · dt      (Q on body-frame error, bias random walk)
//
// Update (accel gravity reference): decimated — runs every N-th sample.
//   h(q) = R(q)ᵀ · [0,0,1]     (unit gravity in body; matches −a_body_unit)
//   H    = [ [h]×, 0 ]          (body-frame error convention)
//   z    = −a_body / ‖a_body‖
//   Innovation-gated: skipped if ‖a_body‖ in g deviates from 1 by more
//   than `accel_gate` — rejects high-g and free-fall, and a small
//   dead-band suppresses update-chatter during hover.
//
// Output units (caller's responsibility to convert):
//   `euler()` returns radians; main.rs converts to degrees for ImuData.
//   Accel input is in g; main.rs converts g → m/s² at the ImuData boundary.

use core::f32::consts::FRAC_PI_2;
use nalgebra::{Matrix3, SMatrix, Vector3};

/// Earth gravity used at the MEKF input/output boundary (ISO 2533).
pub const G_MPS2: f32 = 9.80665;

/// Tuning knobs. Defaults are sane starting points for the ICM-42688P
/// at 8 kHz ODR; expect to re-tune `sigma_a` and `accel_gate` once
/// prop wash / vibration is known on the target airframe.
#[derive(Clone, Copy, Debug)]
pub struct MekfParams {
    /// Gyro white noise, rad/s/√Hz. ICM-42688P datasheet: ~0.0038 rad/s/√Hz
    /// (= 0.22 °/s/√Hz) at ±2000 dps, 8 kHz. Slightly inflated for margin.
    pub sigma_g: f32,
    /// Gyro bias random walk, rad/s/√Hz. Small — bias drifts slowly.
    pub sigma_bw: f32,
    /// Accel measurement noise (unit-vector scale).
    pub sigma_a: f32,
    /// Reject accel update if |‖a‖g − 1| > this. 0.3 rejects >1.3g / <0.7g.
    pub accel_gate: f32,
    /// Initial attitude 1σ (rad) on [roll, pitch, yaw].
    pub init_sigma_att: [f32; 3],
    /// Initial bias 1σ (rad/s) per axis.
    pub init_sigma_bias: f32,
}

impl Default for MekfParams {
    fn default() -> Self {
        Self {
            sigma_g:    0.005,   // rad/s/√Hz  — slightly above datasheet
            sigma_bw:   1.0e-5,  // rad/s/√Hz  — slow bias walk
            sigma_a:    0.08,    // unit vector — accounts for vibration
            accel_gate: 0.3,
            // Seeded from accel → roll/pitch ~observable (~2°), yaw not (~90°).
            init_sigma_att: [0.035, 0.035, FRAC_PI_2],
            init_sigma_bias: 0.02, // ~1.1 °/s
        }
    }
}

/// MEKF state. Hold one instance per airframe; not `Copy` — the
/// covariance is large enough that copies are worth avoiding.
pub struct AttitudeMekf {
    /// Nominal body→nav quaternion [w, x, y, z]. Always unit norm.
    q: [f32; 4],
    /// Nominal gyro bias (rad/s, body frame).
    bias: Vector3<f32>,
    /// Error-state covariance: diag blocks are δθ (3) then δb (3).
    p: SMatrix<f32, 6, 6>,
    params: MekfParams,
}

impl AttitudeMekf {
    pub fn new(params: MekfParams) -> Self {
        let mut p = SMatrix::<f32, 6, 6>::zeros();
        for i in 0..3 {
            p[(i, i)] = params.init_sigma_att[i] * params.init_sigma_att[i];
            p[(3 + i, 3 + i)] = params.init_sigma_bias * params.init_sigma_bias;
        }
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            bias: Vector3::zeros(),
            p,
            params,
        }
    }

    /// Seed the quaternion from a stationary accel reading — level board
    /// assumption, yaw set to zero (accel can't observe yaw, so the filter
    /// will converge on whatever the initial drift-free heading happens
    /// to be; a magnetometer would fix this).
    pub fn initialize_from_accel(&mut self, accel_body_g: [f32; 3]) {
        let ax = accel_body_g[0];
        let ay = accel_body_g[1];
        let az = accel_body_g[2];
        // Standard roll/pitch from accel; atan2 handles quadrants and sign
        // of az (az < 0 at rest in NED body).
        let roll  = libm::atan2f(-ay, -az);
        let pitch = libm::atan2f(ax, libm::sqrtf(ay * ay + az * az));
        let yaw   = 0.0;
        self.q = euler_to_quat(roll, pitch, yaw);
        // Bias starts at zero; the filter learns it via predict/update.
        self.bias = Vector3::zeros();
    }

    /// Predict one step. `gyro_body` is measured gyro in rad/s (body NED).
    /// Pass `dt` in seconds — the task should clamp to a sane range in
    /// case of missed samples; we don't clamp here.
    pub fn predict(&mut self, gyro_body: [f32; 3], dt: f32) {
        // Bias-corrected rate
        let w = Vector3::new(
            gyro_body[0] - self.bias[0],
            gyro_body[1] - self.bias[1],
            gyro_body[2] - self.bias[2],
        );

        // Quaternion update: q ← q ⊗ δq, δq = [1, ω·dt/2] (first-order,
        // valid for ω·dt ≪ 1; at 8 kHz, |ω·dt| ≤ 2000 dps · 125 µs ≈ 4 mrad).
        let half_dt = 0.5 * dt;
        let dq = [1.0, w[0] * half_dt, w[1] * half_dt, w[2] * half_dt];
        self.q = quat_mul(self.q, dq);
        quat_normalize(&mut self.q);

        // Covariance propagate, F · P · Fᵀ + Q·dt, with body-frame error.
        //   F = I + dt · [[ −[ω]×,  −I₃ ],
        //                 [    0,     0  ]]
        // We build F discretely and compute FPFᵀ; at 6x6 this is ~200
        // multiplies, trivial at 8 kHz on the H743 FPU.
        let wx = skew(&w);
        let mut f = SMatrix::<f32, 6, 6>::identity();
        // top-left 3x3 ← I − dt·[ω]×
        for i in 0..3 {
            for j in 0..3 {
                f[(i, j)] -= dt * wx[(i, j)];
            }
        }
        // top-right 3x3 ← −dt·I₃
        for i in 0..3 {
            f[(i, 3 + i)] = -dt;
        }

        let ft = f.transpose();
        self.p = f * self.p * ft;

        // Add Q·dt (diagonal): σ_g² · dt on δθ, σ_bw² · dt on δb.
        let q_theta = self.params.sigma_g * self.params.sigma_g * dt;
        let q_bias  = self.params.sigma_bw * self.params.sigma_bw * dt;
        for i in 0..3 {
            self.p[(i, i)]         += q_theta;
            self.p[(3 + i, 3 + i)] += q_bias;
        }
    }

    /// Gravity-reference accel update. `accel_body_g` is in units of g.
    /// Returns true if the update was applied (innovation gate passed).
    pub fn update_accel(&mut self, accel_body_g: [f32; 3]) -> bool {
        let a = Vector3::new(accel_body_g[0], accel_body_g[1], accel_body_g[2]);
        let norm = a.norm();
        if norm < 1e-6 {
            return false;
        }
        // Innovation gate: skip if off-unity by more than the gate.
        if libm::fabsf(norm - 1.0) > self.params.accel_gate {
            return false;
        }

        // Measurement: −a_body_unit (specific force → gravity direction).
        let z = -a / norm;

        // Predicted gravity unit vector in body: h = R(q)ᵀ · [0,0,1].
        // Using the third column of R(q) gives ẑ_body-in-nav, transposing
        // is the same as using the third row of R(q) — precompute it.
        let h = r_bn_row_z(&self.q);

        // H = [ [h]×, 0₃ ]  (body-frame error convention — see header).
        let hx = skew(&h);

        // Innovation y = z − h  (3-vector).
        let y = z - h;

        // S = H P Hᵀ + R  (3×3). H only touches top 3 rows of P:
        //   HP = [ hx · P_δθδθ,  hx · P_δθδb ]   (3×6)
        //   HPHᵀ = hx · P_δθδθ · hxᵀ            (3×3)
        let p_tt = self.p.fixed_view::<3, 3>(0, 0).into_owned();
        let p_tb = self.p.fixed_view::<3, 3>(0, 3).into_owned();
        let hp_tt = hx * p_tt;
        let mut s = hp_tt * hx.transpose();
        let r_meas = self.params.sigma_a * self.params.sigma_a;
        for i in 0..3 {
            s[(i, i)] += r_meas;
        }

        // Kalman gain K = P Hᵀ S⁻¹ (6×3). P Hᵀ:
        //   top    = P_δθδθ · hxᵀ    (3×3)
        //   bottom = P_δθδbᵀ · hxᵀ   (3×3)   [because H picks the top rows
        //                                      → PHᵀ picks the left cols via
        //                                      transpose; use P_bθ = P_tbᵀ]
        let p_bt = p_tb.transpose();
        let ph_top = p_tt * hx.transpose();
        let ph_bot = p_bt * hx.transpose();
        let s_inv = match s.try_inverse() {
            Some(m) => m,
            None => return false, // ill-conditioned — skip rather than diverge
        };
        let k_top = ph_top * s_inv;
        let k_bot = ph_bot * s_inv;

        // Error-state correction δx = K y
        let d_theta = k_top * y;
        let d_bias  = k_bot * y;

        // Apply attitude correction:  q ← q ⊗ [1, δθ/2]  (body-frame error).
        let dq = [1.0, d_theta[0] * 0.5, d_theta[1] * 0.5, d_theta[2] * 0.5];
        self.q = quat_mul(self.q, dq);
        quat_normalize(&mut self.q);

        // Apply bias correction.
        self.bias += d_bias;

        // Covariance update  P ← (I − K H) P. Again, H only touches the
        // top 3 rows, so  K H = [ [K_top · hx,  0],
        //                         [K_bot · hx,  0] ] ∈ R⁶ˣ⁶.
        let kh_tt = k_top * hx; // 3×3
        let kh_bt = k_bot * hx; // 3×3
        let mut kh = SMatrix::<f32, 6, 6>::zeros();
        for i in 0..3 {
            for j in 0..3 {
                kh[(i, j)]         = kh_tt[(i, j)];
                kh[(3 + i, j)]     = kh_bt[(i, j)];
            }
        }
        let i6 = SMatrix::<f32, 6, 6>::identity();
        self.p = (i6 - kh) * self.p;

        // Force symmetry — floating-point drift accumulates otherwise.
        let pt = self.p.transpose();
        self.p = (self.p + pt) * 0.5;

        true
    }

    /// Euler angles [roll, pitch, yaw] in radians (3-2-1 Tait-Bryan).
    pub fn euler(&self) -> [f32; 3] {
        quat_to_euler(&self.q)
    }

    /// Raw quaternion [w, x, y, z], body→nav.
    pub fn quaternion(&self) -> [f32; 4] {
        self.q
    }

    /// Current gyro bias estimate (rad/s, body frame).
    pub fn bias(&self) -> [f32; 3] {
        [self.bias[0], self.bias[1], self.bias[2]]
    }
}

// ---- Quaternion helpers ----
//
// Convention: q = [w, x, y, z], rotates body → nav. q_a ⊗ q_b applies
// q_b first then q_a (active rotation, Hamilton product).

fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn quat_normalize(q: &mut [f32; 4]) {
    let n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if n2 > 0.0 {
        let inv = 1.0 / libm::sqrtf(n2);
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        *q = [1.0, 0.0, 0.0, 0.0];
    }
}

/// Third row of R(q) (body→nav rotation matrix) — equivalently, the
/// nav-frame z-axis expressed in body coords, or R(q)ᵀ · [0,0,1].
fn r_bn_row_z(q: &[f32; 4]) -> Vector3<f32> {
    let w = q[0]; let x = q[1]; let y = q[2]; let z = q[3];
    Vector3::new(
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        w * w - x * x - y * y + z * z,
    )
}

fn skew(v: &Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
         0.0, -v[2],  v[1],
         v[2],  0.0, -v[0],
        -v[1], v[0],   0.0,
    )
}

fn euler_to_quat(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let (sr, cr) = (libm::sinf(0.5 * roll),  libm::cosf(0.5 * roll));
    let (sp, cp) = (libm::sinf(0.5 * pitch), libm::cosf(0.5 * pitch));
    let (sy, cy) = (libm::sinf(0.5 * yaw),   libm::cosf(0.5 * yaw));
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

fn quat_to_euler(q: &[f32; 4]) -> [f32; 3] {
    let w = q[0]; let x = q[1]; let y = q[2]; let z = q[3];
    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);
    // Pitch (y-axis rotation) — clamp to ±π/2 at the singularity
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if libm::fabsf(sinp) >= 1.0 {
        libm::copysignf(FRAC_PI_2, sinp)
    } else {
        libm::asinf(sinp)
    };
    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);
    [roll, pitch, yaw]
}

// ---- Host-side sanity tests ----
// Compiled out on the target (no_std embedded); run with `cargo test`
// on the host if you want to verify the math.
#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32, tol: f32) -> bool {
        libm::fabsf(a - b) < tol
    }

    #[test]
    fn level_at_rest_converges_to_zero_attitude() {
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.initialize_from_accel([0.0, 0.0, -1.0]);
        for _ in 0..100 {
            m.predict([0.0, 0.0, 0.0], 1.0 / 8000.0);
            m.update_accel([0.0, 0.0, -1.0]);
        }
        let [r, p, _y] = m.euler();
        assert!(approx(r, 0.0, 1e-3), "roll = {}", r);
        assert!(approx(p, 0.0, 1e-3), "pitch = {}", p);
    }

    #[test]
    fn tilted_accel_seeds_roll() {
        // 45° roll right → gravity vector in body is (0, −sin45, −cos45)
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.initialize_from_accel([0.0, -0.7071, -0.7071]);
        let [r, p, _y] = m.euler();
        assert!(approx(r,  core::f32::consts::FRAC_PI_4, 1e-3), "roll = {}", r);
        assert!(approx(p, 0.0, 1e-3), "pitch = {}", p);
    }

    #[test]
    fn quat_mul_identity() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let p = [0.5, 0.5, 0.5, 0.5];
        assert_eq!(quat_mul(q, p), p);
        assert_eq!(quat_mul(p, q), p);
    }
}
