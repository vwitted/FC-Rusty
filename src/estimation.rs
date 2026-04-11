// estimation.rs — Linear 6-state position/velocity Kalman filter
//
// State:   x = [px, py, pz, vx, vy, vz]   (metres, m/s, NED)
// Input:   u = [ax, ay, az]                (m/s², world NED kinematic accel)
// Measure: z_gps  = [px, py, pz]           (NED position, m)
//          z_baro = -pz                    (altitude positive up, m)
//
// The attitude solution lives *outside* this filter — the WT901B's
// onboard Kalman fuses accel+gyro+mag into a quaternion/Euler solution,
// so by the time the caller asks this KF to predict, it has already
// rotated the body-frame specific force into the world frame and added
// gravity. That reduction makes the position filter **linear**, which
// is both cheap enough to run every PID tick and well-conditioned for
// fixed-gain firmware use.
//
// Dynamics (constant-accel-between-samples):
//     p_{k+1} = p_k + v_k·dt + ½·a_k·dt²
//     v_{k+1} = v_k +            a_k·dt
//
//     F = [ I₃   dt·I₃ ]        G = [ ½dt²·I₃ ]
//         [ 0    I₃   ]             [  dt·I₃  ]
//
// Process noise is the CWNA (continuous white-noise acceleration) model
// with spectral density σ_a²:
//     Q = σ_a² · [ ¼dt⁴·I₃   ½dt³·I₃ ]
//                [ ½dt³·I₃     dt²·I₃ ]
//
// Joseph-form updates are not used (we're f32 fixed-dimension, not
// quadruple-product-deep), but covariance symmetry is enforced after
// each update for numerical hygiene.

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

pub const KF_NX: usize = 6;

/// Linear position/velocity Kalman filter.
pub struct PosKf {
    /// State estimate [px, py, pz, vx, vy, vz] (NED).
    pub x: SVector<f32, KF_NX>,
    /// State covariance.
    pub p_cov: SMatrix<f32, KF_NX, KF_NX>,

    /// Process-noise spectral density on acceleration (m²/s³).
    sigma_a: f32,
    /// 1σ GPS horizontal position noise (m).
    sigma_gps_h: f32,
    /// 1σ GPS vertical position noise (m). Usually larger than horizontal.
    sigma_gps_v: f32,
    /// 1σ baro altitude noise (m).
    sigma_baro: f32,
}

impl PosKf {
    /// Create a filter seeded at a known position with tight velocity
    /// uncertainty. Typical use is to call `new_at(...)` with the
    /// take-off position and let the sensors dial the estimate in.
    pub fn new_at(
        position_ned: [f32; 3],
        sigma_a: f32,
        sigma_gps_h: f32,
        sigma_gps_v: f32,
        sigma_baro: f32,
    ) -> Self {
        let mut x = SVector::<f32, KF_NX>::zeros();
        x[0] = position_ned[0];
        x[1] = position_ned[1];
        x[2] = position_ned[2];

        // Moderate initial position uncertainty, small velocity uncertainty.
        // The GPS will dominate the position variance within a few fixes.
        let mut p_cov = SMatrix::<f32, KF_NX, KF_NX>::zeros();
        p_cov[(0, 0)] = 4.0; // 2 m σ
        p_cov[(1, 1)] = 4.0;
        p_cov[(2, 2)] = 4.0;
        p_cov[(3, 3)] = 0.25; // 0.5 m/s σ
        p_cov[(4, 4)] = 0.25;
        p_cov[(5, 5)] = 0.25;

        Self {
            x,
            p_cov,
            sigma_a,
            sigma_gps_h,
            sigma_gps_v,
            sigma_baro,
        }
    }

    /// Current best estimate.
    pub fn state(&self) -> [f32; KF_NX] {
        [self.x[0], self.x[1], self.x[2], self.x[3], self.x[4], self.x[5]]
    }

    /// Altitude in the "positive-up" convention used by the altitude
    /// controller.
    pub fn altitude_up(&self) -> f32 {
        -self.x[2]
    }

    /// Vertical velocity in the "positive-up" convention.
    pub fn vz_up(&self) -> f32 {
        -self.x[5]
    }

    /// Horizontal ground speed (m/s).
    pub fn ground_speed(&self) -> f32 {
        libm::sqrtf(self.x[3] * self.x[3] + self.x[4] * self.x[4])
    }

    /// Predict step. `accel_world_ned` is the current kinematic
    /// acceleration in the world frame — the caller is responsible
    /// for rotating body-frame specific force by the attitude and
    /// adding gravity before calling this.
    pub fn predict(&mut self, accel_world_ned: [f32; 3], dt: f32) {
        // --- State mean: x ← F·x + G·u ---
        let ax = accel_world_ned[0];
        let ay = accel_world_ned[1];
        let az = accel_world_ned[2];

        let px = self.x[0] + self.x[3] * dt + 0.5 * ax * dt * dt;
        let py = self.x[1] + self.x[4] * dt + 0.5 * ay * dt * dt;
        let pz = self.x[2] + self.x[5] * dt + 0.5 * az * dt * dt;
        let vx = self.x[3] + ax * dt;
        let vy = self.x[4] + ay * dt;
        let vz = self.x[5] + az * dt;

        self.x[0] = px;
        self.x[1] = py;
        self.x[2] = pz;
        self.x[3] = vx;
        self.x[4] = vy;
        self.x[5] = vz;

        // --- Covariance: P ← F·P·F^T + Q ---
        // Exploit the block structure to avoid building the full 6×6
        // F matrix. Partition P into 3×3 blocks:
        //   P = [ Ppp  Ppv ]
        //       [ Pvp  Pvv ]
        // With F = [ I dt·I; 0 I ]:
        //   Ppp' = Ppp + dt·(Pvp + Ppv) + dt²·Pvv
        //   Ppv' = Ppv + dt·Pvv
        //   Pvp' = Pvp + dt·Pvv
        //   Pvv' = Pvv
        let ppp = self.p_cov.fixed_view::<3, 3>(0, 0).into_owned();
        let ppv = self.p_cov.fixed_view::<3, 3>(0, 3).into_owned();
        let pvp = self.p_cov.fixed_view::<3, 3>(3, 0).into_owned();
        let pvv = self.p_cov.fixed_view::<3, 3>(3, 3).into_owned();

        let ppp_new: Matrix3<f32> = ppp + dt * (pvp + ppv) + dt * dt * pvv;
        let ppv_new: Matrix3<f32> = ppv + dt * pvv;
        let pvp_new: Matrix3<f32> = pvp + dt * pvv;
        let pvv_new: Matrix3<f32> = pvv;

        self.p_cov.fixed_view_mut::<3, 3>(0, 0).copy_from(&ppp_new);
        self.p_cov.fixed_view_mut::<3, 3>(0, 3).copy_from(&ppv_new);
        self.p_cov.fixed_view_mut::<3, 3>(3, 0).copy_from(&pvp_new);
        self.p_cov.fixed_view_mut::<3, 3>(3, 3).copy_from(&pvv_new);

        // --- CWNA process noise ---
        //   Qpp = σ_a²·¼·dt⁴·I   Qpv = σ_a²·½·dt³·I
        //   Qvp = σ_a²·½·dt³·I   Qvv = σ_a²·    dt²·I
        let s = self.sigma_a * self.sigma_a;
        let q_pp = 0.25 * dt * dt * dt * dt * s;
        let q_pv = 0.5 * dt * dt * dt * s;
        let q_vv = dt * dt * s;

        for i in 0..3 {
            self.p_cov[(i, i)] += q_pp;
            self.p_cov[(i + 3, i + 3)] += q_vv;
            self.p_cov[(i, i + 3)] += q_pv;
            self.p_cov[(i + 3, i)] += q_pv;
        }

        self.symmetrise();
    }

    /// GPS position measurement update.
    ///
    /// `z_gps_ned` is the noisy [px, py, pz] fix in the NED world frame.
    pub fn update_gps(&mut self, z_gps_ned: [f32; 3]) {
        // H = [ I₃ | 0₃ ]  so H·x picks off [px, py, pz].
        let z = Vector3::from_column_slice(&z_gps_ned);
        let y = z - Vector3::new(self.x[0], self.x[1], self.x[2]);

        // S = H·P·H^T + R = Ppp + R
        let ppp = self.p_cov.fixed_view::<3, 3>(0, 0).into_owned();
        let mut s = ppp;
        s[(0, 0)] += self.sigma_gps_h * self.sigma_gps_h;
        s[(1, 1)] += self.sigma_gps_h * self.sigma_gps_h;
        s[(2, 2)] += self.sigma_gps_v * self.sigma_gps_v;

        let s_inv = match s.try_inverse() {
            Some(m) => m,
            None => return, // degenerate; skip update rather than blow up
        };

        // K = P·H^T · S^-1  — P·H^T is the first 3 columns of P (both
        // Ppp and Pvp blocks).
        let p_ht = self.p_cov.fixed_view::<KF_NX, 3>(0, 0).into_owned();
        let k: SMatrix<f32, KF_NX, 3> = p_ht * s_inv;

        // x ← x + K·y
        self.x += k * y;

        // P ← (I - K·H)·P  = P − K·H·P = P − K·[Ppp_row_block]
        // where H·P picks the first three rows of P.
        let h_p = self.p_cov.fixed_view::<3, KF_NX>(0, 0).into_owned();
        self.p_cov -= k * h_p;

        self.symmetrise();
    }

    /// Barometer altitude measurement update.
    ///
    /// `altitude_up` is metres above ground, positive up. Internally
    /// converted to the NED z coordinate (pz = −altitude_up) for the
    /// update equations.
    pub fn update_baro(&mut self, altitude_up: f32) {
        // H picks pz (index 2). We model the measurement as z = −pz.
        // Equivalently: redefine residual y = (-altitude_up) − pz.
        let y_scalar = (-altitude_up) - self.x[2];

        // S = P[2,2] + σ²
        let s = self.p_cov[(2, 2)] + self.sigma_baro * self.sigma_baro;
        if s.abs() < 1e-12 {
            return;
        }
        let s_inv = 1.0 / s;

        // K = P·H^T · S^-1  is column 2 of P scaled by s_inv.
        let mut k = SVector::<f32, KF_NX>::zeros();
        for i in 0..KF_NX {
            k[i] = self.p_cov[(i, 2)] * s_inv;
        }

        // x ← x + K·y
        for i in 0..KF_NX {
            self.x[i] += k[i] * y_scalar;
        }

        // P ← P − K·H·P = P − K·(row 2 of P)
        let row2: [f32; KF_NX] = [
            self.p_cov[(2, 0)],
            self.p_cov[(2, 1)],
            self.p_cov[(2, 2)],
            self.p_cov[(2, 3)],
            self.p_cov[(2, 4)],
            self.p_cov[(2, 5)],
        ];
        for i in 0..KF_NX {
            for j in 0..KF_NX {
                self.p_cov[(i, j)] -= k[i] * row2[j];
            }
        }

        self.symmetrise();
    }

    /// Force P to be symmetric after an update (combats f32 drift).
    fn symmetrise(&mut self) {
        for i in 0..KF_NX {
            for j in (i + 1)..KF_NX {
                let avg = 0.5 * (self.p_cov[(i, j)] + self.p_cov[(j, i)]);
                self.p_cov[(i, j)] = avg;
                self.p_cov[(j, i)] = avg;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Zero accel + a stream of perfect GPS fixes at the true position
    /// should drive the estimate toward the truth and shrink covariance.
    #[test]
    fn gps_updates_converge_to_truth() {
        let mut kf = PosKf::new_at([0.0, 0.0, -5.0], 0.5, 2.0, 5.0, 0.3);

        // Inject 20 noise-free fixes at a different truth.
        let truth = [10.0, -7.0, -3.0];
        for _ in 0..20 {
            kf.predict([0.0, 0.0, 0.0], 0.1);
            kf.update_gps(truth);
        }

        assert!((kf.x[0] - truth[0]).abs() < 0.5, "x={}", kf.x[0]);
        assert!((kf.x[1] - truth[1]).abs() < 0.5, "y={}", kf.x[1]);
        assert!((kf.x[2] - truth[2]).abs() < 0.5, "z={}", kf.x[2]);
    }

    /// Baro updates should fix altitude without touching horizontal position.
    #[test]
    fn baro_only_updates_altitude() {
        let mut kf = PosKf::new_at([0.0, 0.0, 0.0], 0.5, 2.0, 5.0, 0.3);
        // Simulate steady hover 5 m above ground.
        for _ in 0..50 {
            kf.predict([0.0, 0.0, 0.0], 0.02);
            kf.update_baro(5.0);
        }
        assert!((kf.altitude_up() - 5.0).abs() < 0.3);
        assert!(kf.x[0].abs() < 0.01);
        assert!(kf.x[1].abs() < 0.01);
    }

    /// Predict alone — constant horizontal accel for 1 s — should
    /// move the state like simple kinematics.
    #[test]
    fn predict_integrates_kinematics() {
        let mut kf = PosKf::new_at([0.0, 0.0, 0.0], 0.01, 2.0, 5.0, 0.3);
        let dt = 0.01;
        for _ in 0..100 {
            kf.predict([1.0, 0.0, 0.0], dt); // 1 m/s² for 1 s
        }
        // Expected: vx = 1.0 m/s, px = 0.5 m
        assert!((kf.x[3] - 1.0).abs() < 1e-3, "vx={}", kf.x[3]);
        assert!((kf.x[0] - 0.5).abs() < 1e-3, "px={}", kf.x[0]);
    }

    /// Covariance must stay positive semi-definite after updates (diag ≥ 0).
    #[test]
    fn covariance_stays_nonnegative() {
        let mut kf = PosKf::new_at([0.0, 0.0, -5.0], 0.5, 2.0, 5.0, 0.3);
        for i in 0..500 {
            kf.predict([0.1, -0.1, 0.0], 0.005);
            if i % 20 == 0 {
                kf.update_gps([0.0, 0.0, -5.0]);
            }
            if i % 4 == 0 {
                kf.update_baro(5.0);
            }
            for d in 0..KF_NX {
                assert!(kf.p_cov[(d, d)] >= -1e-6, "P[{d},{d}]={}", kf.p_cov[(d, d)]);
            }
        }
    }
}
