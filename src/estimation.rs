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

use nalgebra::{Matrix3, SMatrix, SVector, Vector2, Vector3};

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

    /// GPS horizontal-velocity measurement update.
    ///
    /// `vn`, `ve` are NED ground-velocity components (m/s), typically
    /// derived from NMEA RMC/VTG ground speed × course over ground.
    /// Vertical GPS velocity is intentionally *not* fused — it is noisy
    /// on consumer modules and the baro channel already constrains
    /// vz via cross-covariance with pz.
    ///
    /// This path is independent of position fusion and of GPS home
    /// latching. It can run as soon as the receiver reports a fix —
    /// useful for damping pre-home dead-reckoning drift from O(t²) to
    /// O(t), and for keeping a rough velocity solution alive during
    /// position dropouts.
    pub fn update_gps_velocity(&mut self, vn: f32, ve: f32) {
        // 1σ GPS horizontal-velocity noise. Consumer modules quote
        // 0.05–0.1 m/s on ground speed; 0.3 leaves headroom for the
        // course-induced cross-axis error at low speeds.
        const SIGMA_GPS_VEL: f32 = 0.3;

        // H = picks vn (state 3) and ve (state 4) out of the 6-state
        // vector. y = z − H·x.
        let y = Vector2::new(vn - self.x[3], ve - self.x[4]);

        // S = H·P·H^T + R = P[3:5, 3:5] + σ²·I₂
        let p_vv = self.p_cov.fixed_view::<2, 2>(3, 3).into_owned();
        let mut s = p_vv;
        s[(0, 0)] += SIGMA_GPS_VEL * SIGMA_GPS_VEL;
        s[(1, 1)] += SIGMA_GPS_VEL * SIGMA_GPS_VEL;

        let s_inv = match s.try_inverse() {
            Some(m) => m,
            None => return,
        };

        // K = P·H^T · S^-1.  P·H^T is columns 3..5 of P (6×2).
        let p_ht = self.p_cov.fixed_view::<KF_NX, 2>(0, 3).into_owned();
        let k: SMatrix<f32, KF_NX, 2> = p_ht * s_inv;

        // x ← x + K·y
        self.x += k * y;

        // P ← P − K·H·P  (H·P is rows 3..5 of P, 2×6)
        let h_p = self.p_cov.fixed_view::<2, KF_NX>(3, 0).into_owned();
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

// ---- Geodetic → local NED ----

/// Convert a WGS84 geodetic fix to a local NED offset from a reference
/// ("home") point using the equirectangular flat-earth approximation.
/// Accuracy is sub-metre out to ~1 km from the reference, which covers
/// every sane GPS-rescue scenario — for longer ranges this biases
/// east-west distance via the meridian-convergence cosine.
///
/// Inputs in degrees (lat/lon) and metres above MSL (alt). The returned
/// triple is `[north, east, down]` in metres — ready to hand to
/// `PosKf::update_gps`. Down is `-(alt - alt_ref)` so that the baro's
/// positive-up boot reference and the GPS's MSL reference stay
/// consistent when `alt_ref` is latched at the same instant as the home
/// point.
pub fn geodetic_to_local_ned(
    lat: f64,
    lon: f64,
    alt_msl: f32,
    lat_ref: f64,
    lon_ref: f64,
    alt_ref_msl: f32,
) -> [f32; 3] {
    // WGS84 equatorial radius. Using a single-radius spherical model
    // (vs the full ellipsoid) leaves ~0.2 % error in N at mid-latitudes
    // — well below GPS noise at rescue ranges.
    const R_EARTH: f64 = 6_378_137.0;
    const DEG2RAD: f64 = core::f64::consts::PI / 180.0;

    let lat_ref_rad = lat_ref * DEG2RAD;
    let dlat = (lat - lat_ref) * DEG2RAD;
    let dlon = (lon - lon_ref) * DEG2RAD;

    let north = (dlat * R_EARTH) as f32;
    let east = (dlon * R_EARTH * libm::cos(lat_ref_rad)) as f32;
    let down = -(alt_msl - alt_ref_msl);

    [north, east, down]
}

// ---- GPS home-latch stability check ----
//
// Early NMEA fixes drift for tens of seconds even after the receiver
// reports sats >= 7 and HDOP < 2.0 — modules will frame-narrow as more
// satellites lock, and the reported position can wander by tens of
// metres in the process. Latching home against an early fix anchors
// every subsequent `geodetic_to_local_ned` against a wrong reference,
// and the PosKF then tracks GPS *deltas* correctly but reads as
// "wildly wrong" in absolute terms.
//
// The stability buffer holds the last `CAP` quality-gated samples (the
// caller is responsible for only pushing fixes that already passed
// sats / HDOP gates). Two-stage latch criterion:
//
//   - **Fast path**: at least `FAST_MIN_AGE_S` seconds and `FAST_MIN_N`
//     samples span the buffer, and every sample is within
//     `FAST_RADIUS_M` of the centroid. Catches the "already locked at
//     power-on" case so we don't pay 60 s for a clean receiver.
//   - **Slow path**: at least `SLOW_MIN_AGE_S` seconds and `SLOW_MIN_N`
//     samples, every sample within `SLOW_RADIUS_M` of the centroid.
//     Wandering acquisitions fail the fast test but eventually pass
//     this one once the wander rolls out of the sliding window.
//
// The centroid is returned alongside so the caller can latch home at
// the mean of the stable samples rather than the noisier latest one.

/// Single quality-gated GPS fix awaiting home-latch consideration.
#[derive(Clone, Copy, Debug)]
pub struct StabilitySample {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    /// Monotonic timestamp in milliseconds. Caller chooses the epoch;
    /// only differences matter.
    pub t_ms: u64,
}

/// Tuning for the two-stage latch.
#[derive(Clone, Copy, Debug)]
pub struct StabilityConfig {
    pub fast_min_age_s: u32,
    pub fast_min_samples: usize,
    pub fast_radius_m: f32,
    pub slow_min_age_s: u32,
    pub slow_min_samples: usize,
    pub slow_radius_m: f32,
    /// Samples older than this are evicted from the buffer.
    pub window_s: u32,
}

impl Default for StabilityConfig {
    fn default() -> Self {
        Self {
            fast_min_age_s: 30,
            fast_min_samples: 15,
            fast_radius_m: 10.0,
            slow_min_age_s: 60,
            slow_min_samples: 30,
            slow_radius_m: 100.0,
            window_s: 60,
        }
    }
}

/// Centroid + decision returned by [`evaluate_stability`].
#[derive(Clone, Copy, Debug)]
pub struct StabilityDecision {
    pub latch: bool,
    /// Centroid of the buffer (only meaningful when `latch` is true).
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    /// Max distance from the centroid across all samples, metres.
    /// Useful for diagnostics regardless of `latch`.
    pub spread_m: f32,
    /// Time span of the buffer (oldest to newest), seconds.
    pub age_s: f32,
    pub samples: usize,
}

/// Capacity of the home-latch stability buffer. 64 entries × 1 Hz NMEA
/// covers a 60 s window with headroom; pushes past `CAP` are no-ops
/// (drop the new sample) — by design, the slow path is at-most-60-s,
/// so we'd be discarding samples we don't need.
pub const STABILITY_CAP: usize = 64;

/// Fixed-capacity FIFO of quality-gated GPS samples.
///
/// Insertion-ordered; `push` evicts the oldest entry when full, and
/// `trim_older_than` evicts entries by age. Designed for `no_std` use
/// with no allocator dependency.
pub struct StabilityBuffer {
    samples: [StabilitySample; STABILITY_CAP],
    head: usize,
    len: usize,
}

impl StabilityBuffer {
    pub const fn new() -> Self {
        Self {
            samples: [StabilitySample {
                lat_deg: 0.0,
                lon_deg: 0.0,
                alt_msl_m: 0.0,
                t_ms: 0,
            }; STABILITY_CAP],
            head: 0,
            len: 0,
        }
    }

    #[allow(dead_code)] // public API; used by host tests, kept for callers.
    pub fn len(&self) -> usize {
        self.len
    }

    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[allow(dead_code)]
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    fn idx(&self, i: usize) -> usize {
        (self.head + i) % STABILITY_CAP
    }

    /// Append a sample. When the buffer is full this evicts the oldest
    /// entry — fine because the slow-path window caps out at 60 s and
    /// the buffer is sized to hold that at 1 Hz with headroom.
    pub fn push(&mut self, s: StabilitySample) {
        if self.len < STABILITY_CAP {
            let pos = (self.head + self.len) % STABILITY_CAP;
            self.samples[pos] = s;
            self.len += 1;
        } else {
            // Buffer full: overwrite the oldest slot and advance head.
            self.samples[self.head] = s;
            self.head = (self.head + 1) % STABILITY_CAP;
        }
    }

    /// Drop entries whose timestamp is older than `now_ms - max_age_ms`.
    pub fn trim_older_than(&mut self, now_ms: u64, max_age_ms: u64) {
        let cutoff = now_ms.saturating_sub(max_age_ms);
        while self.len > 0 {
            let oldest = self.samples[self.head];
            if oldest.t_ms < cutoff {
                self.head = (self.head + 1) % STABILITY_CAP;
                self.len -= 1;
            } else {
                break;
            }
        }
    }

    #[allow(dead_code)]
    pub fn oldest(&self) -> Option<StabilitySample> {
        if self.len == 0 { None } else { Some(self.samples[self.head]) }
    }

    #[allow(dead_code)]
    pub fn newest(&self) -> Option<StabilitySample> {
        if self.len == 0 {
            None
        } else {
            let pos = (self.head + self.len - 1) % STABILITY_CAP;
            Some(self.samples[pos])
        }
    }
}

/// Evaluate the home-latch criterion. Returns the decision plus the
/// centroid (only meaningful when `latch == true`) and diagnostic
/// `spread_m` / `age_s` / `samples`.
///
/// The centroid is a flat-mean of lat/lon/alt; for the small angular
/// extents we care about (≤ 100 m radius at any plausible latitude)
/// the difference between a flat mean and a spherical centroid is well
/// below GPS noise.
pub fn evaluate_stability(
    buf: &StabilityBuffer,
    cfg: &StabilityConfig,
    now_ms: u64,
) -> StabilityDecision {
    let n = buf.len;
    if n == 0 {
        return StabilityDecision {
            latch: false,
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_msl_m: 0.0,
            spread_m: 0.0,
            age_s: 0.0,
            samples: 0,
        };
    }

    // Centroid (flat mean).
    let mut sum_lat: f64 = 0.0;
    let mut sum_lon: f64 = 0.0;
    let mut sum_alt: f64 = 0.0;
    for i in 0..n {
        let s = buf.samples[buf.idx(i)];
        sum_lat += s.lat_deg;
        sum_lon += s.lon_deg;
        sum_alt += s.alt_msl_m as f64;
    }
    let mean_lat = sum_lat / n as f64;
    let mean_lon = sum_lon / n as f64;
    let mean_alt = (sum_alt / n as f64) as f32;

    // Max horizontal distance from the centroid, metres.
    const R_EARTH: f64 = 6_378_137.0;
    const DEG2RAD: f64 = core::f64::consts::PI / 180.0;
    let cos_lat = libm::cos(mean_lat * DEG2RAD);
    let mut max_d2_m: f64 = 0.0;
    for i in 0..n {
        let s = buf.samples[buf.idx(i)];
        let dn = (s.lat_deg - mean_lat) * DEG2RAD * R_EARTH;
        let de = (s.lon_deg - mean_lon) * DEG2RAD * R_EARTH * cos_lat;
        let d2 = dn * dn + de * de;
        if d2 > max_d2_m {
            max_d2_m = d2;
        }
    }
    let spread_m = libm::sqrtf(max_d2_m as f32);

    // Buffer span (newest − oldest).
    let oldest_t = buf.samples[buf.idx(0)].t_ms;
    // `now_ms` rather than newest sample so a stream that's gone quiet
    // doesn't keep "ageing" toward latch on stale data.
    let age_ms = now_ms.saturating_sub(oldest_t);
    let age_s = (age_ms as f64 / 1000.0) as f32;

    // Two-stage check (fast first, then slow as a fallback).
    let fast_ok = age_s >= cfg.fast_min_age_s as f32
        && n >= cfg.fast_min_samples
        && spread_m <= cfg.fast_radius_m;
    let slow_ok = age_s >= cfg.slow_min_age_s as f32
        && n >= cfg.slow_min_samples
        && spread_m <= cfg.slow_radius_m;

    StabilityDecision {
        latch: fast_ok || slow_ok,
        lat_deg: mean_lat,
        lon_deg: mean_lon,
        alt_msl_m: mean_alt,
        spread_m,
        age_s,
        samples: n,
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

    /// Velocity updates should converge the velocity state to the truth and
    /// shrink its variance, even without any position fixes.
    #[test]
    fn velocity_updates_converge_without_gps_position() {
        let mut kf = PosKf::new_at([0.0, 0.0, 0.0], 0.5, 2.0, 5.0, 0.3);

        // Inject 30 noise-free velocity measurements at constant cruise.
        // No update_gps() — pure velocity fusion, no position anchor.
        let truth = (3.0_f32, -2.0_f32);
        for _ in 0..30 {
            kf.predict([0.0, 0.0, 0.0], 0.1);
            kf.update_gps_velocity(truth.0, truth.1);
        }

        assert!((kf.x[3] - truth.0).abs() < 0.2, "vn={}", kf.x[3]);
        assert!((kf.x[4] - truth.1).abs() < 0.2, "ve={}", kf.x[4]);

        // Velocity covariance should have shrunk well below initial.
        assert!(kf.p_cov[(3, 3)] < 0.1, "P_vn={}", kf.p_cov[(3, 3)]);
        assert!(kf.p_cov[(4, 4)] < 0.1, "P_ve={}", kf.p_cov[(4, 4)]);
    }

    /// Velocity fusion should not touch the vertical channels (vz, pz).
    #[test]
    fn velocity_updates_dont_touch_vertical() {
        let mut kf = PosKf::new_at([0.0, 0.0, -3.0], 0.1, 2.0, 5.0, 0.3);
        // Seed a non-zero vz via prediction.
        kf.predict([0.0, 0.0, 1.0], 0.5); // vz = 0.5 m/s
        let vz_before = kf.x[5];
        let pz_before = kf.x[2];

        kf.update_gps_velocity(2.0, 0.0);

        // Horizontal velocity should respond, vertical should not.
        assert!(kf.x[3].abs() > 0.0);
        assert!((kf.x[5] - vz_before).abs() < 1e-3, "vz drifted: {} → {}", vz_before, kf.x[5]);
        assert!((kf.x[2] - pz_before).abs() < 0.05, "pz changed by velocity update");
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

    /// Home reference returns exactly zero NED.
    #[test]
    fn geodetic_at_home_is_zero() {
        let ned = geodetic_to_local_ned(
            51.5074, -0.1278, 50.0,   // London
            51.5074, -0.1278, 50.0,
        );
        assert!(ned[0].abs() < 1e-3);
        assert!(ned[1].abs() < 1e-3);
        assert!(ned[2].abs() < 1e-3);
    }

    /// A 1° latitude step is ~111 km north.
    #[test]
    fn geodetic_lat_step_is_111km_north() {
        let ned = geodetic_to_local_ned(
            52.5074, -0.1278, 50.0,
            51.5074, -0.1278, 50.0,
        );
        // R_earth * 1° = 111_319 m, roughly.
        assert!(ned[0] > 110_000.0 && ned[0] < 112_000.0, "n={}", ned[0]);
        assert!(ned[1].abs() < 1.0, "e={}", ned[1]);
    }

    /// At latitude 60° a 1° longitude step is ~55 km (cos(60°) scaling).
    #[test]
    fn geodetic_lon_step_scales_by_cos_lat() {
        // Move 1° east of a reference at 60°N.
        let ned = geodetic_to_local_ned(
            60.0, 11.0, 0.0,
            60.0, 10.0, 0.0,
        );
        // 111_319 · cos(60°) ≈ 55_659 m.
        assert!(ned[1] > 55_000.0 && ned[1] < 56_500.0, "e={}", ned[1]);
        assert!(ned[0].abs() < 1.0, "n={}", ned[0]);
    }

    /// Altitude delta maps to −(alt − alt_ref) in the z channel.
    #[test]
    fn geodetic_alt_goes_to_ned_down() {
        let ned = geodetic_to_local_ned(
            51.5074, -0.1278, 60.0,   // 10 m above ref
            51.5074, -0.1278, 50.0,
        );
        assert!((ned[2] - (-10.0)).abs() < 0.01, "d={}", ned[2]);
    }

    /// Small-offset round-trip sanity at a typical GPS rescue range.
    #[test]
    fn geodetic_100m_range_is_accurate() {
        // ~100 m north, ~100 m east at equator.
        let dlat_deg = 100.0 / 111_319.0;
        let dlon_deg = 100.0 / 111_319.0; // cos(0) = 1 at equator
        let ned = geodetic_to_local_ned(
            dlat_deg, dlon_deg, 0.0,
            0.0, 0.0, 0.0,
        );
        assert!((ned[0] - 100.0).abs() < 0.1, "n={}", ned[0]);
        assert!((ned[1] - 100.0).abs() < 0.1, "e={}", ned[1]);
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

    // ---- Home-latch stability tests ----

    fn s_at(lat: f64, lon: f64, t_ms: u64) -> StabilitySample {
        StabilitySample {
            lat_deg: lat,
            lon_deg: lon,
            alt_msl_m: 50.0,
            t_ms,
        }
    }

    /// Convert a metres offset to a (small) lat/lon delta near the
    /// reference. Useful for building scenarios in physical units.
    fn offset_m(lat_ref: f64, dn_m: f32, de_m: f32) -> (f64, f64) {
        const R_EARTH: f64 = 6_378_137.0;
        const DEG2RAD: f64 = core::f64::consts::PI / 180.0;
        let dlat = (dn_m as f64) / R_EARTH / DEG2RAD;
        let dlon =
            (de_m as f64) / R_EARTH / DEG2RAD / libm::cos(lat_ref * DEG2RAD);
        (lat_ref + dlat, dlon)
    }

    #[test]
    fn stability_fast_path_latches_on_already_locked_module() {
        // Receiver already locked at boot: 1 Hz samples for 30 s, all
        // within 5 m of truth → fast-path fires at ~30 s.
        let mut buf = StabilityBuffer::new();
        let cfg = StabilityConfig::default();
        let truth_lat = 51.5;
        let mut latched_at: Option<u64> = None;

        for sec in 0..40 {
            // Tiny ±3 m oscillation, well inside fast-path radius (10 m).
            let dn = if sec % 2 == 0 { 3.0 } else { -3.0 };
            let de = if sec % 3 == 0 { 2.0 } else { -2.0 };
            let (lat, lon_off) = offset_m(truth_lat, dn, de);
            buf.push(s_at(lat, lon_off, sec * 1000));
            buf.trim_older_than(sec * 1000, (cfg.window_s as u64) * 1000);
            let d = evaluate_stability(&buf, &cfg, sec * 1000);
            if d.latch && latched_at.is_none() {
                latched_at = Some(sec);
                // Centroid should be very close to truth (within 1 m).
                let (_, dlon) = offset_m(truth_lat, 0.0, 0.0);
                assert!((d.lat_deg - truth_lat).abs() < 1e-5,
                        "lat centroid drifted: {} vs {}", d.lat_deg, truth_lat);
                assert!((d.lon_deg - dlon).abs() < 1e-5,
                        "lon centroid drifted");
            }
        }
        let t = latched_at.expect("fast-path should have latched");
        assert!(t <= 31, "fast-path latched too late: {} s", t);
    }

    #[test]
    fn stability_slow_path_latches_after_wander_subsides() {
        // First 20 s of wander (radius ~150 m), then settles inside
        // 50 m for the next 60 s. Fast-path never fires; slow-path
        // latches once the wander rolls out of the 60 s window.
        let mut buf = StabilityBuffer::new();
        let cfg = StabilityConfig::default();
        let truth_lat = 40.0;
        let mut latched_at: Option<u64> = None;

        for sec in 0..120 {
            let (dn, de) = if sec < 20 {
                // Big initial wander
                let phase = (sec as f32) * 0.7;
                (150.0 * libm::cosf(phase), 150.0 * libm::sinf(phase))
            } else {
                // Settled within 50 m centred on truth
                let phase = (sec as f32) * 0.3;
                (20.0 * libm::cosf(phase), 20.0 * libm::sinf(phase))
            };
            let (lat, lon_off) = offset_m(truth_lat, dn, de);
            buf.push(s_at(lat, lon_off, sec * 1000));
            buf.trim_older_than(sec * 1000, (cfg.window_s as u64) * 1000);
            let d = evaluate_stability(&buf, &cfg, sec * 1000);
            if d.latch && latched_at.is_none() {
                latched_at = Some(sec);
            }
        }
        let t = latched_at.expect("slow-path should have latched");
        // Slow-path needs 60 s of post-wander samples; wander ended at
        // 20 s, so earliest plausible latch is ~80 s. Be generous.
        assert!(t >= 75 && t <= 95, "slow-path latched at unexpected time: {} s", t);
    }

    #[test]
    fn stability_never_latches_under_continuous_wander() {
        // Persistent ±150 m noise — should never latch.
        let mut buf = StabilityBuffer::new();
        let cfg = StabilityConfig::default();
        let truth_lat = -33.0;

        for sec in 0..180 {
            let phase = (sec as f32) * 0.9;
            let dn = 150.0 * libm::cosf(phase);
            let de = 150.0 * libm::sinf(phase);
            let (lat, lon_off) = offset_m(truth_lat, dn, de);
            buf.push(s_at(lat, lon_off, sec * 1000));
            buf.trim_older_than(sec * 1000, (cfg.window_s as u64) * 1000);
            let d = evaluate_stability(&buf, &cfg, sec * 1000);
            assert!(!d.latch, "unexpectedly latched under wander at {} s", sec);
        }
    }

    #[test]
    fn stability_buffer_trim_drops_old_entries() {
        let mut buf = StabilityBuffer::new();
        for sec in 0..10 {
            buf.push(s_at(51.5, -0.1, sec * 1000));
        }
        assert_eq!(buf.len(), 10);
        // Drop everything older than 5 s relative to t=9 s.
        buf.trim_older_than(9_000, 5_000);
        // Keeps entries with t_ms >= 4_000 → seconds 4..=9 → 6 entries.
        assert_eq!(buf.len(), 6);
        assert_eq!(buf.oldest().unwrap().t_ms, 4_000);
        assert_eq!(buf.newest().unwrap().t_ms, 9_000);
    }

    #[test]
    fn stability_buffer_eviction_on_overflow() {
        let mut buf = StabilityBuffer::new();
        for i in 0..(STABILITY_CAP + 5) {
            buf.push(s_at(0.0, 0.0, i as u64));
        }
        assert_eq!(buf.len(), STABILITY_CAP);
        // Oldest should now be t_ms = 5 (the first 5 were evicted).
        assert_eq!(buf.oldest().unwrap().t_ms, 5);
        assert_eq!(buf.newest().unwrap().t_ms, (STABILITY_CAP + 4) as u64);
    }
}
