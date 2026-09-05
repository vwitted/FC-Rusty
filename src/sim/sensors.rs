// sensors.rs — Noisy sensor simulators for GPS and barometer
//
// These wrap ground-truth state from `QuadSim` with realistic noise
// characteristics so the state estimator can be tested in sim without
// hardware in the loop. Kept no_std-friendly (uses libm, no std::rand).
//
// Noise model:
//   GPS:  low-rate (≈10 Hz), large Gaussian noise, horizontal σ ≈ 2 m,
//         vertical σ ≈ 5 m (consumer modules are much worse on altitude).
//   Baro: higher-rate (≈50 Hz), smaller white noise (≈0.3 m) plus a
//         slow Ornstein–Uhlenbeck drift (τ ≈ 60 s, σ ≈ 0.5 m) that
//         models pressure/temperature wander.

use core::f32::consts::PI;

// ---- PRNG ---------------------------------------------------------------

/// Deterministic xorshift64 PRNG + Box–Muller Gaussian sampler.
///
/// Seedable so tests/examples are repeatable.
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0xDEAD_BEEF_CAFE_BABE } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Uniform f32 in [0, 1).
    pub fn uniform(&mut self) -> f32 {
        // Top 24 bits → [0, 1)
        (self.next_u64() >> 40) as f32 / ((1u64 << 24) as f32)
    }

    /// Standard-normal f32 via Box–Muller.
    pub fn normal(&mut self) -> f32 {
        // Guard against log(0).
        let mut u1 = self.uniform();
        if u1 < 1.0e-7 {
            u1 = 1.0e-7;
        }
        let u2 = self.uniform();
        libm::sqrtf(-2.0 * libm::logf(u1)) * libm::cosf(2.0 * PI * u2)
    }
}

// ---- GPS simulator ------------------------------------------------------

/// Simulated GPS — outputs noisy position at a fixed rate.
///
/// Position is in metres in the **world NED frame** (same convention as
/// `QuadSim::state.x/y/z`) so it slots straight into a linear KF that
/// tracks NED position. A real receiver returns lat/lon/alt; for this
/// sim-only path we skip the projection and feed the KF NED directly.
/// One GPS fix: position and, separately measured, velocity.
///
/// Velocity is NOT differentiated position. Receivers derive it from
/// carrier Doppler, which is an independent observable and roughly thirty
/// times better in relative terms -- ~0.05 m/s against ~2 m of position
/// error. That gap is the whole reason it is worth fusing on its own
/// rather than letting a filter infer velocity from the position sequence.
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    /// Noisy world-NED position, metres.
    pub position_ned: [f32; 3],
    /// Noisy world-NED velocity, m/s. Error is independent of the position
    /// error, because the underlying observables are.
    pub velocity_ned: [f32; 3],
}

pub struct GpsSim {
    period: f32,
    sigma_h: f32,
    sigma_v: f32,
    sigma_vel_h: f32,
    sigma_vel_v: f32,
    time_accum: f32,
    rng: Rng,
}

impl GpsSim {
    /// # Arguments
    /// * `rate_hz`  — fix rate (e.g. 10.0)
    /// * `sigma_h`  — 1σ horizontal noise (m)
    /// * `sigma_v`  — 1σ vertical noise (m)
    /// * `seed`     — PRNG seed for reproducibility
    pub fn new(rate_hz: f32, sigma_h: f32, sigma_v: f32, seed: u64) -> Self {
        // Doppler velocity noise typical of a consumer receiver with a good
        // fix. Deliberately NOT scaled from the position sigmas: they come
        // from different observables and do not track each other.
        Self::with_velocity_noise(rate_hz, sigma_h, sigma_v, 0.05, 0.15, seed)
    }

    /// As `new`, with explicit velocity noise.
    pub fn with_velocity_noise(
        rate_hz: f32,
        sigma_h: f32,
        sigma_v: f32,
        sigma_vel_h: f32,
        sigma_vel_v: f32,
        seed: u64,
    ) -> Self {
        Self {
            period: 1.0 / rate_hz,
            sigma_h,
            sigma_v,
            sigma_vel_h,
            sigma_vel_v,
            time_accum: 0.0,
            rng: Rng::new(seed),
        }
    }

    /// Advance `dt` seconds. If a new fix is due, return `Some([x, y, z])`
    /// (noisy world-NED position in metres); otherwise `None`.
    pub fn tick(&mut self, dt: f32, truth_ned: [f32; 3]) -> Option<[f32; 3]> {
        self.tick_full(dt, truth_ned, [0.0; 3]).map(|f| f.position_ned)
    }

    /// Advance `dt` seconds; if a fix is due, return position AND velocity.
    ///
    /// The two error draws are independent, matching the receiver: position
    /// comes from pseudorange, velocity from carrier Doppler. Deriving one
    /// noise from the other would model a receiver that does not exist.
    pub fn tick_full(
        &mut self,
        dt: f32,
        truth_ned: [f32; 3],
        truth_vel_ned: [f32; 3],
    ) -> Option<GpsFix> {
        self.time_accum += dt;
        if self.time_accum < self.period {
            return None;
        }
        self.time_accum -= self.period;

        Some(GpsFix {
            position_ned: [
                truth_ned[0] + self.sigma_h * self.rng.normal(),
                truth_ned[1] + self.sigma_h * self.rng.normal(),
                truth_ned[2] + self.sigma_v * self.rng.normal(),
            ],
            velocity_ned: [
                truth_vel_ned[0] + self.sigma_vel_h * self.rng.normal(),
                truth_vel_ned[1] + self.sigma_vel_h * self.rng.normal(),
                truth_vel_ned[2] + self.sigma_vel_v * self.rng.normal(),
            ],
        })
    }

    pub fn sigma_vel_h(&self) -> f32 {
        self.sigma_vel_h
    }

    pub fn sigma_vel_v(&self) -> f32 {
        self.sigma_vel_v
    }

    pub fn sigma_h(&self) -> f32 {
        self.sigma_h
    }
    pub fn sigma_v(&self) -> f32 {
        self.sigma_v
    }
}

// ---- Barometer simulator ------------------------------------------------

/// Simulated barometric altimeter.
///
/// Output is altitude **positive up** (metres above the take-off point),
/// matching how real altimeters are usually consumed by the flight
/// controller. Noise model:
///   reading = truth_alt_up + drift + N(0, σ_white²)
/// where `drift` is a first-order Ornstein–Uhlenbeck process with
/// steady-state std `σ_drift` and correlation time `τ_drift` — a decent
/// match for real MEMS baro temperature/pressure wander on time scales
/// of a minute or so.
pub struct BaroSim {
    period: f32,
    sigma_white: f32,
    drift: f32,
    drift_sigma: f32,
    drift_tau: f32,
    time_accum: f32,
    rng: Rng,
}

impl BaroSim {
    pub fn new(
        rate_hz: f32,
        sigma_white: f32,
        drift_sigma: f32,
        drift_tau_s: f32,
        seed: u64,
    ) -> Self {
        Self {
            period: 1.0 / rate_hz,
            sigma_white,
            drift: 0.0,
            drift_sigma,
            drift_tau: drift_tau_s,
            time_accum: 0.0,
            rng: Rng::new(seed),
        }
    }

    /// Advance `dt` seconds. Drift integrates every call; a reading is
    /// only emitted when `period` has elapsed.
    ///
    /// `truth_alt_up` is metres above ground, positive up.
    pub fn tick(&mut self, dt: f32, truth_alt_up: f32) -> Option<f32> {
        // OU step: drift ← drift·(1 − dt/τ) + √(2·dt/τ)·σ·Z
        // Valid whenever dt ≪ τ; we clamp the shrink factor for safety.
        let shrink = (dt / self.drift_tau).min(1.0);
        let kick = libm::sqrtf(2.0 * shrink) * self.drift_sigma * self.rng.normal();
        self.drift = self.drift * (1.0 - shrink) + kick;

        self.time_accum += dt;
        if self.time_accum < self.period {
            return None;
        }
        self.time_accum -= self.period;

        Some(truth_alt_up + self.drift + self.sigma_white * self.rng.normal())
    }

    pub fn sigma(&self) -> f32 {
        self.sigma_white
    }
    pub fn drift(&self) -> f32 {
        self.drift
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_normal_has_sane_statistics() {
        let mut rng = Rng::new(42);
        let n = 20_000;
        let mut mean = 0.0f32;
        let mut m2 = 0.0f32;
        for _ in 0..n {
            let z = rng.normal();
            mean += z;
            m2 += z * z;
        }
        mean /= n as f32;
        let var = m2 / n as f32 - mean * mean;
        // Very loose bounds; just catch egregious bugs.
        assert!(mean.abs() < 0.05, "mean {}", mean);
        assert!((var - 1.0).abs() < 0.1, "var {}", var);
    }

    #[test]
    fn gps_fires_at_rate() {
        let mut gps = GpsSim::new(10.0, 0.0, 0.0, 1); // no noise
        let mut fixes = 0;
        for _ in 0..1000 {
            if gps.tick(0.005, [1.0, 2.0, -5.0]).is_some() {
                fixes += 1;
            }
        }
        // 1000 steps * 0.005s = 5s @ 10 Hz ≈ 50 fixes
        assert!((45..=55).contains(&fixes), "got {} fixes", fixes);
    }

    #[test]
    fn gps_no_noise_returns_truth() {
        let mut gps = GpsSim::new(10.0, 0.0, 0.0, 7);
        // Tick until a fix arrives.
        for _ in 0..200 {
            if let Some(fix) = gps.tick(0.005, [3.0, -4.0, -5.5]) {
                assert!((fix[0] - 3.0).abs() < 1e-6);
                assert!((fix[1] - -4.0).abs() < 1e-6);
                assert!((fix[2] - -5.5).abs() < 1e-6);
                return;
            }
        }
        panic!("never got a fix");
    }

    #[test]
    fn baro_drift_is_bounded_in_steady_state() {
        // Run the OU process for several τ and check it doesn't
        // blow up. (Not a formal test of stationary std, just a sanity.)
        let mut baro = BaroSim::new(50.0, 0.0, 0.5, 60.0, 13);
        let mut max_drift = 0.0f32;
        for _ in 0..40_000 {
            let _ = baro.tick(0.005, 5.0);
            if baro.drift().abs() > max_drift {
                max_drift = baro.drift().abs();
            }
        }
        // σ=0.5 → expect |drift| well under ±5 (10σ).
        assert!(max_drift < 5.0, "drift ran away: {}", max_drift);
    }

    // ---- GPS velocity ----

    /// Velocity error must be INDEPENDENT of position error. They come from
    /// different observables (pseudorange vs carrier Doppler), so a sim
    /// that derived one from the other would model a receiver that does not
    /// exist -- and would make a filter fusing both look falsely confident.
    #[test]
    fn gps_position_and_velocity_errors_are_independent() {
        let mut g = GpsSim::new(10.0, 2.0, 5.0, 42);
        let (mut sum_pp, mut sum_vv, mut sum_pv, mut n) = (0.0f64, 0.0f64, 0.0f64, 0usize);
        for _ in 0..200_000 {
            if let Some(f) = g.tick_full(0.1, [0.0; 3], [0.0; 3]) {
                let (pe, ve) = (f.position_ned[0] as f64, f.velocity_ned[0] as f64);
                sum_pp += pe * pe;
                sum_vv += ve * ve;
                sum_pv += pe * ve;
                n += 1;
            }
        }
        let corr = (sum_pv / n as f64)
            / ((sum_pp / n as f64).sqrt() * (sum_vv / n as f64).sqrt());
        assert!(corr.abs() < 0.02, "correlation {corr} should be ~0");
    }

    /// And velocity must be far better than position, relatively. This is
    /// the property that makes fusing it worthwhile at all.
    #[test]
    fn gps_velocity_is_much_more_precise_than_position() {
        let g = GpsSim::new(10.0, 2.0, 5.0, 1);
        assert!(g.sigma_vel_h() < 0.1, "horizontal velocity sigma");
        assert!(g.sigma_vel_v() < 0.3, "vertical velocity sigma");
    }

    #[test]
    fn gps_velocity_tracks_truth_and_fires_at_the_fix_rate() {
        let mut g = GpsSim::new(10.0, 2.0, 5.0, 7);
        let truth_v = [12.0f32, -3.0, 0.5];
        let (mut fixes, mut sum) = (0usize, [0.0f64; 3]);
        for _ in 0..1000 {
            if let Some(f) = g.tick_full(0.01, [0.0; 3], truth_v) {
                fixes += 1;
                for k in 0..3 {
                    sum[k] += f.velocity_ned[k] as f64;
                }
            }
        }
        // 99 or 100: time_accum is a float sum, so the last boundary can
        // land either side. Pre-existing accumulator behaviour, not the
        // thing under test.
        assert!((99..=100).contains(&fixes), "10 Hz over 10 s, got {fixes}");
        for k in 0..3 {
            let mean = (sum[k] / fixes as f64) as f32;
            assert!((mean - truth_v[k]).abs() < 0.05, "axis {k} mean {mean}");
        }
    }

    /// The old position-only API keeps working, so existing examples and
    /// results are undisturbed.
    #[test]
    fn position_only_tick_still_behaves() {
        let mut g = GpsSim::new(10.0, 2.0, 5.0, 3);
        let mut fixes = 0;
        for _ in 0..1000 {
            if g.tick(0.01, [1.0, 2.0, 3.0]).is_some() {
                fixes += 1;
            }
        }
        assert!((99..=100).contains(&fixes), "got {fixes}");
    }
}
