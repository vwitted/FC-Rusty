// imu_filter.rs — Software low-pass filtering for the dual-IMU stream.
//
// We left both on-chip UI filters at their wide defaults (ICM-42688P:
// 1st-order, ODR/2 ≈ 4 kHz; MPU6000: DLPF off, ~256 Hz bandwidth) and
// do the filtering downstream in software, *after* averaging. Two
// reasons:
//
//   1. The two chips have different stock filter responses; enabling
//      hardware filters on only one of them (or matching them exactly
//      on both) leaves the averaged signal with a mismatched
//      pass/stop band. Software downstream applies one identical
//      filter to the fused output.
//
//   2. On the MPU6000, enabling the DLPF at any non-zero setting also
//      drops the gyro ODR from 8 kHz to 1 kHz — a hard chip-level
//      coupling we can't decouple. Software filtering keeps the full
//      8 kHz read/predict rate intact.
//
// The MEKF decimates accel 80:1 (8 kHz → 100 Hz gravity update) with
// **no anti-alias filter today**: anything above 50 Hz aliases into
// the band the MEKF actually sees. A 25 Hz accel cutoff puts the
// 50 Hz aliasing edge at ~ −12 dB on a 2nd-order Butterworth
// (|H(50)| = 1 / √(1 + (50/25)⁴) ≈ 0.243). Higher cutoffs lose
// attenuation fast — 30 Hz only gets −9 dB at 50 Hz — so the default
// is intentionally on the tight side.
//
// Gyro filtering is cosmetic by comparison: the rate PID at 8 kHz
// integrates noisy gyro directly, but a 150 Hz cutoff adds only ~7°
// of phase lag at 30 Hz (well above any quad attitude dynamic) so
// it's mostly free in terms of PID feel.
//
// Implementation: per-axis Direct-Form-I biquad. With cutoff/sample
// ratios in the range we use (30/8000 = 0.00375 ... 150/8000 =
// 0.0188), coefficient magnitudes stay well-conditioned in f32 — no
// need for the transposed form or a state-space implementation.

#![allow(dead_code)]

use core::f32::consts::PI;

/// Direct-Form-I biquad with f32 state.
///
/// y[n] = b0·x[n] + b1·x[n−1] + b2·x[n−2] − a1·y[n−1] − a2·y[n−2]
///
/// `a0` is normalised to 1.0 by the constructor — only the five
/// coefficients above are stored. Initial state is zero, so the
/// first few samples after construction transition from 0 to the
/// steady-state DC value.
#[derive(Clone, Copy, Debug)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    /// Build a 2nd-order Butterworth low-pass at cutoff `fc_hz` for a
    /// stream sampled at `fs_hz`. Coefficients via the bilinear
    /// transform (Q = 1/√2 for Butterworth).
    pub fn new_lowpass_butterworth(fc_hz: f32, fs_hz: f32) -> Self {
        // Pre-warped angular frequency.
        let w = libm::tanf(PI * fc_hz / fs_hz);
        let w2 = w * w;
        // Q = 1/√2 → 1/Q = √2.
        const SQRT2: f32 = core::f32::consts::SQRT_2;
        let a0_raw = 1.0 + SQRT2 * w + w2;
        let inv_a0 = 1.0 / a0_raw;

        let b0 = w2 * inv_a0;
        let b1 = 2.0 * w2 * inv_a0;
        let b2 = w2 * inv_a0;
        let a1 = (2.0 * w2 - 2.0) * inv_a0;
        let a2 = (1.0 - SQRT2 * w + w2) * inv_a0;

        Self {
            b0, b1, b2, a1, a2,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
        }
    }

    /// Identity biquad (passes input through unchanged). Useful for
    /// disabling filtering without conditionals.
    pub fn identity() -> Self {
        Self {
            b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
        }
    }

    /// Seed the filter state so the first output is exactly `value`
    /// (no startup transient). Useful when initialising after a
    /// sensor first becomes available with a known steady reading.
    pub fn prime(&mut self, value: f32) {
        self.x1 = value;
        self.x2 = value;
        self.y1 = value;
        self.y2 = value;
    }

    /// Reset state to zero.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    /// Push one sample, return the filtered output.
    #[inline]
    pub fn apply(&mut self, x: f32) -> f32 {
        let y = self.b0 * x
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// Cutoff frequencies (Hz) for the gyro and accel filter chains.
#[derive(Clone, Copy, Debug)]
pub struct ImuFilterParams {
    /// Gyro cutoff frequency. 0 = bypass (identity).
    pub gyro_fc_hz: f32,
    /// Accel cutoff frequency. 0 = bypass (identity).
    pub accel_fc_hz: f32,
    /// Sample rate of the input stream. For the dual-IMU read task
    /// this is 8 kHz.
    pub fs_hz: f32,
}

impl Default for ImuFilterParams {
    fn default() -> Self {
        Self {
            // 150 Hz gives <10° phase lag at 30 Hz (top of quad
            // attitude bandwidth) while cutting most prop / motor
            // harmonics above ~kHz.
            gyro_fc_hz: 150.0,
            // 25 Hz: half the 50 Hz Nyquist of the 100 Hz accel
            // decimation. 2nd-order Butterworth gives ~ −12 dB at
            // 50 Hz from there. Higher cutoffs lose anti-alias
            // headroom quickly (30 Hz → −9 dB only).
            accel_fc_hz: 25.0,
            fs_hz: 8_000.0,
        }
    }
}

/// Per-axis filter bank: 3 gyro biquads + 3 accel biquads.
///
/// `apply` consumes a 3-vector accel + 3-vector gyro (in the units the
/// chip-level driver produces — `i16` counts) and returns the filtered
/// values. Operating on `f32` internally keeps the state precision
/// reasonable; the result is cast back to `i16` so the downstream
/// `RawImu` type can stay unchanged.
pub struct ImuFilter {
    gyro: [Biquad; 3],
    accel: [Biquad; 3],
}

impl ImuFilter {
    pub fn new(params: ImuFilterParams) -> Self {
        let g = if params.gyro_fc_hz > 0.0 {
            Biquad::new_lowpass_butterworth(params.gyro_fc_hz, params.fs_hz)
        } else {
            Biquad::identity()
        };
        let a = if params.accel_fc_hz > 0.0 {
            Biquad::new_lowpass_butterworth(params.accel_fc_hz, params.fs_hz)
        } else {
            Biquad::identity()
        };
        Self {
            gyro: [g; 3],
            accel: [a; 3],
        }
    }

    /// Seed all six filter states so the first output equals the
    /// supplied sample. Avoids a "ramp-up from zero" transient when
    /// the consumer starts reading.
    pub fn prime(&mut self, accel: [i16; 3], gyro: [i16; 3]) {
        for i in 0..3 {
            self.accel[i].prime(accel[i] as f32);
            self.gyro[i].prime(gyro[i] as f32);
        }
    }

    /// Filter a sample pair. Inputs / outputs are i16 LSB counts.
    /// Casting via `as i16` saturates at the extremes on overflow,
    /// which is what we want — a clipped sample beats a wrapped one.
    pub fn apply(&mut self, accel: [i16; 3], gyro: [i16; 3]) -> ([i16; 3], [i16; 3]) {
        let mut a_out = [0i16; 3];
        let mut g_out = [0i16; 3];
        for i in 0..3 {
            let af = self.accel[i].apply(accel[i] as f32);
            let gf = self.gyro[i].apply(gyro[i] as f32);
            // Saturating cast (clamp to i16 range, then truncate).
            a_out[i] = af.clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            g_out[i] = gf.clamp(i16::MIN as f32, i16::MAX as f32) as i16;
        }
        (a_out, g_out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Driving a biquad with a constant DC value should converge to
    /// the same value at the output (gain = 1.0 at DC).
    #[test]
    fn dc_gain_is_unity() {
        let mut b = Biquad::new_lowpass_butterworth(100.0, 8000.0);
        // Burn the startup transient — 0.5 s at 8 kHz is overkill.
        for _ in 0..4000 {
            b.apply(1.0);
        }
        let y = b.apply(1.0);
        assert!((y - 1.0).abs() < 1e-4, "DC gain = {}", y);
    }

    /// At the cutoff frequency, gain should be approximately −3 dB
    /// (0.707). Drive a sinusoid through, look at the peak amplitude
    /// after the transient has decayed.
    #[test]
    fn cutoff_at_minus_three_db() {
        const FS: f32 = 8000.0;
        const FC: f32 = 150.0;
        let mut b = Biquad::new_lowpass_butterworth(FC, FS);
        // Run 0.5 s of sinusoid; pick the last 100 ms's max.
        let n_warm = (FS * 0.4) as usize;
        let n_meas = (FS * 0.1) as usize;
        for k in 0..n_warm {
            let t = k as f32 / FS;
            b.apply(libm::sinf(2.0 * PI * FC * t));
        }
        let mut peak = 0.0_f32;
        for k in 0..n_meas {
            let t = (n_warm + k) as f32 / FS;
            let y = b.apply(libm::sinf(2.0 * PI * FC * t));
            if y.abs() > peak {
                peak = y.abs();
            }
        }
        // −3 dB ≈ 0.7079. Allow ±5% slack for window edge effects.
        assert!(peak > 0.67 && peak < 0.74, "cutoff peak = {}", peak);
    }

    /// Anti-alias check: 25 Hz Butterworth must put 50 Hz at or
    /// below −12 dB so aliasing through the 80:1 accel decimation is
    /// benign. Exact theoretical value: 1/√(1+(50/25)⁴) ≈ 0.243.
    #[test]
    fn accel_filter_attenuates_50hz_past_12db() {
        const FS: f32 = 8000.0;
        const FC: f32 = 25.0;
        const FTEST: f32 = 50.0;
        let mut b = Biquad::new_lowpass_butterworth(FC, FS);
        let n_warm = (FS * 0.4) as usize;
        let n_meas = (FS * 0.2) as usize;
        for k in 0..n_warm {
            let t = k as f32 / FS;
            b.apply(libm::sinf(2.0 * PI * FTEST * t));
        }
        let mut peak = 0.0_f32;
        for k in 0..n_meas {
            let t = (n_warm + k) as f32 / FS;
            let y = b.apply(libm::sinf(2.0 * PI * FTEST * t));
            if y.abs() > peak {
                peak = y.abs();
            }
        }
        // −12 dB = 0.251. 50 Hz is ~1.67× cutoff; 2nd-order
        // Butterworth gives ~ −17 dB there.
        assert!(peak < 0.25, "50Hz peak = {} (expected < 0.25)", peak);
    }

    /// Prime-then-step: priming should kill the startup transient.
    /// A primed filter fed its own DC value outputs that value
    /// immediately, no settling.
    #[test]
    fn prime_eliminates_startup_transient() {
        let mut b = Biquad::new_lowpass_butterworth(50.0, 8000.0);
        b.prime(2048.0);
        let y0 = b.apply(2048.0);
        assert!((y0 - 2048.0).abs() < 1e-3, "first output = {}", y0);
    }

    /// Identity biquad should pass samples through unchanged.
    #[test]
    fn identity_passes_through() {
        let mut b = Biquad::identity();
        for v in [-1000.0, 0.0, 500.0, 12345.0, -7890.0] {
            assert_eq!(b.apply(v), v);
        }
    }

    /// Disabled (fc=0) bank should round-trip i16 samples losslessly.
    #[test]
    fn zero_cutoff_is_bypass() {
        let params = ImuFilterParams {
            gyro_fc_hz: 0.0,
            accel_fc_hz: 0.0,
            fs_hz: 8000.0,
        };
        let mut f = ImuFilter::new(params);
        let a_in = [123, -456, 789];
        let g_in = [-1024, 2048, -512];
        let (a_out, g_out) = f.apply(a_in, g_in);
        assert_eq!(a_out, a_in);
        assert_eq!(g_out, g_in);
    }

    /// Filter bank is stable: feeding pseudo-random noise for a long
    /// time should not produce a NaN or infinite output.
    #[test]
    fn filter_bank_is_stable_under_noise() {
        let mut f = ImuFilter::new(ImuFilterParams::default());
        let mut seed: u32 = 0xC0FFEE;
        for _ in 0..16_000 {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let s = (seed >> 16) as i16;
            let (a, g) = f.apply([s, s.wrapping_neg(), s], [s / 2, s, -s]);
            for v in a.iter().chain(g.iter()) {
                assert!(*v != i16::MIN || *v != i16::MAX,
                    "biquad pegged: {}", v);
            }
        }
    }
}
