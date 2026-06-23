//! Magnetometer hard-iron calibration: online least-squares sphere fit
//! for the offset + bin-coverage completion. Pure no_std, host-tested.
//! Spec: docs/superpowers/specs/2026-06-22-mag-cal-yaw-fix-design.md

use nalgebra::{Matrix4, Vector4};

/// Magnetic declination at the flight location, degrees east-positive.
/// Single source of truth for the true-north anchor. Edit + rebuild to
/// change location.
pub const DECLINATION_DEG: f32 = 0.3;

/// Coverage bins required (of 24 = 8 azimuth × 3 elevation).
const COVERAGE_BINS_REQUIRED: u32 = 20;
/// Minimum total samples before completion.
const MIN_SAMPLES: u32 = 400;
/// Minimum per-axis field span (µT) — rejects a "didn't rotate" finish.
const MIN_SPAN_UT: f32 = 20.0;

/// Cal lifecycle command (nav task → mekf task).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalCommand {
    Start,
    Abort,
}

/// Online hard-iron sphere-fit + coverage tracker.
pub struct MagCalibrator {
    ata: Matrix4<f32>,
    atb: Vector4<f32>,
    min: [f32; 3],
    max: [f32; 3],
    coverage: u32, // 24-bit mask
    count: u32,
}

impl Default for MagCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MagCalibrator {
    pub fn new() -> Self {
        Self {
            ata: Matrix4::zeros(),
            atb: Vector4::zeros(),
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            coverage: 0,
            count: 0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one raw mag sample (any consistent unit; µT here).
    pub fn feed(&mut self, s: [f32; 3]) {
        let (x, y, z) = (s[0], s[1], s[2]);
        let a = Vector4::new(2.0 * x, 2.0 * y, 2.0 * z, 1.0);
        let b = x * x + y * y + z * z;
        self.ata += a * a.transpose();
        self.atb += a * b;
        for i in 0..3 {
            if s[i] < self.min[i] {
                self.min[i] = s[i];
            }
            if s[i] > self.max[i] {
                self.max[i] = s[i];
            }
        }
        let cx = 0.5 * (self.min[0] + self.max[0]);
        let cy = 0.5 * (self.min[1] + self.max[1]);
        let cz = 0.5 * (self.min[2] + self.max[2]);
        let (dx, dy, dz) = (x - cx, y - cy, z - cz);
        let horiz = libm::sqrtf(dx * dx + dy * dy);
        if horiz > 1.0e-3 || libm::fabsf(dz) > 1.0e-3 {
            use core::f32::consts::PI;
            let az = libm::atan2f(dy, dx); // −π..π
            let az_sector = (((az + PI) / (2.0 * PI) * 8.0) as u32).min(7);
            let elev = libm::atan2f(dz, horiz); // −π/2..π/2
            let el_band: u32 = if elev < -0.3 {
                0
            } else if elev > 0.3 {
                2
            } else {
                1
            };
            self.coverage |= 1 << (el_band * 8 + az_sector);
        }
        self.count = self.count.wrapping_add(1);
    }

    pub fn progress(&self) -> u8 {
        ((self.coverage.count_ones() * 100) / 24) as u8
    }

    fn span_ok(&self) -> bool {
        (0..3).all(|i| self.max[i] - self.min[i] >= MIN_SPAN_UT)
    }

    pub fn is_complete(&self) -> bool {
        self.coverage.count_ones() >= COVERAGE_BINS_REQUIRED
            && self.count >= MIN_SAMPLES
            && self.span_ok()
    }

    /// Fitted hard-iron offset (sphere centre). `None` if degenerate.
    pub fn result(&self) -> Option<[f32; 3]> {
        let inv = self.ata.try_inverse()?;
        let p = inv * self.atb;
        if p[0].is_finite() && p[1].is_finite() && p[2].is_finite() {
            Some([p[0], p[1], p[2]])
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate points on a sphere of radius `r` centred at `c`, swept
    /// over a grid of azimuth × elevation, and feed them to the cal.
    fn feed_sphere(cal: &mut MagCalibrator, c: [f32; 3], r: f32, n_az: u32, n_el: u32) {
        use core::f32::consts::PI;
        for ia in 0..n_az {
            let az = (ia as f32) / (n_az as f32) * 2.0 * PI;
            for ie in 0..=n_el {
                let el = -PI / 2.0 + (ie as f32) / (n_el as f32) * PI;
                let x = c[0] + r * libm::cosf(el) * libm::cosf(az);
                let y = c[1] + r * libm::cosf(el) * libm::sinf(az);
                let z = c[2] + r * libm::sinf(el);
                cal.feed([x, y, z]);
            }
        }
    }

    #[test]
    fn recovers_sphere_centre() {
        let mut cal = MagCalibrator::new();
        let c = [12.0, -5.0, 30.0];
        feed_sphere(&mut cal, c, 45.0, 36, 12);
        let off = cal.result().expect("fit");
        for i in 0..3 {
            assert!((off[i] - c[i]).abs() < 0.5, "axis {} got {}", i, off[i]);
        }
    }

    #[test]
    fn full_spread_completes() {
        let mut cal = MagCalibrator::new();
        feed_sphere(&mut cal, [0.0, 0.0, 0.0], 45.0, 36, 12);
        assert!(cal.is_complete());
        assert_eq!(cal.progress(), 100);
    }

    #[test]
    fn horizontal_ring_does_not_complete() {
        // A flat spin (elevation ~0) hits only the middle band — no
        // up/down coverage, so completion must not trigger.
        use core::f32::consts::PI;
        let mut cal = MagCalibrator::new();
        for ia in 0..120 {
            let az = (ia as f32) / 120.0 * 2.0 * PI;
            cal.feed([45.0 * libm::cosf(az), 45.0 * libm::sinf(az), 0.0]);
        }
        assert!(!cal.is_complete());
    }

    #[test]
    fn no_rotation_does_not_complete() {
        let mut cal = MagCalibrator::new();
        for _ in 0..1000 {
            cal.feed([40.0, 0.0, 0.0]);
        }
        assert!(!cal.is_complete()); // span gate fails
    }

    #[test]
    fn degenerate_input_returns_none() {
        let mut cal = MagCalibrator::new();
        cal.feed([1.0, 2.0, 3.0]); // single point → singular AᵀA
        assert!(cal.result().is_none());
    }
}
