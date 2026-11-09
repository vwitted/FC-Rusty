// mag_sim.rs — magnetometer model for the host simulation.
//
// Conventions (the same as the rest of `sim` and as AttitudeMekf):
//   - Nav frame NED. The geomagnetic field in nav is
//       m_nav = |B| [cos I cos D, cos I sin D, sin I]
//     with inclination I positive down (northern hemisphere) and
//     declination D positive east of true north.
//   - Attitude quaternions are Hamilton [w, x, y, z], body -> nav. The body
//     reading is therefore m_body = R(q)^T m_nav, which is what the MEKF's
//     mag update predicts (h = R(q)^T m_ref, attitude_mekf.rs).
//   - Units uT, as the firmware's MAG_DATA (`ut()`).
//
// Error model, applied in this order to the true body field:
//   1. external disturbance, nav frame, inside a time window
//      (steel structure, power line): added to m_nav before rotation;
//   2. soft iron: a 3x3 matrix on the body field;
//   3. hard iron: a constant body offset;
//   4. motor-current interference: a body vector scaled by mean throttle;
//   5. white noise per axis.
//
// THE NUMBERS IN `MagSimParams::PLACEHOLDER` ARE NOT MEASURED. They are
// order-of-magnitude values chosen so the model can be exercised and unit
// tested. The throttle interference in particular is airframe specific and
// must be fitted from a bench capture (corrected field logged by mekf_task
// at several throttle levels) before any result that depends on it is
// trusted.

use crate::sim::sensors::Rng;

/// Magnetometer simulation parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagSimParams {
    /// Output rate, Hz. 0 disables the magnetometer.
    pub rate_hz: f32,
    /// Total field strength, uT.
    pub field_ut: f32,
    /// Inclination, degrees, positive down.
    pub inclination_deg: f32,
    /// Declination, degrees, positive east.
    pub declination_deg: f32,
    /// Hard-iron offset, body, uT.
    pub hard_iron_ut: [f32; 3],
    /// Soft-iron matrix, applied to the body field. Identity = none.
    pub soft_iron: [[f32; 3]; 3],
    /// White noise, 1 sigma per axis, uT.
    pub noise_ut: f32,
    /// Interference at full throttle (mean motor command 1.0), body, uT.
    /// Scales linearly with mean motor command.
    pub throttle_ut: [f32; 3],
    /// External disturbance, nav frame, uT, applied in `disturb_s`.
    pub disturb_nav_ut: [f32; 3],
    /// Disturbance window, seconds [start, end).
    pub disturb_s: (f32, f32),
}

const IDENTITY: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

impl MagSimParams {
    /// Disabled: no samples are produced.
    pub const OFF: Self = Self {
        rate_hz: 0.0,
        field_ut: 0.0,
        inclination_deg: 0.0,
        declination_deg: 0.0,
        hard_iron_ut: [0.0; 3],
        soft_iron: IDENTITY,
        noise_ut: 0.0,
        throttle_ut: [0.0; 3],
        disturb_nav_ut: [0.0; 3],
        disturb_s: (f32::INFINITY, f32::INFINITY),
    };

    /// PLACEHOLDER VALUES, NOT MEASURED. See the module header.
    ///
    /// - 100 Hz: the LIS2MDL rate the firmware runs.
    /// - 50 uT, inclination 65 deg: a representative mid-latitude field.
    /// - Declination: control::mag_cal::DECLINATION_DEG, so a yaw anchored
    ///   by the firmware's own constant is comparable to sim truth.
    /// - Hard iron 2 uT and soft iron within 2 %: plausible residuals after
    ///   a sphere-fit calibration (the uncalibrated offset is set per
    ///   scenario).
    /// - Noise 0.3 uT: the LIS2MDL datasheet order (3 mgauss RMS).
    /// - Throttle interference 3 uT at full throttle, mostly along body z.
    pub const PLACEHOLDER: Self = Self {
        rate_hz: 100.0,
        field_ut: 50.0,
        inclination_deg: 65.0,
        declination_deg: crate::control::mag_cal::DECLINATION_DEG,
        hard_iron_ut: [1.5, -1.0, 0.8],
        soft_iron: [[1.02, 0.01, 0.0], [0.01, 0.99, 0.005], [0.0, 0.005, 1.0]],
        noise_ut: 0.3,
        throttle_ut: [0.8, -0.5, 3.0],
        disturb_nav_ut: [0.0; 3],
        disturb_s: (f32::INFINITY, f32::INFINITY),
    };

    pub fn enabled(&self) -> bool {
        self.rate_hz > 0.0
    }

    /// True geomagnetic field in nav (NED), uT.
    pub fn field_nav(&self) -> [f32; 3] {
        let i = self.inclination_deg.to_radians();
        let d = self.declination_deg.to_radians();
        let h = self.field_ut * libm::cosf(i);
        [h * libm::cosf(d), h * libm::sinf(d), self.field_ut * libm::sinf(i)]
    }
}

/// R(q)^T v for a body -> nav Hamilton quaternion: nav vector into body.
pub fn rotate_nav_to_body(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let r = [
        [w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), w * w - x * x - y * y + z * z],
    ];
    // Transpose: out_i = sum_j r[j][i] v[j].
    [
        r[0][0] * v[0] + r[1][0] * v[1] + r[2][0] * v[2],
        r[0][1] * v[0] + r[1][1] * v[1] + r[2][1] * v[2],
        r[0][2] * v[0] + r[1][2] * v[1] + r[2][2] * v[2],
    ]
}

/// Magnetometer simulator.
pub struct MagSim {
    p: MagSimParams,
    period: f32,
    time_accum: f32,
    t: f32,
    rng: Rng,
}

impl MagSim {
    pub fn new(p: MagSimParams, seed: u64) -> Self {
        let period = if p.enabled() { 1.0 / p.rate_hz } else { f32::INFINITY };
        Self { p, period, time_accum: 0.0, t: 0.0, rng: Rng::new(seed) }
    }

    pub fn params(&self) -> &MagSimParams {
        &self.p
    }

    /// The reading with every error source but noise, uT: the deterministic
    /// part of `tick`'s output.
    pub fn reading_noiseless(&self, q_true: [f32; 4], throttle: f32, t: f32) -> [f32; 3] {
        let mut m_nav = self.p.field_nav();
        if t >= self.p.disturb_s.0 && t < self.p.disturb_s.1 {
            for i in 0..3 {
                m_nav[i] += self.p.disturb_nav_ut[i];
            }
        }
        let b = rotate_nav_to_body(q_true, m_nav);
        let s = &self.p.soft_iron;
        let mut out = [0.0f32; 3];
        for i in 0..3 {
            out[i] = s[i][0] * b[0] + s[i][1] * b[1] + s[i][2] * b[2]
                + self.p.hard_iron_ut[i]
                + self.p.throttle_ut[i] * throttle;
        }
        out
    }

    /// Advance `dt`; when a sample is due, return the body reading, uT.
    /// `throttle` is the mean motor command, 0..1.
    pub fn tick(&mut self, dt: f32, q_true: [f32; 4], throttle: f32) -> Option<[f32; 3]> {
        self.t += dt;
        if !self.p.enabled() {
            return None;
        }
        self.time_accum += dt;
        if self.time_accum < self.period {
            return None;
        }
        self.time_accum -= self.period;
        let mut m = self.reading_noiseless(q_true, throttle, self.t);
        for v in m.iter_mut() {
            *v += self.p.noise_ut * self.rng.normal();
        }
        Some(m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::attitude_mekf::{AttitudeMekf, MekfParams};

    fn clean() -> MagSimParams {
        MagSimParams {
            hard_iron_ut: [0.0; 3],
            soft_iron: IDENTITY,
            noise_ut: 0.0,
            throttle_ut: [0.0; 3],
            declination_deg: 10.0,
            ..MagSimParams::PLACEHOLDER
        }
    }

    fn yaw_quat(yaw_deg: f32) -> [f32; 4] {
        let h = yaw_deg.to_radians() * 0.5;
        [libm::cosf(h), 0.0, 0.0, libm::sinf(h)]
    }

    /// Level and pointing north the body reads the nav field unchanged;
    /// yawed 90 deg (nose east) body x reads the east component and body y
    /// (pointing south) reads minus the north component.
    #[test]
    fn body_reading_is_the_nav_field_rotated_into_body() {
        let s = MagSim::new(clean(), 1);
        let n = clean().field_nav();
        let at0 = s.reading_noiseless([1.0, 0.0, 0.0, 0.0], 0.0, 0.0);
        let at90 = s.reading_noiseless(yaw_quat(90.0), 0.0, 0.0);
        for i in 0..3 {
            assert!((at0[i] - n[i]).abs() < 1e-4, "{at0:?} vs {n:?}");
        }
        assert!((at90[0] - n[1]).abs() < 1e-4, "{at90:?}");
        assert!((at90[1] + n[0]).abs() < 1e-4, "{at90:?}");
        assert!((at90[2] - n[2]).abs() < 1e-4, "{at90:?}");
        // Inclination positive down: the field points into the ground.
        assert!(n[2] > 0.0 && (libm::atan2f(n[2], libm::hypotf(n[0], n[1])).to_degrees() - 65.0).abs() < 1e-3);
    }

    /// At the true attitude, with an exact reference, the MEKF's mag
    /// update sees zero innovation and does not move. A transpose or sign
    /// error between this model and the MEKF's h(q) fails here.
    #[test]
    fn mekf_sees_zero_innovation_at_the_true_attitude() {
        let p = clean();
        let s = MagSim::new(p, 1);
        // An arbitrary attitude: roll 20, pitch -10, yaw 140 (Z-Y-X).
        let q = {
            let r = nalgebra::UnitQuaternion::from_euler_angles(
                20f32.to_radians(),
                -10f32.to_radians(),
                140f32.to_radians(),
            );
            let c = r.coords; // [i, j, k, w]
            [c[3], c[0], c[1], c[2]]
        };
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.set_quaternion(q);
        assert!(m.set_mag_reference(p.field_nav()));
        let q0 = m.quaternion();
        for _ in 0..20 {
            assert!(m.update_mag(s.reading_noiseless(q, 0.0, 0.0)));
        }
        let q1 = m.quaternion();
        for i in 0..4 {
            assert!((q0[i] - q1[i]).abs() < 1e-4, "{q0:?} -> {q1:?}");
        }
    }

    /// From a 30 deg yaw error the mag update pulls the estimate to the
    /// true heading.
    #[test]
    fn mekf_mag_update_converges_a_yaw_error() {
        let p = clean();
        let s = MagSim::new(p, 1);
        let q_true = yaw_quat(40.0);
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.set_quaternion(yaw_quat(10.0));
        assert!(m.set_mag_reference(p.field_nav()));
        for _ in 0..2000 {
            m.predict([0.0; 3], 0.01);
            m.update_accel([0.0, 0.0, -1.0]);
            m.update_mag(s.reading_noiseless(q_true, 0.0, 0.0));
        }
        let yaw = m.euler()[2].to_degrees();
        assert!((yaw - 40.0).abs() < 1.0, "yaw {yaw}");
    }

    #[test]
    fn throttle_interference_is_linear_in_mean_command() {
        let p = MagSimParams { throttle_ut: [1.0, -2.0, 4.0], ..clean() };
        let s = MagSim::new(p, 1);
        let q = [1.0, 0.0, 0.0, 0.0];
        let a = s.reading_noiseless(q, 0.0, 0.0);
        let b = s.reading_noiseless(q, 0.5, 0.0);
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        assert!((d[0] - 0.5).abs() < 1e-5 && (d[1] + 1.0).abs() < 1e-5 && (d[2] - 2.0).abs() < 1e-5);
    }

    #[test]
    fn disturbance_applies_only_inside_its_window() {
        let p = MagSimParams { disturb_nav_ut: [0.0, 20.0, 0.0], disturb_s: (1.0, 2.0), ..clean() };
        let s = MagSim::new(p, 1);
        let q = [1.0, 0.0, 0.0, 0.0];
        let before = s.reading_noiseless(q, 0.0, 0.5);
        let during = s.reading_noiseless(q, 0.0, 1.5);
        let after = s.reading_noiseless(q, 0.0, 2.0);
        assert!((during[1] - before[1] - 20.0).abs() < 1e-4);
        assert_eq!(before, after);
    }

    #[test]
    fn fires_at_rate_and_is_silent_when_off() {
        let mut on = MagSim::new(MagSimParams::PLACEHOLDER, 1);
        let mut off = MagSim::new(MagSimParams::OFF, 1);
        let q = [1.0, 0.0, 0.0, 0.0];
        let (mut n_on, mut n_off) = (0, 0);
        for _ in 0..8000 {
            n_on += on.tick(1.25e-4, q, 0.4).is_some() as u32;
            n_off += off.tick(1.25e-4, q, 0.4).is_some() as u32;
        }
        assert!((99..=101).contains(&n_on), "{n_on}");
        assert_eq!(n_off, 0);
    }
}
