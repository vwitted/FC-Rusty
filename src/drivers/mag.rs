// mag.rs — the parts of a magnetometer driver that are not chip-specific.
//
// Two magnetometers now feed the same pipeline: the LIS2MDL (STMicro,
// 0x1E) and the QMC5883L (QST, 0x0D) on the Radiolink SE100 GPS module.
// Everything downstream of the driver -- MAG_DATA, MagCalibrator,
// AttitudeMekf::update_mag -- consumes `MagSample::ut()` and has no
// business knowing which part produced it.
//
// So the sample type carries its own scale rather than reading a
// per-chip constant. That is the whole difference between the two
// drivers as far as the fusion code is concerned: 0.15 uT/LSB for the
// LIS2MDL, 0.0333 uT/LSB for the QMC5883L at +/-8 G.
//
// Host-testable: nothing here touches embassy or I2C.

/// How a magnetometer is mounted relative to the FC body frame (NED).
///
/// Same shape as the IMU drivers so downstream fusion code does not need
/// to special-case the magnetometer.
///
/// These are sign flips, not general rotations: they cover a part
/// soldered flat in one of four yaw/flip positions, which is every case
/// on this airframe so far. A 90-degree yaw mount would need an axis
/// SWAP and is deliberately not expressible here -- better a missing
/// variant than one that silently drops a component.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum Orientation {
    /// No axis flips — sensor frame == body frame (NED).
    Identity,
    /// Roll 180°: X → +X, Y → −Y, Z → −Z.
    Roll180,
    /// Pitch 180°: X → −X, Y → +Y, Z → −Z.
    Pitch180,
    /// Yaw 180°: X → −X, Y → −Y, Z → +Z.
    Yaw180,
}

impl Orientation {
    pub const fn sign(self) -> [f32; 3] {
        match self {
            Self::Identity => [1.0, 1.0, 1.0],
            Self::Roll180 => [1.0, -1.0, -1.0],
            Self::Pitch180 => [-1.0, 1.0, -1.0],
            Self::Yaw180 => [-1.0, -1.0, 1.0],
        }
    }
}

/// One magnetometer reading, chip-agnostic.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub struct MagSample {
    /// Raw counts, sensor axes, as read off the wire.
    pub raw: [i16; 3],
    /// Microtesla per count for the range the chip is configured in.
    scale_ut_per_lsb: f32,
    /// Body-frame sign flips from `Orientation`.
    sign: [f32; 3],
}

impl MagSample {
    pub const fn new(raw: [i16; 3], scale_ut_per_lsb: f32, sign: [f32; 3]) -> Self {
        Self { raw, scale_ut_per_lsb, sign }
    }

    /// Field in microtesla, rotated into FC body frame (NED). This is
    /// what the estimator and the calibrator consume.
    pub fn ut(&self) -> [f32; 3] {
        [
            self.raw[0] as f32 * self.scale_ut_per_lsb * self.sign[0],
            self.raw[1] as f32 * self.scale_ut_per_lsb * self.sign[1],
            self.raw[2] as f32 * self.scale_ut_per_lsb * self.sign[2],
        ]
    }

    /// Field in mgauss, body frame. 1 uT = 10 mgauss.
    pub fn mgauss(&self) -> [f32; 3] {
        let ut = self.ut();
        [ut[0] * 10.0, ut[1] * 10.0, ut[2] * 10.0]
    }

    /// Field in uT, SENSOR native frame — diagnostic and calibration use,
    /// where the mounting must not be applied twice.
    pub fn ut_sensor(&self) -> [f32; 3] {
        [
            self.raw[0] as f32 * self.scale_ut_per_lsb,
            self.raw[1] as f32 * self.scale_ut_per_lsb,
            self.raw[2] as f32 * self.scale_ut_per_lsb,
        ]
    }

    /// Magnitude of the field in uT. Frame-independent (the sign flips
    /// are orthogonal), so it reads the same either way -- which is why
    /// it is the right sanity check for "is this a plausible reading".
    pub fn magnitude_ut(&self) -> f32 {
        let v = self.ut_sensor();
        // libm unconditionally: the crate is no_std on the host too, so
        // f32::sqrt only exists under cfg(test). Same choice as
        // control::position.
        libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    }
}

/// Why a magnetometer failed to come up.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum MagError {
    /// The bus transaction itself failed (NACK, arbitration, timeout).
    I2c,
    /// The part answered, but not with the identity it should have.
    IdMismatch(u8),
    /// The part answered but did not hold a value written to it, so
    /// nothing is actually there. See `qmc5883l`: a chip ID of 0xFF is
    /// indistinguishable from an idle bus, and a readback is the only
    /// honest presence test.
    NotResponding,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Earth's field is 25-65 uT. A driver that gets the scale wrong by
    /// the usual factor (gauss for tesla, or the wrong range) lands
    /// orders of magnitude outside that, so pinning the arithmetic here
    /// is worth more than it looks.
    #[test]
    fn scale_is_applied_per_axis() {
        let s = MagSample::new([100, -200, 300], 0.1, [1.0, 1.0, 1.0]);
        let ut = s.ut();
        assert!((ut[0] - 10.0).abs() < 1e-6);
        assert!((ut[1] + 20.0).abs() < 1e-6);
        assert!((ut[2] - 30.0).abs() < 1e-6);
    }

    #[test]
    fn orientation_flips_the_right_axes() {
        let raw = [100, 200, 300];
        let ident = MagSample::new(raw, 1.0, Orientation::Identity.sign()).ut();
        let roll = MagSample::new(raw, 1.0, Orientation::Roll180.sign()).ut();
        assert_eq!(ident, [100.0, 200.0, 300.0]);
        assert_eq!(roll, [100.0, -200.0, -300.0]);
    }

    #[test]
    fn every_orientation_is_a_pure_reflection_pair() {
        // Each variant must flip exactly two axes: those are the 180 deg
        // rotations. Flipping one or three is a MIRROR, which would turn
        // a right-handed frame left-handed and invert the sense of yaw
        // without changing the field magnitude -- silent, and exactly the
        // class of bug this project keeps finding.
        for o in [
            Orientation::Identity,
            Orientation::Roll180,
            Orientation::Pitch180,
            Orientation::Yaw180,
        ] {
            let s = o.sign();
            let det = s[0] * s[1] * s[2];
            assert!((det - 1.0).abs() < 1e-6, "{o:?} has determinant {det}, not +1");
        }
    }

    #[test]
    fn magnitude_ignores_mounting() {
        let raw = [300, -400, 0];
        let a = MagSample::new(raw, 0.1, Orientation::Identity.sign());
        let b = MagSample::new(raw, 0.1, Orientation::Yaw180.sign());
        // 3-4-5 triangle: 500 counts * 0.1 = 50.0 uT.
        assert!((a.magnitude_ut() - 50.0).abs() < 1e-4);
        assert!((a.magnitude_ut() - b.magnitude_ut()).abs() < 1e-6);
    }

    #[test]
    fn mgauss_is_ten_times_microtesla() {
        let s = MagSample::new([1000, 0, 0], 0.05, Orientation::Identity.sign());
        assert!((s.ut()[0] - 50.0).abs() < 1e-6);
        assert!((s.mgauss()[0] - 500.0).abs() < 1e-4);
    }
}
