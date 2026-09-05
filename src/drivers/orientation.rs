// orientation.rs — how an IMU chip is mounted relative to the FC body frame.
//
// Split out of icm42688.rs, which needs embassy-stm32 and therefore cannot
// be compiled for host tests. This is pure algebra and the convention it
// encodes is safety-relevant, so it belongs where it can be checked: see
// src/conventions.rs, which asserts each variant is a proper ROTATION
// (determinant +1) rather than a reflection.

// ---- Board orientation ----

/// How the ICM-42688P chip is mounted relative to the FC body frame.
///
/// The DAKEFPVH743 has two IMUs with different physical rotations.
/// ArduPilot hwdef specifies:
///   IMU1 (SPI1): ROTATION_ROLL_180  → sign vector [1, -1, -1]
///   IMU2 (SPI4): ROTATION_PITCH_180 → sign vector [-1, 1, -1]
///
/// The `Identity` variant (sign [1, 1, 1]) is used for pre-averaged
/// samples that are already in body-frame NED.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum Orientation {
    /// IMU1: ROTATION_ROLL_180. Sensor X → +X, Y → −Y, Z → −Z.
    Roll180,
    /// IMU2: ROTATION_PITCH_180. Sensor X → −X, Y → +Y, Z → −Z.
    Pitch180,
    /// Pre-averaged / already in body frame. No axis flips.
    Identity,
}

impl Orientation {
    /// Applies the rotation to map sensor-native axes to FC body frame (NED).
    pub const fn apply(self, v: [f32; 3]) -> [f32; 3] {
        match self {
            Self::Roll180  => [ v[0], -v[1], -v[2]],
            Self::Pitch180 => [-v[0],  v[1], -v[2]],
            Self::Identity => [ v[0],  v[1],  v[2]],
        }
    }
}
