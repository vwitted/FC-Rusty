// mag.rs — the parts of a magnetometer driver that are not chip-specific.
//
// Five magnetometers now feed the same pipeline: the LIS2MDL (STMicro,
// 0x1E), the QMC5883P (QST, 0x2C), and whichever of the QMC5883L (QST,
// 0x0D), HMC5883L (Honeywell, 0x1E) or IST8310 (iSentek, 0x0C-0x0F) a
// given Radiolink SE100 GPS module carries -- the V2 has the IST8310,
// found at 0x0E on the bench.
// Everything downstream of the driver -- MAG_DATA, MagCalibrator,
// AttitudeMekf::update_mag -- consumes `MagSample::ut()` and has no
// business knowing which part produced it.
//
// So the sample type carries its own scale rather than reading a
// per-chip constant. That is the whole difference between the two
// drivers as far as the fusion code is concerned: 0.15 uT/LSB for the
// LIS2MDL, 0.0333 uT/LSB for the QMC5883L at +/-8 G, 0.0267 uT/LSB for
// the QMC5883P at +/-8 G, 0.122 uT/LSB for the HMC5883L at +/-1.9 G,
// 0.3 uT/LSB for the IST8310. Note the two QMC parts differ at the same
// nominal range -- 3000 vs 3750 LSB/G -- which is exactly the kind of
// per-chip constant this type exists to keep out of the fusion code.
//
// Host-testable: nothing here touches embassy or I2C.

/// How a magnetometer is mounted relative to the FC body frame (NED).
///
/// Covers a part soldered flat in any of the eight yaw/flip positions:
/// four 90-degree yaw steps, each either upright or flipped. That is the
/// whole set of mountings for a board-mounted sensor whose Z axis is
/// vertical, which is every case on this airframe.
///
/// # Completeness
///
/// These eight are a GROUP: the rotations that map the vertical axis to
/// plus or minus itself, closed under composition, every one a proper
/// rotation (determinant +1, never a mirror). Both facts are pinned by
/// the tests below, and closure is what lets "turn it round, then flip
/// it" resolve to a single variant instead of needing a new one.
///
/// A mounting NOT in this set -- a sensor tipped onto its side, or at
/// 45 degrees -- is deliberately not expressible. Better a missing
/// variant than one that silently drops a component.
///
/// # Mapping from Betaflight / configurator alignment names
///
/// The trap is that Betaflight's "FLIP" is **two 90-degree PITCH
/// rotations**, stated in `sensor_alignment.h` as
/// `CW0_DEG_FLIP = 5, // _FLIP = 2x90 degree PITCH rotations`. It is not
/// a roll, so the composition does not come out where the name suggests:
/// `CW180FLIP` is `Roll180`, NOT `Pitch180`.
///
/// | Betaflight  | = pitch x yaw       | this enum    | body axes        |
/// |-------------|---------------------|--------------|------------------|
/// | `CW0`       | --                  | `Identity`   | +x, +y, +z       |
/// | `CW90`      | yaw 90              | `Yaw90`      | -y, +x, +z       |
/// | `CW180`     | yaw 180             | `Yaw180`     | -x, -y, +z       |
/// | `CW270`     | yaw 270             | `Yaw270`     | +y, -x, +z       |
/// | `CW0FLIP`   | pitch 180           | `Pitch180`   | -x, +y, -z       |
/// | `CW90FLIP`  | pitch 180 + yaw 90  | `Yaw90Flip`  | +y, +x, -z       |
/// | `CW180FLIP` | pitch 180 + yaw 180 | `Roll180`    | +x, -y, -z       |
/// | `CW270FLIP` | pitch 180 + yaw 270 | `Yaw270Flip` | -y, -x, -z       |
///
/// The four names that are not `YawNN` are the ones that predate the
/// 90-degree variants and are kept because they are what the mounting
/// constants elsewhere already say.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum Orientation {
    /// No change — sensor frame == body frame (NED). Betaflight `CW0`.
    Identity,
    /// Yaw 90°: X → −Y, Y → +X, Z → +Z. Betaflight `CW90`.
    Yaw90,
    /// Yaw 180°: X → −X, Y → −Y, Z → +Z. Betaflight `CW180`.
    Yaw180,
    /// Yaw 270°: X → +Y, Y → −X, Z → +Z. Betaflight `CW270`.
    Yaw270,
    /// Pitch 180°: X → −X, Y → +Y, Z → −Z. Betaflight `CW0FLIP`.
    Pitch180,
    /// Flipped, then yawed 90°: X → +Y, Y → +X, Z → −Z. `CW90FLIP`.
    Yaw90Flip,
    /// Roll 180°: X → +X, Y → −Y, Z → −Z. Betaflight `CW180FLIP`.
    Roll180,
    /// Flipped, then yawed 270°: X → −Y, Y → −X, Z → −Z. `CW270FLIP`.
    Yaw270Flip,
}

impl Orientation {
    /// Rotate a sensor-frame vector into the FC body frame.
    ///
    /// Four of the eight are pure sign flips; the other four also SWAP
    /// the X and Y axes, which is why this cannot be a `[f32; 3]` of
    /// signs the way it was before the 90-degree mountings existed. Any
    /// code that multiplies a stored sign vector component-wise is
    /// therefore wrong for half the variants -- go through this instead.
    pub const fn apply(self, v: [f32; 3]) -> [f32; 3] {
        let [x, y, z] = v;
        match self {
            Self::Identity => [x, y, z],
            Self::Yaw90 => [-y, x, z],
            Self::Yaw180 => [-x, -y, z],
            Self::Yaw270 => [y, -x, z],
            Self::Pitch180 => [-x, y, -z],
            Self::Yaw90Flip => [y, x, -z],
            Self::Roll180 => [x, -y, -z],
            Self::Yaw270Flip => [-y, -x, -z],
        }
    }

    /// True if this mounting swaps X and Y rather than only negating.
    ///
    /// Diagnostic and test support: the four swapping variants are
    /// exactly the odd multiples of 90 degrees of yaw, and they are the
    /// ones a sign-vector representation cannot express.
    #[allow(dead_code)] // test and bring-up support; no flight-path caller yet
    pub const fn swaps_xy(self) -> bool {
        matches!(
            self,
            Self::Yaw90 | Self::Yaw270 | Self::Yaw90Flip | Self::Yaw270Flip
        )
    }

    /// Every variant, for exhaustive checks.
    #[allow(dead_code)] // consumed by the exhaustive tests below
    pub const ALL: [Self; 8] = [
        Self::Identity,
        Self::Yaw90,
        Self::Yaw180,
        Self::Yaw270,
        Self::Pitch180,
        Self::Yaw90Flip,
        Self::Roll180,
        Self::Yaw270Flip,
    ];
}

/// One magnetometer reading, chip-agnostic.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub struct MagSample {
    /// Raw counts, sensor axes, as read off the wire.
    pub raw: [i16; 3],
    /// Microtesla per count for the range the chip is configured in.
    scale_ut_per_lsb: f32,
    /// How the part is mounted. Stored rather than pre-resolved into a
    /// sign vector, because the 90-degree mountings swap axes and a sign
    /// vector cannot represent that.
    orientation: Orientation,
}

impl MagSample {
    pub const fn new(raw: [i16; 3], scale_ut_per_lsb: f32, orientation: Orientation) -> Self {
        Self { raw, scale_ut_per_lsb, orientation }
    }

    /// Field in microtesla, rotated into FC body frame (NED). This is
    /// what the estimator and the calibrator consume.
    pub fn ut(&self) -> [f32; 3] {
        self.orientation.apply(self.ut_sensor())
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
    /// The part reported a saturated axis. The sample is meaningless,
    /// not merely large, and must not be fused. See `hmc5883l`: the
    /// HMC5883L signals this with -4096 in the axis itself, and
    /// `qmc5883p`, whose counts clip at +/-30000.
    Overflow,
}

// ---- Bus scan support ----

/// Every 7-bit address a magnetometer this firmware knows about answers
/// at, with the parts that share it. Two parts at one address is the
/// normal case, not the exception: 0x1E is both the LIS2MDL and the
/// HMC5883L, whose register maps have nothing in common, and 0x0D is
/// both the QMC5883L and one of the IST8310's four pin-selected
/// addresses. The QMC5883P is the exception: 0x2C is its alone among
/// the parts this firmware knows, despite the name it shares with the L.
pub const LIS2MDL_OR_HMC5883L_ADDR: u8 = 0x1E;
pub const QMC5883L_ADDR: u8 = 0x0D;
pub const QMC5883P_ADDR: u8 = 0x2C;
pub const IST8310_ADDRS: [u8; 4] = [0x0C, 0x0D, 0x0E, 0x0F];

/// Name whatever is expected at an address, for the scan log. The scan
/// exists because the SE100's compass has changed part twice across
/// revisions, and a probe that only knocks on the doors it expects tells
/// you nothing when the part is behind a different one.
pub const fn describe_addr(addr: u8) -> &'static str {
    match addr {
        QMC5883L_ADDR => "QMC5883L or IST8310",
        LIS2MDL_OR_HMC5883L_ADDR => "LIS2MDL or HMC5883L",
        0x0C | 0x0E | 0x0F => "IST8310",
        QMC5883P_ADDR => "QMC5883P",
        0x30 | 0x31 => "MMC5983 / RM3100",
        0x76 | 0x77 => "baro (SPL06 / DPS310 / BMP280)",
        0x28..=0x2F => "BNO055 / AK09916 range",
        0x68 | 0x69 => "MPU/ICM IMU or DS3231",
        _ => "unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 3x3 matrix of a transform, as columns: column j is where the
    /// j-th basis vector lands.
    fn matrix(f: impl Fn([f32; 3]) -> [f32; 3]) -> [[f32; 3]; 3] {
        [
            f([1.0, 0.0, 0.0]),
            f([0.0, 1.0, 0.0]),
            f([0.0, 0.0, 1.0]),
        ]
    }

    fn matrix_of(o: Orientation) -> [[f32; 3]; 3] {
        matrix(|v| o.apply(v))
    }

    /// det of a matrix given as three columns: c0 . (c1 x c2).
    fn det(m: [[f32; 3]; 3]) -> f32 {
        let [a, b, c] = m;
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
    }

    /// Earth's field is 25-65 uT. A driver that gets the scale wrong by
    /// the usual factor (gauss for tesla, or the wrong range) lands
    /// orders of magnitude outside that, so pinning the arithmetic here
    /// is worth more than it looks.
    #[test]
    fn scale_is_applied_per_axis() {
        let s = MagSample::new([100, -200, 300], 0.1, Orientation::Identity);
        let ut = s.ut();
        assert!((ut[0] - 10.0).abs() < 1e-6);
        assert!((ut[1] + 20.0).abs() < 1e-6);
        assert!((ut[2] - 30.0).abs() < 1e-6);
    }

    #[test]
    fn orientation_reaches_the_sample() {
        let raw = [100, 200, 300];
        let ident = MagSample::new(raw, 1.0, Orientation::Identity).ut();
        let roll = MagSample::new(raw, 1.0, Orientation::Roll180).ut();
        let yaw90 = MagSample::new(raw, 1.0, Orientation::Yaw90).ut();
        assert_eq!(ident, [100.0, 200.0, 300.0]);
        assert_eq!(roll, [100.0, -200.0, -300.0]);
        // The swap has to survive the trip through MagSample, which is
        // the thing a stored sign vector silently could not do.
        assert_eq!(yaw90, [-200.0, 100.0, 300.0]);
    }

    #[test]
    fn the_betaflight_alignment_table_is_what_the_docs_claim() {
        // Betaflight's FLIP is 2x90 degrees of PITCH (sensor_alignment.h:
        // "CW0_DEG_FLIP = 5, // _FLIP = 2x90 degree PITCH rotations"), so
        // every FLIP row is pitch 180 composed with that row's yaw.
        // Reading FLIP as a roll swaps CW0FLIP and CW180FLIP.
        let v = [2.0, 3.0, 5.0];
        let [x, y, z] = v;
        for (o, want, name) in [
            (Orientation::Identity, [x, y, z], "CW0"),
            (Orientation::Yaw90, [-y, x, z], "CW90"),
            (Orientation::Yaw180, [-x, -y, z], "CW180"),
            (Orientation::Yaw270, [y, -x, z], "CW270"),
            (Orientation::Pitch180, [-x, y, -z], "CW0FLIP"),
            (Orientation::Yaw90Flip, [y, x, -z], "CW90FLIP"),
            (Orientation::Roll180, [x, -y, -z], "CW180FLIP"),
            (Orientation::Yaw270Flip, [-y, -x, -z], "CW270FLIP"),
        ] {
            assert_eq!(o.apply(v), want, "{name} ({o:?})");
        }

        // The specific confusion worth a standing test: CW180FLIP is
        // pitch 180 THEN yaw 180, and that composes to Roll180.
        let cw180flip = matrix(|v| Orientation::Pitch180.apply(Orientation::Yaw180.apply(v)));
        assert_eq!(cw180flip, matrix_of(Orientation::Roll180));
        assert_ne!(cw180flip, matrix_of(Orientation::Pitch180));
    }

    #[test]
    fn every_orientation_is_a_proper_rotation() {
        // Determinant +1. A determinant of -1 is a MIRROR: it would turn
        // a right-handed frame left-handed and invert the sense of yaw
        // without changing the field magnitude -- silent, and exactly the
        // class of bug this project keeps finding.
        for o in Orientation::ALL {
            let d = det(matrix_of(o));
            assert!((d - 1.0).abs() < 1e-6, "{o:?} has determinant {d}, not +1");
        }
    }

    #[test]
    fn the_variants_are_closed_under_composition() {
        // This is what makes "turn it round, then flip it" expressible
        // without a new variant: any two of these compose to a third, so
        // a stack of mountings collapses back into the enum.
        for a in Orientation::ALL {
            for b in Orientation::ALL {
                let c = matrix(|v| a.apply(b.apply(v)));
                assert!(
                    Orientation::ALL.iter().any(|&o| matrix_of(o) == c),
                    "{a:?} * {b:?} escaped the enum",
                );
            }
        }
    }

    #[test]
    fn all_eight_are_distinct() {
        for (i, &a) in Orientation::ALL.iter().enumerate() {
            for &b in &Orientation::ALL[i + 1..] {
                assert_ne!(matrix_of(a), matrix_of(b), "{a:?} and {b:?} are the same rotation");
            }
        }
    }

    #[test]
    fn the_sign_flip_variants_are_the_complete_diagonal_set() {
        // Of the eight sign combinations, the four with determinant +1
        // are rotations and the four with -1 are mirrors. All four proper
        // ones must have a variant -- anything missing would be a
        // mounting we could not express with signs alone.
        let mut found = 0;
        for sx in [1.0f32, -1.0] {
            for sy in [1.0f32, -1.0] {
                for sz in [1.0f32, -1.0] {
                    if sx * sy * sz < 0.0 {
                        continue; // mirror, not a rotation
                    }
                    let want = [[sx, 0.0, 0.0], [0.0, sy, 0.0], [0.0, 0.0, sz]];
                    assert!(
                        Orientation::ALL.iter().any(|&o| matrix_of(o) == want),
                        "proper sign flip [{sx}, {sy}, {sz}] has no variant",
                    );
                    found += 1;
                }
            }
        }
        assert_eq!(found, 4);
    }

    #[test]
    fn exactly_the_odd_yaw_steps_swap_xy() {
        // The swapping half is what a `[f32; 3]` of signs cannot encode,
        // and the reason `MagSample` stores the Orientation itself.
        for o in Orientation::ALL {
            let m = matrix_of(o);
            // X survives as X (up to sign) iff there is no swap.
            let diagonal = m[0][1] == 0.0 && m[1][0] == 0.0;
            assert_eq!(!diagonal, o.swaps_xy(), "{o:?}");
        }
        assert_eq!(
            Orientation::ALL.iter().filter(|o| o.swaps_xy()).count(),
            4,
        );
    }

    #[test]
    fn magnitude_ignores_mounting() {
        // Frame-independent: the sign flips and swaps are orthogonal, so
        // magnitude reads the same whichever mounting is applied. That is
        // why it is the right sanity check for "is this plausible".
        let raw = [300, -400, 0];
        let base = MagSample::new(raw, 0.1, Orientation::Identity);
        // 3-4-5 triangle: 500 counts * 0.1 = 50.0 uT.
        assert!((base.magnitude_ut() - 50.0).abs() < 1e-4);
        for o in Orientation::ALL {
            let s = MagSample::new(raw, 0.1, o);
            assert!(
                (s.magnitude_ut() - base.magnitude_ut()).abs() < 1e-6,
                "{o:?} changed the magnitude",
            );
        }
    }

    #[test]
    fn mgauss_is_ten_times_microtesla() {
        let s = MagSample::new([1000, 0, 0], 0.05, Orientation::Identity);
        assert!((s.ut()[0] - 50.0).abs() < 1e-6);
        assert!((s.mgauss()[0] - 500.0).abs() < 1e-4);
    }

    #[test]
    fn ut_sensor_ignores_mounting() {
        // Calibration works in the sensor frame, so this must NOT have
        // the mounting applied -- applying it twice is the failure.
        let raw = [100, 200, 300];
        for o in Orientation::ALL {
            let s = MagSample::new(raw, 0.5, o);
            assert_eq!(s.ut_sensor(), [50.0, 100.0, 150.0], "{o:?}");
        }
    }
}
