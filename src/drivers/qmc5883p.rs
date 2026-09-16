// qmc5883p.rs — QST QMC5883P 3-axis magnetometer over I2C.
//
// Register map, field encodings and worked examples below are from
// docs/qmc5883p-datasheet.pdf (QST-PD-B002-22, Rev E); section and table
// numbers refer to that document.
//
// The QMC5883P is NOT a drop-in successor to the QMC5883L despite the
// name. Everything a driver touches is different:
//
//   - address 0x2C, not 0x0D (section 8.2, Table 12: the slave address
//     on the wire is 0101100);
//   - the data registers start at 0x01, not 0x00;
//   - there is a real chip ID (0x00 == 0x80), where the L's reads 0xFF
//     and is useless as a presence test;
//   - control registers are at 0x0A/0x0B rather than 0x09/0x0A, and the
//     field packing inside them is different;
//   - the range encoding is four values (30/12/8/2 G), not two, and the
//     sensitivities do not match the L at any shared range;
//   - there is no SET/RESET period register and no pointer-rollover bit;
//     set/reset is a mode field in control register 2 instead.
//
// So this is a separate driver rather than a mode of `qmc5883l`, and the
// two share only `MagSample`.
//
// Configured for Normal mode at 200 Hz, +/-8 G, OSR1 8, OSR2 8 -- which
// is the datasheet's own worked example (section 7.1: "write 0BH = 0x08,
// 0AH = 0xCD").
//
// Range choice: same argument as the QMC5883L driver. Earth's field is
// 0.25-0.65 G, so +/-2 G would fit it with 4x the resolution (15000 vs
// 3750 LSB/G). +/-8 G is still chosen, because this part sees Earth's
// field PLUS whatever the power wiring is doing a few centimetres away,
// and a saturated axis is an unrecoverable reading whereas a coarser one
// is merely noisier. At 8 G one count is 0.0267 uT against an Earth
// field of ~50 uT, so there are still ~1875 counts of signal.
//
// The parsing and configuration logic here is host-testable; only the
// I2C transactions are gated behind the `firmware` feature.

#![allow(dead_code)]

use super::mag::{MagSample, Orientation};

// ---- I2C address ----

/// Fixed 7-bit slave address. Section 8.2.4, Table 12 spells the byte on
/// the wire out bit by bit as 0101100 + R/W, i.e. 7-bit 0x2C. No address
/// pin.
pub const I2C_ADDR: u8 = 0x2C;

// ---- Register map (section 9.1, Table 14) ----

const REG_CHIP_ID: u8 = 0x00;
const REG_X_LSB: u8 = 0x01;
const REG_STATUS: u8 = 0x09;
const REG_CONF1: u8 = 0x0A;
const REG_CONF2: u8 = 0x0B;

/// Axis-sign register. It is NOT in the register map (Table 14 stops at
/// 0x0B), but every setup example in section 7 opens by writing 0x06 to
/// it, and the datasheet describes that as "define the sign for X Y and
/// Z axis". Undocumented beyond that, so it is applied exactly as the
/// datasheet prescribes and not tuned.
///
/// ArduPilot's driver writes these two the wrong way round -- register
/// 0x06 (a read-only data register) gets the value 0x29. Do not use that
/// driver as the reference for this step.
const REG_AXIS_SIGN: u8 = 0x29;
const AXIS_SIGN_VALUE: u8 = 0x06;

/// Section 9.2: the chip ID register reads 0x80.
///
/// Unlike the QMC5883L's 0xFF, this is a real value that an idle
/// pulled-up bus cannot forge, so `init` uses it directly as the
/// presence test and no write-readback trick is needed.
pub const CHIP_ID_VALUE: u8 = 0x80;

// ---- Status register 0x09 (section 9.2, Table 16) ----

/// All three axes loaded into the output registers. Cleared by reading
/// the data registers.
pub const STATUS_DRDY: u8 = 1 << 0;
/// Some axis exceeded the +/-30000 LSB output range. See
/// `OVFL_COUNT_LIMIT`.
pub const STATUS_OVFL: u8 = 1 << 1;

/// The count at which the part declares overflow: the datasheet defines
/// OVFL as "set high when either axis code output exceeds the range of
/// [-30000, 30000] LSB".
///
/// That threshold is a property of the DATA, not of some latched
/// internal state, so `decode_sample` can flag a saturated reading from
/// the six data bytes alone -- no second transaction to read 0x09, and
/// no window in which the status byte refers to a different sample than
/// the one in hand. Same approach as `hmc5883l`, which spots its own
/// -4096 saturation sentinel in the axis.
pub const OVFL_COUNT_LIMIT: i16 = 30000;

// ---- Control register 1, 0x0A (section 9.2, Table 17) ----
//
//   bits 1:0  MODE  00 suspend, 01 normal, 10 single, 11 continuous
//   bits 3:2  ODR   00 10 Hz, 01 50 Hz, 10 100 Hz, 11 200 Hz
//   bits 5:4  OSR1  00 8x, 01 4x, 10 2x, 11 1x   (oversampling)
//   bits 7:6  OSR2  00 1x, 01 2x, 10 4x, 11 8x   (downsampling)
//
// Note OSR1 and OSR2 run in OPPOSITE directions: 0b00 is the largest
// oversampling ratio but the smallest downsampling depth. Both are
// "more filtering is better" at the extremes, which is 0b00 for OSR1 and
// 0b11 for OSR2.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Suspend = 0b00,
    Normal = 0b01,
    Single = 0b10,
    Continuous = 0b11,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Odr {
    Hz10 = 0b00,
    Hz50 = 0b01,
    Hz100 = 0b10,
    Hz200 = 0b11,
}

/// Oversampling ratio (internal filter bandwidth).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Osr1 {
    X8 = 0b00,
    X4 = 0b01,
    X2 = 0b10,
    X1 = 0b11,
}

/// Downsampling depth of the second filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Osr2 {
    X1 = 0b00,
    X2 = 0b01,
    X4 = 0b10,
    X8 = 0b11,
}

/// Assemble control register 1.
pub const fn conf1(osr2: Osr2, osr1: Osr1, odr: Odr, mode: Mode) -> u8 {
    ((osr2 as u8) << 6) | ((osr1 as u8) << 4) | ((odr as u8) << 2) | (mode as u8)
}

// ---- Control register 2, 0x0B (section 9.2, Table 18) ----
//
//   bits 1:0  SET/RESET MODE  00 set and reset on, 01 set only on,
//                             10 set and reset off, 11 set and reset off
//   bits 3:2  RNG             00 +/-30 G, 01 +/-12 G, 10 +/-8 G, 11 +/-2 G
//   bit  6    SELF_TEST       self-clearing after one update
//   bit  7    SOFT_RST

/// Set/reset degaussing behaviour. With it off the offset is not renewed
/// during measurement, which lets thermal and hard-iron drift walk; the
/// datasheet's examples all leave it fully on and so does this driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetReset {
    SetAndResetOn = 0b00,
    SetOnlyOn = 0b01,
    SetAndResetOff = 0b10,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Range {
    G30 = 0b00,
    G12 = 0b01,
    G8 = 0b10,
    G2 = 0b11,
}

impl Range {
    /// Microtesla per count.
    ///
    /// Section 3, the sensitivity table: 1000 LSB/G at 30 G, 2500 at
    /// 12 G, 3750 at 8 G, 15000 at 2 G. 1 gauss = 100 uT, so
    /// uT/LSB = 100 / (LSB/G).
    ///
    /// These do NOT match the QMC5883L at the one range the two parts
    /// share: the L is 3000 LSB/G at 8 G, this part 3750. Reusing the
    /// L's constant here would read 25% high.
    pub const fn ut_per_lsb(self) -> f32 {
        match self {
            Self::G30 => 100.0 / 1000.0,
            Self::G12 => 100.0 / 2500.0,
            Self::G8 => 100.0 / 3750.0,
            Self::G2 => 100.0 / 15000.0,
        }
    }
}

/// Assemble control register 2 for running (no reset, no self-test).
pub const fn conf2(rng: Range, sr: SetReset) -> u8 {
    ((rng as u8) << 2) | (sr as u8)
}

/// Soft reset: restores every register to its default, which includes
/// dropping the part back to Suspend mode. Valid from any mode at any
/// time (section 9.2, Table 18).
const CONF2_SOFT_RST: u8 = 1 << 7;

// ---- Chosen configuration ----

pub const CFG_OSR2: Osr2 = Osr2::X8;
pub const CFG_OSR1: Osr1 = Osr1::X8;
pub const CFG_ODR: Odr = Odr::Hz200;
pub const CFG_RANGE: Range = Range::G8;
pub const CFG_SET_RESET: SetReset = SetReset::SetAndResetOn;

/// Control register 1 as written at init. Equals the datasheet's own
/// example value, 0xCD (section 7.1).
///
/// Normal mode, not Continuous. In Normal mode the output registers
/// refresh at the selected ODR, so 200 Hz here means 200 Hz. Continuous
/// mode ignores ODR and free-runs at whatever the oversampling settings
/// allow (up to 1500 Hz per the section 3 table) for correspondingly
/// more current -- a rate this firmware cannot pin down and does not
/// need, since the mag task polls far slower than either.
pub const CONF1_RUN: u8 = conf1(CFG_OSR2, CFG_OSR1, CFG_ODR, Mode::Normal);

/// Control register 2 as written at init. Equals the datasheet's own
/// example value, 0x08 (section 7.1/7.2).
pub const CONF2_RUN: u8 = conf2(CFG_RANGE, CFG_SET_RESET);

/// Microtesla per count in the configured range.
pub const SENS_UT_PER_LSB: f32 = CFG_RANGE.ut_per_lsb();

// ---- Sample decoding ----

/// Decode the six data bytes into raw counts.
///
/// Each axis is 16-bit two's complement, LSB first, in X, Y, Z order
/// starting at 0x01 (section 9.1, Table 14).
pub const fn decode_raw(buf: [u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([buf[0], buf[1]]),
        i16::from_le_bytes([buf[2], buf[3]]),
        i16::from_le_bytes([buf[4], buf[5]]),
    ]
}

/// True if any axis is past the overflow threshold the part uses for its
/// own OVFL bit. Such a sample is meaningless, not merely large.
pub const fn is_overflow(raw: [i16; 3]) -> bool {
    raw[0] > OVFL_COUNT_LIMIT
        || raw[0] < -OVFL_COUNT_LIMIT
        || raw[1] > OVFL_COUNT_LIMIT
        || raw[1] < -OVFL_COUNT_LIMIT
        || raw[2] > OVFL_COUNT_LIMIT
        || raw[2] < -OVFL_COUNT_LIMIT
}

/// Decode six data bytes into a body-frame sample, or `None` if an axis
/// has saturated.
pub fn decode_sample(buf: [u8; 6], orient: Orientation) -> Option<MagSample> {
    let raw = decode_raw(buf);
    if is_overflow(raw) {
        return None;
    }
    Some(MagSample::new(raw, SENS_UT_PER_LSB, orient.sign()))
}

// ---- Driver ----

#[cfg(feature = "firmware")]
pub use hw::Qmc5883p;

#[cfg(feature = "firmware")]
mod hw {
    use super::*;
    // Only the hardware half returns errors; the decode path cannot fail.
    use super::super::mag::MagError;
    use embassy_stm32::i2c::{Error as I2cError, I2c, Master};
    use embassy_stm32::mode::Blocking;
    use embassy_time::{Duration, Timer};

    fn map_err(_: I2cError) -> MagError {
        MagError::I2c
    }

    pub struct Qmc5883p {
        addr: u8,
        orientation: Orientation,
    }

    impl Qmc5883p {
        /// Read the chip ID register without touching anything else.
        /// Exposed so bring-up can log what actually answered at 0x2C.
        pub fn read_id(i2c: &mut I2c<'_, Blocking, Master>) -> Result<u8, MagError> {
            let mut id = [0u8; 1];
            i2c.blocking_write_read(I2C_ADDR, &[REG_CHIP_ID], &mut id).map_err(map_err)?;
            Ok(id[0])
        }

        /// Verify the chip ID, soft-reset, then configure Normal mode.
        ///
        /// The ID check comes FIRST, before any write. 0x2C is inside the
        /// 0x28-0x2F block that also hosts BNO055 and AK09916 parts (see
        /// `mag::describe_addr`), so a probe that soft-reset first would
        /// be writing 0x80 to register 0x0B of whatever is actually
        /// there. The QMC5883L driver cannot do this -- its chip ID reads
        /// 0xFF and proves nothing -- but this part has a real one, so
        /// the safe order is available and is taken.
        ///
        /// Async because the soft reset and the first conversion both
        /// need real time, and this shares the bus with the baro --
        /// spinning would hold it against a task that wants it.
        pub async fn init(
            i2c: &mut I2c<'_, Blocking, Master>,
            orient: Orientation,
        ) -> Result<Self, MagError> {
            let addr = I2C_ADDR;

            let id = Self::read_id(i2c)?;
            if id != CHIP_ID_VALUE {
                return Err(MagError::IdMismatch(id));
            }

            // Soft reset: this driver cannot assume a cold start. A warm
            // reboot leaves the part in whatever mode the previous
            // firmware left it, and reset is valid from any state.
            //
            // It also satisfies the sequencing rule that Suspend must sit
            // between any two of Normal/Single/Continuous -- reset
            // restores the defaults, and the power-on default mode is
            // Suspend, so the write of CONF1 below is always a
            // Suspend -> Normal transition.
            i2c.blocking_write(addr, &[REG_CONF2, CONF2_SOFT_RST]).map_err(map_err)?;
            Timer::after(Duration::from_millis(10)).await;

            // Order is the datasheet's (section 7.1): axis signs, then
            // range and set/reset, then control register 1 -- writing
            // CONF1 is what leaves Suspend and starts converting, so
            // everything it depends on must already be in place.
            i2c.blocking_write(addr, &[REG_AXIS_SIGN, AXIS_SIGN_VALUE]).map_err(map_err)?;
            i2c.blocking_write(addr, &[REG_CONF2, CONF2_RUN]).map_err(map_err)?;
            i2c.blocking_write(addr, &[REG_CONF1, CONF1_RUN]).map_err(map_err)?;

            // Confirm the part held the configuration. The ID register is
            // read-only, so it proves the address answers but not that
            // writes land; CONF1 is read/write and its post-reset default
            // is 0x00, so reading back the value just written is what
            // distinguishes a configured QMC5883P from a part that
            // ACKed and ignored us.
            let mut back = [0u8; 1];
            i2c.blocking_write_read(addr, &[REG_CONF1], &mut back).map_err(map_err)?;
            if back[0] != CONF1_RUN {
                return Err(MagError::NotResponding);
            }

            // One conversion at 200 Hz is 5 ms; allow for the internal
            // set/reset cycle on top.
            Timer::after(Duration::from_millis(20)).await;

            defmt::info!(
                "QMC5883P init OK @ 0x{=u8:02x} (conf1=0x{=u8:02x}, conf2=0x{=u8:02x}, {=f32} uT/LSB)",
                addr,
                CONF1_RUN,
                CONF2_RUN,
                SENS_UT_PER_LSB,
            );
            Ok(Self { addr, orientation: orient })
        }

        /// Burst-read the six data bytes and return a body-frame sample.
        ///
        /// Must stay a single 6-byte burst: reading the data registers is
        /// what clears DRDY, and splitting it into three 2-byte reads
        /// would let a refresh land between axes and stitch together two
        /// different samples.
        ///
        /// `MagError::Overflow` here means the field exceeded the +/-8 G
        /// range -- the axis is pinned rather than merely noisy, and the
        /// sample must not be fused.
        pub fn read(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<MagSample, MagError> {
            let mut buf = [0u8; 6];
            i2c.blocking_write_read(self.addr, &[REG_X_LSB], &mut buf).map_err(map_err)?;
            decode_sample(buf, self.orientation).ok_or(MagError::Overflow)
        }

        /// Raw status byte: DRDY / OVFL.
        ///
        /// Diagnostic only -- `read` does not consult it, because in
        /// Normal mode the data registers always hold a complete sample
        /// and overflow is visible in the counts themselves. Worth
        /// logging during bring-up: persistent OVFL means the range is
        /// too small for where the part is mounted.
        pub fn status(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<u8, MagError> {
            let mut buf = [0u8; 1];
            i2c.blocking_write_read(self.addr, &[REG_STATUS], &mut buf).map_err(map_err)?;
            Ok(buf[0])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conf1_matches_the_datasheet_worked_example() {
        // Section 7.1: "Write Register 0AH by 0xCD (set normal mode, set
        // ODR=200Hz)", with OSR1 = 8 and OSR2 = 8. If the bit packing is
        // wrong this is the value in the datasheet that catches it.
        assert_eq!(CONF1_RUN, 0xCD, "got 0x{:02X}", CONF1_RUN);
    }

    #[test]
    fn conf2_matches_the_datasheet_worked_example() {
        // Section 7.1/7.2: "Write Register 0BH by 0x08 (Define Set/Reset
        // mode, with Set/Reset On, Field Range 8Guass)".
        assert_eq!(CONF2_RUN, 0x08, "got 0x{:02X}", CONF2_RUN);
        // ...and must not carry the soft-reset or self-test bits into the
        // run config. Self-test in particular injects a bias current and
        // would silently corrupt every heading.
        assert_eq!(CONF2_RUN & 0xC0, 0x00);
    }

    #[test]
    fn conf1_fields_land_in_the_right_bits() {
        // Suspend is mode 00, Osr2::X1 is 00 and Osr1::X8 is 00, so this
        // isolates each field against an all-zero background.
        assert_eq!(conf1(Osr2::X1, Osr1::X8, Odr::Hz10, Mode::Suspend), 0x00);
        assert_eq!(conf1(Osr2::X1, Osr1::X8, Odr::Hz10, Mode::Continuous), 0x03);
        assert_eq!(conf1(Osr2::X1, Osr1::X8, Odr::Hz200, Mode::Suspend), 0x0C);
        assert_eq!(conf1(Osr2::X1, Osr1::X1, Odr::Hz10, Mode::Suspend), 0x30);
        assert_eq!(conf1(Osr2::X8, Osr1::X8, Odr::Hz10, Mode::Suspend), 0xC0);
    }

    #[test]
    fn conf2_fields_land_in_the_right_bits() {
        assert_eq!(conf2(Range::G30, SetReset::SetAndResetOn), 0x00);
        assert_eq!(conf2(Range::G12, SetReset::SetAndResetOn), 0x04);
        assert_eq!(conf2(Range::G8, SetReset::SetAndResetOn), 0x08);
        assert_eq!(conf2(Range::G2, SetReset::SetAndResetOn), 0x0C);
        assert_eq!(conf2(Range::G30, SetReset::SetOnlyOn), 0x01);
        assert_eq!(conf2(Range::G30, SetReset::SetAndResetOff), 0x02);
    }

    #[test]
    fn range_scales_match_the_datasheet() {
        // Section 3: 1000 / 2500 / 3750 / 15000 LSB/G; 1 G = 100 uT.
        assert!((Range::G30.ut_per_lsb() - 0.1).abs() < 1e-6);
        assert!((Range::G12.ut_per_lsb() - 0.04).abs() < 1e-6);
        assert!((Range::G8.ut_per_lsb() - 0.0266667).abs() < 1e-6);
        assert!((Range::G2.ut_per_lsb() - 0.0066667).abs() < 1e-6);
    }

    #[test]
    fn the_p_does_not_share_the_ls_sensitivity() {
        // The two parts differ at their one common range: L = 3000 LSB/G
        // at 8 G, P = 3750. Copying the L's 0.0333 into this driver would
        // read 25% high -- an error a hard-iron calibration largely
        // absorbs, so it would not show up as an obviously broken
        // heading, only as a magnitude that fails the 25-65 uT check.
        let l_8g = 100.0f32 / 3000.0;
        assert!((SENS_UT_PER_LSB - l_8g).abs() > 1e-3);
    }

    #[test]
    fn full_scale_reaches_the_nominal_range() {
        // The part clips at +/-30000 counts, not +/-32768. At 8 G that is
        // 30000 * 0.026667 = 800 uT = 8 G exactly, which is the check
        // that the count limit and the sensitivity agree.
        let fs = OVFL_COUNT_LIMIT as f32 * Range::G8.ut_per_lsb();
        assert!((fs - 800.0).abs() < 0.5, "fs={fs}");
    }

    #[test]
    fn decode_is_little_endian_twos_complement() {
        // X = 0x0001, Y = 0xFFFF (-1), Z = 0x8000 (-32768).
        let raw = decode_raw([0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80]);
        assert_eq!(raw, [1, -1, -32768]);
    }

    #[test]
    fn saturated_samples_are_rejected_not_clamped() {
        // 30000 is the last good count; anything beyond it is a reading
        // the part itself flags as overflowed.
        let ok = [0x30, 0x75, 0, 0, 0, 0]; // 0x7530 = 30000
        assert_eq!(decode_raw(ok)[0], 30000);
        assert!(decode_sample(ok, Orientation::Identity).is_some());

        let over = [0x31, 0x75, 0, 0, 0, 0]; // 30001
        assert!(decode_sample(over, Orientation::Identity).is_none());

        // ...and symmetrically on the negative side. 0x8ACF = -30001.
        let under = [0xCF, 0x8A, 0, 0, 0, 0];
        assert_eq!(decode_raw(under)[0], -30001);
        assert!(decode_sample(under, Orientation::Identity).is_none());

        // Overflow on any axis condemns the whole sample, not just that
        // axis: the estimator consumes the vector.
        let over_z = [0, 0, 0, 0, 0x31, 0x75];
        assert!(decode_sample(over_z, Orientation::Identity).is_none());
    }

    #[test]
    fn a_plausible_earth_field_decodes_to_a_plausible_magnitude() {
        // 50 uT along X at 8 G is 50 / 0.026667 = 1875 counts. This is
        // the check that would catch a gauss/tesla mix-up or the wrong
        // range's sensitivity, which is the failure this driver is most
        // likely to have.
        let s = decode_sample([0x53, 0x07, 0, 0, 0, 0], Orientation::Identity).unwrap();
        assert_eq!(s.raw[0], 1875);
        let m = s.magnitude_ut();
        assert!((25.0..65.0).contains(&m), "magnitude {m} uT is not an Earth field");
    }

    #[test]
    fn orientation_reaches_the_decoded_sample() {
        let buf = [0x53, 0x07, 0, 0, 0, 0];
        let a = decode_sample(buf, Orientation::Identity).unwrap().ut();
        let b = decode_sample(buf, Orientation::Yaw180).unwrap().ut();
        assert!((a[0] + b[0]).abs() < 1e-6, "Yaw180 must negate X");
    }

    #[test]
    fn status_bits_are_distinct() {
        assert_eq!(STATUS_DRDY | STATUS_OVFL, 0b11);
    }

    #[test]
    fn the_address_is_not_one_of_the_other_mags() {
        // 0x2C must not collide with the parts the bus scan already
        // dispatches on, or `probe` would need tie-breaking here too.
        use super::super::mag::{IST8310_ADDRS, LIS2MDL_OR_HMC5883L_ADDR, QMC5883L_ADDR};
        assert_ne!(I2C_ADDR, QMC5883L_ADDR);
        assert_ne!(I2C_ADDR, LIS2MDL_OR_HMC5883L_ADDR);
        assert!(!IST8310_ADDRS.contains(&I2C_ADDR));
    }
}
