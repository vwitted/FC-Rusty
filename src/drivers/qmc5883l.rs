// qmc5883l.rs — QST QMC5883L 3-axis magnetometer over I2C.
//
// The compass inside the Radiolink SE100 GPS module. Register map and
// values below are from docs/qmc5883l-datasheet.pdf; section numbers
// refer to that document.
//
// Configured for continuous mode at 200 Hz, +/-8 G, OSR 512 -- which is
// the datasheet's own worked example (section 7.1, "write 09H = 0x1D").
//
// Range choice: Earth's field is 0.25-0.65 G, so +/-2 G would fit it
// with 4x the resolution (12000 vs 3000 LSB/G). +/-8 G is still chosen,
// because the field this part actually sees is Earth's PLUS whatever the
// power wiring is doing a few centimetres away, and a saturated axis
// (status OVL) is an unrecoverable reading whereas a coarser one is
// merely noisier. At 8 G one count is 0.033 uT against an Earth field of
// ~50 uT, so there are still ~1500 counts of signal.
//
// The parsing and configuration logic here is host-testable; only the
// I2C transactions are gated behind the `firmware` feature.

#![allow(dead_code)]

use super::mag::{MagError, MagSample, Orientation};

// ---- I2C address ----

/// Fixed 7-bit slave address (section 5.4: "The default I2C address is
/// 0D: 0001101"). No address pin.
///
/// Note this collides numerically with `REG_CHIP_ID`, which is also
/// 0x0D. They are unrelated; the coincidence has burned people reading
/// this driver quickly.
pub const I2C_ADDR: u8 = 0x0D;

// ---- Register map (section 9.1, Table 13) ----

const REG_X_LSB: u8 = 0x00;
const REG_STATUS: u8 = 0x06;
const REG_TEMP_LSB: u8 = 0x07;
const REG_CTRL1: u8 = 0x09;
const REG_CTRL2: u8 = 0x0A;
const REG_SET_RESET: u8 = 0x0B;
const REG_CHIP_ID: u8 = 0x0D;

/// Section 9.2.6: the chip ID register returns 0xFF.
///
/// Which is useless as a presence test on its own -- an absent slave, a
/// bus held high by its pull-ups, and a healthy QMC5883L all read 0xFF.
/// `probe` therefore verifies by WRITING and reading back instead. Kept
/// here because reading it is still a cheap first filter and it is what
/// the datasheet documents.
pub const CHIP_ID_VALUE: u8 = 0xFF;

// ---- Status register (section 9.2.2, Table 15) ----

/// New data in all three axes. Cleared by reading any data register.
pub const STATUS_DRDY: u8 = 1 << 0;
/// Some axis saturated at -32768/32767. The reading is not usable.
pub const STATUS_OVL: u8 = 1 << 1;
/// A whole sample was skipped because it was not read in time.
pub const STATUS_DOR: u8 = 1 << 2;

// ---- Control register 1 (section 9.2.4, Table 18) ----
//
//   bits 1:0  MODE  00 standby, 01 continuous
//   bits 3:2  ODR   00 10 Hz, 01 50 Hz, 10 100 Hz, 11 200 Hz
//   bits 5:4  RNG   00 +/-2 G, 01 +/-8 G
//   bits 7:6  OSR   00 512, 01 256, 10 128, 11 64
//
// Note the OSR encoding runs BACKWARDS relative to how it reads: 00 is
// the largest oversampling ratio (512, narrowest filter), not the
// smallest.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Standby = 0b00,
    Continuous = 0b01,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Odr {
    Hz10 = 0b00,
    Hz50 = 0b01,
    Hz100 = 0b10,
    Hz200 = 0b11,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Range {
    G2 = 0b00,
    G8 = 0b01,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Osr {
    X512 = 0b00,
    X256 = 0b01,
    X128 = 0b10,
    X64 = 0b11,
}

impl Range {
    /// Microtesla per count.
    ///
    /// Section 3, Table 2: 12000 LSB/G at +/-2 G, 3000 LSB/G at +/-8 G.
    /// 1 gauss = 100 uT, so uT/LSB = 100 / (LSB/G).
    pub const fn ut_per_lsb(self) -> f32 {
        match self {
            Self::G2 => 100.0 / 12000.0,
            Self::G8 => 100.0 / 3000.0,
        }
    }
}

/// Assemble control register 1.
pub const fn ctrl1(osr: Osr, rng: Range, odr: Odr, mode: Mode) -> u8 {
    ((osr as u8) << 6) | ((rng as u8) << 4) | ((odr as u8) << 2) | (mode as u8)
}

// ---- Control register 2 (section 9.2.4, Table 19) ----

/// Soft reset: restores every register to its default and drops the part
/// back to standby. Section 7.4.
const CTRL2_SOFT_RST: u8 = 1 << 7;

/// Pointer roll-over. With this set the I2C read pointer wraps within
/// 0x00..=0x06, which is what makes a single 7-byte burst of "six data
/// bytes plus status" safe.
const CTRL2_ROL_PNT: u8 = 1 << 6;

/// Interrupt-pin enable, and the sense is INVERTED relative to its name:
/// section 9.2.4 states "INT_ENB: '0': enable interrupt PIN, '1':
/// disable interrupt PIN". We poll and the pin is not wired, so this
/// bit is SET to switch the pin off.
const CTRL2_INT_DISABLE: u8 = 1 << 0;

/// Running configuration for control register 2.
pub const CTRL2_RUN: u8 = CTRL2_ROL_PNT | CTRL2_INT_DISABLE;

// ---- SET/RESET period (section 9.2.5) ----

/// "It is recommended that the register 0BH is written by 0x01." The
/// datasheet gives no meaning for other values, so this is taken as
/// given rather than tuned.
pub const SET_RESET_PERIOD: u8 = 0x01;

// ---- Chosen configuration ----

pub const CFG_OSR: Osr = Osr::X512;
pub const CFG_RANGE: Range = Range::G8;
pub const CFG_ODR: Odr = Odr::Hz200;

/// Control register 1 as written at init. Equals the datasheet's own
/// example value, 0x1D (section 7.1).
pub const CTRL1_RUN: u8 = ctrl1(CFG_OSR, CFG_RANGE, CFG_ODR, Mode::Continuous);

/// Microtesla per count in the configured range.
pub const SENS_UT_PER_LSB: f32 = CFG_RANGE.ut_per_lsb();

// ---- Sample decoding ----

/// Decode the six data bytes into raw counts.
///
/// Each axis is 16-bit two's complement, LSB first (section 9.2.1), and
/// the axes are in X, Y, Z order starting at 0x00.
pub const fn decode_raw(buf: [u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([buf[0], buf[1]]),
        i16::from_le_bytes([buf[2], buf[3]]),
        i16::from_le_bytes([buf[4], buf[5]]),
    ]
}

/// Decode six data bytes into a body-frame sample.
pub fn decode_sample(buf: [u8; 6], orient: Orientation) -> MagSample {
    MagSample::new(decode_raw(buf), SENS_UT_PER_LSB, orient.sign())
}

// ---- Driver ----

#[cfg(feature = "firmware")]
pub use hw::Qmc5883l;

#[cfg(feature = "firmware")]
mod hw {
    use super::*;
    use embassy_stm32::i2c::{Error as I2cError, I2c, Master};
    use embassy_stm32::mode::Blocking;
    use embassy_time::{Duration, Timer};

    fn map_err(_: I2cError) -> MagError {
        MagError::I2c
    }

    pub struct Qmc5883l {
        addr: u8,
        orientation: Orientation,
    }

    impl Qmc5883l {
        /// Soft-reset, verify something is actually there, then configure
        /// continuous mode.
        ///
        /// Async because the soft reset and the first conversion both
        /// need real time, and this shares I2C1 with the baro -- spinning
        /// would hold the bus against a task that wants it.
        pub async fn init(
            i2c: &mut I2c<'_, Blocking, Master>,
            orient: Orientation,
        ) -> Result<Self, MagError> {
            let addr = I2C_ADDR;

            // Soft reset first: this driver cannot assume a cold start.
            // A warm reboot leaves the part in whatever mode the previous
            // firmware left it, and section 9.2.4 says reset is valid
            // from any state.
            i2c.blocking_write(addr, &[REG_CTRL2, CTRL2_SOFT_RST]).map_err(map_err)?;
            Timer::after(Duration::from_millis(10)).await;

            // Presence test by readback, NOT by chip ID -- see
            // CHIP_ID_VALUE. 0x0B is read/write and the reset default is
            // not 0x01, so writing the value we want anyway and reading
            // it back distinguishes a real part from an idle bus that
            // NACKs nothing and returns 0xFF to everything.
            i2c.blocking_write(addr, &[REG_SET_RESET, SET_RESET_PERIOD]).map_err(map_err)?;
            let mut back = [0u8; 1];
            i2c.blocking_write_read(addr, &[REG_SET_RESET], &mut back).map_err(map_err)?;
            if back[0] != SET_RESET_PERIOD {
                return Err(MagError::NotResponding);
            }

            i2c.blocking_write(addr, &[REG_CTRL2, CTRL2_RUN]).map_err(map_err)?;
            // Order matters: section 7.1 sets the SET/RESET period before
            // control register 1, because writing CTRL1 is what starts
            // continuous conversion.
            i2c.blocking_write(addr, &[REG_CTRL1, CTRL1_RUN]).map_err(map_err)?;

            // One conversion at 200 Hz is 5 ms; allow for the internal
            // set/reset cycle on top.
            Timer::after(Duration::from_millis(20)).await;

            defmt::info!(
                "QMC5883L init OK @ 0x{=u8:02x} (ctrl1=0x{=u8:02x}, {=f32} uT/LSB)",
                addr,
                CTRL1_RUN,
                SENS_UT_PER_LSB,
            );
            Ok(Self { addr, orientation: orient })
        }

        /// Burst-read the six data bytes and return a body-frame sample.
        ///
        /// Reading any data register clears DRDY and starts the chip's
        /// data-protection window, which holds the registers stable until
        /// 0x05 has been read -- so this must stay a single 6-byte burst
        /// rather than three 2-byte reads.
        pub fn read(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<MagSample, MagError> {
            let mut buf = [0u8; 6];
            i2c.blocking_write_read(self.addr, &[REG_X_LSB], &mut buf).map_err(map_err)?;
            Ok(decode_sample(buf, self.orientation))
        }

        /// Raw status byte: DRDY / OVL / DOR.
        ///
        /// Worth logging during bring-up. OVL in particular means the
        /// range is too small for where the part is mounted, and the
        /// affected axis is pinned rather than merely noisy.
        pub fn status(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<u8, MagError> {
            let mut buf = [0u8; 1];
            i2c.blocking_write_read(self.addr, &[REG_STATUS], &mut buf).map_err(map_err)?;
            Ok(buf[0])
        }

        /// Relative die temperature, degrees C.
        ///
        /// Section 9.2.3: 100 LSB/C, gain calibrated but OFFSET IS NOT,
        /// so only differences are meaningful. Diagnostic only -- the
        /// part applies its own temperature compensation to the field.
        pub fn read_temp_c_relative(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<f32, MagError> {
            let mut buf = [0u8; 2];
            i2c.blocking_write_read(self.addr, &[REG_TEMP_LSB], &mut buf).map_err(map_err)?;
            Ok(i16::from_le_bytes([buf[0], buf[1]]) as f32 / 100.0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ctrl1_matches_the_datasheet_worked_example() {
        // Section 7.1: "Write Register 09H by 0x1D (Define OSR = 512,
        // Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous
        // measurement mode)". If the bit packing is wrong this is the
        // one value in the datasheet that catches it.
        assert_eq!(CTRL1_RUN, 0x1D, "got 0x{:02X}", CTRL1_RUN);
    }

    #[test]
    fn ctrl1_fields_land_in_the_right_bits() {
        // Standby is mode 00, so this isolates each field.
        assert_eq!(ctrl1(Osr::X512, Range::G2, Odr::Hz10, Mode::Standby), 0x00);
        assert_eq!(ctrl1(Osr::X512, Range::G2, Odr::Hz10, Mode::Continuous), 0x01);
        assert_eq!(ctrl1(Osr::X512, Range::G2, Odr::Hz200, Mode::Standby), 0x0C);
        assert_eq!(ctrl1(Osr::X512, Range::G8, Odr::Hz10, Mode::Standby), 0x10);
        assert_eq!(ctrl1(Osr::X64, Range::G2, Odr::Hz10, Mode::Standby), 0xC0);
    }

    #[test]
    fn ctrl2_disables_the_interrupt_pin_by_setting_the_bit() {
        // The inverted sense is the trap: INT_ENB = 1 DISABLES the pin.
        // Clearing it would leave an unwired pin driving.
        assert_eq!(CTRL2_RUN & 0x01, 0x01);
        // ...and must not carry the soft-reset bit into the run config.
        assert_eq!(CTRL2_RUN & 0x80, 0x00);
    }

    #[test]
    fn range_scales_match_the_datasheet() {
        // Table 2: 12000 LSB/G at 2 G, 3000 LSB/G at 8 G; 1 G = 100 uT.
        assert!((Range::G2.ut_per_lsb() - 0.008333).abs() < 1e-6);
        assert!((Range::G8.ut_per_lsb() - 0.033333).abs() < 1e-6);
        // Full scale must actually reach the nominal range: 8 G = 800 uT.
        let fs = 32768.0 * Range::G8.ut_per_lsb();
        assert!((fs - 1092.0).abs() < 2.0, "fs={fs}");
    }

    #[test]
    fn decode_is_little_endian_twos_complement() {
        // X = 0x0001, Y = 0xFFFF (-1), Z = 0x8000 (-32768, the negative
        // saturation point).
        let raw = decode_raw([0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80]);
        assert_eq!(raw, [1, -1, -32768]);
    }

    #[test]
    fn a_plausible_earth_field_decodes_to_a_plausible_magnitude() {
        // 50 uT along X at 8 G is 50 / 0.03333 = 1500 counts. This is the
        // check that would catch a gauss/tesla or 2G/8G scale mix-up,
        // which is the failure this driver is most likely to have.
        let s = decode_sample([0xDC, 0x05, 0, 0, 0, 0], Orientation::Identity);
        assert_eq!(s.raw[0], 1500);
        let m = s.magnitude_ut();
        assert!((25.0..65.0).contains(&m), "magnitude {m} uT is not an Earth field");
    }

    #[test]
    fn orientation_reaches_the_decoded_sample() {
        let buf = [0xDC, 0x05, 0, 0, 0, 0];
        let a = decode_sample(buf, Orientation::Identity).ut();
        let b = decode_sample(buf, Orientation::Yaw180).ut();
        assert!((a[0] + b[0]).abs() < 1e-6, "Yaw180 must negate X");
    }

    #[test]
    fn status_bits_are_distinct() {
        assert_eq!(STATUS_DRDY | STATUS_OVL | STATUS_DOR, 0b111);
    }
}
