// ist8310.rs — iSentek IST8310 3-axis magnetometer over I2C.
//
// The compass inside the Radiolink SE100 V2 GPS module; the 2026-09-08
// bench scan found it at 0x0E. Register facts below are from
// docs/ist8310-datasheet.pdf (iSentek, v1.2); section numbers refer to
// that document. Cross-checked against the ArduPilot, PX4, Betaflight and
// tstellanova/ist8310 drivers, which agree on everything except the axis
// handedness -- see `decode_raw`.
//
// What makes this part different from the other three compasses here:
//
//   * It has NO continuous mode (section 6.4.6: modes are stand-by and
//     single measurement, "others: reserved"). Every sample has to be
//     asked for, and the part drops back to stand-by when it is done. So
//     `read` is a two-step state machine -- collect the last result,
//     trigger the next -- rather than a burst read of a running
//     converter, and it can legitimately return "nothing new yet".
//
//   * Its address is pin-selectable across 0x0C..=0x0F (section 6.1.1),
//     and 0x0D collides with the QMC5883L. The probe tries both parts
//     there; this one has a real WHO_AM_I, the QMC does not.
//
//   * Its native output frame is left-handed (+x forward, +y right, +z
//     UP), which no `Orientation` can fix because those are all proper
//     rotations. The Z flip that makes it right-handed lives in
//     `decode_raw`, before Orientation is applied.
//
// The parsing and configuration logic here is host-testable; only the
// I2C transactions are gated behind the `firmware` feature.

#![allow(dead_code)]

use super::mag::{MagSample, Orientation};

// ---- I2C address ----

/// The address with CAD1/CAD0 floating, which is the default and what
/// the SE100 V2 uses (measured). Section 6.1.1, Table: VSS/VSS 0x0C,
/// VSS/VDD 0x0D, VDD/VSS 0x0E, VDD/VDD 0x0F.
pub const I2C_ADDR_DEFAULT: u8 = 0x0E;
pub const I2C_ADDRS: [u8; 4] = [0x0C, 0x0D, 0x0E, 0x0F];

// ---- Register map (section 6.4.1) ----

const REG_WAI: u8 = 0x00;
const REG_STAT1: u8 = 0x02;
/// X low byte; X, Y, Z each little-endian from here through 0x08.
const REG_DATA_XL: u8 = 0x03;
const REG_STAT2: u8 = 0x09;
const REG_CNTL1: u8 = 0x0A;
const REG_CNTL2: u8 = 0x0B;
const REG_STR: u8 = 0x0C;
const REG_TEMP_L: u8 = 0x1C;
const REG_AVGCNTL: u8 = 0x41;
const REG_PDCNTL: u8 = 0x42;

/// Section 6.4.2: "Device ID ... Default 10".
pub const WAI_VALUE: u8 = 0x10;

// ---- STAT1 (section 6.4.3) ----

/// Data ready. Set when a single measurement completes; cleared by
/// reading any output data register.
pub const STAT1_DRDY: u8 = 1 << 0;
/// Data overrun: a result was overwritten before it was read.
pub const STAT1_DOR: u8 = 1 << 1;

// ---- CNTL1 (section 6.4.6) ----

pub const CNTL1_STANDBY: u8 = 0x00;
/// "0001: Single Measurement Mode". Writing this starts one conversion;
/// the part clears it back to 0000 when done.
pub const CNTL1_SINGLE: u8 = 0x01;

// ---- CNTL2 (section 6.4.7) ----

/// "SRST: Soft reset, perform Power On Reset (POR) routine ... This bit
/// will be set to zero after POR routine".
pub const CNTL2_SRST: u8 = 1 << 0;
pub const CNTL2_DRP_ACTIVE_HIGH: u8 = 1 << 2;
pub const CNTL2_DREN: u8 = 1 << 3;

// ---- STR (section 6.4.8) ----

pub const STR_SELF_TEST: u8 = 1 << 6;

// ---- AVGCNTL (section 6.4.10) ----
//
//   bits 5:3  Y averaging       000 none, 001 x2, 010 x4 (default),
//   bits 2:0  X and Z averaging 011 x8, 100 x16

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Average {
    X1 = 0b000,
    X2 = 0b001,
    X4 = 0b010,
    X8 = 0b011,
    X16 = 0b100,
}

pub const fn avgcntl(y: Average, xz: Average) -> u8 {
    ((y as u8) << 3) | (xz as u8)
}

// ---- PDCNTL (section 6.4.11) ----

/// "2'b11 Normal (please use this setting)" in bits 7:6. Section 3.1.1
/// calls it an initial setting "for performance optimization".
pub const PDCNTL_NORMAL: u8 = 0b11 << 6;

// ---- Chosen configuration ----

/// Section 3.1.1: "For low noise performance, please set Average Control
/// Register, AVGCNTL(0x41) = 00100100b (24H)". 16x on every axis. This
/// raises the minimum gap between measurements from 5 ms to 6 ms
/// (166 Hz), which is still faster than the 125 Hz poll.
pub const AVGCNTL_RUN: u8 = avgcntl(Average::X16, Average::X16);
pub const PDCNTL_RUN: u8 = PDCNTL_NORMAL;

/// Section 4.2: "Resolution 0.3 uT/LSB", "Sensitivity 3.3 LSB/uT". Fixed;
/// this part has no range setting. Full scale is +/-1600 uT on X/Y and
/// +/-2500 uT on Z, so an Earth field of 50 uT is ~165 counts. Coarse
/// next to the QMC5883L's 1500, and the reason for the 16x averaging.
pub const SENS_UT_PER_LSB: f32 = 0.3;

/// Minimum time between triggering a measurement and its result, with
/// 16x averaging (section 3.1.1).
pub const CONVERSION_MS: u64 = 6;

// ---- Sample decoding ----

/// Decode the six data bytes at 0x03..=0x08 into raw counts in X, Y, Z,
/// as a RIGHT-HANDED frame.
///
/// Wire order is X, Y, Z, each little-endian (section 6.4.4), which is
/// the easy part. The hard part: the sensor's own axes are +x forward,
/// +y right, +z up, a left-handed set. Every flight stack corrects this
/// with one sign flip, but not the same one -- PX4 and ArduPilot negate
/// Z, Betaflight negates Y ("datasheet is incorrect"). Both choices give
/// a right-handed frame; they differ by a 180-degree roll, which is a
/// mounting question and belongs in `Orientation`. Z is flipped here,
/// following the two-of-three majority and PX4's explicit frame comment.
///
/// Why this cannot live in `Orientation`: every variant there has
/// determinant +1 (tested in mag.rs). A left-handed input needs a
/// determinant -1 correction, so without this flip no Orientation could
/// ever make yaw come out right -- it would be mirrored, which is much
/// harder to spot on the bench than inverted.
pub const fn decode_raw(buf: [u8; 6]) -> [i16; 3] {
    let x = i16::from_le_bytes([buf[0], buf[1]]);
    let y = i16::from_le_bytes([buf[2], buf[3]]);
    let z = i16::from_le_bytes([buf[4], buf[5]]);
    // -(-32768) does not fit; saturate rather than wrap to -32768 again.
    let z = if z == i16::MIN { i16::MAX } else { -z };
    [x, y, z]
}

/// Decode six data bytes into a body-frame sample.
pub fn decode_sample(buf: [u8; 6], orient: Orientation) -> MagSample {
    MagSample::new(decode_raw(buf), SENS_UT_PER_LSB, orient.sign())
}

// ---- Driver ----

#[cfg(feature = "firmware")]
pub use hw::Ist8310;

#[cfg(feature = "firmware")]
mod hw {
    use super::*;
    use super::super::mag::MagError;
    use embassy_stm32::i2c::{Error as I2cError, I2c, Master};
    use embassy_stm32::mode::Blocking;
    use embassy_time::{Duration, Timer};

    fn map_err(_: I2cError) -> MagError {
        MagError::I2c
    }

    pub struct Ist8310 {
        addr: u8,
        orientation: Orientation,
    }

    impl Ist8310 {
        /// Read WHO_AM_I at `addr`. Used by the probe to settle the 0x0D
        /// collision with the QMC5883L before either driver writes.
        pub fn read_id(i2c: &mut I2c<'_, Blocking, Master>, addr: u8) -> Result<u8, MagError> {
            let mut id = [0u8; 1];
            i2c.blocking_write_read(addr, &[REG_WAI], &mut id).map_err(map_err)?;
            Ok(id[0])
        }

        /// Soft reset, verify WHO_AM_I, apply the datasheet's recommended
        /// initial settings, and trigger the first measurement.
        pub async fn init(
            i2c: &mut I2c<'_, Blocking, Master>,
            addr: u8,
            orient: Orientation,
        ) -> Result<Self, MagError> {
            // Soft reset first: no cold-start assumption. The register
            // defaults are restored, including CNTL1 = stand-by.
            i2c.blocking_write(addr, &[REG_CNTL2, CNTL2_SRST]).map_err(map_err)?;
            // The datasheet gives no POR duration; PX4 allows 50 ms.
            Timer::after(Duration::from_millis(50)).await;

            let id = Self::read_id(i2c, addr)?;
            if id != WAI_VALUE {
                return Err(MagError::IdMismatch(id));
            }

            i2c.blocking_write(addr, &[REG_AVGCNTL, AVGCNTL_RUN]).map_err(map_err)?;
            i2c.blocking_write(addr, &[REG_PDCNTL, PDCNTL_RUN]).map_err(map_err)?;

            // Read both back. Cheap, and it turns "the writes were ACKed"
            // into "the part holds the config we think it does".
            let mut back = [0u8; 1];
            i2c.blocking_write_read(addr, &[REG_AVGCNTL], &mut back).map_err(map_err)?;
            if back[0] != AVGCNTL_RUN {
                return Err(MagError::NotResponding);
            }
            i2c.blocking_write_read(addr, &[REG_PDCNTL], &mut back).map_err(map_err)?;
            if back[0] != PDCNTL_RUN {
                return Err(MagError::NotResponding);
            }

            let d = Self { addr, orientation: orient };
            d.trigger(i2c)?;
            // So the caller's first `read` finds a result waiting.
            Timer::after(Duration::from_millis(CONVERSION_MS + 4)).await;

            defmt::info!(
                "IST8310 init OK @ 0x{=u8:02x} (avg=0x{=u8:02x} pd=0x{=u8:02x}, {=f32} uT/LSB, single-shot)",
                addr,
                AVGCNTL_RUN,
                PDCNTL_RUN,
                SENS_UT_PER_LSB,
            );
            Ok(d)
        }

        /// Start one measurement. The part returns to stand-by on its own
        /// when the result is in the data registers.
        pub fn trigger(&self, i2c: &mut I2c<'_, Blocking, Master>) -> Result<(), MagError> {
            i2c.blocking_write(self.addr, &[REG_CNTL1, CNTL1_SINGLE]).map_err(map_err)
        }

        /// Collect the previous measurement if it is ready, then trigger
        /// the next one. `Ok(None)` means the conversion has not finished
        /// yet, which is not an error; at a 125 Hz poll with a 6 ms
        /// conversion it should be rare.
        ///
        /// One 7-byte burst from STAT1 through DATAZH, so the DRDY bit
        /// and the data it describes come from the same transaction
        /// (PX4 does the same). Reading the data registers clears DRDY.
        pub fn read(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<Option<MagSample>, MagError> {
            let mut buf = [0u8; 7];
            i2c.blocking_write_read(self.addr, &[REG_STAT1], &mut buf).map_err(map_err)?;
            if buf[0] & STAT1_DRDY == 0 {
                // Still converting; do not re-trigger, that would restart
                // it and we would never see a result.
                return Ok(None);
            }
            let data = [buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]];
            self.trigger(i2c)?;
            Ok(Some(decode_sample(data, self.orientation)))
        }

        /// Raw STAT1: DRDY / DOR.
        pub fn status(&self, i2c: &mut I2c<'_, Blocking, Master>) -> Result<u8, MagError> {
            let mut buf = [0u8; 1];
            i2c.blocking_write_read(self.addr, &[REG_STAT1], &mut buf).map_err(map_err)?;
            Ok(buf[0])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn avgcntl_matches_the_datasheet_recommendation() {
        // Section 3.1.1: "AVGCNTL(0x41) = 00100100b (24H)".
        assert_eq!(AVGCNTL_RUN, 0x24, "got 0x{:02X}", AVGCNTL_RUN);
        // And the register default, x4 on both: 0b010_010.
        assert_eq!(avgcntl(Average::X4, Average::X4), 0x12);
    }

    #[test]
    fn pdcntl_matches_the_datasheet_recommendation() {
        // Section 3.1.1: "PDCTNL(0x42) = 11000000b (C0H)".
        assert_eq!(PDCNTL_RUN, 0xC0);
    }

    #[test]
    fn decode_is_little_endian_x_y_z_with_z_flipped() {
        // Wire: X=1, Y=2, Z=3 little-endian. Right-handed output negates Z.
        let raw = decode_raw([0x01, 0x00, 0x02, 0x00, 0x03, 0x00]);
        assert_eq!(raw, [1, 2, -3]);
        // Negative X: 0xFFFF = -1, untouched.
        assert_eq!(decode_raw([0xFF, 0xFF, 0, 0, 0, 0])[0], -1);
    }

    #[test]
    fn z_flip_saturates_at_the_i16_edge() {
        // Z = -32768 on the wire; -(-32768) must not wrap back to -32768.
        let raw = decode_raw([0, 0, 0, 0, 0x00, 0x80]);
        assert_eq!(raw[2], i16::MAX);
    }

    #[test]
    fn handedness_fix_has_determinant_minus_one() {
        // The whole point of the flip: the sensor frame is left-handed,
        // and Orientation (all det +1) cannot fix that. Check the decode
        // acts as diag(1, 1, -1) on the sign of each axis.
        let e = |i: usize| {
            let mut b = [0u8; 6];
            b[2 * i] = 0x01;
            decode_raw(b)
        };
        let det = e(0)[0] as i32 * e(1)[1] as i32 * e(2)[2] as i32;
        assert_eq!(det, -1);
    }

    #[test]
    fn a_plausible_earth_field_decodes_to_a_plausible_magnitude() {
        // 50 uT along X at 0.3 uT/LSB = 167 counts = 0x00A7.
        let s = decode_sample([0xA7, 0x00, 0, 0, 0, 0], Orientation::Identity);
        assert_eq!(s.raw[0], 167);
        let m = s.magnitude_ut();
        assert!((25.0..65.0).contains(&m), "magnitude {m} uT is not an Earth field");
    }

    #[test]
    fn full_scale_fits_in_i16() {
        // +/-2500 uT on Z at 0.3 uT/LSB = 8333 counts, well inside i16,
        // so there is no overflow sentinel to worry about on this part.
        assert!(2500.0 / SENS_UT_PER_LSB < 32767.0);
    }

    #[test]
    fn addresses_cover_every_cad_combination() {
        assert_eq!(I2C_ADDRS, [0x0C, 0x0D, 0x0E, 0x0F]);
        assert!(I2C_ADDRS.contains(&I2C_ADDR_DEFAULT));
    }
}
