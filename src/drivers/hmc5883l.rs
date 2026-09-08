// hmc5883l.rs — Honeywell HMC5883L 3-axis magnetometer over I2C.
//
// The OTHER compass a Radiolink SE100 may carry. Early SE100s shipped the
// Honeywell part; later ones the QST QMC5883L, and the module is
// externally identical either way. The two chips share a name and a
// footprint and nothing else that matters here:
//
//                     HMC5883L            QMC5883L
//   I2C address       0x1E                0x0D
//   identity          0x0A..0x0C = "H43"  0x0D = 0xFF (useless)
//   data start        0x03                0x00
//   axis order        X, Z, Y             X, Y, Z
//   byte order        big-endian          little-endian
//   max rate          75 Hz               200 Hz
//   overflow          -4096 in the axis   status OVL bit
//
// Getting any of those wrong does not fail loudly: the wrong axis order
// still yields a plausible |B|, so this driver is matched to the chip by
// its ID string and never by assumption. Register facts below are from
// docs/hmc5883l-datasheet.pdf (Honeywell, rev E); table numbers refer to
// that document.
//
// The parsing and configuration logic here is host-testable; only the
// I2C transactions are gated behind the `firmware` feature.

#![allow(dead_code)]

use super::mag::{MagSample, Orientation};

// ---- I2C address ----

/// Fixed 7-bit slave address: "the 7-bit address (0x1E) plus 1 bit
/// read/write identifier, i.e. 0x3D for read and 0x3C for write". No
/// address pin.
///
/// This is the SAME address as the LIS2MDL, so an ACK at 0x1E alone does
/// not say which part is there. The probe reads identity registers to
/// tell them apart; see `Compass::probe` in main.rs.
pub const I2C_ADDR: u8 = 0x1E;

// ---- Register map (Table 2) ----

const REG_CFG_A: u8 = 0x00;
const REG_CFG_B: u8 = 0x01;
const REG_MODE: u8 = 0x02;
/// X MSB. The six data registers run X, Z, Y from here.
const REG_DATA: u8 = 0x03;
const REG_STATUS: u8 = 0x09;
/// Identification registers A, B, C: 0x0A, 0x0B, 0x0C.
pub const REG_ID_A: u8 = 0x0A;

/// Tables 18-20: identification registers A, B, C read the ASCII
/// characters 'H', '4', '3'. Read-only, and the one solid presence test
/// this part offers.
pub const ID_VALUE: [u8; 3] = *b"H43";

// ---- Status register (Table 16) ----

pub const STATUS_RDY: u8 = 1 << 0;
pub const STATUS_LOCK: u8 = 1 << 1;

// ---- Configuration register A (Tables 3-6) ----
//
//   bit 7     reserved, write 0
//   bits 6:5  MA   samples averaged: 00 = 1, 01 = 2, 10 = 4, 11 = 8
//   bits 4:2  DO   output rate: 100 = 15 Hz (default) ... 110 = 75 Hz
//   bits 1:0  MS   00 normal, 01 positive bias, 10 negative bias
//
// Default 0x10.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Average {
    X1 = 0b00,
    X2 = 0b01,
    X4 = 0b10,
    X8 = 0b11,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rate {
    Hz0_75 = 0b000,
    Hz1_5 = 0b001,
    Hz3 = 0b010,
    Hz7_5 = 0b011,
    Hz15 = 0b100,
    Hz30 = 0b101,
    Hz75 = 0b110,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bias {
    Normal = 0b00,
    Positive = 0b01,
    Negative = 0b10,
}

pub const fn cfg_a(avg: Average, rate: Rate, bias: Bias) -> u8 {
    ((avg as u8) << 5) | ((rate as u8) << 2) | (bias as u8)
}

// ---- Configuration register B (Tables 7-9) ----
//
//   bits 7:5  GN   gain; bits 4:0 must be 0.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gain {
    /// +/-0.88 Ga, 1370 LSB/Ga
    Ga0_88 = 0b000,
    /// +/-1.3 Ga, 1090 LSB/Ga (power-on default)
    Ga1_3 = 0b001,
    /// +/-1.9 Ga, 820 LSB/Ga
    Ga1_9 = 0b010,
    /// +/-2.5 Ga, 660 LSB/Ga
    Ga2_5 = 0b011,
    /// +/-4.0 Ga, 440 LSB/Ga
    Ga4_0 = 0b100,
    /// +/-4.7 Ga, 390 LSB/Ga
    Ga4_7 = 0b101,
    /// +/-5.6 Ga, 330 LSB/Ga
    Ga5_6 = 0b110,
    /// +/-8.1 Ga, 230 LSB/Ga
    Ga8_1 = 0b111,
}

impl Gain {
    /// Table 9, "Gain" column.
    pub const fn lsb_per_gauss(self) -> f32 {
        match self {
            Self::Ga0_88 => 1370.0,
            Self::Ga1_3 => 1090.0,
            Self::Ga1_9 => 820.0,
            Self::Ga2_5 => 660.0,
            Self::Ga4_0 => 440.0,
            Self::Ga4_7 => 390.0,
            Self::Ga5_6 => 330.0,
            Self::Ga8_1 => 230.0,
        }
    }

    /// Microtesla per count. 1 gauss = 100 uT.
    pub const fn ut_per_lsb(self) -> f32 {
        100.0 / self.lsb_per_gauss()
    }
}

pub const fn cfg_b(gain: Gain) -> u8 {
    (gain as u8) << 5
}

// ---- Mode register (Tables 10-12) ----

/// MD = 00: continuous measurement. Bit 7 (HS, 3.4 MHz I2C) stays clear.
pub const MODE_CONTINUOUS: u8 = 0x00;
pub const MODE_SINGLE: u8 = 0x01;
pub const MODE_IDLE: u8 = 0x03;

// ---- Chosen configuration ----

/// 8-sample averaging at the fastest continuous rate. The datasheet's
/// init example uses 8 averages at 15 Hz (0x70); we want 75 Hz because
/// the poll loop runs at 125 Hz and the MEKF is happier with fresh
/// samples than with averaged stale ones.
pub const CFG_AVG: Average = Average::X8;
pub const CFG_RATE: Rate = Rate::Hz75;

/// +/-1.9 Ga rather than the +/-1.3 Ga default. Earth's field is at most
/// 0.65 Ga, but this part sits on a GPS mast above the power
/// distribution, and the HMC5883L has only 12 bits (+/-2047 counts): at
/// 1.3 Ga the headroom over a 0.65 Ga field plus a nearby motor lead is
/// thin, and an overflowed axis reads -4096, not "a bit off". 820 LSB/Ga
/// still gives 0.12 uT per count.
pub const CFG_GAIN: Gain = Gain::Ga1_9;

pub const CFG_A_RUN: u8 = cfg_a(CFG_AVG, CFG_RATE, Bias::Normal);
pub const CFG_B_RUN: u8 = cfg_b(CFG_GAIN);

/// Microtesla per count in the configured gain.
pub const SENS_UT_PER_LSB: f32 = CFG_GAIN.ut_per_lsb();

/// "In the event the ADC reading overflows or underflows for the given
/// channel ... this data register will contain the value -4096."
pub const OVERFLOW: i16 = -4096;

// ---- Sample decoding ----

/// Decode the six data bytes at 0x03..=0x08 into raw counts in X, Y, Z
/// order.
///
/// The wire order is X, Z, Y (Table 2), each MSB first. This is the
/// single most common HMC5883L porting bug, and it is invisible in a
/// magnitude check, so it is pinned by test.
pub const fn decode_raw(buf: [u8; 6]) -> [i16; 3] {
    let x = i16::from_be_bytes([buf[0], buf[1]]);
    let z = i16::from_be_bytes([buf[2], buf[3]]);
    let y = i16::from_be_bytes([buf[4], buf[5]]);
    [x, y, z]
}

/// True if any axis carries the overflow sentinel. Such a sample is not
/// "large", it is meaningless, and must not reach the estimator.
pub const fn is_overflowed(raw: [i16; 3]) -> bool {
    raw[0] == OVERFLOW || raw[1] == OVERFLOW || raw[2] == OVERFLOW
}

/// Decode six data bytes into a body-frame sample, or None on overflow.
pub fn decode_sample(buf: [u8; 6], orient: Orientation) -> Option<MagSample> {
    let raw = decode_raw(buf);
    if is_overflowed(raw) {
        return None;
    }
    Some(MagSample::new(raw, SENS_UT_PER_LSB, orient.sign()))
}

// ---- Driver ----

#[cfg(feature = "firmware")]
pub use hw::Hmc5883l;

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

    pub struct Hmc5883l {
        addr: u8,
        orientation: Orientation,
    }

    impl Hmc5883l {
        /// Read the three identification registers. Cheap, and the
        /// caller uses it to pick this driver over the LIS2MDL at the
        /// same address before either driver writes anything.
        pub fn read_id(i2c: &mut I2c<'_, Blocking, Master>) -> Result<[u8; 3], MagError> {
            let mut id = [0u8; 3];
            i2c.blocking_write_read(I2C_ADDR, &[REG_ID_A], &mut id).map_err(map_err)?;
            Ok(id)
        }

        /// Verify the ID string, then configure continuous mode.
        ///
        /// Async because the first sample after a mode write takes
        /// 2/f_DO (Table 12), and this shares the bus with the baro.
        pub async fn init(
            i2c: &mut I2c<'_, Blocking, Master>,
            orient: Orientation,
        ) -> Result<Self, MagError> {
            let addr = I2C_ADDR;

            let id = Self::read_id(i2c)?;
            if id != ID_VALUE {
                return Err(MagError::IdMismatch(id[0]));
            }

            // There is no soft reset. Write every configuration register
            // so a warm reboot cannot leave a stale gain or bias mode in
            // place. Order follows the datasheet's own init example:
            // CRA, CRB, then mode -- the mode write is what starts
            // conversion.
            i2c.blocking_write(addr, &[REG_CFG_A, CFG_A_RUN]).map_err(map_err)?;
            i2c.blocking_write(addr, &[REG_CFG_B, CFG_B_RUN]).map_err(map_err)?;
            i2c.blocking_write(addr, &[REG_MODE, MODE_CONTINUOUS]).map_err(map_err)?;

            // 2/f_DO at 75 Hz is 27 ms; and Table 9: "the very first
            // measurement after a gain change maintains the same gain as
            // the previous setting", so wait for a second one too.
            Timer::after(Duration::from_millis(45)).await;

            defmt::info!(
                "HMC5883L init OK @ 0x{=u8:02x} (cra=0x{=u8:02x} crb=0x{=u8:02x}, {=f32} uT/LSB)",
                addr,
                CFG_A_RUN,
                CFG_B_RUN,
                SENS_UT_PER_LSB,
            );
            Ok(Self { addr, orientation: orient })
        }

        /// Burst-read the six data bytes and return a body-frame sample.
        ///
        /// A single 6-byte read starting at 0x03: "All six data registers
        /// must be read properly before new data can be placed in any of
        /// these data registers", so partial reads would stall the part.
        /// Returns `MagError::NotResponding` for an overflowed sample,
        /// which the caller counts as an error rather than fusing.
        pub fn read(
            &self,
            i2c: &mut I2c<'_, Blocking, Master>,
        ) -> Result<MagSample, MagError> {
            let mut buf = [0u8; 6];
            i2c.blocking_write_read(self.addr, &[REG_DATA], &mut buf).map_err(map_err)?;
            decode_sample(buf, self.orientation).ok_or(MagError::Overflow)
        }

        /// Raw status byte: RDY / LOCK.
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
    fn cfg_a_matches_the_datasheet_example_at_15hz() {
        // Init example step 1: "send 0x3C 0x00 0x70 (8-average, 15 Hz
        // default, normal measurement)".
        assert_eq!(cfg_a(Average::X8, Rate::Hz15, Bias::Normal), 0x70);
        // And the register default, Table 3: 0x10 = 1 average, 15 Hz.
        assert_eq!(cfg_a(Average::X1, Rate::Hz15, Bias::Normal), 0x10);
    }

    #[test]
    fn cfg_a_run_is_8_averages_at_75hz() {
        assert_eq!(CFG_A_RUN, 0x78, "got 0x{:02X}", CFG_A_RUN);
        // Reserved bit 7 must stay clear.
        assert_eq!(CFG_A_RUN & 0x80, 0);
    }

    #[test]
    fn cfg_b_matches_the_datasheet_example_and_default() {
        // Init example step 2: "0xA0 (Gain=5)".
        assert_eq!(cfg_b(Gain::Ga4_7), 0xA0);
        // Table 7: CRB default 0x20 = GN 001 = 1.3 Ga.
        assert_eq!(cfg_b(Gain::Ga1_3), 0x20);
        // Bits 4:0 "must be cleared for correct operation".
        assert_eq!(CFG_B_RUN & 0x1F, 0);
    }

    #[test]
    fn gain_scales_match_table_9() {
        // 1090 LSB/Ga default -> 0.0917 uT/LSB; 820 LSB/Ga -> 0.122.
        assert!((Gain::Ga1_3.ut_per_lsb() - 0.09174).abs() < 1e-4);
        assert!((Gain::Ga1_9.ut_per_lsb() - 0.12195).abs() < 1e-4);
        // The 12-bit output range must actually span the nominal field:
        // 2047 counts at 820 LSB/Ga = 2.5 Ga >= 1.9 Ga.
        let fs_gauss = 2047.0 / Gain::Ga1_9.lsb_per_gauss();
        assert!(fs_gauss >= 1.9, "fs={fs_gauss}");
    }

    #[test]
    fn decode_is_x_z_y_big_endian() {
        // Wire: X=0x0001, Z=0x0002, Y=0x0003. Decoded must come back as
        // [x, y, z] = [1, 3, 2]. Little-endian or X,Y,Z order both fail.
        let raw = decode_raw([0x00, 0x01, 0x00, 0x02, 0x00, 0x03]);
        assert_eq!(raw, [1, 3, 2]);
        // Negative: 0xFFFF = -1.
        assert_eq!(decode_raw([0xFF, 0xFF, 0, 0, 0, 0])[0], -1);
    }

    #[test]
    fn overflow_sentinel_rejects_the_sample() {
        // -4096 = 0xF000 big-endian.
        let buf = [0xF0, 0x00, 0x00, 0x10, 0x00, 0x10];
        assert!(is_overflowed(decode_raw(buf)));
        assert!(decode_sample(buf, Orientation::Identity).is_none());
        // ...and a merely large legitimate value is not an overflow.
        assert!(!is_overflowed([2047, -2048, 0]));
    }

    #[test]
    fn a_plausible_earth_field_decodes_to_a_plausible_magnitude() {
        // 50 uT along X at 820 LSB/Ga: 0.5 Ga * 820 = 410 counts =
        // 0x019A, big-endian. Catches a gauss/tesla or gain mix-up.
        let s = decode_sample([0x01, 0x9A, 0, 0, 0, 0], Orientation::Identity).unwrap();
        assert_eq!(s.raw[0], 410);
        let m = s.magnitude_ut();
        assert!((25.0..65.0).contains(&m), "magnitude {m} uT is not an Earth field");
    }

    #[test]
    fn id_string_is_h43() {
        assert_eq!(ID_VALUE, [0x48, 0x34, 0x33]);
    }
}
