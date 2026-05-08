// Module is compiled but intentionally unreferenced until the breakout
// brings the LIS2MDL online — silence dead_code so the build stays quiet.
#![allow(dead_code)]

// lis2mdl.rs — STMicro LIS2MDL 3-axis magnetometer driver over I2C.
//
// Configured at init for 100 Hz continuous mode in high-resolution
// power mode, with temperature compensation, offset cancellation,
// BDU and the digital low-pass filter all enabled (LPF cutoff =
// ODR/4 = 25 Hz). 25 Hz is well above any useful magnetic-field
// bandwidth on a copter and pulls noise down to ~3 mgauss RMS
// (datasheet Table 9, HR + LPF row).
//
// Address is fixed at 0x1E — there's no AD pin. The bus is borrowed
// at init and on each read so it can be shared with the onboard
// SPL06 baro on the same I2C peripheral.

use embassy_stm32::i2c::{Error as I2cError, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_time::{Duration, Timer};

// ---- I2C address ----

/// Fixed 7-bit I2C slave address (datasheet Table 20). No address pin.
pub const I2C_ADDR: u8 = 0x1E;

// ---- Register map ----

const REG_OFFSET_X_L: u8 = 0x45;
const REG_WHO_AM_I:   u8 = 0x4F;
const REG_CFG_A:      u8 = 0x60;
const REG_CFG_B:      u8 = 0x61;
const REG_CFG_C:      u8 = 0x62;
const REG_INT_CTRL:   u8 = 0x63;
const REG_STATUS:     u8 = 0x67;
const REG_OUTX_L:     u8 = 0x68;
const REG_TEMP_L:     u8 = 0x6E;

const WHO_AM_I_VALUE: u8 = 0x40;

/// Multi-byte reads need the MSB of the sub-address byte set so the
/// chip auto-increments the register pointer.
const SUB_AUTO_INC: u8 = 0x80;

// ---- Configuration values ----

// CFG_REG_A (60h)
//   bit 7: COMP_TEMP_EN — must be 1 (datasheet Table 23 footnote)
//   bit 6: REBOOT
//   bit 5: SOFT_RST
//   bit 4: LP        — 0 = high-resolution
//   bits 3:2: ODR    — 11 = 100 Hz
//   bits 1:0: MD     — 00 = continuous
//
//     0b 1 0 0 0 1 1 0 0 = 0x8C
const CFG_A_RUN_HR_100HZ: u8 = 0x8C;
const CFG_A_SOFT_RESET:   u8 = 1 << 5;

// CFG_REG_B (61h)
//   bit 4: OFF_CANC_ONE_SHOT
//   bit 3: INT_on_DataOFF
//   bit 2: Set_FREQ
//   bit 1: OFF_CANC  — 1 = enable continuous-mode offset cancellation
//   bit 0: LPF       — 1 = digital LPF on, BW = ODR/4 = 25 Hz @ 100 Hz
//
//     0b 0 0 0 0 0 0 1 1 = 0x03
const CFG_B_OFFCANC_LPF: u8 = 0x03;

// CFG_REG_C (62h)
//   bit 4: BDU       — 1 = block reads against partial updates
//
//     0b 0 0 0 1 0 0 0 0 = 0x10
const CFG_C_BDU: u8 = 0x10;

// STATUS_REG (67h) — bit 3 is Zyxda, "X+Y+Z new data available".
const STATUS_ZYXDA: u8 = 1 << 3;

// ---- Scale factors ----

/// Sensitivity from datasheet Table 2: 1.5 mgauss/LSB.
pub const SENS_MGAUSS_PER_LSB: f32 = 1.5;
/// 1 mgauss = 0.1 µT, so 1 LSB = 0.15 µT.
pub const SENS_UT_PER_LSB:     f32 = 0.15;

/// Internal die-temperature sensor: 8 LSB/°C, signed two's-complement,
/// 12-bit resolution sign-extended into 16 bits. Datasheet doesn't
/// specify a zero-point offset; treat absolute value as approximate
/// — used only for diagnostics, the chip applies temperature
/// compensation internally via COMP_TEMP_EN.
pub const TEMP_LSB_PER_C: f32 = 8.0;

// ---- Board orientation ----

/// How the LIS2MDL is mounted relative to the FC body frame (NED).
/// Same shape as the IMU drivers so downstream fusion code doesn't
/// need to special-case the magnetometer.
#[derive(Clone, Copy, Debug, defmt::Format)]
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
            Self::Identity => [ 1.0,  1.0,  1.0],
            Self::Roll180  => [ 1.0, -1.0, -1.0],
            Self::Pitch180 => [-1.0,  1.0, -1.0],
            Self::Yaw180   => [-1.0, -1.0,  1.0],
        }
    }
}

// ---- Public sample type ----

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct MagSample {
    pub raw: [i16; 3],
    sign: [f32; 3],
}

impl MagSample {
    /// Field in microtesla, rotated into FC body frame (NED).
    pub fn ut(&self) -> [f32; 3] {
        [
            self.raw[0] as f32 * SENS_UT_PER_LSB * self.sign[0],
            self.raw[1] as f32 * SENS_UT_PER_LSB * self.sign[1],
            self.raw[2] as f32 * SENS_UT_PER_LSB * self.sign[2],
        ]
    }

    /// Field in mgauss, rotated into FC body frame (NED).
    pub fn mgauss(&self) -> [f32; 3] {
        [
            self.raw[0] as f32 * SENS_MGAUSS_PER_LSB * self.sign[0],
            self.raw[1] as f32 * SENS_MGAUSS_PER_LSB * self.sign[1],
            self.raw[2] as f32 * SENS_MGAUSS_PER_LSB * self.sign[2],
        ]
    }

    /// Field in µT, sensor native frame — diagnostic / calibration use.
    pub fn ut_sensor(&self) -> [f32; 3] {
        [
            self.raw[0] as f32 * SENS_UT_PER_LSB,
            self.raw[1] as f32 * SENS_UT_PER_LSB,
            self.raw[2] as f32 * SENS_UT_PER_LSB,
        ]
    }
}

#[derive(Debug, defmt::Format)]
pub enum InitError {
    I2c,
    WhoAmIMismatch(u8),
}

impl From<I2cError> for InitError {
    fn from(_: I2cError) -> Self { Self::I2c }
}

pub struct Lis2mdl {
    addr: u8,
    orientation: Orientation,
}

impl Lis2mdl {
    /// Verify WHO_AM_I, soft-reset, then configure for 100 Hz
    /// continuous mode with temperature compensation, offset
    /// cancellation, BDU, and the digital LPF (BW = 25 Hz) all on.
    ///
    /// Async because the chip's HR turn-on time is 9.4 ms + 1/ODR
    /// when offset cancellation is enabled (datasheet Table 11), and
    /// we'd rather yield those ~25 ms back to other tasks than spin.
    pub async fn init(
        i2c: &mut I2c<'_, Blocking, Master>,
        orient: Orientation,
    ) -> Result<Self, InitError> {
        let addr = I2C_ADDR;

        // Sanity check before touching CFG. WHO_AM_I is constant 0x40.
        let mut id = [0u8; 1];
        i2c.blocking_write_read(addr, &[REG_WHO_AM_I], &mut id)?;
        if id[0] != WHO_AM_I_VALUE {
            return Err(InitError::WhoAmIMismatch(id[0]));
        }

        // Soft reset clears CFG/user registers; flash trim is preserved
        // and re-applied on the next idle→measurement transition.
        i2c.blocking_write(addr, &[REG_CFG_A, CFG_A_SOFT_RESET])?;
        Timer::after(Duration::from_millis(10)).await;

        // Set up filtering and BDU before powering up so the first
        // valid sample after the turn-on delay is already filtered.
        i2c.blocking_write(addr, &[REG_CFG_B, CFG_B_OFFCANC_LPF])?;
        i2c.blocking_write(addr, &[REG_CFG_C, CFG_C_BDU])?;

        // Power up: continuous mode, 100 Hz, HR, temp-comp on.
        i2c.blocking_write(addr, &[REG_CFG_A, CFG_A_RUN_HR_100HZ])?;
        // 9.4 ms + 1/ODR with OFF_CANC enabled ≈ 19.4 ms; round up.
        Timer::after(Duration::from_millis(25)).await;

        defmt::info!("LIS2MDL init OK @ 0x{=u8:02x}", addr);
        Ok(Self { addr, orientation: orient })
    }

    /// Burst-read OUTX_L..OUTZ_H (6 bytes, little-endian per axis,
    /// datasheet §8.13–8.15) and return a body-frame sample. The
    /// auto-increment bit is required on the sub-address.
    pub fn read(
        &self,
        i2c: &mut I2c<'_, Blocking, Master>,
    ) -> Result<MagSample, InitError> {
        let mut buf = [0u8; 6];
        i2c.blocking_write_read(self.addr, &[REG_OUTX_L | SUB_AUTO_INC], &mut buf)?;

        Ok(MagSample {
            raw: [
                i16::from_le_bytes([buf[0], buf[1]]),
                i16::from_le_bytes([buf[2], buf[3]]),
                i16::from_le_bytes([buf[4], buf[5]]),
            ],
            sign: self.orientation.sign(),
        })
    }

    /// Read the internal die-temperature sensor (°C, approximate —
    /// see `TEMP_LSB_PER_C`). Diagnostic only; the chip applies
    /// temperature compensation to the field reading itself.
    pub fn read_temp_c(
        &self,
        i2c: &mut I2c<'_, Blocking, Master>,
    ) -> Result<f32, InitError> {
        let mut buf = [0u8; 2];
        i2c.blocking_write_read(self.addr, &[REG_TEMP_L | SUB_AUTO_INC], &mut buf)?;
        let raw = i16::from_le_bytes([buf[0], buf[1]]);
        Ok(raw as f32 / TEMP_LSB_PER_C)
    }

    /// True if a fresh X/Y/Z sample is available (STATUS_REG.Zyxda).
    /// Useful for synchronising a polling loop without an INT pin.
    pub fn data_ready(
        &self,
        i2c: &mut I2c<'_, Blocking, Master>,
    ) -> Result<bool, InitError> {
        let mut buf = [0u8; 1];
        i2c.blocking_write_read(self.addr, &[REG_STATUS], &mut buf)?;
        Ok(buf[0] & STATUS_ZYXDA != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensitivity_constants_consistent() {
        // 1 mgauss = 0.1 µT.
        assert!((SENS_UT_PER_LSB - SENS_MGAUSS_PER_LSB * 0.1).abs() < 1e-6);
    }

    #[test]
    fn full_scale_lsb_matches_datasheet() {
        // Table 2: ±49.152 gauss = 49152 mgauss; 16-bit signed → ±32768 LSB
        // → 49152 / 32768 = 1.5 mgauss/LSB.
        let span_mgauss = 32768.0_f32 * SENS_MGAUSS_PER_LSB;
        assert!((span_mgauss - 49152.0).abs() < 1.0, "span={span_mgauss}");
    }

    #[test]
    fn cfg_a_bits_match_intent() {
        // COMP_TEMP_EN must be 1
        assert_eq!(CFG_A_RUN_HR_100HZ & 0x80, 0x80);
        // LP = 0 (high-res)
        assert_eq!(CFG_A_RUN_HR_100HZ & 0x10, 0x00);
        // ODR = 11 (100 Hz)
        assert_eq!(CFG_A_RUN_HR_100HZ & 0x0C, 0x0C);
        // MD = 00 (continuous)
        assert_eq!(CFG_A_RUN_HR_100HZ & 0x03, 0x00);
    }

    #[test]
    fn cfg_b_enables_offcanc_and_lpf() {
        assert_eq!(CFG_B_OFFCANC_LPF & 0x02, 0x02); // OFF_CANC
        assert_eq!(CFG_B_OFFCANC_LPF & 0x01, 0x01); // LPF
    }

    #[test]
    fn orientation_signs_unit_magnitude() {
        for o in [
            Orientation::Identity,
            Orientation::Roll180,
            Orientation::Pitch180,
            Orientation::Yaw180,
        ] {
            for s in o.sign() {
                assert_eq!(s.abs(), 1.0);
            }
        }
    }

    #[test]
    fn sample_applies_orientation_sign() {
        let s = MagSample {
            raw: [100, 200, 300],
            sign: Orientation::Roll180.sign(),
        };
        let ut = s.ut();
        assert!((ut[0] - 100.0 * SENS_UT_PER_LSB).abs() < 1e-6);
        assert!((ut[1] + 200.0 * SENS_UT_PER_LSB).abs() < 1e-6);
        assert!((ut[2] + 300.0 * SENS_UT_PER_LSB).abs() < 1e-6);
    }
}
