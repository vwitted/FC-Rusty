// baro.rs — Onboard barometer driver for the DAKEFPVH743.
//
// The board footprint is populated with either a BMP280 (Bosch) or a
// DPS310 (Infineon) depending on build lot — not register-compatible.
// `detect()` at boot figures out which one is there; on the current
// target (2026-04-20) that's a DPS310 at 0x76.
//
// This file holds:
//   - `detect()`:      WHO_AM_I probe for both chips at both addresses.
//   - `Dps310`:        full DPS310 driver — calibration parse,
//                      continuous-mode config, compensated read.
//
// BMP280 would slot in similarly if a future board populates it; its
// compensation math is different so it'd live under its own struct.

use embassy_stm32::i2c::{Error as I2cError, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_time::{Duration, Timer};

// ---- Detection ----

const CANDIDATE_ADDRS: [u8; 2] = [0x76, 0x77];

const BMP280_REG_ID: u8 = 0xD0;
const BMP280_ID_VALUE: u8 = 0x58;
/// BMP388 populates the same footprint on some revisions — ID 0x50.
const BMP388_ID_VALUE: u8 = 0x50;

const DPS310_REG_ID: u8 = 0x0D;
const DPS310_ID_VALUE: u8 = 0x10;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum BaroChip {
    Bmp280 { addr: u8 },
    Dps310 { addr: u8 },
}

#[derive(Debug, defmt::Format)]
pub enum DetectError {
    NotFound,
    UnknownChip { addr: u8, id: u8 },
}

/// Probe I2C1 for a supported baro. For each candidate address we try
/// both ID registers (BMP280's 0xD0 and DPS310's 0x0D) — the chips use
/// different registers, so reading the wrong one will return either a
/// NACK or an unmapped-address 0x00. Probe lines are logged so a
/// surprise chip gets named rather than hidden.
pub fn detect(i2c: &mut I2c<'_, Blocking, Master>) -> Result<BaroChip, DetectError> {
    let mut unknown: Option<(u8, u8)> = None;

    for &addr in &CANDIDATE_ADDRS {
        let bmp_id = read_reg(i2c, addr, BMP280_REG_ID);
        let dps_id = read_reg(i2c, addr, DPS310_REG_ID);

        defmt::info!(
            "Baro probe @ 0x{=u8:02x}: reg0xD0={=?}, reg0x0D={=?}",
            addr, bmp_id, dps_id,
        );

        if bmp_id == Some(BMP280_ID_VALUE) {
            return Ok(BaroChip::Bmp280 { addr });
        }
        if dps_id == Some(DPS310_ID_VALUE) {
            return Ok(BaroChip::Dps310 { addr });
        }
        if bmp_id == Some(BMP388_ID_VALUE) {
            return Err(DetectError::UnknownChip { addr, id: BMP388_ID_VALUE });
        }

        if unknown.is_none() {
            if let Some(id) = bmp_id {
                unknown = Some((addr, id));
            }
        }
    }

    match unknown {
        Some((addr, id)) => Err(DetectError::UnknownChip { addr, id }),
        None => Err(DetectError::NotFound),
    }
}

pub fn name(chip: BaroChip) -> &'static str {
    match chip {
        BaroChip::Bmp280 { .. } => "BMP280",
        BaroChip::Dps310 { .. } => "DPS310",
    }
}

fn read_reg(i2c: &mut I2c<'_, Blocking, Master>, addr: u8, reg: u8) -> Option<u8> {
    let mut buf = [0u8; 1];
    match i2c.blocking_write_read(addr, &[reg], &mut buf) {
        Ok(()) => Some(buf[0]),
        Err(_) => None,
    }
}

// ---- Public sample type ----

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct BaroSample {
    pub pressure_pa: f32,
    /// On the DPS310 this is a *die*-temperature reading from the
    /// internal sensor, intended for pressure compensation — not for
    /// ambient use. Expect 5–15 °C offset from ambient (datasheet +
    /// Infineon forum). Pressure compensation handles it correctly
    /// via the c01/c11 cross-terms; external ambient temp would need
    /// a separate sensor.
    pub temperature_c: f32,
}

// ---- Pressure → altitude ----

/// Convert pressure to altitude in metres using the US Standard
/// Atmosphere 1976 hypsometric formula (troposphere, valid to 11 km):
///
///     h = 44330.77 · (1 − (P / P₀)^(1 / 5.2558))
///
/// `p_ref_pa` is the reference pressure. Pass the pressure latched at
/// ground level on boot for AGL altitude (positive-up), or 101325.0 for
/// absolute-ish altitude referenced to ISA sea level.
///
/// Near the reference (|h| ≲ 1 km) the formula is smooth and monotonic;
/// at low altitudes a 1 Pa pressure change is ~8.3 cm, matching what
/// we get out of the DPS310 at 16× OSR. Below ground (P > P_ref) the
/// result goes negative as expected.
pub fn pressure_to_altitude_m(p_pa: f32, p_ref_pa: f32) -> f32 {
    if p_ref_pa <= 0.0 || p_pa <= 0.0 {
        return 0.0;
    }
    let ratio = p_pa / p_ref_pa;
    44330.77_f32 * (1.0 - libm::powf(ratio, 1.0 / 5.2558))
}

// ---- DPS310 ----

// Register map (see Infineon DPS310 datasheet rev 1.2, §7)
const DPS310_REG_PRS_B2:    u8 = 0x00; // MSB of pressure
const DPS310_REG_PRS_CFG:   u8 = 0x06;
const DPS310_REG_TMP_CFG:   u8 = 0x07;
const DPS310_REG_MEAS_CFG:  u8 = 0x08;
const DPS310_REG_CFG_REG:   u8 = 0x09;
const DPS310_REG_RESET:     u8 = 0x0C;
const DPS310_REG_COEF_START:u8 = 0x10;
const DPS310_REG_COEF_SRCE: u8 = 0x28;

// MEAS_CFG status bits
const MEAS_CFG_SENSOR_RDY: u8 = 1 << 6;
const MEAS_CFG_COEF_RDY:   u8 = 1 << 7;
/// MEAS_CTRL[2:0] = 0b111 → continuous pressure + temperature.
const MEAS_CTRL_CONT_PT:   u8 = 0b111;

// Scale factors kP / kT from datasheet Table 9, keyed by OSR setting.
//   OSR 1x=0, 2x=1, 4x=2, 8x=3, 16x=4, 32x=5, 64x=6, 128x=7
const K_SCALE: [f32; 8] = [
    524288.0, 1572864.0, 3670016.0, 7864320.0,
    253952.0,  516096.0, 1040384.0, 2088960.0,
];

/// OSR encoding in PRS_CFG / TMP_CFG bits [3:0]. Names reflect the
/// number of internal samples averaged per output reading.
#[derive(Copy, Clone)]
#[allow(dead_code)]
pub enum Osr {
    X1   = 0,
    X2   = 1,
    X4   = 2,
    X8   = 3,
    X16  = 4,
    X32  = 5,
    X64  = 6,
    X128 = 7,
}

/// Output rate in Hz encoded in PRS_CFG / TMP_CFG bits [6:4].
#[derive(Copy, Clone)]
#[allow(dead_code)]
pub enum Rate {
    Hz1   = 0,
    Hz2   = 1,
    Hz4   = 2,
    Hz8   = 3,
    Hz16  = 4,
    Hz32  = 5,
    Hz64  = 6,
    Hz128 = 7,
}

/// Parsed DPS310 calibration coefficients. All values are already sign-
/// extended and converted to f32 so the compensation math is a straight
/// polynomial evaluation with no integer-width bookkeeping.
#[derive(Debug, defmt::Format)]
struct Dps310Cal {
    c0:  f32, // 12-bit signed
    c1:  f32, // 12-bit signed
    c00: f32, // 20-bit signed
    c10: f32, // 20-bit signed
    c01: f32, // 16-bit signed
    c11: f32, // 16-bit signed
    c20: f32, // 16-bit signed
    c21: f32, // 16-bit signed
    c30: f32, // 16-bit signed
}

#[derive(Debug, defmt::Format)]
pub enum Dps310Error {
    I2c,
    CoefTimeout,
    SensorTimeout,
    WhoAmIMismatch(u8),
}

impl From<I2cError> for Dps310Error {
    fn from(_: I2cError) -> Self { Dps310Error::I2c }
}

pub struct Dps310 {
    addr: u8,
    cal:  Dps310Cal,
    k_p:  f32,
    k_t:  f32,
    /// COEF_SRCE bit 7 copied into TMP_CFG bit 7 at init. Must match the
    /// factory trim or temperature (and therefore pressure) compensation
    /// drifts by several degrees / hPa.
    tmp_source_bit: u8,
}

impl Dps310 {
    /// Initialize the DPS310: soft reset, wait for coefficient and
    /// sensor ready, read calibration, configure continuous P+T mode
    /// at 16 Hz with 16× pressure OSR / 1× temperature OSR.
    ///
    /// Pressure OSR 16× gives ~1.2 Pa RMS noise (≈10 cm altitude);
    /// temperature drifts slowly so 1× is enough and lets kT stay on
    /// the cheap side of the scale table. Measurement time at 16× is
    /// ~27.6 ms, comfortably within the 62.5 ms period at 16 Hz rate.
    ///
    /// Async so the ~40 ms of chip-startup polling interleaves with
    /// other tasks (notably the 8 kHz ICM read loop) rather than
    /// spinning the CPU through `cortex_m::asm::delay`.
    pub async fn init(
        i2c: &mut I2c<'_, Blocking, Master>,
        addr: u8,
    ) -> Result<Self, Dps310Error> {
        // Verify ID — cheap, and a sanity check after the probe above.
        let mut id = [0u8; 1];
        i2c.blocking_write_read(addr, &[DPS310_REG_ID], &mut id)?;
        if id[0] != DPS310_ID_VALUE {
            return Err(Dps310Error::WhoAmIMismatch(id[0]));
        }

        // Soft reset (bit 3 of reg 0x0C, 0b1001 is the datasheet magic
        // value). Datasheet §7.12 says wait 12 ms after reset.
        i2c.blocking_write(addr, &[DPS310_REG_RESET, 0x09])?;
        Timer::after(Duration::from_millis(15)).await;

        // Poll MEAS_CFG.COEF_RDY — OTP coefficients aren't available
        // for ~40 ms after reset on a cold start.
        let mut meas = 0u8;
        for _ in 0..20 {
            let mut buf = [0u8; 1];
            i2c.blocking_write_read(addr, &[DPS310_REG_MEAS_CFG], &mut buf)?;
            meas = buf[0];
            if meas & MEAS_CFG_COEF_RDY != 0 {
                break;
            }
            Timer::after(Duration::from_millis(5)).await;
        }
        if meas & MEAS_CFG_COEF_RDY == 0 {
            return Err(Dps310Error::CoefTimeout);
        }

        // Read COEF_SRCE (bit 7) before we touch anything — it drives
        // the choice of internal vs external temperature sensor, which
        // MUST match factory trim for correct compensation.
        let mut src = [0u8; 1];
        i2c.blocking_write_read(addr, &[DPS310_REG_COEF_SRCE], &mut src)?;
        let tmp_source_bit = src[0] & 0x80;

        // Read 18-byte calibration block.
        let mut coef = [0u8; 18];
        i2c.blocking_write_read(addr, &[DPS310_REG_COEF_START], &mut coef)?;
        let cal = Dps310Cal::parse(&coef);

        // Wait for SENSOR_RDY — "sensor is ready for measurements"
        // (datasheet: up to 40 ms after reset).
        let mut ready = false;
        for _ in 0..20 {
            let mut buf = [0u8; 1];
            i2c.blocking_write_read(addr, &[DPS310_REG_MEAS_CFG], &mut buf)?;
            if buf[0] & MEAS_CFG_SENSOR_RDY != 0 {
                ready = true;
                break;
            }
            Timer::after(Duration::from_millis(5)).await;
        }
        if !ready {
            return Err(Dps310Error::SensorTimeout);
        }

        // ---- Configuration ----
        let p_rate = Rate::Hz16;
        let p_osr  = Osr::X16;
        let t_rate = Rate::Hz16;
        let t_osr  = Osr::X1;

        // PRS_CFG: rate[6:4] | osr[3:0]
        let prs_cfg = ((p_rate as u8) << 4) | (p_osr as u8);
        i2c.blocking_write(addr, &[DPS310_REG_PRS_CFG, prs_cfg])?;

        // TMP_CFG: TMP_EXT[7] | rate[6:4] | osr[3:0]
        let tmp_cfg = tmp_source_bit | ((t_rate as u8) << 4) | (t_osr as u8);
        i2c.blocking_write(addr, &[DPS310_REG_TMP_CFG, tmp_cfg])?;

        // CFG_REG: P_SHIFT (bit 2) required when pressure OSR > 8×.
        //          T_SHIFT (bit 3) required when temperature OSR > 8×.
        //          Both interrupt enables left at 0 — we poll.
        let mut cfg = 0u8;
        if matches!(p_osr, Osr::X16 | Osr::X32 | Osr::X64 | Osr::X128) {
            cfg |= 1 << 2;
        }
        if matches!(t_osr, Osr::X16 | Osr::X32 | Osr::X64 | Osr::X128) {
            cfg |= 1 << 3;
        }
        i2c.blocking_write(addr, &[DPS310_REG_CFG_REG, cfg])?;

        // Start continuous P+T mode.
        i2c.blocking_write(addr, &[DPS310_REG_MEAS_CFG, MEAS_CTRL_CONT_PT])?;

        let k_p = K_SCALE[p_osr as usize];
        let k_t = K_SCALE[t_osr as usize];

        defmt::info!(
            "DPS310 init OK: tmp_src={=u8:#x} k_p={=f32} k_t={=f32}",
            tmp_source_bit, k_p, k_t,
        );
        defmt::info!("DPS310 cal: {}", cal);

        Ok(Self { addr, cal, k_p, k_t, tmp_source_bit })
    }

    /// Read the latest compensated pressure + temperature.
    ///
    /// This blocks on 6 bytes of I2C (~150 µs at 400 kHz) so it's cheap
    /// enough to call on a 25 Hz timer from the baro task. The chip
    /// runs in continuous mode so there's no start-measurement latency.
    pub fn read(
        &self,
        i2c: &mut I2c<'_, Blocking, Master>,
    ) -> Result<BaroSample, Dps310Error> {
        // Burst-read pressure (3B) + temperature (3B), both 24-bit
        // big-endian two's complement.
        let mut buf = [0u8; 6];
        i2c.blocking_write_read(self.addr, &[DPS310_REG_PRS_B2], &mut buf)?;

        let p_raw = sign_extend_24(
            ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32),
        );
        let t_raw = sign_extend_24(
            ((buf[3] as u32) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32),
        );

        let p_scaled = p_raw as f32 / self.k_p;
        let t_scaled = t_raw as f32 / self.k_t;

        // Compensation (datasheet §4.9.1). Horner-form for the cubic
        // in P; cross-term c11 adds a temperature-modulated linear
        // term in P.
        let temperature_c = self.cal.c0 * 0.5 + self.cal.c1 * t_scaled;
        let pressure_pa = self.cal.c00
            + p_scaled * (self.cal.c10 + p_scaled * (self.cal.c20 + p_scaled * self.cal.c30))
            + t_scaled * self.cal.c01
            + t_scaled * p_scaled * (self.cal.c11 + p_scaled * self.cal.c21);

        Ok(BaroSample { pressure_pa, temperature_c })
    }

    /// True if the driver was configured to use the external-MEMS
    /// temperature source (vs internal ASIC). Debug only.
    #[allow(dead_code)]
    pub fn uses_external_tmp(&self) -> bool { self.tmp_source_bit != 0 }
}

impl Dps310Cal {
    /// Parse the 18-byte calibration block. The coefficients pack at
    /// non-byte boundaries (12-bit c0/c1, 20-bit c00/c10) so each field
    /// is extracted by hand; all values are sign-extended to i32 then
    /// cast to f32 once up front.
    fn parse(b: &[u8; 18]) -> Self {
        // c0 [11:0] = b[0][7:0] : b[1][7:4]
        let c0_u  = ((b[0] as u32) << 4) | ((b[1] as u32) >> 4);
        let c1_u  = (((b[1] as u32) & 0x0F) << 8) | (b[2] as u32);
        let c00_u = ((b[3] as u32) << 12)
                  | ((b[4] as u32) <<  4)
                  | ((b[5] as u32) >>  4);
        let c10_u = (((b[5] as u32) & 0x0F) << 16)
                  |  ((b[6] as u32) <<  8)
                  |   (b[7] as u32);
        let c01_u = ((b[8]  as u32) << 8) | (b[9]  as u32);
        let c11_u = ((b[10] as u32) << 8) | (b[11] as u32);
        let c20_u = ((b[12] as u32) << 8) | (b[13] as u32);
        let c21_u = ((b[14] as u32) << 8) | (b[15] as u32);
        let c30_u = ((b[16] as u32) << 8) | (b[17] as u32);

        Self {
            c0:  sign_extend(c0_u,  12) as f32,
            c1:  sign_extend(c1_u,  12) as f32,
            c00: sign_extend(c00_u, 20) as f32,
            c10: sign_extend(c10_u, 20) as f32,
            c01: sign_extend(c01_u, 16) as f32,
            c11: sign_extend(c11_u, 16) as f32,
            c20: sign_extend(c20_u, 16) as f32,
            c21: sign_extend(c21_u, 16) as f32,
            c30: sign_extend(c30_u, 16) as f32,
        }
    }
}

/// Sign-extend `value` from `bits` wide to a full i32.
fn sign_extend(value: u32, bits: u32) -> i32 {
    let shift = 32 - bits;
    ((value << shift) as i32) >> shift
}

fn sign_extend_24(value: u32) -> i32 {
    sign_extend(value, 24)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_extend_positive() {
        assert_eq!(sign_extend(0x7FF, 12),  2047);
        assert_eq!(sign_extend(0x000, 12),  0);
    }

    #[test]
    fn sign_extend_negative() {
        assert_eq!(sign_extend(0x800, 12), -2048);
        assert_eq!(sign_extend(0xFFF, 12), -1);
    }

    #[test]
    fn sign_extend_24bit() {
        assert_eq!(sign_extend_24(0x7FFFFF),  8388607);
        assert_eq!(sign_extend_24(0x800000), -8388608);
        assert_eq!(sign_extend_24(0xFFFFFF), -1);
    }

    #[test]
    fn altitude_is_zero_at_reference() {
        let h = pressure_to_altitude_m(101325.0, 101325.0);
        assert!(h.abs() < 0.01, "h={h}");
    }

    #[test]
    fn altitude_sign_convention_is_positive_up() {
        // Lower pressure → higher altitude (positive).
        let h_hi = pressure_to_altitude_m(95_000.0, 101_325.0);
        assert!(h_hi > 0.0 && h_hi < 1000.0, "h_hi={h_hi}");
        // Higher pressure → below reference (negative).
        let h_lo = pressure_to_altitude_m(103_000.0, 101_325.0);
        assert!(h_lo < 0.0 && h_lo > -500.0, "h_lo={h_lo}");
    }

    #[test]
    fn altitude_small_step_matches_8_3_cm_per_pa() {
        // Near the reference, dh/dP ≈ −8.3 cm/Pa. A 10 Pa drop should
        // show ~83 cm of climb within loose bounds.
        let h0 = pressure_to_altitude_m(101_325.0, 101_325.0);
        let h1 = pressure_to_altitude_m(101_315.0, 101_325.0);
        let dh = h1 - h0;
        assert!(dh > 0.70 && dh < 0.95, "dh={dh}");
    }

    #[test]
    fn altitude_clamps_on_bad_ref() {
        assert_eq!(pressure_to_altitude_m(100_000.0, 0.0), 0.0);
        assert_eq!(pressure_to_altitude_m(0.0, 101_325.0), 0.0);
    }
}
