// Module is compiled but intentionally unreferenced until Beta brings
// the breakout online — silence dead_code so the build stays quiet.
#![allow(dead_code)]

// ism6hg256x.rs — STMicro ISM6HG256X 6-axis IMU driver over SPI.
//
// Configures the low-g channel (±16 g) + gyro (±4000 dps) at 7.68 kHz
// ODR in high-performance mode. ±4000 dps is unique to this part vs
// the ICM-42688P's ±2000 max; we use it because the gyro is already
// noise-floor-limited at ±2000 (Rn ≈ 75 mdps RMS at the default LPF1
// bandwidth, vs 70 mdps/LSB) so the wider FS is strictly more
// saturation headroom for free. The high-g channel, sensor fusion
// (SFLP), finite state machine, machine learning core, OIS, and EIS
// blocks are all left disabled — those are post-Beta toys.
//
// Note vs. ICM-42688: ST IMU output registers are little-endian
// (L then H) and use a different data-rate ladder (max 7.68 kHz vs
// the ICM's 8 kHz). The data accessor (`RawImu`) carries an
// orientation sign vector identical in shape to the ICM driver so
// downstream code (MEKF, fusion, averaging) doesn't care which
// chip produced the sample.

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::{Duration, Timer};

// ---- Register map ----

const REG_FUNC_CFG_ACCESS: u8 = 0x01;
const REG_IF_CFG: u8 = 0x03;
const REG_INT1_CTRL: u8 = 0x0D;
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1: u8 = 0x10; // Accel mode + ODR
const REG_CTRL2: u8 = 0x11; // Gyro mode + ODR
const REG_CTRL3: u8 = 0x12; // BOOT, BDU, IF_INC, SW_RESET
const REG_CTRL4: u8 = 0x13; // INT routing, DRDY_PULSED
const REG_CTRL6: u8 = 0x15; // Gyro LPF1 BW + FS
const REG_CTRL8: u8 = 0x17; // Accel HP/LPF2 BW + FS
const REG_OUT_TEMP_L: u8 = 0x20;

const WHO_AM_I_VALUE: u8 = 0x73;
const READ_MASK: u8 = 0x80;

// ---- Configuration values ----

// CTRL1 / CTRL2 — OP_MODE bits [6:4] = 000 (high-performance),
// ODR bits [3:0] = 1100 (7.68 kHz, the chip max).
//   0b 0 000 1100 = 0x0C
const CTRL1_ACCEL_HP_7K68: u8 = 0x0C;
const CTRL2_GYRO_HP_7K68: u8 = 0x0C;

// CTRL3 — BOOT=0, BDU=1, IF_INC=1, SW_RESET=0
//   0b 0 1 0 0 0 1 0 0 = 0x44 (this is also the reset default).
const CTRL3_BDU_AUTOINC: u8 = 0x44;
const CTRL3_SW_RESET: u8 = 0x01;

// CTRL4 — DRDY_PULSED bit1 = 1 → 65 µs pulses on the DRDY interrupt.
// At 7.68 kHz the sample period is ~130 µs, so a 65 µs pulse fits
// cleanly without overlapping the next edge. Latched mode would
// require reading OUTX_H_A to clear, which is fragile in a polled
// loop — same reasoning as the ICM-42688 INT_TPULSE choice.
const CTRL4_DRDY_PULSED: u8 = 0x02;

// CTRL6 — bit3 must be 1 (datasheet "must be set"), FS_G[2:0]=101
// for ±4000 dps, LPF1_G_BW=000 (default bandwidth at 7.68 kHz is
// ~281 Hz, plenty for our 200 Hz rate loop).
//   0b 0 000 1 101 = 0x0D
const CTRL6_GYRO_FS_4000DPS: u8 = 0x0D;

// CTRL8 — HP_LPF2_XL_BW=000 (LPF1 only, ODR/2 cutoff), FS_XL[1:0]=11
// for ±16 g.
//   0b 000 000 11 = 0x03
const CTRL8_ACCEL_FS_16G: u8 = 0x03;

// IF_CFG — I2C_I3C_disable bit0 = 1 to lock the part into SPI-only
// mode. Defaults are fine otherwise (push-pull, active-high INTs,
// 4-wire SPI). Belt-and-braces: CS toggling already idles I²C/I³C.
const IF_CFG_SPI_ONLY: u8 = 0x01;

// INT1_CTRL — INT1_DRDY_XL bit0 = 1 (accel data ready).
// Gyro and accel are synchronised at the same ODR, so one DRDY is
// enough; we pick accel to mirror the convention used elsewhere.
const INT1_DRDY_XL: u8 = 0x01;

// ---- Scale factors (datasheet §4.1, "Mechanical characteristics") ----

/// Gyro at ±4000 dps: sensitivity 140 mdps/LSB → 1 / 0.140 LSB per °/s.
pub const GYRO_LSB_PER_DPS: f32 = 7.142_857;
/// Accel low-g at ±16 g: sensitivity 0.488 mg/LSB → 1 / 0.000488 LSB per g.
pub const ACCEL_LSB_PER_G: f32 = 2048.0;

// ---- Board orientation ----

/// How the ISM6HG256X chip is mounted relative to the FC body frame.
///
/// Same shape as `icm42688::Orientation` so the two drivers are
/// interchangeable. Add new variants here as Beta hardware lands.
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

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct RawImu {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub temp: i16,
    sign: [f32; 3],
}

impl RawImu {
    /// Accel in g, rotated into FC body frame (NED).
    /// When stationary and level, this reads ≈(0, 0, −1) g.
    pub fn accel_g(&self) -> [f32; 3] {
        [
            self.accel[0] as f32 / ACCEL_LSB_PER_G * self.sign[0],
            self.accel[1] as f32 / ACCEL_LSB_PER_G * self.sign[1],
            self.accel[2] as f32 / ACCEL_LSB_PER_G * self.sign[2],
        ]
    }

    /// Gyro in deg/s, rotated into FC body frame (NED).
    pub fn gyro_dps(&self) -> [f32; 3] {
        [
            self.gyro[0] as f32 / GYRO_LSB_PER_DPS * self.sign[0],
            self.gyro[1] as f32 / GYRO_LSB_PER_DPS * self.sign[1],
            self.gyro[2] as f32 / GYRO_LSB_PER_DPS * self.sign[2],
        ]
    }

    /// Accel in g, sensor native frame — diagnostic / self-test use.
    pub fn accel_g_sensor(&self) -> [f32; 3] {
        [
            self.accel[0] as f32 / ACCEL_LSB_PER_G,
            self.accel[1] as f32 / ACCEL_LSB_PER_G,
            self.accel[2] as f32 / ACCEL_LSB_PER_G,
        ]
    }

    /// Gyro in deg/s, sensor native frame — diagnostic / self-test use.
    pub fn gyro_dps_sensor(&self) -> [f32; 3] {
        [
            self.gyro[0] as f32 / GYRO_LSB_PER_DPS,
            self.gyro[1] as f32 / GYRO_LSB_PER_DPS,
            self.gyro[2] as f32 / GYRO_LSB_PER_DPS,
        ]
    }

    /// Temperature in °C. Standard ST IMU formula: 1 LSB = 1/256 °C
    /// with a 25 °C offset. Diagnostic only — not on the control path.
    pub fn temp_c(&self) -> f32 {
        self.temp as f32 / 256.0 + 25.0
    }
}

#[derive(Debug, defmt::Format)]
pub enum InitError {
    WhoAmIMismatch(u8),
    Spi,
}

pub struct Ism6hg256x<'d> {
    spi: Spi<'d, Async>,
    cs: Output<'d>,
    orientation: Orientation,
}

impl<'d> Ism6hg256x<'d> {
    /// Soft-reset, verify WHO_AM_I, configure for ±16 g / ±4000 dps /
    /// 7.68 kHz / high-performance on both channels, then enable the
    /// data-ready interrupt on INT1.
    ///
    /// `orient` specifies how the chip is mounted relative to the FC
    /// body frame; it determines the sign vector applied in
    /// `RawImu::accel_g` and `RawImu::gyro_dps`.
    pub async fn new(
        spi: Spi<'d, Async>,
        cs: Output<'d>,
        orient: Orientation,
    ) -> Result<Self, InitError> {
        let mut dev = Self { spi, cs, orientation: orient };

        dev.cs.set_high();
        Timer::after(Duration::from_millis(10)).await;

        // Soft reset (CTRL3 SW_RESET bit 0). Self-clearing; datasheet
        // doesn't quote a reset duration but the ST family typically
        // settles within a few hundred µs — give it 5 ms to be safe.
        dev.write_reg(REG_CTRL3, CTRL3_SW_RESET).await?;
        Timer::after(Duration::from_millis(5)).await;

        // FUNC_CFG_ACCESS reset to default — we never enter the
        // embedded-functions or sensor-hub register banks. Writing
        // 0 here defends against a stale enable from a prior boot.
        dev.write_reg(REG_FUNC_CFG_ACCESS, 0x00).await?;

        let id = dev.read_reg(REG_WHO_AM_I).await?;
        if id != WHO_AM_I_VALUE {
            return Err(InitError::WhoAmIMismatch(id));
        }

        // Lock to SPI-only. IF_CFG is *not* cleared by SW_RESET, so
        // it's worth being explicit even though the default is 0.
        dev.write_reg(REG_IF_CFG, IF_CFG_SPI_ONLY).await?;

        // BDU on, address auto-increment on. Datasheet default is
        // 0x44 already, but the SW_RESET above will have re-applied
        // it; rewrite for explicitness so future readers see intent.
        dev.write_reg(REG_CTRL3, CTRL3_BDU_AUTOINC).await?;

        // Full-scale + bandwidth before turning the channels on.
        dev.write_reg(REG_CTRL8, CTRL8_ACCEL_FS_16G).await?;
        dev.write_reg(REG_CTRL6, CTRL6_GYRO_FS_4000DPS).await?;

        // INT1 routing + pulse mode.
        dev.write_reg(REG_CTRL4, CTRL4_DRDY_PULSED).await?;
        dev.write_reg(REG_INT1_CTRL, INT1_DRDY_XL).await?;

        // Power on: write the ODR fields → leaves power-down. Gyro
        // turn-on is ~40 ms (datasheet §4.2 Ton); accel is faster.
        // Wait long enough for both to be producing valid data.
        dev.write_reg(REG_CTRL1, CTRL1_ACCEL_HP_7K68).await?;
        dev.write_reg(REG_CTRL2, CTRL2_GYRO_HP_7K68).await?;
        Timer::after(Duration::from_millis(50)).await;

        Ok(dev)
    }

    /// Burst-read the 14-byte block from OUT_TEMP_L (0x20) through
    /// OUTZ_H_A (0x2D): 2 bytes temp, 6 bytes gyro, 6 bytes accel.
    ///
    /// Output is little-endian (L then H) — opposite of the ICM-42688.
    /// The returned `RawImu` carries this sensor's orientation sign
    /// vector so `accel_g()` / `gyro_dps()` return body-frame NED.
    pub async fn read_raw(&mut self) -> Result<RawImu, InitError> {
        let mut buf = [0u8; 15];
        buf[0] = REG_OUT_TEMP_L | READ_MASK;

        self.cs.set_low();
        let res = self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        res.map_err(|_| InitError::Spi)?;

        let d = &buf[1..];
        Ok(RawImu {
            temp: i16::from_le_bytes([d[0], d[1]]),
            gyro: [
                i16::from_le_bytes([d[2], d[3]]),
                i16::from_le_bytes([d[4], d[5]]),
                i16::from_le_bytes([d[6], d[7]]),
            ],
            accel: [
                i16::from_le_bytes([d[8], d[9]]),
                i16::from_le_bytes([d[10], d[11]]),
                i16::from_le_bytes([d[12], d[13]]),
            ],
            sign: self.orientation.sign(),
        })
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, InitError> {
        let mut buf = [reg | READ_MASK, 0u8];
        self.cs.set_low();
        let res = self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        res.map_err(|_| InitError::Spi)?;
        Ok(buf[1])
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), InitError> {
        let buf = [reg & !READ_MASK, value];
        self.cs.set_low();
        let res = self.spi.write(&buf).await;
        self.cs.set_high();
        res.map_err(|_| InitError::Spi)
    }
}
