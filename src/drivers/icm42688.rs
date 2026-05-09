// icm42688.rs — ICM-42688-P 6-axis IMU driver over SPI.
//
// Supports dual-sensor configurations where two ICM-42688P chips are
// mounted in different orientations (e.g. Roll180 + Pitch180 on the
// DAKEFPVH743). Each instance carries its own orientation, and
// `RawImu` stores the per-sample sign vector so downstream code
// (MEKF, averaging) can convert to body-frame NED without knowing
// which sensor produced the reading.
//
// Configuration: ±16 g accel, ±2000 dps gyro, 8 kHz ODR, low-noise
// mode on both. INT1 is configured push-pull active-high pulsed for
// data-ready (used if EXTI is available; otherwise timer-polled).

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::{Duration, Timer};

// ---- Register map (Bank 0) ----

const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_INT_CONFIG: u8 = 0x14;
const REG_TEMP_DATA1: u8 = 0x1D; // 14 bytes from here: T(2), A(6), G(6)
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_INT_CONFIG1: u8 = 0x64;
const REG_INT_SOURCE0: u8 = 0x65;
const REG_WHO_AM_I: u8 = 0x75;

const WHO_AM_I_VALUE: u8 = 0x47;
const READ_MASK: u8 = 0x80;

// ---- Full-scale config values ----
//   GYRO_FS_SEL[7:5]=000 → ±2000 dps
//   GYRO_ODR[3:0]=0b0011 → 8 kHz
//   ACCEL_FS_SEL[7:5]=000 → ±16 g
//   ACCEL_ODR[3:0]=0b0011 → 8 kHz
const GYRO_CFG: u8 = 0x03;
const ACCEL_CFG: u8 = 0x03;

// PWR_MGMT0: GYRO_MODE[3:2]=11 (LN) | ACCEL_MODE[1:0]=11 (LN)
const PWR_LN_BOTH: u8 = 0x0F;

// INT_CONFIG (reg 0x14) — INT1 bits [2:0]:
//   bit 0 POLARITY: 0=active-low, 1=active-high  → want 1
//   bit 1 DRIVE_CIRCUIT: 0=open-drain, 1=push-pull → want 1
//   bit 2 MODE: 0=pulsed, 1=latched              → want 0 (pulsed)
// → push-pull, active-high, pulsed = 0b011 = 0x03
const INT_CFG: u8 = 0x03;

// INT_CONFIG1 (reg 0x64) — edge-miss fixes, both required:
//   bit 4 INT_ASYNC_RESET: default 1, datasheet says "set to 0"
//   bit 6 INT_TPULSE_DURATION: 0=100µs (default), 1=8µs (required at
//         ODR > 4 kHz — 100µs pulse overlaps the 125µs period at 8 kHz
//         and we only see half the edges).
// → 0b01000000 = 0x40
const INT_CFG1: u8 = 0x40;

// INT_SOURCE0: UI_DRDY_INT1_EN bit3
const INT_SRC_DRDY: u8 = 0x08;

// ---- Scale factors ----
/// Gyro: ±2000 dps / 32768 counts → 16.384 LSB/(°/s)
pub const GYRO_LSB_PER_DPS: f32 = 16.384;
/// Accel: ±16 g / 32768 counts → 2048 LSB/g
pub const ACCEL_LSB_PER_G: f32 = 2048.0;

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
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Orientation {
    /// IMU1: ROTATION_ROLL_180. Sensor X → +X, Y → −Y, Z → −Z.
    Roll180,
    /// IMU2: ROTATION_PITCH_180. Sensor X → −X, Y → +Y, Z → −Z.
    Pitch180,
    /// MPU6000: ROTATION_YAW_90. Sensor X → −Y, Y → +X, Z → +Z.
    Yaw90,
    /// Pre-averaged / already in body frame. No axis flips.
    Identity,
}

impl Orientation {
    /// Applies the rotation to map sensor-native axes to FC body frame (NED).
    pub const fn apply(self, v: [f32; 3]) -> [f32; 3] {
        match self {
            Self::Roll180  => [ v[0], -v[1], -v[2]],
            Self::Pitch180 => [-v[0],  v[1], -v[2]],
            Self::Yaw90    => [-v[1],  v[0],  v[2]],
            Self::Identity => [ v[0],  v[1],  v[2]],
        }
    }
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct RawImu {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub temp: i16,
    /// Orientation of the sensor used to rotate into body-frame NED.
    pub orientation: Orientation,
}

impl RawImu {
    /// Accel in g, rotated into FC body frame (NED).
    /// When stationary and level, this reads ≈(0, 0, −1) g.
    pub fn accel_g(&self) -> [f32; 3] {
        let v = [
            self.accel[0] as f32 / ACCEL_LSB_PER_G,
            self.accel[1] as f32 / ACCEL_LSB_PER_G,
            self.accel[2] as f32 / ACCEL_LSB_PER_G,
        ];
        self.orientation.apply(v)
    }

    /// Gyro in deg/s, rotated into FC body frame (NED).
    pub fn gyro_dps(&self) -> [f32; 3] {
        let v = [
            self.gyro[0] as f32 / GYRO_LSB_PER_DPS,
            self.gyro[1] as f32 / GYRO_LSB_PER_DPS,
            self.gyro[2] as f32 / GYRO_LSB_PER_DPS,
        ];
        self.orientation.apply(v)
    }

    /// Accel in g, sensor native frame — for driver/diagnostic use only.
    pub fn accel_g_sensor(&self) -> [f32; 3] {
        [
            self.accel[0] as f32 / ACCEL_LSB_PER_G,
            self.accel[1] as f32 / ACCEL_LSB_PER_G,
            self.accel[2] as f32 / ACCEL_LSB_PER_G,
        ]
    }

    /// Gyro in deg/s, sensor native frame — for driver/diagnostic use only.
    pub fn gyro_dps_sensor(&self) -> [f32; 3] {
        [
            self.gyro[0] as f32 / GYRO_LSB_PER_DPS,
            self.gyro[1] as f32 / GYRO_LSB_PER_DPS,
            self.gyro[2] as f32 / GYRO_LSB_PER_DPS,
        ]
    }

    /// Datasheet 14.9: T(°C) = (raw / 132.48) + 25.
    pub fn temp_c(&self) -> f32 {
        self.temp as f32 / 132.48 + 25.0
    }

    /// Create a fused sample by averaging two body-frame readings.
    ///
    /// Both inputs must already carry their correct orientation sign
    /// vectors. The average is computed in body-frame float space and
    /// the result gets `Identity` sign (values are already in NED).
    ///
    /// The raw `i16` fields in the returned `RawImu` are set to zero
    /// since they're meaningless after cross-orientation averaging;
    /// only the float accessors (`accel_g`, `gyro_dps`, `temp_c`)
    /// should be used on fused samples.
    pub fn averaged(a: &RawImu, b: &RawImu) -> RawImu {
        let ag = a.accel_g();
        let bg = b.accel_g();
        let ad = a.gyro_dps();
        let bd = b.gyro_dps();

        // Average in float body-frame space, then store as "raw" by
        // scaling back to counts with Identity sign so the existing
        // accel_g() / gyro_dps() paths reconstruct the exact values.
        let fused_accel = [
            ((ag[0] + bg[0]) * 0.5 * ACCEL_LSB_PER_G) as i16,
            ((ag[1] + bg[1]) * 0.5 * ACCEL_LSB_PER_G) as i16,
            ((ag[2] + bg[2]) * 0.5 * ACCEL_LSB_PER_G) as i16,
        ];
        let fused_gyro = [
            ((ad[0] + bd[0]) * 0.5 * GYRO_LSB_PER_DPS) as i16,
            ((ad[1] + bd[1]) * 0.5 * GYRO_LSB_PER_DPS) as i16,
            ((ad[2] + bd[2]) * 0.5 * GYRO_LSB_PER_DPS) as i16,
        ];
        let fused_temp = ((a.temp as i32 + b.temp as i32) / 2) as i16;

        RawImu {
            accel: fused_accel,
            gyro: fused_gyro,
            temp: fused_temp,
            orientation: Orientation::Identity,
        }
    }
}

#[derive(Debug, defmt::Format)]
pub enum InitError {
    WhoAmIMismatch(u8),
    Spi,
}

pub struct Icm42688<'d> {
    spi: Spi<'d, Async>,
    cs: Output<'d>,
    orientation: Orientation,
}

impl<'d> Icm42688<'d> {
    /// Soft-reset, verify WHO_AM_I, configure for ±16 g / ±2000 dps /
    /// 8 kHz / low-noise on both sensors, then power them up.
    ///
    /// `orient` specifies how this particular chip is mounted on the
    /// board — determines the sign vector applied in `RawImu::accel_g`
    /// and `RawImu::gyro_dps`.
    pub async fn new(
        spi: Spi<'d, Async>,
        cs: Output<'d>,
        orient: Orientation,
    ) -> Result<Self, InitError> {
        let mut dev = Self { spi, cs, orientation: orient };

        // CS idle high
        dev.cs.set_high();
        Timer::after(Duration::from_millis(1)).await;

        // Soft reset — bit0 of DEVICE_CONFIG
        dev.write_reg(REG_DEVICE_CONFIG, 0x01).await?;
        Timer::after(Duration::from_millis(2)).await;

        let id = dev.read_reg(REG_WHO_AM_I).await?;
        if id != WHO_AM_I_VALUE {
            return Err(InitError::WhoAmIMismatch(id));
        }

        // Configure ranges + ODR *before* powering the sensors up.
        dev.write_reg(REG_GYRO_CONFIG0, GYRO_CFG).await?;
        dev.write_reg(REG_ACCEL_CONFIG0, ACCEL_CFG).await?;
        dev.write_reg(REG_INT_CONFIG, INT_CFG).await?;
        dev.write_reg(REG_INT_CONFIG1, INT_CFG1).await?;
        dev.write_reg(REG_INT_SOURCE0, INT_SRC_DRDY).await?;

        // Power on — gyro + accel in low-noise mode.
        dev.write_reg(REG_PWR_MGMT0, PWR_LN_BOTH).await?;
        // Gyro startup is 30–45 ms per datasheet §14.1; accel <10 ms.
        Timer::after(Duration::from_millis(50)).await;

        Ok(dev)
    }

    /// Read the 14-byte TEMP+ACCEL+GYRO data block in one transaction.
    ///
    /// The returned `RawImu` carries this sensor's orientation sign
    /// vector, so `accel_g()` / `gyro_dps()` return body-frame NED.
    pub async fn read_raw(&mut self) -> Result<RawImu, InitError> {
        let mut buf = [0u8; 15];
        buf[0] = REG_TEMP_DATA1 | READ_MASK;

        self.cs.set_low();
        let res = self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        res.map_err(|_| InitError::Spi)?;

        let d = &buf[1..];
        Ok(RawImu {
            temp: i16::from_be_bytes([d[0], d[1]]),
            accel: [
                i16::from_be_bytes([d[2], d[3]]),
                i16::from_be_bytes([d[4], d[5]]),
                i16::from_be_bytes([d[6], d[7]]),
            ],
            gyro: [
                i16::from_be_bytes([d[8], d[9]]),
                i16::from_be_bytes([d[10], d[11]]),
                i16::from_be_bytes([d[12], d[13]]),
            ],
            orientation: self.orientation,
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
