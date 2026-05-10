// mpu6000.rs — MPU-6000 6-axis IMU driver over SPI.
//
// Target board: GEPRCTAKERH743
// Configured for ±16 g accel, ±2000 dps gyro, 8 kHz ODR gyro, 1 kHz accel.

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_time::{Duration, Timer};
use super::icm42688::{InitError, Orientation, RawImu};

// ---- Register map ----
const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_XOUT_H: u8 = 0x3B; // 14 bytes from here: A(6), T(2), G(6)
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_WHO_AM_I: u8 = 0x75;

const WHO_AM_I_VALUE: u8 = 0x68;
const READ_MASK: u8 = 0x80;

// ---- Full-scale config values ----
// ±2000 dps
const GYRO_CFG: u8 = 0x18;
// ±16 g
const ACCEL_CFG: u8 = 0x18;

// CONFIG: DLPF_CFG = 0 (256Hz bw, 8kHz ODR for gyro, 1kHz for accel)
const CONFIG_VAL: u8 = 0x00;
// SMPLRT_DIV = 0 (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) -> 8kHz)
const SMPLRT_DIV_VAL: u8 = 0x00;

// USER_CTRL: Disable I2C interface (I2C_IF_DIS = 0x10)
const USER_CTRL_VAL: u8 = 0x10;

// PWR_MGMT_1: Reset = 0x80, Auto clock select (PLL with X gyro) = 0x01
const PWR_RESET: u8 = 0x80;
const PWR_CLKSEL: u8 = 0x01;

pub struct Mpu6000<'d> {
    spi: Spi<'d, Async>,
    cs: Output<'d>,
    orientation: Orientation,
}

impl<'d> Mpu6000<'d> {
    pub async fn new(
        spi: Spi<'d, Async>,
        cs: Output<'d>,
        orient: Orientation,
    ) -> Result<Self, InitError> {
        let mut dev = Self { spi, cs, orientation: orient };

        // Temporarily lower SPI frequency to 1 MHz for setup writes
        // (MPU6000 requires <= 1 MHz for register writes)
        let mut cfg = embassy_stm32::spi::Config::default();
        cfg.frequency = embassy_stm32::time::Hertz(1_000_000);
        dev.spi.set_config(&cfg).unwrap();

        // CS idle high
        dev.cs.set_high();
        Timer::after(Duration::from_millis(1)).await;

        // Soft reset
        dev.write_reg(REG_PWR_MGMT_1, PWR_RESET).await?;
        Timer::after(Duration::from_millis(100)).await;

        // Wake up and select X-axis gyro clock
        dev.write_reg(REG_PWR_MGMT_1, PWR_CLKSEL).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Disable I2C interface
        dev.write_reg(REG_USER_CTRL, USER_CTRL_VAL).await?;
        Timer::after(Duration::from_millis(1)).await;

        let id = dev.read_reg(REG_WHO_AM_I).await?;
        if id != WHO_AM_I_VALUE {
            return Err(InitError::WhoAmIMismatch(id));
        }

        // Configure ranges + ODR
        dev.write_reg(REG_CONFIG, CONFIG_VAL).await?;
        dev.write_reg(REG_SMPLRT_DIV, SMPLRT_DIV_VAL).await?;
        dev.write_reg(REG_GYRO_CONFIG, GYRO_CFG).await?;
        dev.write_reg(REG_ACCEL_CONFIG, ACCEL_CFG).await?;

        // Give sensors time to settle
        Timer::after(Duration::from_millis(50)).await;

        // Restore SPI frequency to 10 MHz for reading
        cfg.frequency = embassy_stm32::time::Hertz(10_000_000);
        dev.spi.set_config(&cfg).unwrap();

        Ok(dev)
    }

    /// Read the 14-byte ACCEL+TEMP+GYRO data block.
    pub async fn read_raw(&mut self) -> Result<RawImu, InitError> {
        let mut buf = [0u8; 15];
        buf[0] = REG_ACCEL_XOUT_H | READ_MASK;

        self.cs.set_low();
        let res = self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        res.map_err(|_| InitError::Spi)?;

        let d = &buf[1..];
        
        // MPU6000 Temp calculation: T(°C) = (raw / 340.0) + 36.53.
        // We return `temp` as a pseudo-raw value so that `RawImu::temp_c` computes it correctly.
        // `RawImu::temp_c` uses: (raw / 132.48) + 25.0 (ICM42688 scale).
        // So we need to reverse this to store in `RawImu`:
        // raw_icm = (T_actual - 25.0) * 132.48
        let mpu_temp_raw = i16::from_be_bytes([d[6], d[7]]);
        let temp_c = mpu_temp_raw as f32 / 340.0 + 36.53;
        let icm_temp_raw = ((temp_c - 25.0) * 132.48) as i16;

        Ok(RawImu {
            accel: [
                i16::from_be_bytes([d[0], d[1]]),
                i16::from_be_bytes([d[2], d[3]]),
                i16::from_be_bytes([d[4], d[5]]),
            ],
            gyro: [
                i16::from_be_bytes([d[8], d[9]]),
                i16::from_be_bytes([d[10], d[11]]),
                i16::from_be_bytes([d[12], d[13]]),
            ],
            temp: icm_temp_raw,
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
        let mut buf = [reg & !READ_MASK, value];
        self.cs.set_low();
        let res = self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        res.map_err(|_| InitError::Spi)
    }
}
