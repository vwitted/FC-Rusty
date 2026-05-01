// main.rs — Flight controller entry point
//
// Target: STM32H743VIT6 (480 MHz, Cortex-M7F, 100-pin LQFP)
// Board:  DAKEFPVH743 (STM32H743)
//         5V BEC during IMU rework — see git log for context)
// Framework: Embassy async executor
//
// Pin map (DAKEFPVH743):
//   USART1  TX=PA9              → T1 pad, defmt output (raw-reg logger, 115200)
//   USART2  RX=PD6              → R2 pad, CRSF receiver (416666 baud)
//   USART6  TX=PC6   RX=PC7     → T6/R6 pads, GPS (UBX binary)
//   UART4   RX=PA1              → ESC telemetry (internal, not wired yet)
//
// The onboard ICM-42688P (SPI1) is used as the primary IMU.
//
// Motors (DShot600 via a single timer multi-channel burst):
//   TIM2_CH1 → PA0 → M1
//   TIM2_CH2 → PA1 → M2
//   TIM2_CH3 → PA2 → M3
//   TIM2_CH4 → PA3 → M4
//   See src/drivers/dshot_hw.rs for DMA stream / timing details.
//
// Flashing: board has no SWD; use DFU over USB-C. Hold BOOT while
// plugging in, then run `scripts/flash-dfu.sh` (see that script for
// the one-off cargo-binutils / dfu-util install).

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Uart, UartRx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker};

use panic_probe as _; // panic handler that works with probe

// Our modules
mod drivers {
    pub mod baro;
    pub mod crsf;
    pub mod dshot_diag;
    pub mod dshot_hw;
    pub mod icm42688;
    pub mod nmea;
    pub mod ubx;
    pub mod wt901b;
}
mod control {
    pub mod altitude;
    pub mod arming;
    pub mod mixer;
    pub mod mpc;
    pub mod pid;
}
mod attitude_mekf;
mod estimation;
mod logger;
mod rc_task;

use attitude_mekf::{AttitudeMekf, G_MPS2, MekfParams};
use control::altitude::{AltitudeController, AltitudeGains};
use control::arming::{ArmState, ArmingStateMachine};
use control::mixer::{ControlDemand, QUAD_X};
use control::mpc::AttitudeMpc;
use control::pid::{PidGains, PidLimits, RatePidController};
use drivers::baro::{self, BaroSample};
use drivers::crsf::RcChannels;
use drivers::dshot_hw::DshotQuad;
use uf_dshot::{Command, DshotSpeed, DshotTx, EncodedFrame};
use drivers::icm42688::RawImu;
use drivers::nmea::{FixMode, GpsData, NmeaParser};
use drivers::wt901b::{
    ImuData, UPDATED_ACCEL, UPDATED_ANGLE, UPDATED_GYRO, UPDATED_QUAT, Wt901bParser,
};
use estimation::{PosKf, geodetic_to_local_ned};

// ---- Interrupt bindings ----

// USART1 is owned by `logger::init_usart1()` (raw register TX for defmt)
// — no Embassy interrupt handler needed here.
bind_interrupts!(struct Irqs {

    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
});

// ---- Shared state between tasks ----
// Signals are "latest value wins" — perfect for real-time sensor data.

/// Latest IMU data from the WT901B task (fused angles + raw rates).
static IMU_DATA: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Latest raw samples from the ICM-42688P task (body-frame NED).
/// Phase 2: populated at ~8 kHz, not yet consumed by the control loop.
/// Phase 3 (MEKF) will consume this and republish to IMU_DATA.
static RAW_IMU: Signal<CriticalSectionRawMutex, RawImu> = Signal::new();

/// Counters for the ICM monitor task. Live regardless of whether the
/// read task is making progress, so a silent INT pin still shows up
/// as `0 samples/s, 0 errors/s` instead of no log at all.
static ICM_SAMPLES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static ICM_ERRORS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Latest GPS data from the GPS task
static GPS_DATA: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

/// Parallel GPS signal dedicated to `pos_kf_task`. The NMEA task
/// publishes to both so the 200 Hz control loop and the KF task don't
/// race each other for the same single-shot sample.
static GPS_DATA_FOR_KF: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

/// Latest baro sample (pressure_pa + temperature_c) from the baro task.
/// Consumed by `pos_kf_task` to drive the position-KF `update_baro`.
static BARO_DATA: Signal<CriticalSectionRawMutex, BaroSample> = Signal::new();

/// Latest fused IMU data intended for the position KF (*separate* from
/// `IMU_DATA`, which the control loop already consumes). Signalled by
/// `mekf_task` on its 100 Hz accel-update branch — paying one extra
/// signal-write every 80th sample is cheap and avoids consumer
/// contention with the control loop on `IMU_DATA`.
static IMU_DATA_FOR_KF: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Fused position / velocity estimate published by `pos_kf_task`.
#[derive(Clone, Copy, Debug, defmt::Format)]
struct PosEstimate {
    /// Position in NED world frame (m), relative to the home point once
    /// the GPS home latch completes. Before home latch the horizontal
    /// components are from dead-reckoning the accel since boot — only
    /// useful once `home_latched` is true.
    position_ned: [f32; 3],
    /// Velocity in NED world frame (m/s).
    velocity_ned: [f32; 3],
    /// Altitude above boot reference (m, positive up).
    altitude_up: f32,
    /// Vertical velocity (m/s, positive up).
    vz_up: f32,
    /// Reference pressure latched from the first seconds of boot baro
    /// readings (Pa). 0.0 until latching completes.
    p_ref_pa: f32,
    /// Milliseconds since the last baro update was applied.
    baro_age_ms: u32,
    /// False until the p_ref latch completes and the first update_baro
    /// has fired. Control loop altitude hold should gate on this.
    ready: bool,
    /// True once a sufficiently good GPS fix has been captured as the
    /// home origin. Horizontal `position_ned` is meaningful only when
    /// this is true; any GPS-rescue or position-hold behaviour must
    /// gate on it.
    home_latched: bool,
}

static POS_ESTIMATE: Signal<CriticalSectionRawMutex, PosEstimate> = Signal::new();



// RC signals are defined in rc_task.rs:
// rc_task::RC_CHANNELS, rc_task::RC_LINK, rc_task::RC_LAST_SEEN

// ---- Main entry point ----

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ---- Clock configuration ----
    // STM32H743 with 8 MHz HSE crystal on the DAKEFPVH743.
    //   HSE 8 MHz → PLL_M=1 → 8 MHz → PLL_N=120 → VCO 960 MHz
    //   PLL_P=2 → SYSCLK 480 MHz  (H743 max)
    //   PLL_Q=20 → USB 48 MHz     (960/20 = 48, exact)
    //   AHB  = 240 MHz (prescaler 2)
    //   APB1 = 120 MHz (prescaler 2), APB1 timers = 240 MHz
    //   APB2 = 120 MHz (prescaler 2), APB2 timers = 240 MHz
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllDiv, PllPreDiv,
        PllSource, Sysclk, VoltageScale,
    };
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,   // 8 MHz / 1 = 8 MHz VCO input
        mul: PllMul::MUL120,       // 8 MHz × 120 = 960 MHz VCO
        divp: Some(PllDiv::DIV2),  // 960 / 2 = 480 MHz SYSCLK
        divq: Some(PllDiv::DIV20), // 960 / 20 = 48 MHz USB
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // 240 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 120 MHz (timers 240 MHz)
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 120 MHz (timers 240 MHz)
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.voltage_scale = VoltageScale::Scale0;

    let p = embassy_stm32::init(config);

    // Disable D-cache as early as possible
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.SCB.disable_dcache(&mut core.CPUID);

    // Bring up USART1 TX (PA9) for defmt output before anything else
    // so the first defmt::info! below actually lands on the wire.
    logger::init_usart1();

    // ---- Status LED Heartbeat ----
    // DAKEFPVH743 has LED0 on PD10 (active low). We spawn a quick blink
    // task so we have an immediate visual indicator that the firmware
    // has booted and is running, without needing to check the UART logs.
    use embassy_stm32::gpio::{Level, Output, Speed};
    let led = Output::new(p.PD10, Level::High, Speed::Low);
    spawner.spawn(blink_task(led)).unwrap();

    defmt::info!("Flight controller starting");

    // ---- Configure and spawn the RC receiver task ----
    // CRSF on USART2 RX (PD6), 416666 baud — R2 pad on the DAKEFPVH743.
    let rc_uart = UartRx::new(
        p.USART2,
        Irqs,
        p.PD6,      // RX pin (USART2)
        p.DMA1_CH5, // USART2_RX → DMA1 Stream 5
        rc_task::crsf_uart_config(),
    )
    .unwrap();

    spawner.spawn(rc_task::run(rc_uart)).unwrap();
    defmt::info!("RC task spawned");



    // ---- ICM-42688P on SPI1 (Phase 2: 8 kHz INT-driven reads) ----
    // SCK=PA5, MISO=PA6, MOSI=PA7, CS=PB2, INT1=PC4.
    // SPI @ 10 MHz target (embassy picks nearest ≤: 6.75 MHz on APB2).
    {
        use embassy_stm32::exti::ExtiInput;
        use embassy_stm32::gpio::{Level, Output, Pull, Speed};
        use embassy_stm32::spi::{Config as SpiConfig, Spi};
        use embassy_stm32::time::Hertz;

        let mut spi_cfg = SpiConfig::default();
        spi_cfg.frequency = Hertz(10_000_000);

        let spi = Spi::new(
            p.SPI1, p.PA5,      // SCK
            p.PA7,      // MOSI
            p.PA6,      // MISO
            p.DMA2_CH3, // SPI1_TX (ch3)
            p.DMA2_CH0, // SPI1_RX (ch3)
            spi_cfg,
        );

        let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
        let drdy = ExtiInput::new(p.PC4, p.EXTI4, Pull::None);

        match drivers::icm42688::Icm42688::new(spi, cs).await {
            Ok(imu) => {
                defmt::info!("ICM-42688P initialised OK");
                spawner.spawn(icm_read_task(imu, drdy)).unwrap();
                spawner.spawn(icm_monitor_task()).unwrap();
                spawner.spawn(mekf_task()).unwrap();
            }
            Err(e) => {
                defmt::error!("ICM-42688P init failed: {:?}", e);
            }
        }
    }

    // ---- Baro on I2C1 (DPS310 or BMP280, auto-detected) ----
    // SCL=PB10, SDA=PB11. Blocking mode.
    //
    // The task owns the raw peripherals (not a pre-built I2c) so it can
    // drop the driver and bitbang SCL to unstick the bus when the STM32
    // I2C peripheral latches BUSY/ARLO — observed mid-run in the field.
    spawner.spawn(baro_task(p.I2C2, p.PB10, p.PB11)).unwrap();
    spawner.spawn(pos_kf_task()).unwrap();

    // ---- Configure and spawn the GPS task ----
    // GPS on USART6 TX=PC6 RX=PC7 at factory 9600 baud, speaking
    // factory-default NMEA. We parse NMEA directly — no module
    // reconfiguration required. See `drivers::nmea`.
    let gps_uart_config = {
        let mut c = usart::Config::default();
        c.baudrate = 9600;
        c
    };

    let gps_uart = Uart::new(
        p.USART6,
        p.PC7, // RX
        p.PC6, // TX
        Irqs,
        p.DMA2_CH6, // TX DMA
        p.DMA2_CH1, // RX DMA
        gps_uart_config,
    )
    .unwrap();

    let (_gps_tx, gps_rx) = gps_uart.split();
    spawner.spawn(gps_task(gps_rx)).unwrap();
    defmt::info!("GPS task spawned (NMEA at 9600)");

    // DShot ESC outputs (all 4 channels on TIM2)
    // M1=PA0, M2=PA1, M3=PA2, M4=PA3.
    let dshot = DshotQuad::new(
        p.TIM2,
        p.PA0,
        p.PA1,
        p.PA2,
        p.PA3,
        p.DMA1_CH7, // TIM2_UP
        DshotSpeed::Dshot600,
    );

    dshot.log_config();
    defmt::info!("DShot (TIM2+TIM3+TIM4, DShot600) initialised");

    // ---- Run the control loop on the main task ----
    // This is deliberate: the control loop is the highest priority
    // work, so it runs on the main executor rather than being
    // spawned as a separate task.
    control_loop(dshot).await;
}

// ---- Heartbeat Task ----

#[embassy_executor::task]
async fn blink_task(mut led: embassy_stm32::gpio::Output<'static>) {
    loop {
        led.set_low(); // Turn LED ON
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        led.set_high(); // Turn LED OFF
        embassy_time::Timer::after(embassy_time::Duration::from_millis(900)).await;
    }
}

// ---- GPS Task ----
// Reads NMEA sentences from the GPS module, publishes via GPS_DATA signal.

#[embassy_executor::task]
async fn gps_task(mut rx: UartRx<'static, embassy_stm32::mode::Async>) {
    let mut parser = NmeaParser::new();
    let mut buf = [0u8; 128];
    let mut sentence_count: u32 = 0;
    let mut last_report = Instant::now();
    let mut announced_first = false;

    defmt::info!("GPS task started (NMEA)");

    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                for &byte in &buf {
                    if parser.push_byte(byte).is_some() {
                        sentence_count += 1;
                        GPS_DATA.signal(parser.data);
                        // Second consumer: `pos_kf_task` needs GPS
                        // updates too and can't share a single-shot
                        // signal with the 200 Hz control loop.
                        GPS_DATA_FOR_KF.signal(parser.data);

                        if !announced_first {
                            defmt::info!("GPS: first NMEA sentence parsed — stream is alive");
                            announced_first = true;
                        }

                        // Report GPS stats every 5 seconds, regardless of fix.
                        let now = Instant::now();
                        if now.duration_since(last_report) >= Duration::from_secs(5) {
                            defmt::info!(
                                "GPS: {} sentences, fix={} mode={} sats={} hdop={:?} lat={:?} lon={:?} alt={:?}m",
                                sentence_count,
                                parser.data.fix as u8,
                                parser.data.fix_mode as u8,
                                parser.data.satellites,
                                parser.data.hdop,
                                parser.data.latitude as f32,
                                parser.data.longitude as f32,
                                parser.data.altitude_m,
                            );
                            last_report = now;
                        }
                    }
                }
            }
            Err(e) => {
                defmt::warn!("GPS UART error: {:?}", e);
                embassy_time::Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

// ---- ICM-42688P Read Task (INT-driven 8 kHz) ----
// Waits on rising edge of DRDY (INT1 → PC4), reads the 14-byte data
// block, and publishes the RawImu to RAW_IMU. The MEKF task consumes
// RAW_IMU at 8 kHz and republishes fused attitude to IMU_DATA for the
// control loop.

#[embassy_executor::task]
async fn icm_read_task(
    mut imu: drivers::icm42688::Icm42688<'static>,
    mut drdy: embassy_stm32::exti::ExtiInput<'static>,
) {
    use core::sync::atomic::Ordering;
    defmt::info!("ICM read task started (INT-driven)");

    loop {
        drdy.wait_for_rising_edge().await;
        match imu.read_raw().await {
            Ok(r) => {
                RAW_IMU.signal(r);
                ICM_SAMPLES.fetch_add(1, Ordering::Relaxed);
            }
            Err(_) => {
                ICM_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

// ---- ICM Monitor Task ----
// Runs on a 1 Hz ticker independently of the read task, so we see
// sample/error counters even if the read task is stuck (e.g. INT pin
// not firing). Raw IMU values are logged by the MEKF task which
// consumes RAW_IMU at full rate.

#[embassy_executor::task]
async fn icm_monitor_task() {
    use core::sync::atomic::Ordering;
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        let s = ICM_SAMPLES.swap(0, Ordering::Relaxed);
        let e = ICM_ERRORS.swap(0, Ordering::Relaxed);
        defmt::info!("ICM {} samples/s, {} errors/s", s, e);
    }
}

// ---- MEKF Task (Phase 3) ----
// Consumes ICM-42688P raw samples at 8 kHz, runs the 6-state error-state
// Kalman filter (3 attitude + 3 gyro bias), and republishes the fused
// result to IMU_DATA so the existing control loop picks it up with no
// consumer-side changes. See src/attitude_mekf.rs for the math.
//
// Rate scheduling:
//   - Predict runs every sample (8 kHz, ~125 µs) using RAW_IMU gyro.
//   - Accel gravity update runs every `ACCEL_DECIMATION`-th sample
//     (default 80 → 100 Hz). Accel is noisy and low-bandwidth; updating
//     at full rate wastes cycles and amplifies vibration-induced drift.
//
// Output conventions match the old WT901B producer so the control loop
// is agnostic to which sensor is driving IMU_DATA:
//   - ImuData.accel  in m/s² (ICM g × 9.80665 at the boundary)
//   - ImuData.gyro   in °/s, bias-corrected
//   - ImuData.angle  Euler roll/pitch/yaw in degrees, from the filter
//   - ImuData.quaternion as [w, x, y, z], body→nav
//   - ImuData.altitude_cm / pressure / mag are zero — baro will be
//     fused in a separate task (Phase 4); mag is unused for now.

#[embassy_executor::task]
async fn mekf_task() {
    use core::f32::consts::PI;
    const DEG2RAD: f32 = PI / 180.0;
    const RAD2DEG: f32 = 180.0 / PI;
    // 8 kHz predict / 100 Hz update → 80:1 decimation. Tune if accel
    // vibration at ODR aliases into the update band.
    const ACCEL_DECIMATION: u32 = 80;

    defmt::info!("MEKF task started — waiting for first RAW_IMU sample");

    let mut mekf = AttitudeMekf::new(MekfParams::default());

    // Seed attitude from the first accel reading (assumes the board
    // is stationary at boot — a 50 ms power-on settle inside the
    // ICM driver plus the tasks-spawning delay covers this in practice).
    let first = RAW_IMU.wait().await;
    mekf.initialize_from_accel(first.accel_g());
    defmt::info!(
        "MEKF seeded: euler=[{=f32},{=f32},{=f32}]deg",
        mekf.euler()[0] * RAD2DEG,
        mekf.euler()[1] * RAD2DEG,
        mekf.euler()[2] * RAD2DEG,
    );

    let mut last_predict = Instant::now();
    let mut sample_count: u32 = 0;
    let mut last_report = Instant::now();
    let mut updates_applied: u32 = 0;
    let mut updates_rejected: u32 = 0;

    loop {
        let raw = RAW_IMU.wait().await;

        let now = Instant::now();
        // Clamp dt to sane bounds — a missed sample stretches dt to
        // ~250 µs which the filter handles; anything beyond 2 ms is a
        // stall we shouldn't integrate through.
        let dt_us = (now - last_predict).as_micros() as f32;
        let dt = (dt_us * 1.0e-6).clamp(50.0e-6, 2.0e-3);
        last_predict = now;

        let g_dps = raw.gyro_dps();
        let gyro_rad = [g_dps[0] * DEG2RAD, g_dps[1] * DEG2RAD, g_dps[2] * DEG2RAD];
        mekf.predict(gyro_rad, dt);

        let on_kf_tick = sample_count % ACCEL_DECIMATION == 0;
        if on_kf_tick {
            if mekf.update_accel(raw.accel_g()) {
                updates_applied = updates_applied.wrapping_add(1);
            } else {
                updates_rejected = updates_rejected.wrapping_add(1);
            }
        }
        sample_count = sample_count.wrapping_add(1);

        // Bias-corrected gyro in °/s — this is what downstream PID expects.
        let bias_rad = mekf.bias();
        let gyro_corr_dps = [
            (gyro_rad[0] - bias_rad[0]) * RAD2DEG,
            (gyro_rad[1] - bias_rad[1]) * RAD2DEG,
            (gyro_rad[2] - bias_rad[2]) * RAD2DEG,
        ];

        let a_g = raw.accel_g();
        let euler_rad = mekf.euler();
        let imu = ImuData {
            accel: [a_g[0] * G_MPS2, a_g[1] * G_MPS2, a_g[2] * G_MPS2],
            temperature: raw.temp_c(),
            gyro: gyro_corr_dps,
            angle: [
                euler_rad[0] * RAD2DEG,
                euler_rad[1] * RAD2DEG,
                euler_rad[2] * RAD2DEG,
            ],
            mag: [0; 3],
            pressure: 0,
            altitude_cm: 0,
            quaternion: mekf.quaternion(),
            updated: UPDATED_ACCEL | UPDATED_GYRO | UPDATED_ANGLE | UPDATED_QUAT,
        };
        IMU_DATA.signal(imu);
        // Feed the position KF at 100 Hz — matches the accel-update
        // cadence so the signalled sample is the freshest fused one.
        if on_kf_tick {
            IMU_DATA_FOR_KF.signal(imu);
        }

        // 1 Hz health log. Includes sample count (should be ~8000),
        // fused Euler so we can see tilt test output live, and bias
        // magnitude so we can watch bias converge.
        if (now - last_report) >= Duration::from_secs(1) {
            let b_dps_mag = libm::sqrtf(
                (bias_rad[0] * RAD2DEG) * (bias_rad[0] * RAD2DEG)
                    + (bias_rad[1] * RAD2DEG) * (bias_rad[1] * RAD2DEG)
                    + (bias_rad[2] * RAD2DEG) * (bias_rad[2] * RAD2DEG),
            );
            defmt::info!(
                "MEKF {} samples/s, upd={}/{}rej, euler=[{=f32},{=f32},{=f32}]deg, |bias|={=f32}dps",
                sample_count,
                updates_applied,
                updates_rejected,
                euler_rad[0] * RAD2DEG,
                euler_rad[1] * RAD2DEG,
                euler_rad[2] * RAD2DEG,
                b_dps_mag,
            );
            sample_count = 0;
            updates_applied = 0;
            updates_rejected = 0;
            last_report = now;
        }
    }
}

// ---- Position KF Task (Phase 4b + 4c.1) ----
// 6-state position/velocity Kalman filter driven by:
//   - IMU prediction at 100 Hz (matches the MEKF accel-update cadence
//     so every prediction uses a gravity-corrected attitude).
//   - GPS position updates at ~1 Hz (NMEA default) — fuses once home
//     is latched.
//   - Baro altitude updates at ~25 Hz — fuses only after GPS has
//     anchored the filter and baro has self-calibrated (see below).
//
// Altitude frame = GPS. The original design averaged the first N baro
// readings at boot as ground pressure and let altitude hold engage
// baro-only. That was abandoned after the onboard DPS310 proved
// intermittent (2026-04-20): a drone that arms and takes off on a
// sensor that might vanish mid-flight is unsafe. The current design
// is:
//   1. Home origin latches on the first GPS fix clearing `FIX3D`,
//      `≥ MIN_SATS_FOR_LATCH`, and `HDOP < MAX_HDOP_FOR_LATCH`.
//   2. Subsequent GPS fixes fuse as local NED (home_lat/lon/alt_msl
//      as the reference). KF altitude converges toward home = 0 m AGL.
//   3. Once at least `MIN_GPS_FUSES_FOR_BARO_CAL` GPS updates have
//      fused, the *next* baro sample triggers p_ref self-calibration:
//      compute the reference pressure that makes baro altitude agree
//      with the KF's current altitude. From that sample forward, baro
//      fuses normally (and its σ=0.3 m dominates the altitude axis
//      over GPS σ=5 m).
//   4. If baro is never alive, the filter runs GPS-only — noisier
//      vertical but safe.
//
// Home is never re-latched — GPS rescue has to return to the *original*
// origin, not wherever the receiver sees us now.

#[embassy_executor::task]
async fn pos_kf_task() {
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};

    const HZ: u64 = 100;
    const PERIOD_MS: u64 = 1000 / HZ;
    const DT: f32 = 1.0 / HZ as f32;

    // GPS home-latch gates. Relaxed for Alpha testing in poor-signal
    // environments. Post-Alpha: tighten to 6 sats / HDOP < 2.5 or lower.
    const MIN_SATS_FOR_LATCH: u8 = 5;
    const MAX_HDOP_FOR_LATCH: f32 = 3.5;

    // Wait for at least this many GPS fuses (post-home) before
    // self-calibrating the baro. Each fuse pulls KF altitude closer to
    // truth; one is technically enough since the init covariance (2 m)
    // puts a single σ_gps_v=5 m fix at Kalman gain ~0.14 — but two
    // fuses get us well inside σ_gps_v before we lock in the baro
    // reference, which is cheap insurance against a one-off noisy fix.
    const MIN_GPS_FUSES_FOR_BARO_CAL: u32 = 2;

    // σ_a = 0.5 m/s² matches the sim tuning — loose enough to track
    // gust transients without treating baro noise as truth. σ_baro =
    // 0.3 m is the DPS310 spec at 16× OSR plus a bit of headroom.
    // σ_gps_h / σ_gps_v are typical consumer-module noise; the KF will
    // rightly let baro dominate altitude via the much-smaller σ_baro.
    let mut kf = PosKf::new_at(
        [0.0, 0.0, 0.0],
        0.5, // σ_a
        2.0, // σ_gps_h
        5.0, // σ_gps_v
        0.3, // σ_baro
    );

    // Baro self-calibration state. p_ref is the pressure that, if
    // plugged into the hypsometric formula with the baro's current
    // reading, yields the KF altitude at the moment of calibration.
    // `baro_calibrated` gates baro fusion — before it flips true,
    // baro samples are ignored.
    let mut p_ref_pa: f32 = 0.0;
    let mut baro_calibrated = false;

    // Home-origin latch state. Stored as f64 for lat/lon to preserve
    // sub-metre resolution; the geodetic helper handles the cast.
    let mut home_lat: f64 = 0.0;
    let mut home_lon: f64 = 0.0;
    let mut home_alt_msl: f32 = 0.0;
    let mut home_latched = false;
    // How many GPS updates have fused since home latched. Gates baro
    // self-calibration so we don't lock in the reference against a
    // wildly noisy first fix.
    let mut gps_fuses_post_home: u32 = 0;

    let mut last_imu: Option<ImuData> = None;
    let mut last_baro_t = Instant::now();
    let mut baro_updates_sec: u32 = 0;
    let mut gps_updates_sec: u32 = 0;
    let mut last_gps: Option<GpsData> = None;

    let mut ticker = Ticker::every(Duration::from_millis(PERIOD_MS));
    let mut last_report = Instant::now();

    defmt::info!("PosKF task started (100 Hz predict; baro + GPS fusion)");

    loop {
        ticker.next().await;

        // ---- Pull latest IMU for predict ----
        if let Some(imu) = IMU_DATA_FOR_KF.try_take() {
            last_imu = Some(imu);
        }

        // ---- Predict with world-frame kinematic accel ----
        // Rotate body specific force by the MEKF quaternion (body→nav),
        // then add gravity to recover inertial accel in NED world.
        // If we haven't seen an IMU yet, coast with zero accel.
        if let Some(imu) = last_imu {
            let [qw, qx, qy, qz] = imu.quaternion;
            let q = UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz));
            let sf_body = Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
            let sf_world = q * sf_body;
            // NED: +Z down, so kinematic = specific force + [0, 0, +g].
            let a_world = [sf_world.x, sf_world.y, sf_world.z + G_MPS2];
            kf.predict(a_world, DT);
        } else {
            kf.predict([0.0, 0.0, 0.0], DT);
        }

        // ---- GPS update (sensor-driven; None on non-1-Hz ticks) ----
        // Home-point latch fires once on the first fix that clears the
        // quality gates; subsequent fixes get converted to local NED
        // and fused. No re-latch — the original origin is authoritative
        // for anything that ever needs to return to it.
        if let Some(gps) = GPS_DATA_FOR_KF.try_take() {
            last_gps = Some(gps);
            let good_fix = gps.fix_mode == FixMode::Fix3D
                && gps.satellites >= MIN_SATS_FOR_LATCH
                && gps.hdop > 0.0
                && gps.hdop < MAX_HDOP_FOR_LATCH;

            if !home_latched {
                if good_fix {
                    home_lat = gps.latitude;
                    home_lon = gps.longitude;
                    home_alt_msl = gps.altitude_m;
                    home_latched = true;
                    defmt::info!(
                        "PosKF home latched: lat={=f32} lon={=f32} alt_msl={=f32}m | sats={} hdop={=f32}",
                        gps.latitude as f32,
                        gps.longitude as f32,
                        gps.altitude_m,
                        gps.satellites,
                        gps.hdop,
                    );
                }
            } else if good_fix {
                // Convert to local NED and fuse. Z-channel uses
                // GPS altitude (σ=5 m) so baro still dominates the
                // short-term once calibrated; the GPS keeps altitude
                // honest over the long term via cross-covariance.
                let ned = geodetic_to_local_ned(
                    gps.latitude,
                    gps.longitude,
                    gps.altitude_m,
                    home_lat,
                    home_lon,
                    home_alt_msl,
                );
                kf.update_gps(ned);
                gps_updates_sec = gps_updates_sec.wrapping_add(1);
                gps_fuses_post_home = gps_fuses_post_home.saturating_add(1);
            }
        }

        // ---- Baro update (sensor-driven; None on non-25-Hz ticks) ----
        // Two-phase: self-calibrate once GPS has anchored the KF, then
        // fuse as an altitude sensor. No boot-time averaging — the
        // reference pressure is whatever makes baro altitude agree with
        // the GPS-anchored KF at the moment of calibration.
        if let Some(baro) = BARO_DATA.try_take() {
            if !baro_calibrated {
                if home_latched && gps_fuses_post_home >= MIN_GPS_FUSES_FOR_BARO_CAL {
                    // p_ref = p_now / (1 - kf_alt_up/44330.77)^5.2558
                    let kf_alt_up = kf.altitude_up();
                    let ratio_base = 1.0 - kf_alt_up / 44330.77_f32;
                    // Guard the pow() against a nonsensical KF alt
                    // (shouldn't happen post-fuse, but cheap to check).
                    if ratio_base > 0.0 && baro.pressure_pa > 0.0 {
                        p_ref_pa = baro.pressure_pa / libm::powf(ratio_base, 5.2558);
                        baro_calibrated = true;
                        last_baro_t = Instant::now();
                        defmt::info!(
                            "PosKF baro self-cal: p_ref={=f32}Pa @ kf_alt={=f32}m (p_now={=f32}Pa)",
                            p_ref_pa,
                            kf_alt_up,
                            baro.pressure_pa,
                        );
                    }
                }
                // Pre-calibration: don't fuse, don't count toward baro_updates_sec.
            } else {
                let alt_up = baro::pressure_to_altitude_m(baro.pressure_pa, p_ref_pa);
                kf.update_baro(alt_up);
                last_baro_t = Instant::now();
                baro_updates_sec = baro_updates_sec.wrapping_add(1);
            }
        }

        // ---- Readiness ----
        // GPS home latch is the sole gate. Altitude is GPS-anchored from
        // that point; baro is a nice-to-have that tightens the estimate
        // once it self-calibrates. Arming requires `home_latched` too
        // (see `ArmingStateMachine`) — so the control loop never sees
        // `ready` and `home_latched` disagree at arm time.
        let ready = home_latched;

        // ---- Publish estimate ----
        let s = kf.state();
        let est = PosEstimate {
            position_ned: [s[0], s[1], s[2]],
            velocity_ned: [s[3], s[4], s[5]],
            altitude_up: kf.altitude_up(),
            vz_up: kf.vz_up(),
            p_ref_pa,
            baro_age_ms: last_baro_t.elapsed().as_millis() as u32,
            ready,
            home_latched,
        };
        POS_ESTIMATE.signal(est);

        // ---- 1 Hz health log ----
        if last_report.elapsed() >= Duration::from_secs(1) {
            let (sats, hdop, fix) = last_gps
                .map(|g| (g.satellites, g.hdop, g.fix_mode as u8))
                .unwrap_or((0, 99.99, 0));
            if ready {
                defmt::info!(
                    "PosKF ready: alt={=f32}m vz={=f32}m/s | N={=f32}m E={=f32}m | baro_cal={} | {} baro/s {} gps/s",
                    est.altitude_up,
                    est.vz_up,
                    est.position_ned[0],
                    est.position_ned[1],
                    baro_calibrated,
                    baro_updates_sec,
                    gps_updates_sec,
                );
            } else {
                defmt::info!(
                    "PosKF waiting: GPS sats={} hdop={=f32} fix={} | {} gps/s",
                    sats,
                    hdop,
                    fix,
                    gps_updates_sec,
                );
            }
            baro_updates_sec = 0;
            gps_updates_sec = 0;
            last_report = Instant::now();
        }
    }
}

// ---- Baro Task (Phase 4) ----
// Owns I2C1 + SCL/SDA pins directly (not a pre-built I2c) so it can
// drop the driver and bitbang SCL to unstick the bus when the STM32
// I2C peripheral latches BUSY/ARLO on some platforms.
// If it times out at the 25 Hz tick rate, we use a recovery sequence.
// timeouts at the 25 Hz tick rate (≈25 errs/s), no reads.
//
// Recovery sequence:
//   1. Drop the `I2c` — this disconnects pins and disables I2C1 RCC.
//   2. Drive SCL as **open-drain** output (never push-pull!), toggle 9×
//      at ~100 kHz. Slave finishes whatever partial byte it was holding
//      SDA low for.
//   3. Manual STOP: drive SDA low then release high, both with SCL
//      high — also open-drain. Slave returns to idle.
//   4. Rebuild I2c, rerun detect + DPS310 init.
//
// !! SAFETY: open-drain is NOT optional. Slaves are allowed to clock-
// stretch (hold SCL low) and to drive SDA low during a transaction. A
// push-pull output fighting that short-circuits the MCU's PMOS through
// the slave's NMOS — that killed the onboard DPS310 on this board on
// 2026-04-20. Never `Output::new` on an I2C pin; always
// `OutputOpenDrain::new`.
//
// The bitbang runs on *every* rebuild after the first, not just the
// read-streak path — field-observed second hang had detect pass and
// init fail with I2C error, and without bitbanging between retry
// attempts the rebuild would just re-hang indefinitely.
//
// The outer loop gives up after `MAX_INIT_ATTEMPTS` failures so a
// totally absent baro stops spamming the log — reboot after the
// external baro is wired in. A dead bus should not take the FC down,
// and pos_kf_task handles missing baro via GPS-only altitude.

const BARO_ERR_STREAK_RECOVERY: u32 = 10; // ~0.4 s at 25 Hz
const BARO_TIMEOUT_MS: u64 = 5; // shorter wastes less CPU when stuck
const BARO_MAX_INIT_ATTEMPTS: u32 = 5; // give up after this many detect/init failures

#[embassy_executor::task]
async fn baro_task(
    mut i2c_per: embassy_stm32::Peri<'static, embassy_stm32::peripherals::I2C2>,
    mut scl: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB10>,
    mut sda: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB11>,
) {
    use drivers::baro::{self, BaroChip, Dps310};
    use embassy_stm32::gpio::{Level, OutputOpenDrain, Speed};
    use embassy_stm32::i2c::{Config as I2cConfig, I2c};
    use embassy_stm32::time::Hertz;
    use embassy_time::Timer;

    let make_cfg = || {
        let mut c = I2cConfig::default();
        c.frequency = Hertz(400_000);
        c.timeout = Duration::from_millis(BARO_TIMEOUT_MS);
        c
    };

    let mut recovery_count: u32 = 0;
    let mut init_failures: u32 = 0;
    let mut first_iter = true;

    // Outer loop: (re)build I2c, detect + init, run read loop until it
    // asks for recovery. The reborrow()s keep ownership of the raw Peris
    // here so we can drop the I2c and bitbang SCL directly.
    loop {
        if init_failures >= BARO_MAX_INIT_ATTEMPTS {
            defmt::warn!(
                "Baro: {} consecutive init failures — giving up (reboot after wiring external baro)",
                init_failures,
            );
            return;
        }

        // Always bitbang before rebuild (except on the very first boot
        // build). Cheap (~100 µs), idempotent when the bus is already
        // idle, essential when detect/init failed and left it stuck.
        //
        // Both SCL and SDA use OutputOpenDrain — slaves are allowed to
        // drive either line low, and a push-pull output fighting that
        // shorts the MCU output stage through the slave's NMOS. See the
        // safety note on this task.
        if !first_iter {
            Timer::after(Duration::from_millis(2)).await;
            {
                let mut scl_out = OutputOpenDrain::new(scl.reborrow(), Level::High, Speed::Low);
                for _ in 0..9 {
                    scl_out.set_low();
                    Timer::after(Duration::from_micros(5)).await;
                    scl_out.set_high();
                    Timer::after(Duration::from_micros(5)).await;
                }
                let mut sda_out = OutputOpenDrain::new(sda.reborrow(), Level::Low, Speed::Low);
                Timer::after(Duration::from_micros(5)).await;
                sda_out.set_high();
                Timer::after(Duration::from_micros(5)).await;
            }
            Timer::after(Duration::from_millis(10)).await;
        }
        first_iter = false;

        let mut i2c = I2c::new_blocking(
            i2c_per.reborrow(),
            scl.reborrow(),
            sda.reborrow(),
            make_cfg(),
        );

        let chip = match baro::detect(&mut i2c) {
            Ok(c) => {
                defmt::info!("Baro detected: {} ({:?})", baro::name(c), c);
                c
            }
            Err(e) => {
                init_failures = init_failures.saturating_add(1);
                defmt::warn!(
                    "Baro detect failed: {:?} ({}/{}) — bitbang + retry in 1 s",
                    e,
                    init_failures,
                    BARO_MAX_INIT_ATTEMPTS,
                );
                drop(i2c);
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        let addr = match chip {
            BaroChip::Dps310 { addr } => addr,
            BaroChip::Bmp280 { addr: _ } => {
                defmt::warn!("BMP280 detected but driver not yet implemented");
                return;
            }
        };

        let dps = match Dps310::init(&mut i2c, addr).await {
            Ok(d) => d,
            Err(e) => {
                init_failures = init_failures.saturating_add(1);
                defmt::error!(
                    "DPS310 init failed: {:?} ({}/{}) — bitbang + retry in 1 s",
                    e,
                    init_failures,
                    BARO_MAX_INIT_ATTEMPTS,
                );
                drop(i2c);
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        // A successful init means the chip is healthy; reset the
        // give-up counter so a later bus-stuck recovery doesn't count
        // against the boot-time init budget.
        init_failures = 0;

        // ---- Read loop ----
        let mut ticker = Ticker::every(Duration::from_millis(40)); // 25 Hz
        let mut reads: u32 = 0;
        let mut errs: u32 = 0;
        let mut streak: u32 = 0;
        let mut last_report = Instant::now();
        let mut last_sample: Option<(BaroSample, Instant)> = None;

        let recover = loop {
            ticker.next().await;
            match dps.read(&mut i2c) {
                Ok(s) => {
                    last_sample = Some((s, Instant::now()));
                    BARO_DATA.signal(s);
                    reads = reads.wrapping_add(1);
                    streak = 0;
                }
                Err(_) => {
                    errs = errs.wrapping_add(1);
                    streak = streak.saturating_add(1);
                }
            }

            if Instant::now() - last_report >= Duration::from_secs(1) {
                match (reads, last_sample) {
                    (0, Some((s, t))) => {
                        let age_ms = (Instant::now() - t).as_millis() as u32;
                        defmt::info!(
                            "Baro 0 reads/s, {} errs — bus stuck (last P={=f32}Pa T={=f32}C age={=u32}ms)",
                            errs,
                            s.pressure_pa,
                            s.temperature_c,
                            age_ms,
                        );
                    }
                    (0, None) => {
                        defmt::info!("Baro 0 reads/s, {} errs — bus stuck (no sample yet)", errs);
                    }
                    _ => {
                        let (p, t) = last_sample
                            .map(|(s, _)| (s.pressure_pa, s.temperature_c))
                            .unwrap_or((0.0, 0.0));
                        defmt::info!(
                            "Baro {} reads/s, {} errs — P={=f32}Pa T={=f32}C",
                            reads,
                            errs,
                            p,
                            t,
                        );
                    }
                }
                reads = 0;
                errs = 0;
                last_report = Instant::now();
            }

            if streak >= BARO_ERR_STREAK_RECOVERY {
                break true;
            }
        };

        if recover {
            recovery_count = recovery_count.saturating_add(1);
            defmt::warn!(
                "Baro: I2C bus stuck ({} consecutive errs), recovering (n={})",
                streak,
                recovery_count,
            );
            drop(i2c);
            // Fall through — top of outer loop runs the bitbang + rebuild.
        }
    }
}



// ---- Control Loop ----
// The heart of the flight controller. Reads sensor data and
// RC input, computes control outputs, and drives the motors.
//
// Architecture:
//   200 Hz: PID rate inner loop + mixer + DShot output
//    50 Hz: MPC attitude outer loop + altitude hold (every 4th cycle)
//
// This runs as the main task — it never returns.

async fn control_loop(mut dshot: DshotQuad<'static>) -> ! {
    use core::f32::consts::PI;
    const DEG2RAD: f32 = PI / 180.0;
    const RAD2DEG: f32 = 180.0 / PI;

    // ---- Arming state machine ----
    // TODO: re-enable GPS requirement before outdoor flight
    let mut arming = ArmingStateMachine::new();
    arming.require_gps = false; // bench mode — no GPS indoors

    // ---- MPC attitude outer loop (50 Hz) ----
    let mut mpc = AttitudeMpc::new();
    let mut rate_sp_degs = [0.0f32; 3]; // persisted between MPC solves

    // ---- Altitude hold (50 Hz) ----
    let hover_throttle: f32 = 0.294; // tune per aircraft: mass*g / max_thrust
    let alt_gains = AltitudeGains {
        kp: 0.15,
        kd: 0.1,
        ki: 0.05,
    };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let mut current_thrust = hover_throttle;

    // ---- PID rate inner loop (200 Hz) ----
    // Gains tuned for real hardware with motor lag (~30ms ESC+motor).
    // Adjust Kp/Ki/Kd during bench testing with props off first.
    let rate_gains = PidGains {
        kp: 0.02,
        ki: 0.005,
        kd: 0.001,
    };
    let yaw_gains = PidGains {
        kp: 0.03,
        ki: 0.005,
        kd: 0.0,
    };
    let limits = PidLimits {
        integral_max: 0.3,
        output_max: 0.5,
        d_lpf_tau_s: 0.008,
    };
    let mut rate_pid = RatePidController::new(rate_gains, rate_gains, yaw_gains, limits);

    // ---- Sensor state ----
    let mut last_rc = RcChannels {
        channels: [992; 16],
    };
    let mut last_imu = ImuData::new();
    let mut last_gps = GpsData::new();
    let mut last_pos_est: Option<PosEstimate> = None;
    let mut imu_last_seen = Instant::now();
    let mut control_demand = ControlDemand::default();
    let mut last_armed = false;

    // ---- Loop timing instrumentation ----
    // Tracks how long each control loop iteration takes.
    // If loop_time exceeds 5ms (200 Hz budget), we're overrunning.
    let mut loop_time_us_max: u32 = 0;
    let mut loop_time_us_sum: u32 = 0;
    let mut mpc_time_us_max: u32 = 0;
    let mut mpc_time_us_last: u32 = 0;
    let mut mpc_iters_last: u32 = 0;
    let mut mpc_iters_max: u32 = 0;
    let mut overrun_count: u32 = 0;
    let mut timing_sample_count: u32 = 0;

    // ---- Main loop: 200 Hz ----
    let mut ticker = Ticker::every(Duration::from_millis(5));
    let mut cycle_count: u32 = 0;
    let dt: f32 = 0.005; // 200 Hz

    // ---- MPC warm-up ----
    // Run one throwaway solve so the first in-flight solve (on arm)
    // isn't a cold-start. This surfaces any init-time crashes here
    // on the bench instead of at the worst possible moment, and lets
    // `mpc_max` start counting from a representative value instead of
    // a pathological first-call spike.
    {
        let warmup_start = Instant::now();
        mpc.set_reference([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let _ = mpc.solve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        mpc.reset();
        let warmup_us = warmup_start.elapsed().as_micros() as u32;
        defmt::info!("MPC warm-up solve completed in {}us", warmup_us);
    }

    defmt::info!("Control loop starting at 200 Hz (5ms budget)");

    loop {
        ticker.next().await;
        cycle_count = cycle_count.wrapping_add(1);
        let loop_start = Instant::now();

        // ---- 1. Read latest sensor data ----
        if let Some(imu) = IMU_DATA.try_take() {
            last_imu = imu;
            imu_last_seen = Instant::now();
        }
        if let Some(gps) = GPS_DATA.try_take() {
            last_gps = gps;
        }
        if let Some(est) = POS_ESTIMATE.try_take() {
            last_pos_est = Some(est);
        }
        if let Some(rc) = rc_task::RC_CHANNELS.try_take() {
            last_rc = rc;
        }

        // ---- 2. Arming state machine ----
        let arm_switch = last_rc.channels[4] > 1500;
        let throttle_raw = RcChannels::to_unit(last_rc.channels[2]);
        let imu_age_ms = imu_last_seen.elapsed().as_millis() as u32;
        let rc_age_ms = rc_task::rc_last_seen_ms();
        // GPS-home is a hard arm-time gate (see arming.rs comments).
        // Before the PosKF has published anything, treat as not-ready.
        let gps_home_ready = last_pos_est.map(|e| e.home_latched).unwrap_or(false);

        let arm_state = arming.update(
            arm_switch,
            throttle_raw,
            last_imu.angle[0], // roll deg
            last_imu.angle[1], // pitch deg
            imu_age_ms,
            rc_age_ms,
            gps_home_ready,
        );
        let armed = arm_state == ArmState::Armed;
        if armed && !last_armed {
            let pos_ready = last_pos_est.map(|e| e.ready).unwrap_or(false);
            if !pos_ready {
                defmt::warn!(
                    "ARMED without PosKF lock — MANUAL THROTTLE pass-through (no altitude hold)"
                );
            }
        }
        last_armed = armed;

        // Bench diagnostic: if the switch is high but we're still disarmed,
        // report which pre-arm check(s) failed at 1 Hz so the user can see
        // the blocker without guessing.
        if arm_switch && !armed && cycle_count % 200 == 0 {
            let c = arming.run_checks(
                throttle_raw,
                last_imu.angle[0],
                last_imu.angle[1],
                imu_age_ms,
                rc_age_ms,
                gps_home_ready,
            );
            defmt::info!(
                "arm rejected: thr_low={} level={} imu={} rc={} gps={} | thr={}% roll={}° pitch={}° imu_age={}ms rc_age={}ms ch4={} ch5={}",
                c.throttle_low,
                c.attitude_level,
                c.imu_fresh,
                c.rc_link_active,
                c.gps_home_ready,
                (throttle_raw * 100.0) as i32,
                last_imu.angle[0] as i32,
                last_imu.angle[1] as i32,
                imu_age_ms,
                rc_age_ms,
                last_rc.channels[4],
                last_rc.channels[5],
            );
        }



        // ---- 3. Control computation ----
        if armed {
            // RC stick → desired attitude
            let max_angle: f32 = 30.0;
            let roll_input = RcChannels::to_normalised(last_rc.channels[0]);
            let pitch_input = RcChannels::to_normalised(last_rc.channels[1]);
            let yaw_input = RcChannels::to_normalised(last_rc.channels[3]);

            let desired_roll_rad = roll_input * max_angle * DEG2RAD;
            let desired_pitch_rad = pitch_input * max_angle * DEG2RAD;
            let desired_yaw_rad = 0.0; // yaw hold; yaw rate from stick handled separately

            // ---- 50 Hz outer loops (every 4th cycle) ----
            if cycle_count % 4 == 0 {
                // MPC attitude: set reference from RC sticks
                mpc.set_reference(
                    [desired_roll_rad, desired_pitch_rad, desired_yaw_rad],
                    [0.0, 0.0, yaw_input * 200.0 * DEG2RAD], // yaw rate setpoint
                );

                let angles_rad = [
                    last_imu.angle[0] * DEG2RAD,
                    last_imu.angle[1] * DEG2RAD,
                    last_imu.angle[2] * DEG2RAD,
                ];
                let rates_rad = [
                    last_imu.gyro[0] * DEG2RAD,
                    last_imu.gyro[1] * DEG2RAD,
                    last_imu.gyro[2] * DEG2RAD,
                ];

                let mpc_start = Instant::now();
                let mpc_out = mpc.solve(angles_rad, rates_rad);
                mpc_time_us_last = mpc_start.elapsed().as_micros() as u32;
                if mpc_time_us_last > mpc_time_us_max {
                    mpc_time_us_max = mpc_time_us_last;
                }
                mpc_iters_last = mpc_out.iterations as u32;
                if mpc_iters_last > mpc_iters_max {
                    mpc_iters_max = mpc_iters_last;
                }

                rate_sp_degs = [
                    mpc_out.rate_setpoints_rads[0] * RAD2DEG,
                    mpc_out.rate_setpoints_rads[1] * RAD2DEG,
                    mpc_out.rate_setpoints_rads[2] * RAD2DEG,
                ];

                // Altitude hold — closes on the PosKF estimate (baro-fused
                // today, baro+GPS once `update_gps` is wired in Phase 4c).
                // Gated on `ready`: the p_ref latch takes ~1 s from first
                // baro sample, and closing the loop before then would
                // chase garbage. With no PosKF lock (bench / GPS-denied
                // arm), fall through to direct stick → thrust so motor
                // bring-up can verify throttle response without GPS.
                if let Some(est) = last_pos_est.filter(|e| e.ready) {
                    let target_alt = est.altitude_up; // hold current altitude
                    current_thrust =
                        alt_ctrl.update(target_alt, est.altitude_up, est.vz_up, dt * 4.0);
                } else {
                    current_thrust = throttle_raw.clamp(0.0, 1.0);
                }
            }

            // ---- 200 Hz PID rate inner loop ----
            let pid_output = rate_pid.update(rate_sp_degs, last_imu.gyro, dt);

            control_demand = ControlDemand {
                thrust: current_thrust,
                roll: pid_output[0],
                pitch: pid_output[1],
                yaw: pid_output[2],
            };
        } else {
            // Disarmed — zero everything, reset controllers
            control_demand = ControlDemand::default();
            rate_pid.reset();
            mpc.reset();
            alt_ctrl.reset();
            rate_sp_degs = [0.0; 3];
            current_thrust = hover_throttle;
        }

        // ---- 4. Mixer ----
        let motor_outputs = QUAD_X.apply(&control_demand);

        // ---- 5. DShot output ----
        let mut frames: [EncodedFrame; 4] = [DshotTx::standard().command(Command::MotorStop); 4];
        if armed {
            for i in 0..4 {
                let v = motor_outputs.motors[i];
                if v <= 0.0 {
                    frames[i] = DshotTx::standard().command(Command::MotorStop);
                } else {
                    let throttle = (v * 1999.0) as u16;
                    frames[i] = DshotTx::standard().throttle_clamped(throttle);
                }
            }
        }
        dshot.send(frames).await;

        // ---- 6. Loop timing ----
        let loop_us = loop_start.elapsed().as_micros() as u32;
        loop_time_us_sum += loop_us;
        timing_sample_count += 1;
        if loop_us > loop_time_us_max {
            loop_time_us_max = loop_us;
        }
        if loop_us > 5000 {
            overrun_count += 1;
        }

        // ---- 7. Telemetry (2 Hz) ----
        if cycle_count % 100 == 0 {
            let loop_avg = if timing_sample_count > 0 {
                loop_time_us_sum / timing_sample_count
            } else {
                0
            };

            // stick_thr    = raw RC stick throttle input (0-100%)
            // thrust_cmd   = altitude controller's current thrust command (0-100%);
            //                while disarmed this is held at `hover_throttle`, so
            //                seeing a non-zero value with armed=false is expected.
            let stick_thr_pct = (throttle_raw * 100.0) as u8;
            let thrust_cmd_pct = (current_thrust * 100.0) as u8;
            defmt::info!(
                "armed={} roll={:?}° pitch={:?}° yaw={:?}° stick_thr={}% thrust_cmd={}% sats={}",
                armed,
                last_imu.angle[0],
                last_imu.angle[1],
                last_imu.angle[2],
                stick_thr_pct,
                thrust_cmd_pct,
                last_gps.satellites,
            );
            defmt::info!(
                "loop: avg={}us max={}us mpc_max={}us mpc_last={}us mpc_iters={}/{} overruns={}",
                loop_avg,
                loop_time_us_max,
                mpc_time_us_max,
                mpc_time_us_last,
                mpc_iters_last,
                mpc_iters_max,
                overrun_count,
            );

            // Reset timing stats each reporting period
            loop_time_us_max = 0;
            loop_time_us_sum = 0;
            mpc_time_us_max = 0;
            mpc_iters_max = 0;
            timing_sample_count = 0;
        }
    }
}
