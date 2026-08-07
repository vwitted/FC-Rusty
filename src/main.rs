// main.rs — Flight controller entry point
//
// Target: STM32H743VIT6 (480 MHz, Cortex-M7F, 100-pin LQFP)
// Board:  DAKEFPVH743 (STM32H743)
//         5V BEC during IMU rework — see git log for context)
// Framework: Embassy async executor
//
// Pin map (DAKEFPVH743 — matches board connector labels):
//   USART1  TX=PA9   RX=PA10    → T1/R1 pads, GPS (SERIAL1, NMEA 9600)
//   USART2  (reserved)          → T2/R2 pads, available (SERIAL2)
//   USART3  (reserved)          → T3/R3 pads, ESC telem (SERIAL3, free)
//   UART4   TX=PD1   RX=PD0    → T4/R4, DisplayPort/VTX (SERIAL4, free)
//   UART5   RX=PB5              → R5 pad, CRSF receiver (SERIAL5, 416666)
//   USART6  TX=PC6              → T6 pad, defmt logger (SERIAL6, 115200)
//   UART7   (available)         → T7/R7 pads, user/GP (SERIAL7, free)
//   UART8   (available)         → T8/R8 pads, user/GP (SERIAL8, free)
//
// The onboard dual ICM-42688P sensors are used as the IMU:
//   IMU1 on SPI1 (SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4)  — ROTATION_ROLL_180
//   IMU2 on SPI4 (SCK=PE12, MISO=PE13, MOSI=PE14, CS=PB1) — ROTATION_PITCH_180
// Both are read at 8 kHz and averaged for √2 noise reduction.
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
// Under `motor-test` the entire flight stack is cfg'd out (the flight `main`
// is gated off), so its tasks, helpers, and imports are intentionally unused.
// Silence that noise for the bench build only; the flight build is unaffected.
#![cfg_attr(feature = "motor-test", allow(unused))]

use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Uart, UartRx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};

use panic_probe as _; // panic handler that works with probe

// Our modules
mod drivers {
    pub mod baro;
    pub mod crsf;
    pub mod dshot_bb_decode;
    pub mod dshot_bb_frame;
    pub mod dshot_bitbang;
    pub mod dshot_diag;
    pub mod dshot_frame;
    pub mod dshot_hw;
    pub mod dshot_telemetry;
    pub mod icm42688;
    pub mod ism6hg256x;
    pub mod lis2mdl;
    pub mod nmea;
    pub mod ubx;
    pub mod wt901b;
}
mod control {
    pub mod altitude;
    pub mod arm_origin;
    pub mod arming;
    pub mod cal_led;
    pub mod mag_cal;
    pub mod mixer;
    pub mod mpc;
    pub mod pid;
    pub mod position;
}
mod attitude_mekf;
mod estimation;
mod imu_filter;
mod logger;
mod rc_task;

#[cfg(feature = "motor-test")]
mod motor_test;

mod persist {
    pub mod record;
    pub mod flash;
}

use attitude_mekf::{AttitudeMekf, G_MPS2, MekfParams};
use imu_filter::{ImuFilter, ImuFilterParams};
use control::altitude::{AltitudeController, AltitudeGains};
use control::arming::{ArmState, ArmingStateMachine};
use control::arm_origin::ArmOriginSync;
use control::cal_led::{led_on, CalLed};
use control::mag_cal::{CalCommand, MagCalibrator, DECLINATION_DEG};
use control::mixer::{AirmodeGate, ControlDemand, QUAD_X};
use control::mpc::{AttitudeMpc, MPC_DT, MPC_PERIOD_US};
use control::pid::{PidGains, PidLimits, RatePidController};
use control::position::{PositionController, PositionGains};
use drivers::baro::{self, BaroSample};
use drivers::crsf::RcChannels;
use drivers::lis2mdl::{Lis2mdl, MagSample, Orientation as MagOrientation};
use drivers::dshot_frame::{DshotFrame, DshotSpeed};
use drivers::dshot_hw::DshotQuad;
use drivers::icm42688::RawImu;
use drivers::nmea::{FixMode, GpsData, NmeaParser};
use drivers::wt901b::{
    ImuData, UPDATED_ACCEL, UPDATED_ANGLE, UPDATED_GYRO, UPDATED_QUAT,
};
use estimation::{geodetic_to_local_ned, PosKf};

// ---- Flight modes ----

/// Active flight mode — determines how RC sticks, the position
/// controller, and the altitude controller interact in the control
/// loop. Selected by RC channel 5 (3-position mode switch), channel 6
/// (GPS rescue override), or the arming FSM’s failsafe flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
enum FlightMode {
    /// Sticks → angle (MPC) → rate (PID). Direct throttle.
    Acro,
    /// Sticks → angle (MPC) → rate (PID). Altitude held by
    /// controller; throttle stick commands climb/descend rate.
    AltHold,
    /// Altitude + horizontal position held. Sticks command velocity.
    /// Without GPS home, horizontal hold is best-effort dead-reckoning
    /// damped by NMEA velocity fusion — drift accumulates linearly
    /// rather than quadratically and is acceptable for tens of seconds.
    PosHold,
    /// Autonomous return to GPS home at a safe altitude. Triggered by switch.
    GpsHome,
    /// Failsafe mode (RC lost, GPS home available): hover at home.
    GpsRescue,
    /// Failsafe mode (RC lost, no GPS home, baro alive): closed-loop
    /// controlled descent. Level attitude, alt-hold target ramps down,
    /// auto-disarm at low altitude or 30-s timeout.
    FailsafeLand,
    /// Failsafe mode (RC lost, no altitude reference at all): open-loop
    /// blind descent. Level attitude, fixed throttle slightly below
    /// hover, auto-disarm at 30-s timeout.
    FailsafeBlind,
}

// ---- GPS rescue parameters ----
/// Altitude to climb to during GPS rescue (metres, positive-up).
const RESCUE_ALT_M: f32 = 50.0;
/// Horizontal distance to home at which we consider “arrived” (metres).
const RESCUE_ARRIVAL_RADIUS_M: f32 = 5.0;
/// Descent rate during auto-land phase (m/s, positive value).
const RESCUE_LAND_RATE_MPS: f32 = 0.5;
/// Time loitering at home before auto-landing if RC stays lost (seconds).
const RESCUE_LAND_TIMEOUT_S: f32 = 30.0;
/// Altitude below which auto-land disarms (metres, positive-up).
const RESCUE_DISARM_ALT_M: f32 = 1.0;
/// Throttle stick deadband for alt-hold climb/descend (0–1 normalised).
const ALT_HOLD_DEADBAND: f32 = 0.05;
/// Max climb/descend rate when throttle stick is fully deflected (m/s).
const ALT_HOLD_MAX_RATE_MPS: f32 = 2.0;
/// Max velocity when sticks are fully deflected in PosHold (m/s).
const POS_HOLD_MAX_VEL_MPS: f32 = 5.0;

// ---- Failsafe descent (no GPS home) ----
/// Descent rate during FailsafeLand (closed-loop, baro-driven), m/s.
const FAILSAFE_DESCENT_RATE_MPS: f32 = 0.7;
/// Altitude (above arm reference) below which FailsafeLand auto-disarms.
const FAILSAFE_LAND_DISARM_ALT_M: f32 = 0.3;
/// Open-loop throttle for FailsafeBlind, expressed as a fraction of
/// hover_throttle (so 0.9 means "10 % below hover" → gentle descent).
/// FailsafeBlind has no auto-disarm — without altitude data we
/// can't tell when to cut motors. The descent runs until the pilot
/// regains RC or the battery cuts. Impact-signature-based disarm is
/// a Beta-target backlog item (see PROJECT_STATUS.md).
const FAILSAFE_BLIND_THROTTLE_FRAC: f32 = 0.9;

// ---- DShot configuration ----
/// Set `true` for bidir DShot (line idles HIGH, telemetry-request bit
/// set, inverted CRC, and an input-capture RX phase after each frame).
/// Must match the ESC's EEPROM config from BLHeli/BlueJay/AM32. When
/// false the driver runs the plain non-bidir TX-only path.
const DSHOT_BIDIR: bool = true;

// ---- Interrupt bindings ----

// USART6 is owned by `logger::init_usart6()` (raw register TX for defmt)
// — no Embassy interrupt handler needed here.
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    UART5 => usart::InterruptHandler<peripherals::UART5>;
});

// ---- Shared state between tasks ----
// Signals are "latest value wins" — perfect for real-time sensor data.

/// Latest IMU data from the WT901B task (fused angles + raw rates).
static IMU_DATA: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Latest raw samples from the ICM-42688P dual-IMU task (body-frame NED,
/// averaged across both sensors when both are online).
/// Consumed at ~8 kHz by the MEKF attitude filter.
static RAW_IMU: Signal<CriticalSectionRawMutex, RawImu> = Signal::new();

/// Dedicated signal for the navigation task so it doesn't steal IMU_DATA from the fast inner loop.
static IMU_DATA_FOR_NAV: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Command output from the 50 Hz navigation outer loop, read by the 8 kHz fast inner loop.
#[derive(Clone, Copy)]
pub struct OuterLoopCommand {
    pub thrust: f32,
    pub rate_sp_degs: [f32; 3],
    pub armed: bool,
}

static OUTER_CMD: Watch<CriticalSectionRawMutex, OuterLoopCommand, 2> = Watch::new();


/// Counters for the ICM monitor task. Live regardless of whether the
/// read task is making progress, so a silent INT pin still shows up
/// as `0 samples/s, 0 errors/s` instead of no log at all.
static ICM_SAMPLES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static ICM_ERRORS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Per-sensor diagnostic snapshots. Updated at ~1 Hz by the dual read
/// task so the monitor can log individual IMU1 / IMU2 values alongside
/// the fused output for cross-sensor validation.
#[derive(Clone, Copy, Debug, defmt::Format)]
struct ImuDiag {
    /// Body-frame accel (g) from IMU1
    a1: [f32; 3],
    /// Body-frame accel (g) from IMU2
    a2: [f32; 3],
    /// Body-frame gyro (dps) from IMU1
    g1: [f32; 3],
    /// Body-frame gyro (dps) from IMU2
    g2: [f32; 3],
    /// Fused accel (g)
    a_fused: [f32; 3],
    /// Fused gyro (dps)
    g_fused: [f32; 3],
    /// Temp °C from IMU1 / IMU2
    t1: f32,
    t2: f32,
}
static IMU_DIAG: Signal<CriticalSectionRawMutex, ImuDiag> = Signal::new();

/// Latest GPS data from the GPS task
static GPS_DATA: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

/// Parallel GPS signal dedicated to `pos_kf_task`. The NMEA task
/// publishes to both so the 200 Hz control loop and the KF task don't
/// race each other for the same single-shot sample.
static GPS_DATA_FOR_KF: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

/// Latest baro sample (pressure_pa + temperature_c) from the baro task.
/// Consumed by `pos_kf_task` to drive the position-KF `update_baro`.
static BARO_DATA: Signal<CriticalSectionRawMutex, BaroSample> = Signal::new();

/// Latest LIS2MDL magnetometer sample (body-frame, oriented). Signalled
/// by the baro task at 100 Hz (the chip's continuous-mode ODR — the
/// LIS2MDL shares the I2C1 bus with the baro and is owned by the same
/// task so no bus arbitration is needed). Consumed by `mekf_task`
/// where it feeds `AttitudeMekf::update_mag`, making yaw observable.
/// Absent when the breakout failed to init — `mekf_task` simply runs
/// without the mag update branch in that case.
static MAG_DATA: Signal<CriticalSectionRawMutex, MagSample> = Signal::new();

/// Latest fused IMU data intended for the position KF (*separate* from
/// `IMU_DATA`, which the control loop already consumes). Signalled by
/// `mekf_task` on its 100 Hz accel-update branch — paying one extra
/// signal-write every 80th sample is cheap and avoids consumer
/// contention with the control loop on `IMU_DATA`.
static IMU_DATA_FOR_KF: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Fused position / velocity estimate published by `pos_kf_task`.
#[derive(Clone, Copy, Debug, defmt::Format)]
struct PosEstimate {
    /// Position in NED world frame (m), relative to the GPS home point
    /// once the GPS home latch completes. Before home latch the
    /// horizontal components are dead-reckoned and only the altitude
    /// channel is meaningful.
    position_ned: [f32; 3],
    /// Velocity in NED world frame (m/s).
    velocity_ned: [f32; 3],
    /// Altitude (m, positive up). Reference is whichever sensor latched
    /// first: pressure altitude relative to arm (baro present) or
    /// height above GPS-home origin (GPS only).
    altitude_up: f32,
    /// Vertical velocity (m/s, positive up).
    vz_up: f32,
    /// Reference pressure latched at arm time (Pa). 0.0 if baro is
    /// absent or arm has not yet fired.
    p_ref_pa: f32,
    /// Milliseconds since the last baro update was applied.
    baro_age_ms: u32,
    /// True once at least one altitude sensor is anchored:
    ///   - baro: p_ref latched at arm and at least one fuse fired, OR
    ///   - GPS:  home latched and fixes are fusing.
    /// Altitude-hold and any throttle controller that consumes
    /// `altitude_up` / `vz_up` must gate on this.
    altitude_ready: bool,
    /// True once a sufficiently good GPS fix has been captured as the
    /// home origin. Horizontal `position_ned` is meaningful only when
    /// this is true; any GPS-rescue or position-hold behaviour must
    /// gate on it.
    home_latched: bool,
    /// Monotonic counter incremented each time the PosKF consumes the arm
    /// latch (re-anchors origins + zeros state). The navigation task uses
    /// it (via `ArmOriginSync`) to withhold target capture until the
    /// arm-time re-origin has actually landed — otherwise it would capture
    /// stale pre-zero targets and lurch on arm.
    arm_origin_seq: u32,
}

static POS_ESTIMATE: Signal<CriticalSectionRawMutex, PosEstimate> = Signal::new();

/// Fired by `navigation_task` on the Disarmed→Armed transition. The
/// `pos_kf_task` consumes this to latch the baro reference pressure
/// against the current sample (so `altitude_up` reads ~0 at arm) and
/// to zero the KF's vertical state. Position-NED is left untouched so
/// any GPS-home anchoring is preserved.
static ARM_LATCH: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Magnetometer-cal lifecycle: navigation task → mekf task.
static CAL_CONTROL: Signal<CriticalSectionRawMutex, CalCommand> = Signal::new();
/// Completed cal to persist (disarmed): mekf task → persist task.
static CAL_SAVE: Signal<CriticalSectionRawMutex, persist::record::Config> = Signal::new();
/// Trusted true heading from GPS COG (rad): navigation task → mekf task.
static YAW_COG: Signal<CriticalSectionRawMutex, f32> = Signal::new();
/// Boot-loaded calibration: main → mekf task.
static STORED_CAL: Signal<CriticalSectionRawMutex, persist::record::Config> = Signal::new();
/// Cal-feedback LED phase: mekf task → blink task. Watch so the renderer
/// can poll the current phase every tick without consuming it.
static CAL_LED: Watch<CriticalSectionRawMutex, CalLed, 2> = Watch::new();



// RC signals are defined in rc_task.rs:
// rc_task::RC_CHANNELS, rc_task::RC_LINK, rc_task::RC_LAST_SEEN

// ---- Main entry point ----

/// STM32H743 clock tree for the DAKEFPV (8 MHz HSE → 480 MHz SYSCLK).
/// Shared by the flight and motor-test entry points so the clock config
/// can never drift between them.
fn board_config() -> embassy_stm32::Config {
    //   HSE 8 MHz → PLL_N=120 → VCO 960 MHz; PLL_P=2 → 480 MHz SYSCLK;
    //   PLL_Q=20 → 48 MHz USB. AHB 240 MHz; APB* 120 MHz (timers 240 MHz).
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
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL120,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV20),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config
}

/// Bench motor-test entry point — drives DShot directly, no flight stack.
#[cfg(feature = "motor-test")]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(board_config());

    // D-cache off for DMA coherency (DShot is DMA-driven).
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.SCB.disable_dcache(&mut core.CPUID);

    // defmt over USART6 (PC6), same as the flight path.
    logger::init_usart6();

    motor_test::run(p).await;
}

#[cfg(not(feature = "motor-test"))]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(board_config());

    // Disable D-cache as early as possible
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.SCB.disable_dcache(&mut core.CPUID);

    // Bring up USART6 TX (PC6) for defmt output before anything else
    // so the first defmt::info! below actually lands on the wire.
    logger::init_usart6();

    // Load persisted config (sub-project A). None ⇒ uncalibrated defaults,
    // identical to prior behaviour. Read once here, before control loops.
    let mut cfg_flash = persist::flash::driver(p.FLASH);
    let config = persist::flash::read(&mut cfg_flash).unwrap_or_default();
    defmt::info!(
        "persist: mag_calibrated={} decl={=f32}rad hard_iron=[{=f32},{=f32},{=f32}]",
        config.mag_calibrated,
        config.declination_rad,
        config.mag_hard_iron_ut[0],
        config.mag_hard_iron_ut[1],
        config.mag_hard_iron_ut[2],
    );
    #[cfg(feature = "persist-selftest")]
    {
        // Two-boot protocol:
        //   Boot 1 (blank sector): `config` read above is the default
        //   (mag_calibrated=false). We then write the marker below.
        //   Boot 2 (after power-cycle): the read above returns the marker
        //   (mag_calibrated=true, decl=0.1234), proving persistence.
        if config.mag_calibrated && (config.declination_rad - 0.1234).abs() < 1e-4 {
            defmt::info!("persist-selftest: PASS — marker survived reboot");
        } else {
            defmt::warn!("persist-selftest: no marker yet — writing it now, power-cycle to verify");
            let marker = persist::record::Config {
                mag_hard_iron_ut: [1.0, 2.0, 3.0],
                declination_rad: 0.1234,
                mag_calibrated: true,
            };
            match persist::flash::write(&mut cfg_flash, &marker) {
                Ok(()) => defmt::info!("persist-selftest: marker written OK"),
                Err(e) => defmt::error!("persist-selftest: write failed {:?}", e),
            }
        }
    }

    // Hand the boot calibration to the MEKF and the flash handle to the
    // persist task (which writes future cals while disarmed).
    STORED_CAL.signal(config);
    spawner.spawn(persist_task(cfg_flash)).unwrap();

    // ---- Status LED Heartbeat ----
    // DAKEFPVH743 has LED0 on PD10 (active low). We spawn a quick blink
    // task so we have an immediate visual indicator that the firmware
    // has booted and is running, without needing to check the UART logs.
    use embassy_stm32::gpio::{Level, Output, Speed};
    let led = Output::new(p.PD10, Level::High, Speed::Low);
    spawner.spawn(blink_task(led)).unwrap();

    defmt::info!("Flight controller starting");

    // ---- Configure and spawn the RC receiver task ----
    // CRSF on UART5 RX (PB5), 416666 baud — R5 pad on the DAKEFPVH743
    // (SERIAL5, the board's dedicated RC input port).
    let rc_uart = UartRx::new(
        p.UART5,
        Irqs,
        p.PB5,      // RX pin (UART5)
        p.DMA1_CH5, // UART5_RX DMA
        rc_task::crsf_uart_config(),
    )
    .unwrap();

    spawner.spawn(rc_task::run(rc_uart)).unwrap();
    defmt::info!("RC task spawned");



    // ---- Dual ICM-42688P IMUs (timer-polled 8 kHz reads) ----
    // IMU1 on SPI1: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4  (ROTATION_ROLL_180)
    // IMU2 on SPI4: SCK=PE12, MISO=PE13, MOSI=PE14, CS=PB1 (ROTATION_PITCH_180)
    // Both sensors are read back-to-back and averaged for √2 noise reduction.
    // No EXTI pins are mapped on this board, so we use an 8 kHz ticker.
    {
        use drivers::icm42688::Orientation;
        use embassy_stm32::gpio::{Level, Output, Speed};
        use embassy_stm32::spi::{Config as SpiConfig, Spi};
        use embassy_stm32::time::Hertz;

        let mut spi_cfg = SpiConfig::default();
        spi_cfg.frequency = Hertz(10_000_000);

        // ---- IMU1 (SPI1) ----
        let spi1 = Spi::new(
            p.SPI1, p.PA5,      // SCK
            p.PA7,      // MOSI
            p.PA6,      // MISO
            p.DMA2_CH3, // SPI1_TX
            p.DMA2_CH0, // SPI1_RX
            spi_cfg,
        );
        let cs1 = Output::new(p.PA4, Level::High, Speed::VeryHigh);

        match drivers::icm42688::Icm42688::new(spi1, cs1, Orientation::Roll180).await {
            Ok(imu1) => {
                defmt::info!("ICM-42688P IMU1 (SPI1, Roll180) initialised OK");

                // ---- IMU2 (SPI4) ----
                let spi4 = Spi::new(
                    p.SPI4, p.PE12,     // SCK
                    p.PE14,     // MOSI
                    p.PE13,     // MISO
                    p.DMA1_CH0, // SPI4_TX
                    p.DMA1_CH1, // SPI4_RX
                    spi_cfg,
                );
                let cs2 = Output::new(p.PB1, Level::High, Speed::VeryHigh);

                match drivers::icm42688::Icm42688::new(spi4, cs2, Orientation::Pitch180).await {
                    Ok(imu2) => {
                        defmt::info!("ICM-42688P IMU2 (SPI4, Pitch180) initialised OK");
                        spawner.spawn(dual_icm_read_task(imu1, imu2)).unwrap();
                    }
                    Err(e) => {
                        defmt::warn!("ICM-42688P IMU2 init failed: {:?} — running single-IMU", e);
                        spawner.spawn(single_icm_read_task(imu1)).unwrap();
                    }
                }

                spawner.spawn(icm_monitor_task()).unwrap();
                spawner.spawn(mekf_task()).unwrap();
            }
            Err(e) => {
                defmt::error!("ICM-42688P IMU1 init failed: {:?} — NO IMU AVAILABLE", e);
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
    // GPS on USART1 TX=PA9 RX=PA10 at factory 9600 baud, speaking
    // factory-default NMEA. SERIAL1 / T1+R1 pads — the board's
    // dedicated GPS connector. See `drivers::nmea`.
    let gps_uart_config = {
        let mut c = usart::Config::default();
        c.baudrate = 9600;
        c
    };

    let gps_uart = Uart::new(
        p.USART1,
        p.PA10, // RX
        p.PA9,  // TX
        Irqs,
        p.DMA2_CH6, // TX DMA
        p.DMA2_CH1, // RX DMA
        gps_uart_config,
    )
    .unwrap();

    let (_gps_tx, gps_rx) = gps_uart.split();
    spawner.spawn(gps_task(gps_rx)).unwrap();
    defmt::info!("GPS task spawned (NMEA at 9600)");

    // DShot ESC outputs (all 4 channels on TIM2).
    //   TIM2 CH1 → PA0 → M1 (DMA1_CH2)
    //   TIM2 CH2 → PA1 → M2 (DMA1_CH3)
    //   TIM2 CH3 → PA2 → M3 (DMA1_CH4)
    //   TIM2 CH4 → PA3 → M4 (DMA1_CH7)
    // Per-channel CC DMA (BF-style port), four DMA1 streams.
    let dshot = DshotQuad::new(
        p.TIM2,
        p.PA0,      // CH1 → M1
        p.PA1,      // CH2 → M2
        p.PA2,      // CH3 → M3
        p.PA3,      // CH4 → M4
        p.DMA1_CH2, // TIM2_CH1 → M1
        p.DMA1_CH3, // TIM2_CH2 → M2
        p.DMA1_CH4, // TIM2_CH3 → M3
        p.DMA1_CH7, // TIM2_CH4 → M4
        DshotSpeed::Dshot600,
        DSHOT_BIDIR,
    );

    dshot.log_config();
    defmt::info!(
        "DShot initialised on TIM2 (4 per-channel CC DMA streams, bidir={=bool})",
        DSHOT_BIDIR
    );

    // ---- Run the 50 Hz outer loop as a spawned task ----
    spawner.spawn(navigation_task()).unwrap();

    // ---- Run the fast 8 kHz inner loop on the main task ----
    // This is deliberate: the control loop is the highest priority
    // work, so it runs on the main executor rather than being
    // spawned as a separate task.
    control_loop(dshot).await;
}

// ---- Heartbeat Task ----

#[embassy_executor::task]
async fn blink_task(mut led: embassy_stm32::gpio::Output<'static>) {
    let mut rx = CAL_LED.receiver().unwrap();
    let mut phase = CalLed::Idle;
    let mut phase_start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_millis(25));
    loop {
        if let Some(new_phase) = rx.try_get() {
            // Reset the pattern clock only on a *variant* change, so
            // Calibrating(p) progress updates don't restart the blink.
            if core::mem::discriminant(&new_phase) != core::mem::discriminant(&phase) {
                phase_start = Instant::now();
            }
            phase = new_phase;
        }
        let elapsed = phase_start.elapsed().as_millis() as u32;
        if led_on(phase, elapsed) {
            led.set_low(); // active-low: ON
        } else {
            led.set_high(); // OFF
        }
        ticker.next().await;
    }
}

// ---- Persist Task ----
// Owns the flash handle; writes a completed magnetometer calibration to
// the persist store. Triggered only by cal completion, which is
// disarmed-only — so the multi-second sector erase happens on the ground.
#[embassy_executor::task]
async fn persist_task(
    mut flash: embassy_stm32::flash::Flash<'static, embassy_stm32::flash::Blocking>,
) {
    loop {
        let cfg = CAL_SAVE.wait().await;
        match persist::flash::write(&mut flash, &cfg) {
            Ok(()) => defmt::info!("persist: CAL SAVED to flash"),
            Err(e) => defmt::error!("persist: cal save failed {:?}", e),
        }
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

// ---- Dual ICM-42688P Read Task (timer-polled 8 kHz) ----
// Reads both IMUs back-to-back on each tick, averages the body-frame
// outputs, and publishes the fused RawImu to RAW_IMU. The MEKF task
// consumes RAW_IMU unchanged.
//
// Sequential reads take ~40 µs total (2 × 15-byte SPI @ 10 MHz + CS
// toggling), well within the 125 µs budget at 8 kHz.

#[embassy_executor::task]
async fn dual_icm_read_task(
    mut imu1: drivers::icm42688::Icm42688<'static>,
    mut imu2: drivers::icm42688::Icm42688<'static>,
) {
    use core::sync::atomic::Ordering;
    defmt::info!("Dual ICM read task started (8 kHz ticker)");

    let mut ticker = Ticker::every(Duration::from_micros(125));
    let mut last_diag = Instant::now();

    // Software LPF on the fused IMU stream. Filters the post-average
    // signal once with one identical chain rather than relying on the
    // (different) on-chip filters of the MPU6000 and ICM-42688P.
    // Defaults are 150 Hz gyro / 25 Hz accel — see `imu_filter.rs` for
    // the design notes. Primed lazily on the first successful sample
    // so the consumer doesn't see a startup ramp from zero.
    let mut filter = ImuFilter::new(ImuFilterParams::default());
    let mut filter_primed = false;

    loop {
        ticker.next().await;

        let r1 = imu1.read_raw().await;
        let r2 = imu2.read_raw().await;

        match (r1, r2) {
            (Ok(a), Ok(b)) => {
                let fused = drivers::icm42688::RawImu::averaged(&a, &b);
                if !filter_primed {
                    filter.prime(fused.accel, fused.gyro);
                    filter_primed = true;
                }
                let (a_filt, g_filt) = filter.apply(fused.accel, fused.gyro);
                let filtered = drivers::icm42688::RawImu {
                    accel: a_filt,
                    gyro: g_filt,
                    temp: fused.temp,
                    orientation: fused.orientation,
                };
                RAW_IMU.signal(filtered);
                ICM_SAMPLES.fetch_add(1, Ordering::Relaxed);

                // Snapshot per-sensor diagnostics at ~1 Hz. The
                // a_fused / g_fused fields reflect the *filtered*
                // fused output (what the MEKF actually sees), while
                // a1 / a2 / g1 / g2 stay raw per-sensor.
                let now = Instant::now();
                if (now - last_diag) >= Duration::from_secs(1) {
                    IMU_DIAG.signal(ImuDiag {
                        a1: a.accel_g(),
                        a2: b.accel_g(),
                        g1: a.gyro_dps(),
                        g2: b.gyro_dps(),
                        a_fused: filtered.accel_g(),
                        g_fused: filtered.gyro_dps(),
                        t1: a.temp_c(),
                        t2: b.temp_c(),
                    });
                    last_diag = now;
                }
            }
            (Ok(a), Err(_)) => {
                // IMU2 read failed — use IMU1 only this cycle.
                // Still filter so the consumer sees a consistent
                // spectral response across dropouts.
                if !filter_primed {
                    filter.prime(a.accel, a.gyro);
                    filter_primed = true;
                }
                let (a_filt, g_filt) = filter.apply(a.accel, a.gyro);
                let filtered = drivers::icm42688::RawImu {
                    accel: a_filt,
                    gyro: g_filt,
                    temp: a.temp,
                    orientation: a.orientation,
                };
                RAW_IMU.signal(filtered);
                ICM_SAMPLES.fetch_add(1, Ordering::Relaxed);
                ICM_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
            (Err(_), Ok(b)) => {
                if !filter_primed {
                    filter.prime(b.accel, b.gyro);
                    filter_primed = true;
                }
                let (a_filt, g_filt) = filter.apply(b.accel, b.gyro);
                let filtered = drivers::icm42688::RawImu {
                    accel: a_filt,
                    gyro: g_filt,
                    temp: b.temp,
                    orientation: b.orientation,
                };
                RAW_IMU.signal(filtered);
                ICM_SAMPLES.fetch_add(1, Ordering::Relaxed);
                ICM_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
            (Err(_), Err(_)) => {
                ICM_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

// ---- Single ICM-42688P Read Task (fallback, timer-polled 8 kHz) ----
// Used when IMU2 fails to initialise. Identical to the dual task but
// reads only IMU1.

#[embassy_executor::task]
async fn single_icm_read_task(
    mut imu: drivers::icm42688::Icm42688<'static>,
) {
    use core::sync::atomic::Ordering;
    defmt::info!("Single ICM read task started (8 kHz ticker, IMU2 unavailable)");

    let mut ticker = Ticker::every(Duration::from_micros(125));
    // Same software LPF as the dual-IMU path; primed lazily.
    let mut filter = ImuFilter::new(ImuFilterParams::default());
    let mut filter_primed = false;

    loop {
        ticker.next().await;
        match imu.read_raw().await {
            Ok(r) => {
                if !filter_primed {
                    filter.prime(r.accel, r.gyro);
                    filter_primed = true;
                }
                let (a_filt, g_filt) = filter.apply(r.accel, r.gyro);
                let filtered = drivers::icm42688::RawImu {
                    accel: a_filt,
                    gyro: g_filt,
                    temp: r.temp,
                    orientation: r.orientation,
                };
                RAW_IMU.signal(filtered);
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

        // If a diagnostic snapshot is available, log per-sensor detail.
        if let Some(d) = IMU_DIAG.try_take() {
            // Max absolute accel disagreement across axes (g)
            let da = [
                libm::fabsf(d.a1[0] - d.a2[0]),
                libm::fabsf(d.a1[1] - d.a2[1]),
                libm::fabsf(d.a1[2] - d.a2[2]),
            ];
            let max_da = if da[0] > da[1] { if da[0] > da[2] { da[0] } else { da[2] } }
                         else { if da[1] > da[2] { da[1] } else { da[2] } };

            // Max absolute gyro disagreement across axes (dps)
            let dg = [
                libm::fabsf(d.g1[0] - d.g2[0]),
                libm::fabsf(d.g1[1] - d.g2[1]),
                libm::fabsf(d.g1[2] - d.g2[2]),
            ];
            let max_dg = if dg[0] > dg[1] { if dg[0] > dg[2] { dg[0] } else { dg[2] } }
                         else { if dg[1] > dg[2] { dg[1] } else { dg[2] } };

            defmt::info!(
                "IMU1 accel=[{=f32},{=f32},{=f32}]g  gyro=[{=f32},{=f32},{=f32}]dps  t={=f32}C",
                d.a1[0], d.a1[1], d.a1[2],
                d.g1[0], d.g1[1], d.g1[2],
                d.t1,
            );
            defmt::info!(
                "IMU2 accel=[{=f32},{=f32},{=f32}]g  gyro=[{=f32},{=f32},{=f32}]dps  t={=f32}C",
                d.a2[0], d.a2[1], d.a2[2],
                d.g2[0], d.g2[1], d.g2[2],
                d.t2,
            );
            defmt::info!(
                "FUSED accel=[{=f32},{=f32},{=f32}]g  gyro=[{=f32},{=f32},{=f32}]dps  |da|max={=f32}g |dg|max={=f32}dps",
                d.a_fused[0], d.a_fused[1], d.a_fused[2],
                d.g_fused[0], d.g_fused[1], d.g_fused[2],
                max_da, max_dg,
            );
        }
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

// True when the board is near-stationary and ~1 g — a safe moment to
// anchor yaw (attitude/roll/pitch are trustworthy and the field is steady).
fn mag_anchor_ready(raw: &RawImu) -> bool {
    let a = raw.accel_g();
    let amag = libm::sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    let g = raw.gyro_dps();
    let gmag = libm::sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    (amag - 1.0).abs() < 0.1 && gmag < 5.0
}

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
    let mut mag_applied: u32 = 0;
    let mut mag_rejected: u32 = 0;
    let mut last_mag_ut: [f32; 3] = [0.0; 3];

    const SIGMA_YAW_COG: f32 = 0.26; // ~15°, generous: COG ≈ heading only
    let mut calibrator = MagCalibrator::new();
    let mut cal_active = false;
    let mut anchor_pending = false;
    let mut last_cal_log = Instant::now();
    let cal_led_tx = CAL_LED.sender();

    loop {
        let raw = RAW_IMU.wait().await;

        // Apply a boot-loaded calibration once it arrives.
        if let Some(cfg) = STORED_CAL.try_take() {
            if cfg.mag_calibrated {
                mekf.set_hard_iron(cfg.mag_hard_iron_ut);
                anchor_pending = true;
                defmt::info!(
                    "MEKF loaded stored cal: offset=[{=f32},{=f32},{=f32}]",
                    cfg.mag_hard_iron_ut[0], cfg.mag_hard_iron_ut[1], cfg.mag_hard_iron_ut[2],
                );
            }
        }
        // Cal start/abort from the navigation task.
        if let Some(cmd) = CAL_CONTROL.try_take() {
            match cmd {
                CalCommand::Start => {
                    calibrator.reset();
                    cal_active = true;
                    cal_led_tx.send(CalLed::Calibrating(0));
                    defmt::info!("MEKF cal: started — rotate the craft through all axes");
                }
                CalCommand::Abort => {
                    if cal_active {
                        defmt::info!("MEKF cal: aborted");
                    }
                    cal_active = false;
                    cal_led_tx.send(CalLed::Idle);
                }
            }
        }
        // Fuse a fresh trusted COG heading, if any.
        if let Some(yaw_cog) = YAW_COG.try_take() {
            mekf.update_yaw_reference(yaw_cog, SIGMA_YAW_COG);
        }

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

        // Magnetometer handling: try_take is non-blocking and runs on
        // whichever IMU sample happens to coincide with a fresh mag
        // reading (~100 Hz mag vs 8 kHz IMU). Three modes:
        //   - cal_active: collect raw samples for the sphere fit; do NOT
        //     fuse mag (the craft is being rotated through all axes).
        //   - uninitialised: seed a relative-boot-heading reference (the
        //     pre-calibration fallback, unchanged from before).
        //   - normal: anchor to true north once (when level-and-still and
        //     a cal is pending), then fuse mag updates as usual.
        if let Some(mag) = MAG_DATA.try_take() {
            let ut = mag.ut();
            last_mag_ut = ut;
            if cal_active {
                calibrator.feed(ut);
                if last_cal_log.elapsed().as_millis() >= 500 {
                    cal_led_tx.send(CalLed::Calibrating(calibrator.progress()));
                    defmt::info!("MEKF cal: coverage {}%", calibrator.progress());
                    last_cal_log = Instant::now();
                }
                if calibrator.is_complete() {
                    match calibrator.result() {
                        Some(off) => {
                            mekf.set_hard_iron(off);
                            cal_active = false;
                            anchor_pending = true;
                            cal_led_tx.send(CalLed::AwaitingLevel);
                            let cfg = persist::record::Config {
                                mag_hard_iron_ut: off,
                                declination_rad: DECLINATION_DEG.to_radians(),
                                mag_calibrated: true,
                            };
                            CAL_SAVE.signal(cfg);
                            defmt::info!(
                                "MEKF cal: COMPLETE offset=[{=f32},{=f32},{=f32}] — hold level to anchor",
                                off[0], off[1], off[2],
                            );
                        }
                        None => {
                            cal_active = false;
                            cal_led_tx.send(CalLed::Fault);
                            defmt::error!("MEKF cal: degenerate fit — aborted, keeping prior cal");
                        }
                    }
                }
            } else if !mekf.mag_initialized() {
                if mekf.initialize_mag_from_first(ut) {
                    defmt::info!("MEKF mag reference seeded (relative boot heading)");
                }
            } else {
                if anchor_pending && mag_anchor_ready(&raw) {
                    mekf.anchor_heading(ut, DECLINATION_DEG.to_radians());
                    anchor_pending = false;
                    cal_led_tx.send(CalLed::Saved);
                    defmt::info!(
                        "MEKF anchored to true north: yaw={=f32}deg",
                        mekf.euler()[2] * RAD2DEG,
                    );
                }
                if mekf.update_mag(ut) {
                    mag_applied = mag_applied.wrapping_add(1);
                } else {
                    mag_rejected = mag_rejected.wrapping_add(1);
                }
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
        // Pack body-frame mag into the legacy ImuData mag slot in
        // mgauss × 10 (so 1 LSB = 0.1 mgauss = 0.01 µT) — wide enough
        // to hold ±32k mgauss in i16. Downstream consumers don't fuse
        // this; it's there for logging and future user-facing displays.
        let mag_i16 = [
            (last_mag_ut[0] * 100.0) as i16,
            (last_mag_ut[1] * 100.0) as i16,
            (last_mag_ut[2] * 100.0) as i16,
        ];
        let imu = ImuData {
            accel: [a_g[0] * G_MPS2, a_g[1] * G_MPS2, a_g[2] * G_MPS2],
            temperature: raw.temp_c(),
            gyro: gyro_corr_dps,
            angle: [
                euler_rad[0] * RAD2DEG,
                euler_rad[1] * RAD2DEG,
                euler_rad[2] * RAD2DEG,
            ],
            mag: mag_i16,
            pressure: 0,
            altitude_cm: 0,
            quaternion: mekf.quaternion(),
            updated: UPDATED_ACCEL | UPDATED_GYRO | UPDATED_ANGLE | UPDATED_QUAT,
        };
        IMU_DATA.signal(imu);
        IMU_DATA_FOR_NAV.signal(imu);
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
                "MEKF {} samples/s, accel_upd={}/{}rej, mag_upd={}/{}rej, euler=[{=f32},{=f32},{=f32}]deg, |bias|={=f32}dps",
                sample_count,
                updates_applied,
                updates_rejected,
                mag_applied,
                mag_rejected,
                euler_rad[0] * RAD2DEG,
                euler_rad[1] * RAD2DEG,
                euler_rad[2] * RAD2DEG,
                b_dps_mag,
            );
            sample_count = 0;
            updates_applied = 0;
            updates_rejected = 0;
            mag_applied = 0;
            mag_rejected = 0;
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
//   - Baro altitude updates at ~25 Hz — fuses once `p_ref` is latched
//     at arm time.
//
// The two sensor paths are independent. The KF runs on whichever is
// available — GPS-only, baro-only, both, or (briefly) neither. The
// soft arm gate in `arming.rs` requires *one* of the two to be live.
//
// GPS path:
//   1. Each incoming NMEA fix is gated on `FIX3D && sats >= MIN_SATS &&
//      hdop < HDOP_THRESH`.
//   2. Home origin latches against the third consecutive lat/lon-fresh
//      good fix (three-streak guards against single-fix cold-start
//      glitches without paying a 30-s buffer window). Home is
//      *provisional* until ARM re-anchors it.
//   3. Every subsequent good fix fuses as local NED relative to home.
//      The R-matrix is scaled by HDOP at fuse time, so noisier fixes
//      contribute proportionally less.
//
// Baro path:
//   1. The first baro sample seeds a *provisional* `p_ref_pa`. From
//      that point on, every sample fuses via `update_baro`. Pre-arm
//      altitude is meaningful (bounded to baro noise) instead of
//      drifting on pure IMU integration.
//   2. On the Disarmed→Armed transition the current pressure becomes
//      `p_ref_pa` and the KF state is reset to (0,0,0,0,0,0). Home is
//      also re-anchored to the current GPS fix if one is healthy.
//      Altitude and position thus read ~0 at arm regardless of how
//      far the provisional origins drifted pre-arm.
//
// History notes:
//   - Pre-2026-05-15: home required a two-stage spatial-stability
//     buffer (30 s fast / 60 s slow) and baro fusion was gated on a
//     post-arm flag. The buffer never latched against the receiver's
//     actual sats/hdop output; baro never fused pre-arm, so pre-arm
//     altitude drifted unbounded. Both replaced with the simpler
//     HDOP-only gate + always-fusing baro above.
//   - Pre-Alpha: baro p_ref coupled to GPS-anchored KF altitude as a
//     workaround for the previous board's flaky DPS310. The H743's
//     SPL06 is reliable enough to drop that coupling.

#[embassy_executor::task]
async fn pos_kf_task() {
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};

    const HZ: u64 = 100;
    const PERIOD_MS: u64 = 1000 / HZ;
    const DT: f32 = 1.0 / HZ as f32;

    // GPS per-fix quality gate. Below `MIN_SATS` or above `HDOP_THRESH`
    // the fix is dropped (no fuse, no streak increment). `HDOP_THRESH`
    // also caps the per-fuse R-matrix scale below.
    const MIN_SATS: u8 = 4;
    const HDOP_THRESH: f32 = 2.5;
    // Home-latch streak: require this many consecutive lat/lon-fresh
    // good fixes before latching home. Three is enough to dodge the
    // single-fix cold-start glitches some modules emit when they
    // first start solving, without paying a fixed time window.
    const STREAK_TO_LATCH: u8 = 3;
    // Base σ values fed into `update_gps_scaled`. Scaled by the
    // current HDOP (clamped to [1.0, HDOP_THRESH]) so each fix's
    // influence tracks its reported uncertainty.
    const BASE_SIGMA_GPS_H: f32 = 2.0;
    const BASE_SIGMA_GPS_V: f32 = 5.0;

    // σ_a = 0.5 m/s² matches the sim tuning — loose enough to track
    // gust transients without treating baro noise as truth. σ_baro =
    // 0.3 m is the SPL06 spec at 1× OSR plus a bit of headroom.
    // σ_gps_h / σ_gps_v are typical consumer-module noise; the KF will
    // rightly let baro dominate altitude via the much-smaller σ_baro.
    let mut kf = PosKf::new_at(
        [0.0, 0.0, 0.0],
        0.5, // σ_a
        BASE_SIGMA_GPS_H,
        BASE_SIGMA_GPS_V,
        0.3, // σ_baro
    );

    // Baro state. `p_ref_pa` is provisional from the first baro
    // sample, then re-anchored on the Disarmed→Armed event so
    // altitude reads ~0 at arm. `p_ref_at_arm` is just a marker — it
    // *doesn't* gate fusion, which runs whenever a `p_ref_pa` exists.
    let mut p_ref_pa: f32 = 0.0;
    let mut p_ref_provisional = true;
    let mut p_ref_at_arm = false;
    let mut last_baro_pressure: Option<f32> = None;

    // Bumped each time the arm latch is consumed (re-origin done). Published
    // in the estimate so the navigation task knows the post-arm zero has
    // landed before it captures altitude/position targets.
    let mut arm_origin_seq: u32 = 0;

    // Home-origin latch state. Stored as f64 for lat/lon to preserve
    // sub-metre resolution; the geodetic helper handles the cast.
    // `home_latched` retains its prior semantics for downstream
    // consumers ("home origin is set, GPS pos fusion is active").
    let mut home_lat: f64 = 0.0;
    let mut home_lon: f64 = 0.0;
    let mut home_alt_msl: f32 = 0.0;
    let mut home_latched = false;
    // Counts consecutive lat/lon-fresh good fixes; reset on any bad
    // fix or stale (duplicate-lat/lon) signal. Latches at
    // `STREAK_TO_LATCH`.
    let mut good_fix_streak: u8 = 0;

    let mut last_imu: Option<ImuData> = None;
    // `None` until the first baro sample arrives. The arming gate
    // (`baro_ready`) and the readiness flag (`altitude_ready`) both
    // derive freshness from this — initialising to `Instant::now()`
    // would mean "fresh" at task spawn before any sample existed,
    // a startup race that could let the arm FSM clear baro-only
    // arming with the baro completely absent.
    let mut last_baro_t: Option<Instant> = None;
    let mut baro_updates_sec: u32 = 0;
    // Three GPS counters so the 1 Hz log distinguishes "signal events
    // received" (driven by the gps_task's UART wake rate) from "actual
    // measurements fused" (one per GGA / RMC cycle once dedupe kicks in).
    let mut gps_signals_sec: u32 = 0;
    let mut gps_pos_fuses_sec: u32 = 0;
    let mut gps_vel_fuses_sec: u32 = 0;
    let mut last_gps: Option<GpsData> = None;

    // Dedupe state — guards against per-sentence NMEA storms re-fusing
    // the same GGA reading 3–5 times. NaN sentinel means "no prior
    // fuse"; any real GPS coord compares unequal.
    let mut last_fused_lat: f64 = f64::NAN;
    let mut last_fused_lon: f64 = f64::NAN;
    let mut last_fused_speed: f32 = f32::NAN;
    let mut last_fused_course: f32 = f32::NAN;

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

        // ---- GPS update (sensor-driven) ----
        //
        // Single-stage HDOP gate: a fix must clear FIX3D / MIN_SATS /
        // HDOP_THRESH to be considered. Home latches against the third
        // consecutive lat/lon-fresh good fix; from that point every
        // good lat/lon-fresh fix fuses as `update_gps_scaled`, with
        // σ_h / σ_v scaled by the fix's HDOP so each measurement
        // contributes proportional to its reported uncertainty.
        //
        // Dedupe (lat/lon != last_fused_*) prevents per-sentence NMEA
        // signal storms from re-fusing the same GGA 3–5× and
        // over-shrinking P_pp; a duplicate signal also resets the
        // streak so it doesn't count toward the latch.
        if let Some(gps) = GPS_DATA_FOR_KF.try_take() {
            last_gps = Some(gps);
            gps_signals_sec = gps_signals_sec.wrapping_add(1);

            let good_fix = gps.fix_mode == FixMode::Fix3D
                && gps.satellites >= MIN_SATS
                && gps.hdop > 0.0
                && gps.hdop < HDOP_THRESH;

            let pos_fresh =
                gps.latitude != last_fused_lat || gps.longitude != last_fused_lon;

            if good_fix && pos_fresh {
                good_fix_streak = good_fix_streak.saturating_add(1);

                if !home_latched && good_fix_streak >= STREAK_TO_LATCH {
                    home_lat = gps.latitude;
                    home_lon = gps.longitude;
                    home_alt_msl = gps.altitude_m;
                    home_latched = true;
                    defmt::info!(
                        "PosKF home latched (provisional): lat={=f32} lon={=f32} alt_msl={=f32}m | sats={} hdop={=f32}",
                        gps.latitude as f32,
                        gps.longitude as f32,
                        gps.altitude_m,
                        gps.satellites,
                        gps.hdop,
                    );
                }

                if home_latched {
                    let ned = geodetic_to_local_ned(
                        gps.latitude,
                        gps.longitude,
                        gps.altitude_m,
                        home_lat,
                        home_lon,
                        home_alt_msl,
                    );
                    // Scale R by HDOP. hdop=1 → unity; hdop≥HDOP_THRESH
                    // is the noisiest fix we accept, capped so the worst
                    // case still has *some* influence.
                    let r_scale = gps.hdop.clamp(1.0, HDOP_THRESH);
                    let sigma_h = BASE_SIGMA_GPS_H * r_scale;
                    let sigma_v = BASE_SIGMA_GPS_V * r_scale;
                    kf.update_gps_scaled(ned, sigma_h, sigma_v);
                    gps_pos_fuses_sec = gps_pos_fuses_sec.wrapping_add(1);
                    last_fused_lat = gps.latitude;
                    last_fused_lon = gps.longitude;
                    // Per-fuse trace. One line per actual fuse (~1/s
                    // post-dedupe). meas_N/E is the measurement, post_N/E
                    // is the filter's state after fusing. Tight match =
                    // filter tracks GPS; big delta = filter is smoothing
                    // heavily toward its prior.
                    let post_n = kf.x[0];
                    let post_e = kf.x[1];
                    defmt::info!(
                        "gps_fuse: lat={=f32} lon={=f32} hdop={=f32} | meas_N={=f32}m E={=f32}m | post_N={=f32}m E={=f32}m",
                        gps.latitude as f32,
                        gps.longitude as f32,
                        gps.hdop,
                        ned[0],
                        ned[1],
                        post_n,
                        post_e,
                    );
                }
            } else if !good_fix {
                // Any bad fix breaks the streak — the cold-start glitch
                // we're guarding against would otherwise sneak through
                // between two unrelated good fixes.
                good_fix_streak = 0;
            }
            // (duplicate-pos signals neither advance nor reset the streak;
            // they're a no-op for latch purposes.)

            // Velocity fusion runs independently of position (only on
            // fresh RMC/VTG) so stationary GPS doesn't keep re-fusing
            // (0, 0) every signal and over-shrinking P_vv. Below
            // ~0.3 m/s the receiver's course report is noise — clamp
            // to (0, 0), which still actively damps drift while
            // stationary but only once per fresh sample.
            let vel_fresh = gps.ground_speed_ms != last_fused_speed
                || gps.course_deg != last_fused_course;
            if vel_fresh && gps.fix_mode != FixMode::NoFix && gps.satellites >= 3 {
                let (vn, ve) = if gps.ground_speed_ms < 0.3 {
                    (0.0, 0.0)
                } else {
                    let crs = gps.course_deg.to_radians();
                    (
                        gps.ground_speed_ms * libm::cosf(crs),
                        gps.ground_speed_ms * libm::sinf(crs),
                    )
                };
                kf.update_gps_velocity(vn, ve);
                gps_vel_fuses_sec = gps_vel_fuses_sec.wrapping_add(1);
                last_fused_speed = gps.ground_speed_ms;
                last_fused_course = gps.course_deg;
            }
        }

        // ---- Baro update (sensor-driven; None on non-25-Hz ticks) ----
        // The first sample seeds a *provisional* p_ref so pre-arm
        // altitude is bounded by baro noise instead of running away on
        // pure IMU integration. From the first sample on, every sample
        // fuses via `update_baro`. ARM re-anchors p_ref to the current
        // pressure (see below) so post-arm altitude reads ~0.
        if let Some(baro) = BARO_DATA.try_take() {
            last_baro_pressure = Some(baro.pressure_pa);
            last_baro_t = Some(Instant::now());
            if p_ref_provisional {
                p_ref_pa = baro.pressure_pa;
                p_ref_provisional = false;
                defmt::info!(
                    "PosKF provisional p_ref set: {=f32}Pa",
                    p_ref_pa,
                );
            }
            let alt_up = baro::pressure_to_altitude_m(baro.pressure_pa, p_ref_pa);
            kf.update_baro(alt_up);
            baro_updates_sec = baro_updates_sec.wrapping_add(1);
        }

        // ---- Arm-time frame re-origin ----
        // Fired once on Disarmed→Armed by the navigation task. Re-anchor
        // both provisional origins to the current sensor state and zero
        // the full KF state so altitude and NED position both read ~0
        // immediately after arm — regardless of how far the pre-arm
        // estimates drifted on a noisy first-fix or a wandering baro.
        if ARM_LATCH.try_take().is_some() {
            // Re-anchor p_ref to current pressure (or warn if no baro).
            if let Some(p) = last_baro_pressure {
                p_ref_pa = p;
                p_ref_provisional = false;
                p_ref_at_arm = true;
                defmt::info!(
                    "PosKF arm: p_ref re-anchored to {=f32}Pa",
                    p_ref_pa,
                );
            } else {
                defmt::warn!(
                    "PosKF arm: no baro sample — p_ref stays at boot default",
                );
            }
            // Re-anchor home to current GPS fix if it's healthy. If GPS
            // isn't usable we leave home where it was (either pre-arm
            // provisional or unset).
            if let Some(gps) = last_gps.filter(|g| {
                g.fix_mode == FixMode::Fix3D
                    && g.satellites >= MIN_SATS
                    && g.hdop > 0.0
                    && g.hdop < HDOP_THRESH
            }) {
                home_lat = gps.latitude;
                home_lon = gps.longitude;
                home_alt_msl = gps.altitude_m;
                home_latched = true;
                last_fused_lat = gps.latitude;
                last_fused_lon = gps.longitude;
                defmt::info!(
                    "PosKF arm: home re-anchored to lat={=f32} lon={=f32} alt_msl={=f32}m",
                    gps.latitude as f32,
                    gps.longitude as f32,
                    gps.altitude_m,
                );
            } else if !home_latched {
                defmt::warn!(
                    "PosKF arm: no usable GPS — home origin unset, NED frame not anchored",
                );
            }
            // Zero the full KF state so position/velocity read as
            // (0,0,0,0,0,0) at the freshly-anchored origin. Without
            // this the KF would carry pre-arm drift into the first
            // post-arm samples and the operator would see a transient
            // pull-toward-zero on the very first fuses.
            for i in 0..6 {
                kf.x[i] = 0.0;
            }

            // Signal to the navigation task that the re-origin for this arm
            // has landed (the published estimate now reflects the zero).
            arm_origin_seq = arm_origin_seq.wrapping_add(1);
        }

        // ---- Readiness ----
        // Altitude is meaningful if either sensor is currently anchored:
        // recent baro samples *or* a latched GPS home. Keyed on
        // `baro_fresh` (live freshness) rather than `p_ref_at_arm`
        // (one-shot arm marker) so we (a) don't misreport stale-true
        // if the baro dies mid-flight, and (b) include the pre-arm
        // window where baro is alive but we haven't armed yet.
        //
        // Freshness threshold: 1 s. The baro task signals at 125 Hz,
        // so a 1-s gap means at least ~125 missed samples — well past
        // the in-task recovery streak (~0.4 s) so we're not flapping
        // during normal bus-stuck recoveries.
        let baro_fresh = last_baro_t
            .map(|t| t.elapsed() < Duration::from_secs(1))
            .unwrap_or(false);
        let altitude_ready = baro_fresh || home_latched;

        // ---- Publish estimate ----
        let s = kf.state();
        let est = PosEstimate {
            position_ned: [s[0], s[1], s[2]],
            velocity_ned: [s[3], s[4], s[5]],
            altitude_up: kf.altitude_up(),
            vz_up: kf.vz_up(),
            p_ref_pa,
            // u32::MAX before the first baro sample — distinguishes
            // "never seen" from "stale by N ms" downstream.
            baro_age_ms: last_baro_t
                .map(|t| t.elapsed().as_millis() as u32)
                .unwrap_or(u32::MAX),
            altitude_ready,
            home_latched,
            arm_origin_seq,
        };
        POS_ESTIMATE.signal(est);

        // ---- 1 Hz health log ----
        if last_report.elapsed() >= Duration::from_secs(1) {
            let (sats, hdop, fix) = last_gps
                .map(|g| (g.satellites, g.hdop, g.fix_mode as u8))
                .unwrap_or((0, 99.99, 0));
            if altitude_ready {
                defmt::info!(
                    "PosKF: alt={=f32}m vz={=f32}m/s | N={=f32}m E={=f32}m | baro_fresh={} p_ref_at_arm={} home={} | {} baro/s | gps {}sig {}pos {}vel /s",
                    est.altitude_up,
                    est.vz_up,
                    est.position_ned[0],
                    est.position_ned[1],
                    baro_fresh,
                    p_ref_at_arm,
                    home_latched,
                    baro_updates_sec,
                    gps_signals_sec,
                    gps_pos_fuses_sec,
                    gps_vel_fuses_sec,
                );
            } else {
                defmt::info!(
                    "PosKF waiting: GPS sats={} hdop={=f32} fix={} baro_seen={} | gps {}sig {}pos {}vel /s",
                    sats,
                    hdop,
                    fix,
                    last_baro_pressure.is_some(),
                    gps_signals_sec,
                    gps_pos_fuses_sec,
                    gps_vel_fuses_sec,
                );
            }
            baro_updates_sec = 0;
            gps_signals_sec = 0;
            gps_pos_fuses_sec = 0;
            gps_vel_fuses_sec = 0;
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

const BARO_ERR_STREAK_RECOVERY: u32 = 50; // ~0.4 s at 125 Hz
const BARO_TIMEOUT_MS: u64 = 5; // shorter wastes less CPU when stuck
const BARO_MAX_INIT_ATTEMPTS: u32 = 5; // give up after this many detect/init failures

#[embassy_executor::task]
async fn baro_task(
    mut i2c_per: embassy_stm32::Peri<'static, embassy_stm32::peripherals::I2C2>,
    mut scl: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB10>,
    mut sda: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB11>,
) {
    use drivers::baro::{self, BaroChip, Spl06};
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
            BaroChip::Spl06  { addr } => addr,
            BaroChip::Dps310 { addr } => addr, // fallback; not on this board
            BaroChip::Bmp280 { addr: _ } => {
                defmt::warn!("BMP280 detected but driver not yet implemented");
                return;
            }
        };

        let spl = match Spl06::init(&mut i2c, addr).await {
            Ok(d) => d,
            Err(e) => {
                init_failures = init_failures.saturating_add(1);
                defmt::error!(
                    "SPL06 init failed: {:?} ({}/{}) — bitbang + retry in 1 s",
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

        // ---- LIS2MDL magnetometer (same bus, optional) ----
        // The mag shares I2C1 with the baro. We init it here so it can't
        // get stuck waiting for the bus owner — if absent or DOA, we
        // continue without mag fusion (yaw stays unobservable in the
        // MEKF, exactly the pre-magnetometer behaviour). The chip is in
        // 100 Hz continuous mode after init, so we just poll it once per
        // tick (125 Hz) and let the chip's own ODR cap effective rate.
        // Orientation::Identity is correct for the LIS2MDL breakout
        // soldered with its X+ aligned to body forward; revise here if
        // the airframe mounts it differently.
        let mag = match Lis2mdl::init(&mut i2c, MagOrientation::Identity).await {
            Ok(d) => {
                defmt::info!("LIS2MDL magnetometer online @ 100 Hz");
                Some(d)
            }
            Err(e) => {
                defmt::warn!(
                    "LIS2MDL init failed: {:?} — continuing without mag (yaw will drift)",
                    e,
                );
                None
            }
        };

        // ---- Read loop ----
        // SPL06 configured at 128 Hz; tick at 8 ms (125 Hz) to consume
        // each new sample without skipping. LIS2MDL is 100 Hz so polling
        // at 125 Hz yields ~20% duplicate samples — harmless, the MEKF
        // does a try_take and just skips when no new data arrived.
        let mut ticker = Ticker::every(Duration::from_millis(8)); // 125 Hz
        let mut reads: u32 = 0;
        let mut errs: u32 = 0;
        let mut mag_reads: u32 = 0;
        let mut mag_errs: u32 = 0;
        let mut streak: u32 = 0;
        let mut last_report = Instant::now();
        let mut last_sample: Option<(BaroSample, Instant)> = None;

        let recover = loop {
            ticker.next().await;
            match spl.read(&mut i2c) {
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

            // Mag read shares the bus and is independent of baro success.
            // A mag I2C error doesn't extend the baro recovery streak —
            // they're separate devices and we don't want one bad mag
            // read forcing a baro bus reset.
            if let Some(m) = mag.as_ref() {
                match m.read(&mut i2c) {
                    Ok(s) => {
                        MAG_DATA.signal(s);
                        mag_reads = mag_reads.wrapping_add(1);
                    }
                    Err(_) => {
                        mag_errs = mag_errs.wrapping_add(1);
                    }
                }
            }

            if Instant::now() - last_report >= Duration::from_secs(1) {
                match (reads, last_sample) {
                    (0, Some((s, t))) => {
                        let age_ms = (Instant::now() - t).as_millis() as u32;
                        defmt::info!(
                            "Baro 0 reads/s, {} errs — bus stuck (last P={=f32}Pa T={=f32}C age={=u32}ms); mag {}/s, {} errs",
                            errs,
                            s.pressure_pa,
                            s.temperature_c,
                            age_ms,
                            mag_reads,
                            mag_errs,
                        );
                    }
                    (0, None) => {
                        defmt::info!(
                            "Baro 0 reads/s, {} errs — bus stuck (no sample yet); mag {}/s, {} errs",
                            errs, mag_reads, mag_errs,
                        );
                    }
                    _ => {
                        let (p, t) = last_sample
                            .map(|(s, _)| (s.pressure_pa, s.temperature_c))
                            .unwrap_or((0.0, 0.0));
                        defmt::info!(
                            "Baro {} reads/s, {} errs — P={=f32}Pa T={=f32}C; mag {}/s, {} errs",
                            reads,
                            errs,
                            p,
                            t,
                            mag_reads,
                            mag_errs,
                        );
                    }
                }
                reads = 0;
                errs = 0;
                mag_reads = 0;
                mag_errs = 0;
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

#[embassy_executor::task]
async fn navigation_task() {
    use core::f32::consts::PI;
    const DEG2RAD: f32 = PI / 180.0;
    const RAD2DEG: f32 = 180.0 / PI;

    // ---- Arming state machine ----
    let mut arming = ArmingStateMachine::new();
    // Soft arm gate — baro alone is enough on a normal FC. Bench mode
    // only needs `require_altitude_ref = false` if neither baro nor
    // GPS hardware is present.

    // ---- MPC attitude outer loop (100 Hz) ----
    let mut mpc = AttitudeMpc::new();

    // ---- Altitude hold (100 Hz) ----
    let hover_throttle: f32 = 0.294; // tune per aircraft: mass*g / max_thrust
    let alt_gains = AltitudeGains {
        kp: 0.15,
        kd: 0.1,
        ki: 0.05,
    };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let mut current_thrust = hover_throttle;

    // ---- Position hold / GPS rescue (50 Hz) ----
    let mut pos_ctrl = PositionController::new(PositionGains::default());



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

    // ---- Flight mode state ----
    let mut flight_mode = FlightMode::Acro;
    let mut prev_mode = FlightMode::Acro;
    let mut alt_target: f32 = 0.0;      // metres, positive-up
    let mut pos_target: [f32; 2] = [0.0, 0.0]; // [north, east] metres
    let mut rescue_loiter_start: Option<Instant> = None;
    let mut rescue_landing = false;

    // Arm-time re-origin gate + "have the current mode's targets been
    // captured yet" flag. Together they withhold altitude/position target
    // capture until the PosKF has zeroed for this arm, then capture exactly
    // once — even if the mode was entered during the arm/re-origin window.
    let mut arm_origin_sync = ArmOriginSync::new();
    let mut targets_captured = false;

    // GPS-COG yaw gating: only trust course when genuinely flying forward.
    const V_MIN_COG: f32 = 2.0; // m/s
    const FWD_STICK_MIN: f32 = 0.3; // normalised pitch-forward
    let mut cal_sw_prev = false;
    let mut armed_prev_cal = false;

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

    // ---- Main loop: 100 Hz ----
    // Period and dt come from the MPC module so the outer control loop and
    // the MPC's discretised model can never run at different rates.
    let mut ticker = Ticker::every(Duration::from_micros(MPC_PERIOD_US));
    let mut cycle_count: u32 = 0;
    let dt: f32 = MPC_DT; // 100 Hz (== mpc::MPC_PERIOD_US)

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
        if let Some(imu) = IMU_DATA_FOR_NAV.try_take() {
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
        // Either sensor satisfies the soft altitude-reference gate.
        // Baro counts as "ready" when the PosKF has at least seen it
        // recently (age < 500 ms). Before the PosKF has published
        // anything, treat both as not-ready.
        let baro_ready = last_pos_est.map(|e| e.baro_age_ms < 500).unwrap_or(false);
        let gps_home_latched = last_pos_est.map(|e| e.home_latched).unwrap_or(false);

        let arm_state = arming.update(
            arm_switch,
            throttle_raw,
            last_imu.angle[0], // roll deg
            last_imu.angle[1], // pitch deg
            imu_age_ms,
            rc_age_ms,
            baro_ready,
            gps_home_latched,
        );
        let armed = arm_state == ArmState::Armed;
        if armed && !last_armed {
            // Tell the PosKF to latch p_ref against the current baro
            // sample (if any) and zero the vertical KF state.
            ARM_LATCH.signal(());

            // Surface the available references so the pilot knows
            // which modes will engage.
            let alt_ready = last_pos_est.map(|e| e.altitude_ready).unwrap_or(false);
            match (alt_ready, gps_home_latched) {
                (true, true)  => {} // full lock — quiet
                (true, false) => defmt::info!("ARMED with baro only — alt-hold OK, no position modes"),
                (false, true) => defmt::warn!("ARMED with GPS only — position modes OK; alt source is GPS"),
                (false, false) => defmt::warn!(
                    "ARMED with no altitude reference — manual throttle, no alt-hold or position modes"
                ),
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
                baro_ready,
                gps_home_latched,
            );
            defmt::info!(
                "arm rejected: thr_low={} level={} imu={} rc={} alt_ref={} | thr={}% roll={}° pitch={}° imu_age={}ms rc_age={}ms baro={} gps_home={} ch4={} ch5={}",
                c.throttle_low,
                c.attitude_level,
                c.imu_fresh,
                c.rc_link_active,
                c.altitude_ref_ready,
                (throttle_raw * 100.0) as i32,
                last_imu.angle[0] as i32,
                last_imu.angle[1] as i32,
                imu_age_ms,
                rc_age_ms,
                baro_ready,
                gps_home_latched,
                last_rc.channels[4],
                last_rc.channels[5],
            );
        }



        // ---- 3. Flight mode selection ----
        // `altitude_ready` (baro OR GPS) gates altitude-aware modes;
        // `home_latched` (GPS) gates NED-frame modes. Failsafe picks
        // the best descent strategy for what we still have.
        let altitude_ready = last_pos_est.map(|e| e.altitude_ready).unwrap_or(false);
        let home_latched = last_pos_est.map(|e| e.home_latched).unwrap_or(false);

        if !armed {
            flight_mode = FlightMode::Acro;
        } else if arming.failsafe_active {
            // Pick the best descent we can run with what's available.
            flight_mode = if home_latched {
                FlightMode::GpsRescue
            } else if altitude_ready {
                FlightMode::FailsafeLand
            } else {
                FlightMode::FailsafeBlind
            };
        } else if RcChannels::to_us(last_rc.channels[6]) > 1500 && home_latched {
            flight_mode = FlightMode::GpsHome; // Return to home on switch
        } else if RcChannels::to_us(last_rc.channels[5]) > 1600 && altitude_ready {
            // PosHold: GPS home gives true position hold; without it,
            // PosKF velocity-fusion damps DR drift and the controller
            // does best-effort horizontal hold for tens of seconds.
            flight_mode = FlightMode::PosHold;
        } else if RcChannels::to_us(last_rc.channels[5]) > 1200 && altitude_ready {
            flight_mode = FlightMode::AltHold;
        } else {
            flight_mode = FlightMode::Acro;
        }

        // ---- Magnetometer cal trigger (AUX4 = channel index 7) ----
        // Disarmed-only. Rising edge starts; falling edge or a fresh arm
        // aborts. The cal itself runs in the MEKF task.
        let cal_sw = last_rc.channels[7] > 1500;
        if cal_sw && !cal_sw_prev && !armed {
            CAL_CONTROL.signal(CalCommand::Start);
        } else if (!cal_sw && cal_sw_prev) || (cal_sw && armed && !armed_prev_cal) {
            CAL_CONTROL.signal(CalCommand::Abort);
        }
        cal_sw_prev = cal_sw;
        armed_prev_cal = armed;

        // ---- GPS-COG yaw reference (gated) ----
        // COG equals heading only in deliberate forward flight, so require
        // armed + good 3D fix + above V_MIN + forward pitch stick. The
        // MEKF fuses it as a generous-sigma scalar yaw update.
        // NOTE: confirm the forward-stick sign on the bench (channels[1]
        // forward should be positive here); flip if your TX is reversed.
        let fwd_stick = RcChannels::to_normalised(last_rc.channels[1]);
        if armed
            && last_gps.has_3d_fix()
            && last_gps.ground_speed_ms > V_MIN_COG
            && fwd_stick > FWD_STICK_MIN
        {
            YAW_COG.signal(last_gps.course_deg.to_radians());
        }

        // ---- Mode entry: capture targets on transition ----
        // Log the change and reset the capture flag here; the actual target
        // capture below is gated separately on the PosKF re-origin, so a
        // mode entered during the arm/re-origin window still captures once
        // the zero lands rather than being missed.
        if flight_mode != prev_mode {
            defmt::info!("Flight mode: {} -> {}", prev_mode, flight_mode);
            prev_mode = flight_mode;
            targets_captured = false;
        }

        // True once the PosKF has zeroed for the current arm. Must be called
        // every tick so the arm-edge tracking stays correct.
        let reoriginated = arm_origin_sync
            .reoriginated(armed, last_pos_est.map(|e| e.arm_origin_seq).unwrap_or(0));

        if !targets_captured {
            // AltHold + FailsafeLand need altitude; PosHold + GPS modes
            // need NED. FailsafeBlind needs nothing (open loop).
            let target_gate = match flight_mode {
                FlightMode::AltHold | FlightMode::FailsafeLand => altitude_ready,
                FlightMode::PosHold => altitude_ready, // best-effort horizontal w/o home
                FlightMode::GpsRescue | FlightMode::GpsHome => home_latched,
                FlightMode::Acro | FlightMode::FailsafeBlind => false,
            };
            // Wait for the arm-time re-origin before sampling the estimate:
            // otherwise we'd latch a stale pre-zero altitude/position and
            // lurch the instant the KF zeroes. Mid-flight mode switches see
            // `reoriginated == true` already, so they capture immediately.
            if let Some(est) = last_pos_est.filter(|_| target_gate && reoriginated) {
                match flight_mode {
                    FlightMode::AltHold => {
                        alt_target = est.altitude_up;
                        alt_ctrl.reset();
                    }
                    FlightMode::PosHold => {
                        alt_target = est.altitude_up;
                        pos_target = [est.position_ned[0], est.position_ned[1]];
                        alt_ctrl.reset();
                    }
                    FlightMode::GpsRescue => {
                        // Hover in place (lock current position and altitude)
                        alt_target = est.altitude_up;
                        pos_target = [est.position_ned[0], est.position_ned[1]];
                        alt_ctrl.reset();
                    }
                    FlightMode::GpsHome => {
                        // Climb to rescue alt or hold current if already higher.
                        alt_target = if est.altitude_up > RESCUE_ALT_M {
                            est.altitude_up
                        } else {
                            RESCUE_ALT_M
                        };
                        pos_target = [0.0, 0.0]; // home is NED origin
                        rescue_loiter_start = None;
                        rescue_landing = false;
                        alt_ctrl.reset();
                    }
                    FlightMode::FailsafeLand => {
                        // Start descent from current altitude; the per-
                        // tick handler ramps alt_target down at
                        // FAILSAFE_DESCENT_RATE_MPS.
                        alt_target = est.altitude_up;
                        alt_ctrl.reset();
                    }
                    FlightMode::Acro | FlightMode::FailsafeBlind => {}
                }
                targets_captured = true;
            }
        }

        // ---- 4. Control computation ----
        if armed {
            let max_angle: f32 = 30.0;
            let roll_input = RcChannels::to_normalised(last_rc.channels[0]);
            let pitch_input = RcChannels::to_normalised(last_rc.channels[1]);
            let yaw_input = RcChannels::to_normalised(last_rc.channels[3]);
            let throttle_raw = RcChannels::to_unit(last_rc.channels[2]);

            // ---- 100 Hz outer loops (every cycle) ----
            // The MPC and altitude/position controllers run once per
            // navigation tick. The MPC's A/B model is discretised for
            // exactly this period (mpc::MPC_DT), so dt_outer == dt. The
            // block yields the freshly-computed rate setpoints.
            let rate_sp_degs = {
                let dt_outer: f32 = dt; // 100 Hz

                // Current IMU state in radians
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
                let yaw_rad = angles_rad[2];

                // -- Determine desired roll/pitch/yaw and thrust per mode --
                let (desired_roll_rad, desired_pitch_rad, desired_yaw_rad, yaw_rate_dps);

                match flight_mode {
                    FlightMode::Acro => {
                        desired_roll_rad = roll_input * max_angle * DEG2RAD;
                        desired_pitch_rad = pitch_input * max_angle * DEG2RAD;
                        desired_yaw_rad = 0.0;
                        yaw_rate_dps = yaw_input * 200.0;
                        // Direct throttle pass-through
                        current_thrust = throttle_raw.clamp(0.0, 1.0);
                    }
                    FlightMode::AltHold => {
                        desired_roll_rad = roll_input * max_angle * DEG2RAD;
                        desired_pitch_rad = pitch_input * max_angle * DEG2RAD;
                        desired_yaw_rad = 0.0;
                        yaw_rate_dps = yaw_input * 200.0;
                        // Throttle stick → climb/descend rate → alt target adjustment
                        let thr_centered = throttle_raw - 0.5; // -0.5..+0.5
                        if libm::fabsf(thr_centered) > ALT_HOLD_DEADBAND {
                            let rate = thr_centered * 2.0 * ALT_HOLD_MAX_RATE_MPS;
                            alt_target += rate * dt_outer;
                        }
                        if let Some(est) = last_pos_est.filter(|e| e.altitude_ready) {
                            current_thrust =
                                alt_ctrl.update(alt_target, est.altitude_up, est.vz_up, dt_outer);
                        }
                    }
                    FlightMode::PosHold => {
                        yaw_rate_dps = yaw_input * 200.0;
                        // Sticks → velocity → position target offset
                        if libm::fabsf(roll_input) > ALT_HOLD_DEADBAND
                            || libm::fabsf(pitch_input) > ALT_HOLD_DEADBAND
                        {
                            let cos_yaw = libm::cosf(yaw_rad);
                            let sin_yaw = libm::sinf(yaw_rad);
                            let vn = (cos_yaw * pitch_input + sin_yaw * roll_input)
                                * POS_HOLD_MAX_VEL_MPS;
                            let ve = (-sin_yaw * pitch_input + cos_yaw * roll_input)
                                * POS_HOLD_MAX_VEL_MPS;
                            pos_target[0] += vn * dt_outer;
                            pos_target[1] += ve * dt_outer;
                        }
                        // Throttle → altitude target
                        let thr_centered = throttle_raw - 0.5;
                        if libm::fabsf(thr_centered) > ALT_HOLD_DEADBAND {
                            alt_target += thr_centered * 2.0 * ALT_HOLD_MAX_RATE_MPS * dt_outer;
                        }
                        if let Some(est) = last_pos_est.filter(|e| e.home_latched) {
                            let pos_out = pos_ctrl.update(
                                [est.position_ned[0], est.position_ned[1]],
                                [est.velocity_ned[0], est.velocity_ned[1]],
                                pos_target,
                                yaw_rad,
                            );
                            desired_roll_rad = pos_out.roll_rad;
                            desired_pitch_rad = pos_out.pitch_rad;
                            current_thrust =
                                alt_ctrl.update(alt_target, est.altitude_up, est.vz_up, dt_outer);
                        } else {
                            desired_roll_rad = 0.0;
                            desired_pitch_rad = 0.0;
                            current_thrust = hover_throttle;
                        }
                        desired_yaw_rad = 0.0;
                    }
                    FlightMode::GpsRescue => {
                        // Failsafe: just hover in place
                        yaw_rate_dps = 0.0;
                        desired_yaw_rad = 0.0;
                        if let Some(est) = last_pos_est.filter(|e| e.home_latched) {
                            let pos_out = pos_ctrl.update(
                                [est.position_ned[0], est.position_ned[1]],
                                [est.velocity_ned[0], est.velocity_ned[1]],
                                pos_target,
                                yaw_rad,
                            );
                            desired_roll_rad = pos_out.roll_rad;
                            desired_pitch_rad = pos_out.pitch_rad;
                            current_thrust =
                                alt_ctrl.update(alt_target, est.altitude_up, est.vz_up, dt_outer);
                        } else {
                            desired_roll_rad = 0.0;
                            desired_pitch_rad = 0.0;
                            current_thrust = hover_throttle;
                        }
                    }
                    FlightMode::GpsHome => {
                        // Return to home
                        yaw_rate_dps = 0.0;
                        desired_yaw_rad = 0.0;
                        if let Some(est) = last_pos_est.filter(|e| e.home_latched) {
                            let dist_home = libm::sqrtf(est.position_ned[0] * est.position_ned[0] + est.position_ned[1] * est.position_ned[1]);

                            // Auto-land sequence
                            if rescue_landing {
                                alt_target -= RESCUE_LAND_RATE_MPS * dt_outer;
                                if est.altitude_up < RESCUE_DISARM_ALT_M {
                                    defmt::info!("GPS Home: auto-land complete, disarming");
                                    arming.force_disarm();
                                }
                            } else if dist_home < RESCUE_ARRIVAL_RADIUS_M {
                                // Arrived — start loiter timer
                                if rescue_loiter_start.is_none() {
                                    defmt::info!("GPS Home: arrived at home, loitering");
                                    rescue_loiter_start = Some(Instant::now());
                                }
                                if let Some(t) = rescue_loiter_start {
                                    let loiter_s = t.elapsed().as_millis() as f32 / 1000.0;
                                    // If we are actually in failsafe (or just loiter timeout on switch), land
                                    if loiter_s > RESCUE_LAND_TIMEOUT_S {
                                        defmt::info!("GPS Home: loiter timeout, auto-landing");
                                        rescue_landing = true;
                                    }
                                }
                            }

                            let pos_out = pos_ctrl.update(
                                [est.position_ned[0], est.position_ned[1]],
                                [est.velocity_ned[0], est.velocity_ned[1]],
                                pos_target,
                                yaw_rad,
                            );
                            desired_roll_rad = pos_out.roll_rad;
                            desired_pitch_rad = pos_out.pitch_rad;
                            current_thrust =
                                alt_ctrl.update(alt_target, est.altitude_up, est.vz_up, dt_outer);
                        } else {
                            desired_roll_rad = 0.0;
                            desired_pitch_rad = 0.0;
                            current_thrust = hover_throttle;
                        }
                    }
                    FlightMode::FailsafeLand => {
                        // RC lost, no GPS home, baro alive: closed-loop
                        // controlled descent at FAILSAFE_DESCENT_RATE_MPS.
                        // Disarm when altitude crosses the floor (we're
                        // back near the arm reference). No timeout —
                        // altitude-floor is the sole stop condition.
                        yaw_rate_dps = 0.0;
                        desired_yaw_rad = 0.0;
                        desired_roll_rad = 0.0;
                        desired_pitch_rad = 0.0;
                        alt_target -= FAILSAFE_DESCENT_RATE_MPS * dt_outer;
                        if let Some(est) = last_pos_est.filter(|e| e.altitude_ready) {
                            current_thrust =
                                alt_ctrl.update(alt_target, est.altitude_up, est.vz_up, dt_outer);
                            if est.altitude_up < FAILSAFE_LAND_DISARM_ALT_M {
                                defmt::info!("FailsafeLand: altitude floor reached, disarming");
                                arming.force_disarm();
                            }
                        } else {
                            // Altitude went stale mid-descent — fall
                            // through to blind throttle for this tick.
                            // Mode-selection will switch us to
                            // FailsafeBlind on the next pass.
                            current_thrust = hover_throttle * FAILSAFE_BLIND_THROTTLE_FRAC;
                        }
                    }
                    FlightMode::FailsafeBlind => {
                        // RC lost AND no altitude reference. Open-loop
                        // throttle slightly below hover, level attitude.
                        // No auto-disarm — without altitude data we
                        // can't tell when to stop. The descent runs
                        // until pilot recovers RC or battery cuts.
                        // (Beta backlog: impact-signature disarm.)
                        yaw_rate_dps = 0.0;
                        desired_yaw_rad = 0.0;
                        desired_roll_rad = 0.0;
                        desired_pitch_rad = 0.0;
                        current_thrust = hover_throttle * FAILSAFE_BLIND_THROTTLE_FRAC;
                    }
                }

                // ---- MPC solve (all modes) ----
                mpc.set_reference(
                    [desired_roll_rad, desired_pitch_rad, desired_yaw_rad],
                    [0.0, 0.0, yaw_rate_dps * DEG2RAD],
                );

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

                [
                    mpc_out.rate_setpoints_rads[0] * RAD2DEG,
                    mpc_out.rate_setpoints_rads[1] * RAD2DEG,
                    mpc_out.rate_setpoints_rads[2] * RAD2DEG,
                ]
            };

            // ---- Publish to fast inner loop ----
            OUTER_CMD.sender().send(OuterLoopCommand {
                thrust: current_thrust,
                rate_sp_degs,
                armed: true,
            });
        } else {
            // Disarmed — zero everything, reset controllers
            mpc.reset();
            alt_ctrl.reset();
            current_thrust = hover_throttle;
            rescue_loiter_start = None;
            rescue_landing = false;

            OUTER_CMD.sender().send(OuterLoopCommand {
                thrust: 0.0,
                rate_sp_degs: [0.0; 3],
                armed: false,
            });
        }

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

            let stick_thr_pct = (RcChannels::to_unit(last_rc.channels[2]) * 100.0) as u8;
            let thrust_cmd_pct = (current_thrust * 100.0) as u8;
            defmt::info!(
                "mode={} armed={} roll={:?}° pitch={:?}° yaw={:?}° thr={}% cmd={}% alt_t={=f32}m sats={}",
                flight_mode,
                armed,
                last_imu.angle[0],
                last_imu.angle[1],
                last_imu.angle[2],
                stick_thr_pct,
                thrust_cmd_pct,
                alt_target,
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

// ---- Fast Inner Loop (8 kHz) ----
// Runs strictly synchronised to the MEKF output (which runs at 8 kHz).
// Reads the latest target rates and thrust from the outer loop, runs
// the rate PID, and pushes commands to the ESCs via DShot.

async fn control_loop(mut dshot: DshotQuad<'static>) -> ! {
    defmt::info!("Fast inner loop started (8 kHz synced to IMU_DATA)");

    // ---- PID rate inner loop (8 kHz) ----
    // Gain scaling: since dt is 40x smaller (125us vs 5ms), the D-term filter
    // needs adjustment if it was tuned for 200 Hz. For now we use the same gains,
    // but tuning will likely be required.
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

    let dt: f32 = 0.000125; // 8 kHz
    let mut receiver = OUTER_CMD.receiver().unwrap();

    // Airmode is withheld until throttle first crosses this floor after
    // arming, so an armed quad on the ground can't spin up on stick input.
    const AIRMODE_ACTIVATE_THROTTLE: f32 = 0.05; // 5% collective
    let mut airmode_gate = AirmodeGate::new();

    loop {
        // Wait for the next 8 kHz IMU sample
        let imu = IMU_DATA.wait().await;

        // Get the latest commands from the outer loop
        // We use try_get() so it never blocks the fast loop
        let cmd = receiver.try_get().unwrap_or(OuterLoopCommand {
            thrust: 0.0,
            rate_sp_degs: [0.0; 3],
            armed: false,
        });

        let airmode = airmode_gate.update(cmd.armed, cmd.thrust, AIRMODE_ACTIVATE_THROTTLE);

        if cmd.armed {
            let motor_outputs = if airmode {
                // Airborne (throttle has crossed the floor this arm): full
                // airmode keeps roll/pitch/yaw authority at low thrust.
                let pid_output = rate_pid.update(cmd.rate_sp_degs, imu.gyro, dt);
                QUAD_X.apply(&ControlDemand {
                    thrust: cmd.thrust,
                    roll: pid_output[0],
                    pitch: pid_output[1],
                    yaw: pid_output[2],
                })
            } else {
                // Armed but still grounded (pre first throttle-up):
                // collective thrust only, no torque, and hold the rate PID
                // in reset so its integrator can't wind up before takeoff.
                rate_pid.reset();
                QUAD_X.apply_no_airmode(&ControlDemand {
                    thrust: cmd.thrust,
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                })
            };

            // `from_normalised` emits MotorStop for v ≤ 0; otherwise a
            // throttle frame. Bidir flag controls telem-bit + CRC.
            let frames: [DshotFrame; 4] = core::array::from_fn(|i| {
                DshotFrame::from_normalised(motor_outputs.motors[i], DSHOT_BIDIR)
            });

            let telemetry = dshot.send_throttles_and_receive(frames).await;

            // Telemetry log at ~10 Hz (every 800 frames at 8 kHz). Each
            // per-motor result is Erpm{period_us} / NoEdge / InvalidGcr
            // / InvalidCrc — the key signal that bidir RX is decoding.
            use core::sync::atomic::{AtomicU32, Ordering};
            static TELEM_LOG_N: AtomicU32 = AtomicU32::new(0);
            let n = TELEM_LOG_N.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
            if DSHOT_BIDIR && n.is_multiple_of(800) {
                defmt::info!(
                    "DShot RX: M1={=?} M2={=?} M3={=?} M4={=?}",
                    telemetry[0],
                    telemetry[1],
                    telemetry[2],
                    telemetry[3],
                );
            }
        } else {
            rate_pid.reset();
            let frames: [DshotFrame; 4] = [DshotFrame::motor_stop(DSHOT_BIDIR); 4];
            let _ = dshot.send_throttles_and_receive(frames).await;
        }
    }
}
