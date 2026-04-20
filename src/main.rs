// main.rs — Flight controller entry point
//
// Target: STM32F722RET6 (216 MHz, Cortex-M7F, 64-pin LQFP)
// Board:  Radiolink F722 (after the SpeedyBee F7 V3 suffered a dead
//         5V BEC during IMU rework — see git log for context)
// Framework: Embassy async executor
//
// Pin map (Radiolink F722):
//   USART1  TX=PA9   RX=PA10    → T1/R1 pads, WT901B IMU (full duplex)
//   USART2  RX=PA3              → R2 pad, CRSF receiver (416666 baud)
//   USART3  TX=PB10             → T3 pad, defmt output (raw-reg logger, 115200)
//   USART6  TX=PC6   RX=PC7     → T6/R6 pads, GPS (UBX binary)
//   UART4   RX=PA1              → ESC telemetry (internal, not wired yet)
//
// The onboard ICM-42688P (SPI1) and baro (BMP280 on I2C1) are unused
// for now; WT901B over UART remains the primary IMU. Moving to the
// SPI IMU is a post-Alpha optimisation (would need a new driver plus
// an AHRS filter to recover Euler angles).
//
// Motors (multi-timer DShot600 via three parallel DMA streams):
//   TIM2_CH1 → PA15 → M1 (rear-right,  CW)
//   TIM2_CH2 → PB3  → M2 (front-right, CCW)
//   TIM3_CH1 → PB4  → M3 (rear-left,   CCW)
//   TIM4_CH1 → PB6  → M4 (front-left,  CW)
//   See src/drivers/dshot_hw.rs for DMA stream / timing details.
//
// Flashing: board has no SWD; use DFU over USB-C. Hold BOOT while
// plugging in, then run `scripts/flash-dfu.sh` (see that script for
// the one-off cargo-binutils / dfu-util install).

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::usart::{self, Uart, UartRx};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker};

use panic_probe as _; // panic handler that works with probe

// Our modules
mod drivers {
    pub mod crsf;
    pub mod dshot;
    pub mod dshot_hw;
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
mod logger;
mod rc_task;

use drivers::crsf::RcChannels;
use drivers::dshot::{DshotFrame, DshotSpeed};
use drivers::dshot_hw::DshotQuad;
use drivers::nmea::{NmeaParser, GpsData};
use drivers::wt901b::{Wt901bParser, ImuData};
use control::arming::{ArmingStateMachine, ArmState};
use control::mixer::{ControlDemand, QUAD_X};
use control::pid::{PidGains, PidLimits, RatePidController};
use control::mpc::AttitudeMpc;
use control::altitude::{AltitudeController, AltitudeGains};

// ---- Interrupt bindings ----

// USART3 is owned by `logger::init_usart3()` (raw register TX for defmt)
// — no Embassy interrupt handler needed here.
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
});

// ---- Shared state between tasks ----
// Signals are "latest value wins" — perfect for real-time sensor data.

/// Latest IMU data from the WT901B task
static IMU_DATA: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Latest GPS data from the GPS task
static GPS_DATA: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

/// Commands from the control loop to the IMU command task (TX side).
/// Used for in-field magnetometer calibration via AUX channel.
#[derive(Clone, Copy)]
enum ImuCommand {
    /// Enter mag-field calibration mode (CALSW=0x02, unlocked).
    StartMagCal,
    /// Exit calibration mode and persist bias to flash.
    SaveMagCal,
    /// Exit calibration mode without saving.
    AbortMagCal,
}

static IMU_COMMAND: Signal<CriticalSectionRawMutex, ImuCommand> = Signal::new();

// RC signals are defined in rc_task.rs:
// rc_task::RC_CHANNELS, rc_task::RC_LINK, rc_task::RC_LAST_SEEN

// ---- Main entry point ----

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ---- Clock configuration ----
    // STM32F722RET6 with 8 MHz HSE crystal on the SpeedyBee F7 V3.
    //   HSE 8 MHz → PLL_M=4 → 2 MHz → PLL_N=216 → VCO 432 MHz
    //   PLL_P=2 → SYSCLK 216 MHz  (F722 max)
    //   PLL_Q=9 → USB 48 MHz      (432/9 = 48, exact)
    //   AHB  = 216 MHz (prescaler 1)
    //   APB1 = 54 MHz  (prescaler 4), APB1 timers = 108 MHz
    //   APB2 = 108 MHz (prescaler 2), APB2 timers = 216 MHz
    use embassy_stm32::rcc::{
        Hse, HseMode, Pll, PllMul, PllPreDiv, PllPDiv, PllQDiv,
        PllSource, Sysclk, APBPrescaler, AHBPrescaler,
    };
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,    // 8 MHz / 4 = 2 MHz VCO input
        mul: PllMul::MUL216,        // 2 MHz × 216 = 432 MHz VCO
        divp: Some(PllPDiv::DIV2),  // 432 / 2 = 216 MHz SYSCLK
        divq: Some(PllQDiv::DIV9),  // 432 / 9 = 48 MHz USB
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;    // 216 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV4;   // 54 MHz (timers 108 MHz)
    config.rcc.apb2_pre = APBPrescaler::DIV2;   // 108 MHz (timers 216 MHz)

    let p = embassy_stm32::init(config);

    // Bring up USART3 TX (PB10) for defmt output before anything else
    // so the first defmt::info! below actually lands on the wire.
    logger::init_usart3();

    defmt::info!("Flight controller starting");

    // ---- Configure and spawn the RC receiver task ----
    // CRSF on USART2 RX (PA3), 416666 baud — R2 pad on the Radiolink F722.
    let rc_uart = UartRx::new(
        p.USART2,
        Irqs,
        p.PA3,           // RX pin
        p.DMA1_CH5,      // USART2_RX → DMA1 Stream 5
        rc_task::crsf_uart_config(),
    ).unwrap();

    spawner.spawn(rc_task::run(rc_uart)).unwrap();
    defmt::info!("RC task spawned");

    // ---- WT901B IMU on USART1 (full duplex) ----
    // Start at factory 9600 baud, configure over UART, then switch
    // to 115200. TX=PA9 (T1 pad), RX=PA10 (R1 pad).
    let imu_uart_config = {
        let mut c = usart::Config::default();
        c.baudrate = 9600;
        c
    };

    let imu_uart = Uart::new(
        p.USART1,
        p.PA10,          // RX
        p.PA9,           // TX
        Irqs,
        p.DMA2_CH7,      // USART1_TX → DMA2 Stream 7
        p.DMA2_CH2,      // USART1_RX → DMA2 Stream 2
        imu_uart_config,
    ).unwrap();

    // Split into TX+RX, auto-detect baud + configure. Hand TX to a
    // dedicated command task so the control loop can trigger in-field
    // mag calibration over AUX channel 7 (see imu_command_task).
    let (mut imu_tx, mut imu_rx) = imu_uart.split();
    let imu_baud = drivers::wt901b::configure(&mut imu_tx, &mut imu_rx).await;

    spawner.spawn(imu_task(imu_rx)).unwrap();
    spawner.spawn(imu_command_task(imu_tx)).unwrap();
    defmt::info!("IMU task spawned at {} baud", imu_baud);

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
        p.PC7,           // RX
        p.PC6,           // TX
        Irqs,
        p.DMA2_CH6,      // TX DMA
        p.DMA2_CH1,      // RX DMA
        gps_uart_config,
    ).unwrap();

    let (_gps_tx, gps_rx) = gps_uart.split();
    spawner.spawn(gps_task(gps_rx)).unwrap();
    defmt::info!("GPS task spawned (NMEA at 9600)");

    // ---- DShot ESC outputs (multi-timer across TIM2/TIM3/TIM4) ----
    // M1=PA15 (TIM2_CH1), M2=PB3 (TIM2_CH2),
    // M3=PB4  (TIM3_CH1), M4=PB6 (TIM4_CH1).
    let dshot = DshotQuad::new(
        p.TIM2, p.TIM3, p.TIM4,
        p.PA15, p.PB3, p.PB4, p.PB6,
        p.DMA1_CH7,      // TIM2_UP
        p.DMA1_CH2,      // TIM3_UP
        p.DMA1_CH6,      // TIM4_UP
        DshotSpeed::Dshot600,
    );
    defmt::info!("DShot (TIM2+TIM3+TIM4, DShot600) initialised");

    // ---- Run the control loop on the main task ----
    // This is deliberate: the control loop is the highest priority
    // work, so it runs on the main executor rather than being
    // spawned as a separate task.
    control_loop(dshot).await;
}

// ---- GPS Task ----
// Reads NMEA sentences from the GPS module, publishes via GPS_DATA signal.

#[embassy_executor::task]
async fn gps_task(
    mut rx: UartRx<'static, embassy_stm32::mode::Async>,
) {
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

                        if !announced_first {
                            defmt::info!(
                                "GPS: first NMEA sentence parsed — stream is alive"
                            );
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

// ---- IMU Task ----
// Reads WT901B data, optionally configures it on startup,
// then publishes parsed data via the IMU_DATA signal.

#[embassy_executor::task]
async fn imu_task(
    mut rx: UartRx<'static, embassy_stm32::mode::Async>,
) {
    defmt::info!("IMU task started");

    let mut parser = Wt901bParser::new();
    let mut buf = [0u8; 32];
    let mut pkt_count: u32 = 0;

    // =========================================================================
    // TEMP DEBUG — IMU framing diagnosis on Radiolink F722 (2026-04-20)
    // Remove this entire block once soldering / framing is confirmed good.
    // Look for: `// END TEMP DEBUG` marker below.
    // =========================================================================
    let mut read_count: u32 = 0;
    let mut bytes_since_report: u32 = 0;
    let mut pkts_since_report: u32 = 0;
    let mut last_report = embassy_time::Instant::now();
    // END TEMP DEBUG (part 1/3)

    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                // TEMP DEBUG (part 2/3) — remove with part 1 & 3
                read_count += 1;
                bytes_since_report += buf.len() as u32;
                if read_count % 50 == 0 {
                    defmt::info!("[IMU-DEBUG] raw[0..16]: {=[u8]:02x}", buf[..16]);
                }
                // END TEMP DEBUG (part 2/3)

                for &byte in &buf {
                    if parser.push_byte(byte).is_some() {
                        pkt_count += 1;
                        pkts_since_report += 1; // TEMP DEBUG
                        IMU_DATA.signal(parser.data);

                        // Log first successful packet so we know it's alive
                        if pkt_count == 1 {
                            defmt::info!(
                                "IMU first packet! accel=[{:?},{:?},{:?}] gyro=[{:?},{:?},{:?}]",
                                parser.data.accel[0], parser.data.accel[1], parser.data.accel[2],
                                parser.data.gyro[0], parser.data.gyro[1], parser.data.gyro[2],
                            );
                        }
                    }
                }

                // TEMP DEBUG (part 3/3) — remove with part 1 & 2
                if last_report.elapsed() >= Duration::from_secs(1) {
                    defmt::info!(
                        "[IMU-DEBUG] 1s window: {} bytes, {} pkts parsed (total {})",
                        bytes_since_report, pkts_since_report, pkt_count
                    );
                    bytes_since_report = 0;
                    pkts_since_report = 0;
                    last_report = embassy_time::Instant::now();
                }
                // END TEMP DEBUG (part 3/3)
            }
            Err(e) => {
                defmt::warn!("IMU UART error: {:?}", e);
                embassy_time::Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

// ---- IMU Command Task ----
// Owns the WT901B's UART TX half and dispatches mag-cal commands
// posted to IMU_COMMAND from the control loop. Each command waits
// 200 ms between bytes (WT901B datasheet requirement) before
// returning, so this task must NOT be on the control-loop's
// critical path — which is why it lives here, not inline.

#[embassy_executor::task]
async fn imu_command_task(
    mut tx: usart::UartTx<'static, embassy_stm32::mode::Async>,
) {
    use drivers::wt901b::{config, SAVE, UNLOCK};
    use embassy_time::Timer;

    let step = Duration::from_millis(200);

    loop {
        let cmd = IMU_COMMAND.wait().await;
        match cmd {
            ImuCommand::StartMagCal => {
                let _ = tx.write(&UNLOCK).await;
                Timer::after(step).await;
                let _ = tx.write(&config::start_mag_calibration()).await;
                Timer::after(step).await;
            }
            ImuCommand::SaveMagCal => {
                let _ = tx.write(&UNLOCK).await;
                Timer::after(step).await;
                let _ = tx.write(&config::exit_calibration()).await;
                Timer::after(step).await;
                let _ = tx.write(&SAVE).await;
                Timer::after(step).await;
            }
            ImuCommand::AbortMagCal => {
                let _ = tx.write(&UNLOCK).await;
                Timer::after(step).await;
                let _ = tx.write(&config::exit_calibration()).await;
                Timer::after(step).await;
            }
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
    let mut arming = ArmingStateMachine::new();

    // ---- MPC attitude outer loop (50 Hz) ----
    let mut mpc = AttitudeMpc::new();
    let mut rate_sp_degs = [0.0f32; 3]; // persisted between MPC solves

    // ---- Altitude hold (50 Hz) ----
    let hover_throttle: f32 = 0.294; // tune per aircraft: mass*g / max_thrust
    let alt_gains = AltitudeGains { kp: 0.15, kd: 0.1, ki: 0.05 };
    let mut alt_ctrl = AltitudeController::new(alt_gains, hover_throttle);
    let mut current_thrust = hover_throttle;

    // ---- PID rate inner loop (200 Hz) ----
    // Gains tuned for real hardware with motor lag (~30ms ESC+motor).
    // Adjust Kp/Ki/Kd during bench testing with props off first.
    let rate_gains = PidGains { kp: 0.02, ki: 0.005, kd: 0.001 };
    let yaw_gains = PidGains { kp: 0.03, ki: 0.005, kd: 0.0 };
    let limits = PidLimits { integral_max: 0.3, output_max: 0.5, d_lpf_tau_s: 0.008 };
    let mut rate_pid = RatePidController::new(rate_gains, rate_gains, yaw_gains, limits);

    // ---- Sensor state ----
    let mut last_rc = RcChannels { channels: [992; 16] };
    let mut last_imu = ImuData::new();
    let mut last_gps = GpsData::new();
    let mut imu_last_seen = Instant::now();
    let mut control_demand = ControlDemand::default();

    // ---- Magnetometer calibration state (AUX ch7) ----
    // Switch high = enter cal mode; switch low = save if held >10s,
    // otherwise abort. `ch7_seen_low` guards against accidental start
    // when the user boots with the switch already high.
    const MAG_CAL_CH7_HI: u16 = 1500;
    const MAG_CAL_MIN_SECS: u64 = 10;
    let mut last_ch7_high = false;
    let mut ch7_seen_low = false;
    let mut mag_cal_start: Option<Instant> = None;

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
        if let Some(rc) = rc_task::RC_CHANNELS.try_take() {
            last_rc = rc;
        }

        // ---- 2. Arming state machine ----
        let arm_switch = last_rc.channels[4] > 1500;
        let throttle_raw = RcChannels::to_unit(last_rc.channels[2]);
        let imu_age_ms = imu_last_seen.elapsed().as_millis() as u32;
        let rc_age_ms = rc_task::rc_last_seen_ms();

        let arm_state = arming.update(
            arm_switch,
            throttle_raw,
            last_imu.angle[0],  // roll deg
            last_imu.angle[1],  // pitch deg
            imu_age_ms,
            rc_age_ms,
        );
        let armed = arm_state == ArmState::Armed;

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
            );
            defmt::info!(
                "arm rejected: thr_low={} level={} imu={} rc={} | thr={}% roll={}° pitch={}° imu_age={}ms rc_age={}ms ch4={} ch5={}",
                c.throttle_low, c.attitude_level, c.imu_fresh, c.rc_link_active,
                (throttle_raw * 100.0) as i32,
                last_imu.angle[0] as i32,
                last_imu.angle[1] as i32,
                imu_age_ms, rc_age_ms,
                last_rc.channels[4], last_rc.channels[5],
            );
        }

        // ---- 2b. Mag calibration edge detection on AUX ch7 ----
        let ch7_high = last_rc.channels[6] > MAG_CAL_CH7_HI;
        if !ch7_high {
            ch7_seen_low = true;
        }
        let ch7_rising = ch7_high && !last_ch7_high && ch7_seen_low;
        let ch7_falling = !ch7_high && last_ch7_high;
        last_ch7_high = ch7_high;

        if ch7_rising && mag_cal_start.is_none() {
            if armed {
                defmt::warn!("MAG CAL: ignored — disarm first");
            } else if throttle_raw > 0.05 {
                defmt::warn!("MAG CAL: ignored — throttle not idle");
            } else {
                mag_cal_start = Some(Instant::now());
                defmt::info!(
                    "MAG CAL: START — rotate through all axes for >{}s, flip ch7 low to save",
                    MAG_CAL_MIN_SECS,
                );
                IMU_COMMAND.signal(ImuCommand::StartMagCal);
            }
        }

        if ch7_falling {
            if let Some(start) = mag_cal_start.take() {
                let secs = start.elapsed().as_secs();
                if secs >= MAG_CAL_MIN_SECS {
                    defmt::info!("MAG CAL: saved after {}s", secs);
                    IMU_COMMAND.signal(ImuCommand::SaveMagCal);
                } else {
                    defmt::warn!("MAG CAL: too short ({}s), not saved", secs);
                    IMU_COMMAND.signal(ImuCommand::AbortMagCal);
                }
            }
        }

        if let Some(start) = mag_cal_start {
            if cycle_count % 200 == 0 {
                defmt::info!("MAG CAL: rotating... {}s", start.elapsed().as_secs());
            }
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

                // Altitude hold (uses barometric altitude from IMU or GPS)
                // TODO: fuse baro + GPS altitude for better estimate
                let alt_m = last_imu.altitude_cm as f32 / 100.0;
                let vz_up = 0.0; // TODO: estimate from baro rate or GPS vz
                let target_alt = alt_m; // hold current altitude for now
                current_thrust = alt_ctrl.update(target_alt, alt_m, vz_up, dt * 4.0);
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
        let frames: [DshotFrame; 4] = if armed {
            [
                DshotFrame::from_normalised(motor_outputs.motors[0], false),
                DshotFrame::from_normalised(motor_outputs.motors[1], false),
                DshotFrame::from_normalised(motor_outputs.motors[2], false),
                DshotFrame::from_normalised(motor_outputs.motors[3], false),
            ]
        } else {
            [DshotFrame::disarmed(); 4]
        };
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
