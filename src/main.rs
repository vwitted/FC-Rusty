// main.rs — Flight controller entry point
//
// Target: STM32F407VET6 (168 MHz, Cortex-M4F, 100-pin LQFP)
// Board:  WeAct-style dev board
// Framework: Embassy async executor
//
// ========== PIN ASSIGNMENTS ==========
//
// UART peripherals:
//   USART1  RX=PA10              → CRSF receiver (416666 baud, RX only)
//                                  DMA: DMA2 Stream 5 Ch 4 (RX)
//
//   USART3  TX=PB10  RX=PB11    → WT901B IMU (115200 baud, TX+RX)
//                                  DMA: DMA1 Stream 3 Ch 4 (TX)
//                                       DMA1 Stream 1 Ch 4 (RX)
//
//   USART6  TX=PC6   RX=PC7     → GPS module (9600/115200 baud, TX+RX)
//                                  DMA: DMA2 Stream 6 Ch 5 (TX)
//                                       DMA2 Stream 1 Ch 5 (RX)
//
// DShot ESC outputs (TIM3, 4 channels):
//   TIM3_CH1  PA6  → Motor 1 (front-right)
//   TIM3_CH2  PA7  → Motor 2 (rear-left)
//   TIM3_CH3  PB0  → Motor 3 (front-left)
//   TIM3_CH4  PB1  → Motor 4 (rear-right)
//                                  DMA: DMA1 Stream 4 Ch 5 (CH1)
//                                       DMA1 Stream 5 Ch 5 (CH2)
//                                       DMA1 Stream 7 Ch 5 (CH3)
//                                       DMA1 Stream 2 Ch 5 (CH4)
//
// Reserved / board-specific:
//   PA11, PA12  → USB (dev board)
//   PA13, PA14  → SWD debug (ST-Link)
//   PH0, PH1   → HSE crystal (8 MHz)
//   PC13        → On-board LED (active low on most boards)
//
// Free pins for future use:
//   PA0-PA5, PA8-PA9, PA15      → SPI sensors, buzzer, LED strip, etc.
//   PB2-PB9, PB12-PB15          → SPI (gyro), I2C (baro/mag), etc.
//   PC0-PC5, PC8-PC15           → ADC (battery voltage), SD card, etc.
//   PD0-PD15, PE0-PE15          → plenty of GPIO
//
// DMA stream allocation (no conflicts):
//   DMA1: S1=USART3_RX  S2=TIM3_CH4  S3=USART3_TX
//         S4=TIM3_CH1   S5=TIM3_CH2   S7=TIM3_CH3
//   DMA2: S1=USART6_RX  S5=USART1_RX  S6=USART6_TX
//
// =====================================

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::usart::{self, UartRx, UartTx};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker};

use defmt_rtt as _; // logging over RTT (debug probe)
use panic_probe as _; // panic handler that works with probe

// Our modules
mod drivers {
    pub mod crsf;
    pub mod dshot;
    pub mod nmea;
    pub mod wt901b;
}
mod control {
    pub mod altitude;
    pub mod arming;
    pub mod mixer;
    pub mod mpc;
    pub mod pid;
}
mod rc_task;

use drivers::crsf::RcChannels;
use drivers::dshot::{DshotFrame, DshotSpeed};
use drivers::nmea::{NmeaParser, GpsData};
use drivers::wt901b::{Wt901bParser, ImuData};
use control::arming::{ArmingStateMachine, ArmState};
use control::mixer::{ControlDemand, QUAD_X};
use control::pid::{PidGains, PidLimits, RatePidController};
use control::mpc::AttitudeMpc;
use control::altitude::{AltitudeController, AltitudeGains};

// ---- Interrupt bindings ----

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
});

// ---- Shared state between tasks ----
// Signals are "latest value wins" — perfect for real-time sensor data.

/// Latest IMU data from the WT901B task
static IMU_DATA: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

/// Latest GPS data from the GPS task
static GPS_DATA: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

// RC signals are defined in rc_task.rs:
// rc_task::RC_CHANNELS, rc_task::RC_LINK, rc_task::RC_LAST_SEEN

// ---- Main entry point ----

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ---- Clock configuration ----
    // STM32F407VET6 with 8 MHz HSE crystal on the dev board.
    //   HSE 8 MHz → PLL_M=8 → 1 MHz → PLL_N=336 → VCO 336 MHz
    //   PLL_P=2 → SYSCLK 168 MHz
    //   PLL_Q=7 → USB 48 MHz (not used yet, but correct for future)
    //   AHB  = 168 MHz (prescaler 1)
    //   APB1 = 42 MHz  (prescaler 4), APB1 timers = 84 MHz
    //   APB2 = 84 MHz  (prescaler 2), APB2 timers = 168 MHz
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
        prediv: PllPreDiv::DIV8,    // 8 MHz / 8 = 1 MHz
        mul: PllMul::MUL336,        // 1 MHz × 336 = 336 MHz VCO
        divp: Some(PllPDiv::DIV2),  // 336 / 2 = 168 MHz SYSCLK
        divq: Some(PllQDiv::DIV7),  // 336 / 7 = 48 MHz USB
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;   // 168 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV4;   // 42 MHz (timers 84 MHz)
    config.rcc.apb2_pre = APBPrescaler::DIV2;   // 84 MHz (timers 168 MHz)

    let p = embassy_stm32::init(config);

    defmt::info!("Flight controller starting");

    // ---- Configure and spawn the RC receiver task ----
    // CRSF on USART1 RX (PA10), 416666 baud
    let rc_uart = UartRx::new(
        p.USART1,
        Irqs,
        p.PA10,          // RX pin
        p.DMA2_CH5,      // DMA2 Stream 5 Ch 4
        rc_task::crsf_uart_config(),
    ).unwrap();

    spawner.spawn(rc_task::run(rc_uart)).unwrap();
    defmt::info!("RC task spawned");

    // ---- Configure and spawn the IMU task ----
    // WT901B on USART3 TX=PB10 RX=PB11, 115200 baud
    let imu_uart_config = {
        let mut c = usart::Config::default();
        c.baudrate = 115200;
        c
    };

    let (imu_tx, imu_rx) = {
        let uart = embassy_stm32::usart::Uart::new(
            p.USART3,
            p.PB11,        // RX pin
            p.PB10,        // TX pin
            Irqs,
            p.DMA1_CH3,   // DMA1 Stream 3 Ch 4 (TX)
            p.DMA1_CH1,   // DMA1 Stream 1 Ch 4 (RX)
            imu_uart_config,
        ).unwrap();
        uart.split()
    };

    spawner.spawn(imu_task(imu_tx, imu_rx)).unwrap();
    defmt::info!("IMU task spawned");

    // ---- Configure and spawn the GPS task ----
    // GPS on USART6 TX=PC6 RX=PC7, 9600 baud (NMEA default)
    let gps_uart_config = {
        let mut c = usart::Config::default();
        c.baudrate = 9600;
        c
    };

    let gps_rx = UartRx::new(
        p.USART6,
        Irqs,
        p.PC7,           // RX pin
        p.DMA2_CH1,      // DMA2 Stream 1 Ch 5 (RX)
        gps_uart_config,
    ).unwrap();

    spawner.spawn(gps_task(gps_rx)).unwrap();
    defmt::info!("GPS task spawned");

    // ---- Run the control loop on the main task ----
    // This is deliberate: the control loop is the highest priority
    // work, so it runs on the main executor rather than being
    // spawned as a separate task.
    control_loop().await;
}

// ---- GPS Task ----
// Reads NMEA sentences from the GPS module, publishes via GPS_DATA signal.

#[embassy_executor::task]
async fn gps_task(
    mut rx: UartRx<'static, embassy_stm32::mode::Async>,
) {
    let mut parser = NmeaParser::new();
    let mut buf = [0u8; 128]; // NMEA sentences up to 82 chars

    defmt::info!("GPS task started");

    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                for &byte in &buf {
                    if parser.push_byte(byte).is_some() {
                        GPS_DATA.signal(parser.data);
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
    mut tx: UartTx<'static, embassy_stm32::mode::Async>,
    mut rx: UartRx<'static, embassy_stm32::mode::Async>,
) {
    // Give the IMU a moment to boot
    embassy_time::Timer::after(Duration::from_millis(500)).await;

    // Configure the WT901B for our needs:
    // 1. Unlock
    // 2. Set 200 Hz output rate
    // 3. Set output content: accel + gyro + angle + quaternion + baro
    // 4. Set bandwidth to 188 Hz (good for 200 Hz output)
    // 5. Save

    use drivers::wt901b::{UNLOCK, SAVE, config};

    let commands: &[[u8; 5]] = &[
        UNLOCK,
        config::set_output_rate(0x0B),           // 200 Hz
        config::set_output_content(0x024E),       // acc+gyro+angle+baro+quat
        config::set_bandwidth(0x01),              // 188 Hz
        SAVE,
    ];

    for cmd in commands {
        let _ = tx.write(cmd).await;
        embassy_time::Timer::after(Duration::from_millis(50)).await;
    }

    defmt::info!("WT901B configured");

    // Now read data continuously
    let mut parser = Wt901bParser::new();
    let mut buf = [0u8; 32];

    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                for &byte in &buf {
                    if parser.push_byte(byte).is_some() {
                        // Publish the latest data snapshot
                        IMU_DATA.signal(parser.data);
                    }
                }
            }
            Err(e) => {
                defmt::warn!("IMU UART error: {:?}", e);
                embassy_time::Timer::after(Duration::from_millis(1)).await;
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

async fn control_loop() -> ! {
    use core::f32::consts::PI;
    const DEG2RAD: f32 = PI / 180.0;
    const RAD2DEG: f32 = 180.0 / PI;

    // ---- DShot setup ----
    // TIM3 at 84 MHz (APB1 timer clock), DShot600
    // TODO: configure TIM3 hardware + DMA channels for PA6/PA7/PB0/PB1
    let dshot_speed = DshotSpeed::Dshot600;
    let timer_clock = 84_000_000u32;
    let t1h = dshot_speed.t1h_ticks(timer_clock);
    let t0h = dshot_speed.t0h_ticks(timer_clock);
    let mut dma_bufs = [[0u16; 18]; 4];

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

    // ---- Loop timing instrumentation ----
    // Tracks how long each control loop iteration takes.
    // If loop_time exceeds 5ms (200 Hz budget), we're overrunning.
    let mut loop_time_us_max: u32 = 0;
    let mut loop_time_us_sum: u32 = 0;
    let mut mpc_time_us_max: u32 = 0;
    let mut mpc_time_us_last: u32 = 0;
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
        // TIM3 CH1-4 → PA6 (M1), PA7 (M2), PB0 (M3), PB1 (M4)
        if armed {
            for i in 0..4 {
                let frame = DshotFrame::from_normalised(
                    motor_outputs.motors[i],
                    false,
                );
                frame.fill_dma_buffer(&mut dma_bufs[i], t1h, t0h);
            }
            // TODO: trigger DMA transfers on TIM3 channels
            // DMA1 Stream 4 (CH1), Stream 5 (CH2), Stream 7 (CH3), Stream 2 (CH4)
        } else {
            for i in 0..4 {
                let frame = DshotFrame::disarmed();
                frame.fill_dma_buffer(&mut dma_bufs[i], t1h, t0h);
            }
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
                "loop: avg={}us max={}us mpc_max={}us mpc_last={}us overruns={}",
                loop_avg,
                loop_time_us_max,
                mpc_time_us_max,
                mpc_time_us_last,
                overrun_count,
            );

            // Reset timing stats each reporting period
            loop_time_us_max = 0;
            loop_time_us_sum = 0;
            mpc_time_us_max = 0;
            timing_sample_count = 0;
        }
    }
}
