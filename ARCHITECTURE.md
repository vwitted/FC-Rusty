# Flight Controller Architecture

## Overview

An Embassy-based async flight controller in Rust, targeting the
STM32F405 (Cortex-M4F, 168 MHz). The design separates concerns into
independent async tasks that communicate via lock-free channels.

## Crate / Module Structure

```
fc-firmware/
├── Cargo.toml
├── src/
│   ├── main.rs              # Entry point, task spawning, pin assignment
│   │
│   ├── drivers/              # Hardware abstraction (one module per device)
│   │   ├── mod.rs
│   │   ├── crsf.rs           # CRSF receiver protocol (UART Rx)
│   │   ├── wt901b.rs         # WitMotion WT901B IMU (UART or I2C)
│   │   │                     # Provides: accel, gyro, mag, baro, quaternion
│   │   │                     # Onboard Kalman filter, up to 200 Hz
│   │   ├── ublox.rs          # UBlox GPS (UART, UBX binary protocol)
│   │   └── dshot.rs          # DShot ESC output (Timer + DMA)
│   │
│   ├── state/                # State estimation and sensor fusion
│   │   ├── mod.rs
│   │   ├── types.rs          # Core data types (Attitude, Position, etc.)
│   │   ├── estimator.rs      # State estimator (initially passthrough from
│   │   │                     # WitMotion, later own EKF if needed)
│   │   └── calibration.rs    # Sensor calibration (mag, accel offsets)
│   │
│   ├── control/              # The control system
│   │   ├── mod.rs
│   │   ├── mode.rs           # Flight mode logic (Acro, Angle, PosHold...)
│   │   ├── setpoint.rs       # RC input → reference trajectory conversion
│   │   ├── mpc.rs            # MPC outer loop (wraps tinympc-rs)
│   │   ├── pid.rs            # PID inner loop (rate controller)
│   │   └── mixer.rs          # Abstract demands → per-motor commands
│   │
│   ├── comms/                # Telemetry and configuration
│   │   ├── mod.rs
│   │   ├── mavlink.rs        # MAVLink for GCS (optional, over UART/USB)
│   │   └── blackbox.rs       # Flight logging (to flash or SD)
│   │
│   └── config/               # System configuration
│       ├── mod.rs
│       ├── params.rs         # Tunable parameters (PID gains, MPC weights)
│       └── vehicle.rs        # Vehicle geometry (mixer matrix, mass, etc.)
```

## Task Model (Embassy)

Each major subsystem runs as an independent async task. Tasks communicate
via Embassy's `Channel` (MPSC) or `Signal` (latest-value-wins) primitives.
No shared mutable state, no mutexes in the hot path.

```
┌─────────────────────────────────────────────────────────────┐
│                        SPAWNED TASKS                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [imu_task]         Reads WT901B at up to 200 Hz             │
│       │             Publishes: AttitudeEstimate               │
│       │             Also provides baro alt + mag heading      │
│       │             (not every field fresh every packet at     │
│       │              max rate — driver caches last known)      │
│       ▼                                                     │
│  Signal<AttitudeEstimate>                                   │
│       │                                                     │
│       │  ┌──────────┐                                       │
│       │  │ rc_task   │  Reads CRSF frames from UART Rx      │
│       │  │           │  Publishes: RcChannels                │
│       │  │  ~150 Hz  │                                       │
│       │  └─────┬─────┘                                      │
│       │        │                                             │
│       │        ▼                                             │
│       │  Signal<RcChannels>                                 │
│       │        │                                             │
│       ▼        ▼                                             │
│  ┌─────────────────────────┐                                │
│  │     control_task        │  THE MAIN LOOP                 │
│  │                         │                                │
│  │  1. Read latest state   │  Runs at IMU output rate       │
│  │  2. Read latest RC      │  or at a fixed tick rate       │
│  │  3. Mode logic →        │                                │
│  │     setpoints           │                                │
│  │  4. MPC solve (if       │                                │
│  │     outer loop tick)    │                                │
│  │  5. PID inner loop      │                                │
│  │  6. Mixer               │                                │
│  │  7. Write motor cmds    │                                │
│  └───────────┬─────────────┘                                │
│              │                                               │
│              ▼                                               │
│  [motor_output]  DShot via Timer+DMA                        │
│                  (could be inline or separate task)          │
│                                                             │
│  [gps_task]      Reads UBlox, publishes GpsFix              │
│                  ~5-10 Hz, feeds estimator                   │
│                                                             │
│  [logging_task]  Consumes state + control data              │
│                  Writes to flash/SD at lower priority        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Core Data Types

These flow between tasks via channels/signals:

```rust
/// What the IMU/estimator produces (from WT901B)
pub struct AttitudeEstimate {
    pub quaternion: [f32; 4],     // Orientation (onboard Kalman fused)
    pub gyro_rates: [f32; 3],    // Angular velocity (rad/s)
    pub accel: [f32; 3],         // Linear acceleration (m/s²)
    pub mag: [f32; 3],           // Magnetometer (Gauss)
    pub baro_altitude: f32,      // Barometric altitude (m)
    pub baro_pressure: f32,      // Barometric pressure (hPa)
    pub timestamp_us: u64,
}

/// What the RC receiver produces
pub struct RcChannels {
    pub channels: [u16; 16],     // Raw channel values
    pub rssi: u8,
    pub link_quality: u8,
    pub timestamp_us: u64,
}

/// What the mode/setpoint logic produces for the controller
pub struct ControlReference {
    pub mode: FlightMode,
    pub target_rates: [f32; 3],  // Desired roll/pitch/yaw rates
    pub target_angles: [f32; 3], // Desired roll/pitch/yaw (if angle mode)
    pub target_thrust: f32,      // Collective thrust command
}

/// What the control loop produces
pub struct MotorCommands {
    pub motors: [u16; 4],        // DShot values per motor
    pub armed: bool,
}

/// Flight modes
pub enum FlightMode {
    /// Sticks control angular rates directly
    Acro,
    /// Sticks control angles, controller holds attitude
    Angle,
    /// Sticks control velocity, controller holds position (needs GPS)
    PosHold,
    /// Controller returns to launch point (needs GPS)
    ReturnToHome,
}
```

## Control Loop Detail

The control loop runs in two layers, which can be in the same task
but execute at different rates:

```
Every IMU sample (~200 Hz from WT901B):
┌─────────────────────────────────────────┐
│  Read AttitudeEstimate                  │
│  Read RcChannels                        │
│  Mode logic → ControlReference          │
│                                         │
│  if mpc_tick:          (every N-th cycle, e.g. 50 Hz)
│  │  Build x_ref from ControlReference   │
│  │  Set initial state from estimate     │
│  │  mpc.solve() → outer loop setpoints  │
│  │                                      │
│  PID inner loop:       (every cycle)    │
│  │  error = outer_setpoint - gyro_rates │
│  │  pid_output = pid.update(error, dt)  │
│  │                                      │
│  Mixer:                                 │
│  │  motor_cmds = mix(thrust, pid_out)   │
│  │                                      │
│  Write DShot commands                   │
└─────────────────────────────────────────┘
```

## Design Decisions / Open Questions

1. **WT901B as primary IMU**: Gets us flying fast with onboard
   Kalman-fused attitude. Includes gyro, accel, mag, and baro in
   one 15mm package. Limits inner loop rate to ~200 Hz. Acceptable
   for MPC at 50 Hz + simple PID at 200 Hz. At max rate, not all
   data fields arrive every packet — driver should cache last known
   values. If we later need faster raw gyro rates, add a dedicated
   IMU (e.g. ICM-42688) on SPI.

2. **Single control task vs split inner/outer**: Starting with a
   single task that runs both loops at different rates is simpler.
   Can split later if needed.

3. **DShot implementation**: Timer + DMA on STM32F405. Well
   supported in embassy-stm32 — TIM1/TIM2/TIM3/TIM4 all have
   DMA-capable channels suitable for DShot.

4. **STM32F405 platform**: First-class Embassy support via
   embassy-stm32. Cortex-M4F at 168 MHz, 192 kB SRAM, 1 MB
   flash. Proven to run TinyMPC (Crazyflie uses this exact chip).
   Plenty of UARTs (for CRSF, WT901B, GPS, telemetry), SPI and
   I2C buses, and timer channels for DShot.

5. **tinympc-rs integration**: Use as a cargo dependency. The
   vehicle model (A, B matrices) goes in config/vehicle.rs.
   Cost weights (Q, R) go in config/params.rs. MPC wrapper in
   control/mpc.rs handles the solve loop and reference generation.

6. **Arming safety**: Need an arming state machine — disarmed by
   default, arm only when throttle low + specific stick gesture
   or switch. All motor output zero when disarmed.

7. **Failsafe**: What happens when RC link is lost? Timer-based
   detection from CRSF link quality. Options: hold position,
   return to home, or cut motors (configurable).
```
