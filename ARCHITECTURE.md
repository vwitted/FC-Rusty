# FC-Rusty — Architecture

## Overview

An Embassy-based async flight controller in Rust, targeting the
**DAKEFPV H743** (STM32H743VIT6, Cortex-M7F @ 480 MHz, 1 MB SRAM).
The design separates concerns into independent async tasks that
communicate via lock-free signals. No shared mutable state, no mutexes
in the hot path.

---

## Module Structure

```
fc-firmware/
├── Cargo.toml
├── memory.x                    # Linker script (SRAM1 reserved for DMA buffers)
├── src/
│   ├── main.rs              # Entry point, task spawning, pin assignment
│   │
│   ├── drivers/              # Hardware abstraction (one module per device)
│   │   ├── mod.rs
│   │   ├── crsf.rs           # CRSF receiver protocol (UART Rx)
│   │   ├── icm42688.rs       # Dual ICM-42688P IMUs (SPI)
│   │   │                     # Provides raw accel, gyro, temperature
│   │   │                     # Runs at 8 kHz
│   │   ├── ublox.rs          # UBlox GPS (UART, UBX binary protocol)
│   │   └── dshot.rs          # DShot ESC output (Timer + DMA)
│   │
│   ├── state/                # State estimation and sensor fusion
│   │   ├── mod.rs
│   │   ├── types.rs          # Core data types (Attitude, Position, etc.)
│   │   ├── estimator.rs      # State estimator (initially passthrough from
│   │   │                     # internal baro (DPS310), accel (ICM-42688P), MEKF)
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
│  [dual_icm_task]    Reads ICM-42688P SPI sensors at 8 kHz        │
│       │             Publishes: RawImu                            │
│       ▼                                                          │
│  Signal<RawImu>                                                  │
│       │                                                          │
│       ▼                                                          │
│  [mekf_task]        Kalman filter fuses raw IMU data at 8 kHz    │
│       │             Publishes: ImuData (fused attitude)          │
│       ▼                                                          │
│  Signal<ImuData>                                                 │
│  Signal<ImuDataForNav>                                           │
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
│  │     navigation_task     │  OUTER LOOP                    │
│  │                         │                                │
│  │  1. Read ImuDataForNav  │  Runs at 100 Hz                │
│  │  2. Read RcChannels     │                                │
│  │  3. Mode logic          │                                │
│  │  4. PosHold / AltHold   │                                │
│  │  5. MPC solve           │                                │
│  └───────────┬─────────────┘                                │
│              │ Watch<OuterLoopCommand>                      │
│              ▼                                              │
│  ┌─────────────────────────┐                                │
│  │     control_loop        │  FAST INNER LOOP               │
│  │                         │                                │
│  │  1. Wait for ImuData    │  Runs at 8 kHz synced to IMU   │
│  │  2. Read OuterLoopCmd   │                                │
│  │  3. Rate PID controller │                                │
│  │  4. Mixer               │                                │
│  │  5. Write DShot cmds    │                                │
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
/// What the MEKF estimator produces (from ICM-42688P)
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
100 Hz Outer Loop (`navigation_task`):
┌─────────────────────────────────────────┐
│  Read ImuDataForNav (Signal)            │
│  Read RcChannels (Signal)               │
│  Read GPS / PosEstimate                 │
│  Mode logic → Pos/Alt Hold              │
│                                         │
│  mpc.solve() → rate setpoints           │
│  Write OuterLoopCommand (Watch)         │
└─────────────────────────────────────────┘

8 kHz Inner Loop (`control_loop`):
┌─────────────────────────────────────────┐
│  Wait for ImuData (Signal)              │
│  try_get() OuterLoopCommand (Watch)     │
│                                         │
│  error = outer_setpoint - gyro_rates    │
│  pid_output = pid.update(error, dt)     │
│                                         │
│  Mixer:                                 │
│  motor_cmds = mix(thrust, pid_out)      │
│                                         │
│  Write DShot commands                   │
└─────────────────────────────────────────┘
```

## Design Decisions / Open Questions

1. **Using dual IMUs, with barometer, GPS as core inertial sensing**
   Kalman-fused attitude via MEKF (8 kHz).
   MPC outer loop runs at 100 Hz, with simple PID inner loop at 8 kHz.
   This fulfills the original goal to maximize rates on the STM32H7.
2. **Split inner/outer tasks**: The architecture has transitioned from a single unified control task to a split inner/outer loop. This guarantees the heavy MPC math never stalls the strict 8 kHz PID tracking. Communication uses an `embassy_sync::watch::Watch` to share targets lock-free.

3. **DShot implementation**: Timer + DMA. Relies heavily on our own implementation for DMA and Timer configuration for H7.

4. **tinympc-rs integration**: Use as a cargo dependency. The
   vehicle model (A, B matrices) goes in config/vehicle.rs.
   Cost weights (Q, R) go in config/params.rs. MPC wrapper in
   control/mpc.rs handles the solve loop and reference generation.

5. **Arming safety**: Need an arming state machine — disarmed by
   default, arm only when throttle low + specific stick gesture
   or switch. All motor output zero when disarmed.

6. **Failsafe**: What happens when RC link is lost? Timer-based
   detection from CRSF link quality. Options: hold position,
   return to home, or cut motors (configurable).

```
