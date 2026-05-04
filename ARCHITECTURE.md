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
│   ├── main.rs                 # Entry point, clock config, task spawning, pin map,
│   │                           # control_loop (runs on main task — highest priority)
│   ├── rc_task.rs              # CRSF receiver task + shared RC signals
│   ├── logger.rs               # Raw USART6 TX writer for defmt output
│   ├── attitude_mekf.rs        # Quaternion MEKF (gyro-bias state), 8 kHz predict
│   ├── estimation.rs           # 6-state position KF (GPS + baro + IMU)
│   │
│   ├── drivers/
│   │   ├── icm42688.rs         # ICM-42688P SPI driver; dual-sensor averaged reads
│   │   ├── baro.rs             # Baro driver (DPS310-compat path); SPL06-001 driver needed
│   │   ├── crsf.rs             # CRSF RC protocol (UART Rx)
│   │   ├── nmea.rs             # NMEA GPS parser (GGA/RMC/GSA/VTG)
│   │   ├── ubx.rs              # u-blox UBX binary parser — written, not yet active
│   │   ├── dshot_hw.rs         # DShot600: TIM2 DMAR burst, all 4 channels
│   │   ├── dshot_diag.rs       # Boot-time + runtime register diagnostics
│   │   └── wt901b.rs           # WT901B parser; ImuData type still used internally
│   │
│   ├── control/
│   │   ├── pid.rs              # 3-axis rate PID (200 Hz inner loop)
│   │   ├── mpc.rs              # Attitude MPC — wraps tinympc-rs (50 Hz outer loop)
│   │   ├── altitude.rs         # Altitude hold PID (50 Hz, gated on PosKF.ready)
│   │   ├── position.rs         # Position PD for GPS rescue / pos-hold — written,
│   │   │                       # not yet wired into the control loop
│   │   ├── arming.rs           # Pre-arm checks + arming FSM + failsafe
│   │   ├── mixer.rs            # Quad-X mixer; airmode + phantom-thrust prevention
│   │   └── tinympc-rs/         # Vendored no_std MPC solver (ADMM, const-generic)
│   │
│   └── sim/                    # Host-only (no_std disabled)
│       ├── sim.rs              # 6DOF rigid-body physics, τ=30ms motor lag
│       ├── sim_hover.rs        # sim_hover example harness
│       └── sensors.rs          # GPS/baro sensor models with noise
```

---

## Task Model

Each sensor subsystem runs as an independent async task. Tasks publish
via `Signal<_, T>` (latest-value-wins) — correct for real-time sensor
data where a missed sample is preferable to a stale-queue buildup.
The control loop runs on the main Embassy task so it is never
deprioritised by the executor.

```
┌─────────────────────────────────────────────────────────────────────┐
│                           SPAWNED TASKS                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  [blink_task]         LED heartbeat (PD10, 100ms on / 900ms off)    │
│                                                                     │
│  [rc_task::run]       CRSF on UART5 RX (PB5), 416666 baud          │
│       │               Publishes: RC_CHANNELS, RC_LINK, RC_LAST_SEEN │
│       │               (module-level statics in rc_task.rs)          │
│       │                                                             │
│  [dual_icm_read_task] Both ICM-42688Ps at 8 kHz (ticker-driven)    │
│       │               IMU1: SPI1, ROTATION_ROLL_180                 │
│       │               IMU2: SPI4, ROTATION_PITCH_180                │
│       │               Reads back-to-back, averages body-frame →     │
│       │               √2 noise floor improvement.                   │
│       │               Falls back to single_icm_read_task if IMU2   │
│       │               fails to init.                                │
│       │               Publishes: RAW_IMU (8 kHz), IMU_DIAG (1 Hz)  │
│       │                                                             │
│  [icm_monitor_task]   1 Hz — sample/error counters + diag snapshot │
│                                                                     │
│  [mekf_task]          Consumes RAW_IMU at 8 kHz                    │
│       │               Predict: every sample (gyro integration)      │
│       │               Update:  every 80th sample → 100 Hz (accel)  │
│       │               Publishes: IMU_DATA (8 kHz)                   │
│       │                          IMU_DATA_FOR_KF (100 Hz, on accel  │
│       │                          update ticks only)                 │
│       │                                                             │
│  [gps_task]           USART1 (PA10 RX), 9600 baud, NMEA            │
│       │               Publishes to two signals so the control loop  │
│       │               and pos_kf_task don't race for the same       │
│       │               single-shot sample:                           │
│       │               GPS_DATA (→ control_loop)                     │
│       │               GPS_DATA_FOR_KF (→ pos_kf_task)               │
│       │                                                             │
│  [baro_task]          I2C2 (PB10/PB11), DPS310, 25 Hz              │
│       │               Owns raw peripheral for bus-recovery bitbang  │
│       │               Publishes: BARO_DATA                          │
│       │                                                             │
│  [pos_kf_task]        100 Hz ticker                                 │
│       │               Predict:  IMU_DATA_FOR_KF (accel + attitude)  │
│       │               Update:   GPS_DATA_FOR_KF (~1 Hz, post-latch) │
│       │               Update:   BARO_DATA (25 Hz, post self-cal)    │
│       │               Publishes: POS_ESTIMATE                       │
│       │                                                             │
├─────────────────────────────────────────────────────────────────────┤
│                      MAIN TASK (highest priority)                   │
│                                                                     │
│  control_loop()       200 Hz ticker                                 │
│       Reads: IMU_DATA, GPS_DATA, POS_ESTIMATE, RC_CHANNELS          │
│       Runs:  arming FSM                                             │
│              → MPC attitude outer loop (50 Hz, every 4th tick)      │
│              → altitude hold (50 Hz, every 4th tick)                │
│              → rate PID inner loop (200 Hz)                         │
│              → mixer → DShot output                                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Shared Signals

| Signal            | Type          | Producer        | Consumer(s)                  |
|-------------------|---------------|-----------------|------------------------------|
| `RAW_IMU`         | `RawImu`      | icm_read_task   | mekf_task                    |
| `IMU_DIAG`        | `ImuDiag`     | icm_read_task   | icm_monitor_task             |
| `IMU_DATA`        | `ImuData`     | mekf_task       | control_loop                 |
| `IMU_DATA_FOR_KF` | `ImuData`     | mekf_task       | pos_kf_task                  |
| `GPS_DATA`        | `GpsData`     | gps_task        | control_loop                 |
| `GPS_DATA_FOR_KF` | `GpsData`     | gps_task        | pos_kf_task                  |
| `BARO_DATA`       | `BaroSample`  | baro_task       | pos_kf_task                  |
| `POS_ESTIMATE`    | `PosEstimate` | pos_kf_task     | control_loop                 |
| `RC_CHANNELS`     | `RcChannels`  | rc_task         | control_loop                 |

---

## Control Loop Detail

```
Every tick (200 Hz / 5 ms budget):
┌──────────────────────────────────────────────────────────────────┐
│  1. Read latest signals (try_take — non-blocking)                │
│     IMU_DATA, GPS_DATA, POS_ESTIMATE, RC_CHANNELS                │
│                                                                  │
│  2. Arming FSM                                                   │
│     Pre-arm checks: thr_low, attitude_level, imu_fresh,          │
│     rc_link_active, [gps_home_ready]                             │
│     Disarmed: zero all demands, reset PID/MPC/alt controllers    │
│                                                                  │
│  3a. Outer loops (every 4th tick → 50 Hz):                       │
│      │                                                           │
│      │  MPC attitude                                             │
│      │  RC sticks → desired roll/pitch/yaw_rate                  │
│      │  mpc.solve(angles_rad, rates_rad) → rate_setpoints        │
│      │                                                           │
│      │  Altitude hold                                            │
│      │  if PosKF.ready → alt_ctrl.update(target, est, dt)        │
│      │  else            → direct stick → thrust (manual mode)    │
│      │                                                           │
│  3b. Rate PID (every tick → 200 Hz):                             │
│      rate_pid.update(rate_setpoints, gyro_rates, dt)             │
│      → torque demands [roll, pitch, yaw]                         │
│                                                                  │
│  4. Mixer                                                        │
│     QUAD_X.apply(thrust, torque) → motor outputs [m1..m4]        │
│                                                                  │
│  5. DShot output                                                 │
│     armed   → throttle_clamped(motor_value × 1999)               │
│     disarmed → MotorStop on all channels                         │
│     dshot.send(frames).await                                     │
│                                                                  │
│  6. 2 Hz telemetry log (every 100th tick)                        │
└──────────────────────────────────────────────────────────────────┘
```

### Planned cascade additions

`control/position.rs` is written and unit-tested, ready to wire in:

```
PosKF (100 Hz) ← GPS (~1 Hz) + baro (25 Hz) + IMU predict
      │
      ▼
Position PD (5 Hz)    ← [to be wired; pos-hold / GPS rescue]
      │
      ▼
Attitude MPC (50 Hz) → rate setpoints
      │
      ▼
Rate PID (200 Hz) → torque demands → mixer → DShot → motors
```

---

## DShot Implementation

Embassy's `waveform_up` / `waveform_up_multi_channel` APIs produced
malformed waveforms (DMAR register contention). The fix writes directly
to TIM2's DMAR register via a 4-beat DMA burst triggered by TIM2's
Update event:

- All four motor channels on a **single TIM2** (PA0–PA3, CH1–CH4).
- Frame/CRC encoding via the `uf_dshot` crate.
- DMA buffer in SRAM1 (D2 domain, `0x3000_0000`). DMA1 can reach SRAM1;
  it cannot reach DTCM — buffers must not be placed there.
- `DBL=3` → 4 CCR writes per Update event → all channels fire
  simultaneously.
- D-cache disabled at boot (`SCB.disable_dcache()`).

---

## Key Data Types

```rust
/// Raw averaged output from the dual-ICM read task.
/// Body-frame NED, before MEKF filtering.
pub struct RawImu { /* accel_g(), gyro_dps(), temp_c() */ }

/// MEKF output — also the ImuData type from wt901b.rs (reused for
/// compatibility). This is what the control loop and PosKF consume.
pub struct ImuData {
    pub accel:       [f32; 3],    // m/s², body frame
    pub gyro:        [f32; 3],    // °/s, bias-corrected
    pub angle:       [f32; 3],    // roll/pitch/yaw, degrees
    pub quaternion:  [f32; 4],    // [w, x, y, z], body→nav
    pub temperature: f32,
    // mag, pressure, altitude_cm: zero when driven by MEKF
}

/// Parsed NMEA output.
pub struct GpsData {
    pub fix_mode:    FixMode,     // NoFix / Fix2D / Fix3D
    pub satellites:  u8,
    pub hdop:        f32,
    pub latitude:    f64,         // degrees
    pub longitude:   f64,         // degrees
    pub altitude_m:  f32,         // MSL metres
}

/// Fused position/velocity estimate from pos_kf_task.
pub struct PosEstimate {
    pub position_ned: [f32; 3],  // NED metres, relative to home origin
    pub velocity_ned: [f32; 3],  // NED m/s
    pub altitude_up:  f32,       // metres AGL, positive up
    pub vz_up:        f32,       // vertical velocity m/s, positive up
    pub p_ref_pa:     f32,       // baro reference pressure (Pa)
    pub baro_age_ms:  u32,
    pub ready:        bool,      // true once GPS home is latched
    pub home_latched: bool,
}

/// RC input from the CRSF task.
pub struct RcChannels {
    pub channels: [u16; 16],     // raw CRSF values (172–1811 range)
}
```

---

## Design Decisions

1. **Dual ICM-42688P.** Both onboard IMUs are read back-to-back and
   averaged, halving the noise floor (√2) for only ~40 µs of extra SPI
   time per 125 µs tick. Graceful fallback to single-IMU if IMU2 fails.

2. **MEKF predict at 8 kHz, accel-update at 100 Hz.** Running accel
   updates at full rate would amplify vibration; 80:1 decimation is the
   right trade-off for the ICM's noise characteristics.

3. **Control loop on the main task.** All spawned tasks are sensor
   producers. The control-loop consumer is the main task, which the
   Embassy executor will never preempt for a spawned task.

4. **Two GPS signals.** `GPS_DATA` and `GPS_DATA_FOR_KF` are separate
   `Signal` instances fed by the same NMEA parse. A single signal would
   create a race between `control_loop` and `pos_kf_task` — whichever
   calls `try_take` first wins and the other sees `None`.

5. **GPS home as altitude anchor.** Boot-time baro averaging was
   abandoned: a baro-only altitude floor is unsafe if the sensor fails
   mid-flight (as happened on 2026-04-20). Current design: GPS home
   latches the altitude origin; baro self-calibrates against the
   GPS-anchored KF state, then dominates short-term altitude.

6. **Open-drain I2C bus recovery.** The I2C peripheral can latch
   BUSY/ARLO if a slave holds SDA mid-transaction. Recovery bitbangs 9
   SCL pulses then a manual STOP via `OutputOpenDrain` — never
   `Output`. Push-pull against a clock-stretching slave shorts the MCU
   output stage (killed the F722's DPS310 on 2026-04-20).
