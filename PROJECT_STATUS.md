# FC-Rusty — Project Status & Roadmap

A Rust flight controller targeting the **Radiolink F722** (STM32F722RET6,
Cortex-M7F @ 216 MHz). The north-star is **stable, high-authority
attitude control via MPC**; everything else — estimation, sensors,
arming, comms — exists to feed or protect that loop.

Status is written as a snapshot, not a log. Update this document
whenever a material hardware or design change lands (see `CLAUDE.md`).

---

## Current Hardware State

### Radiolink F722 peripherals

| Peripheral        | Role                           | Status                                            |
|-------------------|--------------------------------|---------------------------------------------------|
| USART3 TX (PB10)  | defmt logger (115200, T3 pad)  | ✅ verified                                        |
| USART2 RX (PA3)   | CRSF RC receiver (R2, 416666)  | ✅ verified — 6 channels parsing                   |
| USART6 (PC6/PC7)  | GPS (NMEA, 9600, T6/R6)        | ✅ verified — 3D fix, home latches at ≥6 sats      |
| SPI1 + PB2 + PC4  | ICM-42688P IMU (onboard)       | ✅ verified — 8 kHz reads, MEKF fusing             |
| I2C1 (PB8/PB9)    | DPS310 barometer (onboard)     | ❌ **dead 2026-04-20** (push-pull bitbang killed it) |
| USART1 (PA9/PA10) | WT901B IMU (external, T1/R1)   | ⚠️ deprioritised — driver retained as fallback     |
| UART4 RX (PA1)    | ESC telemetry                  | ⚪ not wired                                       |
| Motors PA15/PB3/PB4/PB6 | DShot600 (TIM2/3/4)      | ⚪ **never driven a real ESC**                    |
| W25Nx1G on SPI3   | Blackbox flash                 | ⚪ post-Alpha                                     |

An identical external DPS310 is on hand for replacement when the user
solders it to the I2C bus; the firmware is agnostic to baro presence
and runs GPS-only if it is absent.

### Why Radiolink and not the prior boards

- **F407VET6 dev board**: original bring-up target; full stack ran
  end-to-end on probe-rs. Retired when we moved to a real flight board.
- **SpeedyBee F7V3**: 5V BEC rail damaged during IMU rework; no longer
  flight-usable. Moved to Radiolink F722 — same silicon, same motor
  pinout, prior firmware work carried over unchanged.

---

## What's Verified on Hardware

- **Attitude**: ICM-42688P at 8 kHz, MEKF fusing accel (100 Hz update)
  and gyro (8 kHz predict). Gyro bias bounded to 0.3–0.5 dps; innovation
  gate rejects ~0% at rest and ~25% under aggressive motion. Sensor
  frame mapped to NED via `BODY_SIGN=[+1,-1,-1]`.
- **Position**: 6-state linear PosKF (pn, pe, pz, vn, ve, vz) running
  at 100 Hz predict.
  - GPS home latches on the first fix with `FIX3D && sats ≥ 6 && HDOP < 2`.
  - GPS altitude fuses with σ_gps_v = 5 m; baro (when alive) with
    σ_baro = 0.3 m. Baro dominates the short term; GPS keeps altitude
    honest long-term via cross-covariance.
  - Outdoor verification 2026-04-20: clean `alt-ready → ready`
    transition, baro 26 reads/s with 0 errors across the run,
    post-home-latch IMU-only drift (~1500 m) corrected in one GPS
    tick as expected.
- **Comms**: CRSF RC (6 channels), NMEA GPS, defmt over USART3.
- **Control loop**: 200 Hz closed loop running on MPC+PID against
  real sensors; timing 61 µs avg / 122 µs max with no overruns. MPC
  warm-up 213 µs; in-flight MPC scheduled at 50 Hz (160:1 decimation
  against the 8 kHz inner rate).

---

## Code-Done but Unflashed

- **GPS-gated arm + baro self-calibration** — commit `a7feea0` on
  `feat/icm42688-mekf`.
  - Arming requires `gps_home_ready`. Mid-flight GPS loss does not
    disarm (arm-time gate only).
  - pos_kf_task drops the boot-time p_ref average. Once home has
    latched and ≥2 GPS fixes have fused, the next baro sample
    self-calibrates: `p_ref = p_now / (1 - kf_alt_up/44330.77)^5.2558`.
  - Rationale: the onboard baro's intermittency makes a baro-only
    take-off unsafe. GPS home is the new altitude floor.
  - Verification checklist lives in the session pickup memory; the
    short version: `gps=false` in "arm rejected" until home latches,
    `baro_cal=false` until the 2-fuse window passes, no altitude
    jump at calibration.

---

## What's Next

1. **Flash + outdoor verify `a7feea0`.** Then push.
2. **Motor bring-up on F722.** Never run DShot against a real ESC on
   this hardware. Biggest remaining unknown. ⚠ **Critical safety note:**
   the ESC 'V' pad is Vbat (11–25 V LiPo), not 5 V — bridging it to
   the FC 5 V rail killed a previous dev board, the GPS, and the
   ST-Link. Triple-check before powering up.
3. **Close the loop on hardware.** Rate-PID-only hover first, then
   enable the MPC outer loop and tune for transfer from sim.
4. **External DPS310 soldering** (user task) → baro fusion comes back.

---

## Backlog (north-star-aligned, not yet urgent)

- **Accel bias estimation in PosKF.** The 6-state filter predicts
  kinematics from raw body specific force with no accel-bias state;
  outdoor verify showed ~0.4 m/s/s drift between GPS fixes. Benign
  while GPS σ=2 m dominates, but will bite in two scenarios:
  - GPS dropouts > a few seconds → drift then re-acquire jumps.
  - Autonomous landing / loiter hold / precise RTH → a 0.4 m/s/s
    leak is ~25 m in a minute with intermittent GPS.
  - Design: extend to 9-state (pn pe pz vn ve vz **bax bay baz**),
    subtract estimated bias from predict, random-walk process noise
    with τ ≈ hundreds of seconds. Nominal magnitudes per ICM-42688P
    spec: ~40 mg offset, ~0.1 mg/°C drift → steady-state |b_a| ~ 0.4
    m/s² is plausible.
  - **Schedule: after motor bring-up, before any precision feature.**
- **Bidirectional DShot + RPM into the MPC model.** Betaflight uses
  eRPM telemetry for notch filtering; we could go further and use it
  as an adaptive thrust-mapping term in the MPC's B matrix. See
  "Post-Alpha directions" below.
- **Pilot-facing "lost both sensors" warning.** Currently a combined
  baro+GPS loss would just leave the KF coasting on IMU with no
  explicit downgrade. Not implemented.

---

## Implemented Modules (at Alpha)

All host-tested (`cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`).

| Module              | File                      | Description                                                   |
|---------------------|---------------------------|---------------------------------------------------------------|
| Rate PID            | `control/pid.rs`          | 3-axis, derivative-on-measurement, D-term LPF, anti-windup    |
| Attitude MPC        | `control/mpc.rs`          | 6-state roll/pitch/yaw+rates, tinympc ADMM, 50 Hz             |
| Altitude hold       | `control/altitude.rs`     | PID + hover feedforward, anti-windup, gated on PosKF.ready    |
| Position PD         | `control/position.rs`     | Horizontal hold, world→body rotation, tilt-limited            |
| Quad-X mixer        | `control/mixer.rs`        | Airmode + no-airmode paths, phantom-thrust prevention         |
| Arming FSM          | `control/arming.rs`       | Pre-arm (thr/lvl/imu/rc/gps), failsafe, re-arm lockout        |
| PosKF               | `estimation.rs`           | 6-state linear KF; GPS + baro + IMU predict                   |
| MEKF                | `attitude_mekf.rs`        | Quaternion MEKF with gyro-bias state, 8 kHz predict           |
| CRSF parser         | `drivers/crsf.rs`         | Byte streaming, 11-bit unpack, link stats, CRC8               |
| NMEA parser         | `drivers/nmea.rs`         | GGA/RMC/GSA/VTG, 3D fix detection, checksum                   |
| WT901B parser       | `drivers/wt901b.rs`       | All packet types; retained as fallback IMU                    |
| DPS310 driver       | `drivers/baro.rs`         | I2C @ 16× OSR, bus-recovery bitbang (open-drain)              |
| DShot encoder       | `drivers/dshot.rs`        | Frame, CRC, DMA buffer, speed/timing                          |
| Physics sim         | `sim/sim.rs`              | 6DOF rigid body, τ=30ms motor lag, NED, ground collision      |
| Sensor sim          | `sim/sensors.rs`          | GPS (10 Hz + noise), baro (50 Hz + noise/drift), xorshift64   |
| TinyMPC solver      | `tinympc-rs/`             | ADMM, no_std, const-generic dimensions                        |

### Control cascade (proven in sim and — with the exception of motors — on hardware)

```
PosKf (100 Hz) ← GPS (1 Hz NMEA) + baro (25 Hz) + IMU predict
      │
      ▼
Position PD (5 Hz) → desired roll/pitch
      │
      ▼
Attitude MPC (50 Hz) → rate setpoints
      │
      ▼
Rate PID (200 Hz) → torque demands → mixer → motors
```

### Simulation examples

`cargo run --example <name> --no-default-features`:

| Example           | Description                                               | Status                               |
|-------------------|-----------------------------------------------------------|--------------------------------------|
| `sim_hover`       | PID-only hover at 5 m                                     | Stable, ±0.02 m altitude             |
| `sim_mpc_hover`   | MPC+PID hover at 5 m                                      | MPC converges in 3–5 iterations      |
| `sim_kf_hover`    | Full stack, noisy GPS+baro → KF → altitude hold           | KF altitude within ~30 mm of truth   |
| `sim_gps_rescue`  | Fly (20, 10) m → home at 5 m altitude                     | Arrives within 0.23 m of home        |

---

## Post-Alpha Directions

Parked here so they aren't forgotten. **None of these should be
started until a flyable Alpha exists.**

### F722 headroom

Versus the original F405 target:
- 216 MHz vs 168 MHz (+29%)
- 256 KB contiguous RAM vs 128 KB + 64 KB CCM
- FPv5-SP vs FPv4-SP
- **ITCM (16 KB) + DTCM (64 KB)** — zero-wait-state memories
- **ART accelerator** — instruction prefetch/cache on flash

Ideas ordered by ratio of payoff to risk:

1. **MPC hot loop in ITCM.** TinyMPC's ADMM iteration is our tightest
   inner loop; zero-wait ITCM should measurably cut solve time. Cheap,
   low-risk.
2. **MPC rate 50 Hz → 100 Hz → 200 Hz.** Gated on solve-time headroom
   from (1). Cuts attitude-tracking lag; if matched to the PID rate,
   the rate loop sees a fresh setpoint every tick.
3. **Longer MPC horizon.** More RAM + faster solve = look further
   ahead; helps on aggressive manoeuvres where the controller currently
   sees constraints too late.
4. **True NMPC (12-state, SQP / multiple-shooting / RTI).** Biggest
   payoff, biggest work item. Needs a nonlinear solver (acados-style),
   not tinympc. Gate on (1)+(2) showing us the solve-time budget.
5. **Accel-bias state in PosKF** — see Backlog above. Independent of
   the MPC work.
6. **Bidirectional DShot + adaptive B matrix.** Use measured RPM to
   correct the thrust mapping at runtime. RPM-based notch filtering
   on the gyro falls out for free. Phases:
   - Bidirectional DShot capture (timer input capture + DMA)
   - RPM filtering (port the notch-filter concept)
   - RPM → thrust estimation for MPC model correction
   - Disturbance estimate into MPC state

### VTX / OSD

- **Analog OSD (MAX7456):** SPI-driven character overlay. Not on this
  board; would need board rev.
- **Digital OSD (MSP DisplayPort over UART):** FC sends OSD data to
  the VTX, which renders digitally. Path of least resistance. ~30 Hz,
  one UART.
- **VTX control:** SmartAudio (4800 baud, Tx-only) or IRC Tramp (9600,
  Tx/Rx). Low priority — manual button configuration works until it's
  implemented.

### Blackbox logging

Record ~200 Hz flight data to the onboard W25Nx1G flash. Essential
for post-flight tuning. Key fields: IMU raw+fused, RC input, control
demand, motor outputs, MPC solve time/convergence, GPS fix quality.
Format could be Betaflight-compatible (reuse Blackbox Explorer) or
custom.

### Configuration interface

- **MSP over USB** — compatible with Betaflight configurator.
  Ambitious but useful.
- **MAVLink over UART** — Mission Planner / QGC compatibility.
- **defmt CLI over USB serial** — minimal effort.
- **Parameter storage in flash** — persist tuning across reboots.

### ESC telemetry — post-Alpha motor health

See (6) above. Beyond notch filtering:
- **Adaptive thrust mapping** — runtime lookup (DShot → RPM → thrust),
  linearises actuator response, directly improves MPC accuracy.
- **Motor failure detection** — if one motor's RPM diverges wildly
  from commanded, the MPC's constraint handling makes real-time
  mixer reconfiguration theoretically possible.
