# FC-Rusty — Project Status & Roadmap

A Rust flight controller targeting the ~~Radiolink F722 STM32F722RET6~~
~~Cortex-M7F @ 216 MHz~~ STM32H743VIT6, specifically a DAKEFPV F7-H7 stack. The north-star is **stable, high-authority
attitude control via MPC**; everything else — estimation, sensors,
arming, comms — exists to feed or protect that loop.

This document is the project status log. Update this document whenever a material hardware or design change lands (see `CLAUDE.md`).

## What's Verified on Hardware

(THe specifics here are on older hardware, but the overall functionality is implemented on the H7, with improvements in some cases.

- **Attitude**: ICM-42688P at 8 kHz, MEKF fusing accel (100 Hz update)
  and gyro (8 kHz predict). Gyro bias bounded to 0.3–0.5 dps; innovation
  gate rejects ~0% at rest and ~25% under aggressive motion. Sensor
  frame mapped to NED via `BODY_SIGN=[+1,-1,-1]`.
- **Position**: 6-state linear PosKF (pn, pe, pz, vn, ve, vz) running
  at 100 Hz predict.
  - GPS home latches on the first fix with `FIX3D && sats ≥ 5 && HDOP < 3.5`
    (relaxed for Alpha testing; see backlog for post-Alpha tightening).
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

## Deprecated - Look to revert functionality as GPS and Baro both work well

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

Correct functionality now will be to rely on either baro or GPS as an altitude start (GPS with fix requirements as documented) and for this data to be fused as independent sources of truth with regards altitude specifically

---

## What's Next

1. ~~**Flash + outdoor verify `a7feea0`.**~~ Flashed 2026-04-21.
   GPS-latch thresholds relaxed to 5 sats / HDOP < 3.5 for Alpha
   testing. Push once verified.
   Tighten GPS thresholds post Alpha for more reliable prearm (7 sats / HDOP < 2.5)
   **~~Motor bring-up on F722~~**
   ~~blocked, under investigation~~
   Resolved: Record kept for reference as of around 25-04-26:
   Driver or peripheral config is implicated, not the ESCs. Three
   of the four ESCs are proven healthy on TIM2's signal; a fourth
   is unproven. Arm attempts at 29 % thrust did not produce clean
   spin on any motor. Full session observations and hypotheses in
   `docs/motor-bringup-log.md`. Captured
   oscilloscope waveforms and eventually traced malformed bitstream to various embassy issues populating DMAR. We implemented this functionality directly to avoid the issue (src/drivers/dshot_hw.rs) so the ESCs now receive the correct bitstream and the motors spin up on arm.
  
2. **Close the loop on hardware.** Rate-PID-only hover first, then
   enable the MPC outer loop and tune for transfer from sim.

 (Currently very unclear on whether point 2 is stale comment or true - please verify)

## ****Alpha Complete 03-05-2026****

### post-Alpha tweaks

- GPS thresholds tightened to 7 sats / HDOP < 2.0
- Re-enable arming on baro only, but if GPS fix is available set home co-ords.
- Assign CRSF channels for user-initiated GPS Rescue, pos-hold and alt-hold functionality.
- ESC Bidirectional Dshot functionality
- revert throttle changes implemented for bench motor testing (posssibly this is done by adding a stick scaling factor in the mixer)

### Items for Beta build

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
  - **Bidirectional DShot + RPM into the MPC model.** (Betaflight uses
    eRPM telemetry for notch filtering; we could go further and use it
    as an adaptive thrust-mapping term in the MPC's B matrix.) See
    "Post-Alpha directions" below.

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

### H743 headroom

- **ITCM/DTCM/ART** — zero-wait-state memories

- Ideas ordered by ratio of payoff to risk:

1. **MPC hot loop in ITCM.** TinyMPC's ADMM iteration is our tightest
   inner loop; zero-wait ITCM should measurably cut solve time. Cheap,
   low-risk.
2. **MPC rate 50 Hz → 100 Hz → 200 Hz or beyond.** Gated on solve-time headroom
   from (1) and potentially 3. Cuts attitude-tracking lag; if matched to the PID rate,
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

- **Analog OSD:** SPI-driven character overlay. Onboard hardware.

- **Digital OSD** FC sends OSD data to
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

### Current sensing

Board needs to be able to sense the current draw from the motors and battery voltage to provide feedback to the Flight Controller and to the pilot.

### Configuration interface

- **MSP over USB** — compatible with Betaflight configurator.
  Ambitious but useful.
- **MAVLink over UART** — Mission Planner / QGC compatibility.
- **Parameter storage in flash** — persist tuning across reboots.

### ESC telemetry — post-Alpha motor health

See (6) above. Beyond notch filtering:

- **Adaptive thrust mapping** — runtime lookup (DShot → RPM → thrust),
  linearises actuator response, directly improves MPC accuracy.
- **Motor failure detection** — if one motor's RPM diverges wildly
  from commanded, the MPC's constraint handling makes real-time
  mixer reconfiguration theoretically possible.
