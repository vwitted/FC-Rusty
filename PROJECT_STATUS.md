# FC-Rusty — Project Status

A Rust flight controller targeting the **DAKEFPV H743** (STM32H743VIT6,
Cortex-M7F @ 480 MHz). The north-star is **stable, high-authority
attitude control via MPC**; everything else — estimation, sensors,
arming, comms — exists to feed or protect that loop.

Status is a snapshot, not a log. Update this document whenever a
material hardware or design change lands (see `CLAUDE.md`).

---

## Current Hardware — DAKEFPV H743

### Peripherals

| Peripheral             | Role                              | Status                                        |
|------------------------|-----------------------------------|-----------------------------------------------|
| USART6 TX (PC6)        | defmt logger (115200, T6 pad)     | ✅ verified                                    |
| UART5 RX (PB5)         | CRSF RC receiver (R5, 416666)     | ✅ verified — 6 channels parsing               |
| USART1 (PA9/PA10)      | GPS (NMEA, 9600, T1/R1)           | ✅ verified — 3D fix, home latches             |
| SPI1 + PA4             | ICM-42688P IMU1 (onboard)         | ✅ verified — 8 kHz reads, MEKF fusing         |
| SPI4 + PB1             | ICM-42688P IMU2 (onboard)         | ✅ verified — 8 kHz reads, averaged with IMU1  |
| I2C2 (PB10/PB11)       | SPL06 barometer (onboard)         | ✅ proper driver — 128 Hz, correct calibration              |
| TIM2 (PA0/PA1/PA2/PA3) | DShot600, motors M1–M4            | ✅ verified — all four motors spin on arm      |
| PD10                   | Status LED (active low)           | ✅ heartbeat blink task                        |
| USB-C                  | DFU flashing                      | ✅ verified — no SWD on this board             |
| UART4 (PD1/PD0)        | DisplayPort / VTX (T4/R4)         | ⚪ not wired                                   |
| USART3                 | ESC telemetry (T3/R3)             | ⚪ not wired                                   |
| UART7 / UART8          | General purpose (T7/R7, T8/R8)   | ⚪ available                                   |
| SPI2 (PB13/14/15) + PB12 | AT7456E OSD (MAX7456-compatible) | ⚪ no driver yet                               |

### Why this board

- **Radiolink F722** (STM32F722RET6): prior board. DShot was unresolvable
  via Embassy's `waveform_up` / `waveform_up_multi_channel` APIs — scope
  traces showed malformed waveforms from DMAR contention. Fix: write to
  DMAR directly. Opportunity taken to move to a more capable board.
- **DAKEFPV H743**: STM32H743VIT6 at 480 MHz, 1 MB SRAM, dual onboard
  ICM-42688P (√2 noise reduction), all four motor outputs on a single
  TIM2 (no multi-timer synchronisation problem), USB-C DFU flashing.
  The DShot fix (direct DMAR burst write) carries straight over and
  works cleanly on the H743.

---

## What's Verified on Hardware

(The specifics here are largely validated on older hardware, but the overall functionality has now been implemented and enhanced for the STM32H743).

- **Attitude**: Dual ICM-42688P sensors read at 8 kHz via SPI. MEKF fusing accel (100 Hz update)
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
- **Control loop**: Asynchronous dual-loop architecture communicating via lock-free `Watch` channel:
  - **Outer Loop (100 Hz)**: `navigation_task` handles Attitude MPC, altitude hold, position hold, and RC processing.
  - **Inner Loop (8 kHz)**: `control_loop` executes the rate PID and DShot output, fully synchronized to the MEKF gyro predicts without arbitrary timers.

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

   *(Status Verification: This point remains **TRUE**. While the logic and loop decoupling have been heavily validated in the simulation environments (`sim_gps_rescue`, `sim_hover`), we are still awaiting the delivery of the final target hardware to begin the physical motor bring-up and physical PID tuning.)*

## ****Alpha Complete 03-05-2026****

### post-Alpha tweaks

- [ ] GPS thresholds tightened to 7 sats / HDOP < 2.0
- [x] Re-enable arming on baro only, but if GPS fix is available set home co-ords. (Implemented in `arming.rs` FSM)
- [x] Assign CRSF channels for user-initiated GPS Rescue, pos-hold and alt-hold functionality. (Implemented: CH5 for mode, CH6 for RTH trigger)
- [ ] ESC Bidirectional Dshot functionality
- [ ] revert throttle changes implemented for bench motor testing (posssibly this is done by adding a stick scaling factor in the mixer)

### Items for Beta build

- **Accel bias estimation in PosKF.** The 6-state filter predicts
  kinematics from raw body specific force with no accel-bias state.
  Outdoor testing showed ~0.4 m/s/s drift between GPS fixes. Benign
  while GPS σ=2 m dominates, but will degrade GPS-rescue accuracy and
  any loiter/precision features.
  - Design: extend to 9-state (pn pe pz vn ve vz bax bay baz), subtract
    estimated bias from predict, random-walk process noise with τ ≈
    hundreds of seconds.
  - **Schedule: before any precision autonomous feature.**

- **Bidirectional DShot + eRPM into the MPC.** Measured eRPM from the
  ESCs closes the loop on real motor behaviour, with several payoff layers:
  1. **RPM-based notch filter** on the gyro signal — removes motor/prop
     resonances that currently pass through to the attitude estimate.
  2. **Adaptive thrust mapping** — runtime DShot→RPM→thrust lookup
     linearises the actuator response and directly improves MPC accuracy.
     The MPC's B matrix currently uses a fixed model; substituting
     measured data makes it adaptive.
  3. **Motor failure detection** — diverging RPM from commanded value
     detected in real time; MPC's constraint handling makes mixer
     reconfiguration theoretically possible.
  USART3 (ESC telemetry pad) is free; needs wiring and a bidirectional
  DShot + telemetry-decode driver.

- **Magnetometer calibration.** The onboard magnetometer is not yet used
  (zeroed out in `ImuData`). Before it can correct yaw drift, a
  calibration mode is needed: ~1 minute where the pilot can move the
  drone through full rotation on all three axes, collecting min/max field
  readings for sphere-fitting (hard-iron + soft-iron correction). The FC
  needs to detect when sufficient coverage has been collected and store
  the result. No implementation exists yet.

- ~~**SPL06 barometer driver.**~~ Done. `drivers/baro.rs` now has a
  proper `Spl06` struct reading calibration from the correct register
  start (0x18, not the DPS310's 0x10). Running at 128 Hz / 1× OSR;
  baro task ticks at 125 Hz. DPS310 code retained for reference.

- **Position hold.** `control/position.rs` is written and unit-tested but
  not yet wired into the control loop. Gates on reliable baro + GPS
  fusion and sufficient PID tuning in flight.

---

## Implemented Modules

All host-tested (`cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`).

| Module           | File                     | Description                                                   |
|------------------|--------------------------|---------------------------------------------------------------|
| Rate PID         | `control/pid.rs`         | 3-axis, derivative-on-measurement, D-term LPF, anti-windup   |
| Attitude MPC     | `control/mpc.rs`         | 6-state roll/pitch/yaw+rates, tinympc ADMM, 50 Hz            |
| Altitude hold    | `control/altitude.rs`    | PID + hover feedforward, anti-windup, gated on PosKF.ready   |
| Position PD      | `control/position.rs`    | Horizontal hold, world→body rotation, tilt-limited — **written, not yet wired** |
| Quad-X mixer     | `control/mixer.rs`       | Airmode + no-airmode paths, phantom-thrust prevention         |
| Arming FSM       | `control/arming.rs`      | Pre-arm (thr/lvl/imu/rc/gps), failsafe, re-arm lockout       |
| PosKF            | `estimation.rs`          | 6-state linear KF; GPS + baro + IMU predict                  |
| MEKF             | `attitude_mekf.rs`       | Quaternion MEKF with gyro-bias state, 8 kHz predict          |
| CRSF parser      | `drivers/crsf.rs`        | Byte streaming, 11-bit unpack, link stats, CRC8              |
| NMEA parser      | `drivers/nmea.rs`        | GGA/RMC/GSA/VTG, 3D fix detection, checksum                  |
| UBX parser       | `drivers/ubx.rs`         | u-blox binary protocol — written, not yet active             |
| WT901B parser    | `drivers/wt901b.rs`      | All packet types; `ImuData` type still used internally        |
| SPL06 driver     | `drivers/baro.rs`        | 128 Hz / 1× OSR, correct cal from 0x18; DPS310 retained     |
| DShot driver     | `drivers/dshot_hw.rs`    | TIM2 DMAR burst, DShot600, all 4 channels simultaneously     |
| Physics sim      | `sim/sim.rs`             | 6DOF rigid body, τ=30ms motor lag, NED, ground collision     |
| Sensor sim       | `sim/sensors.rs`         | GPS (10 Hz + noise), baro (50 Hz + noise/drift), xorshift64  |
| TinyMPC solver   | `control/tinympc-rs/`    | ADMM, no_std, const-generic dimensions                        |

### Control cascade

```
PosKF (100 Hz) ← GPS (1 Hz NMEA) + baro (25 Hz) + IMU predict
      │
      ▼
Position PD (5 Hz)  ← [written, not yet wired]
      │
      ▼
Attitude MPC (50 Hz) → rate setpoints
      │
      ▼
Rate PID (200 Hz) → torque demands → mixer → DShot → motors
```

### Simulation examples

`cargo run --example <name> --no-default-features`:

| Example          | Description                                          | Status                             |
|------------------|------------------------------------------------------|------------------------------------|
| `sim_hover`      | PID-only hover at 5 m                                | Stable, ±0.02 m altitude           |
| `sim_mpc_hover`  | MPC+PID hover at 5 m                                 | MPC converges in 3–5 iterations    |
| `sim_kf_hover`   | Full stack, noisy GPS+baro → KF → altitude hold      | KF altitude within ~30 mm of truth |
| `sim_gps_rescue` | Fly (20, 10) m → home at 5 m altitude                | Arrives within 0.23 m of home      |

---

## Post-Alpha Directions

Parked here so they aren't forgotten. **None of these should be started
until the post-Alpha tweaks above are done.**

### H743 headroom

The H743 is significantly more capable than the F722 it replaced:
480 MHz vs 216 MHz, 1 MB SRAM, ITCM/DTCM zero-wait-state memories,
dual FPU. Ideas ordered by payoff/risk ratio:

1. **MPC hot loop in ITCM.** TinyMPC's ADMM iteration is our tightest
   inner loop; zero-wait ITCM should measurably cut solve time. Low risk.
2. **MPC rate 50 Hz → 100 Hz → 200 Hz.** Gated on (1). Cuts
   attitude-tracking lag; at 200 Hz the rate loop gets a fresh setpoint
   every tick.
3. **Longer MPC horizon.** More RAM + faster solve = look further ahead;
   helps on aggressive manoeuvres.
4. **True NMPC (12-state, SQP / RTI).** Biggest payoff, biggest work.
   Needs a nonlinear solver (acados-style). Gate on (1)+(2) showing us
   the solve-time budget.
5. **Bidirectional DShot + adaptive B matrix.** See Backlog above for the
   full rationale. The MPC aspect specifically: feed measured eRPM through
   a thrust model (eRPM → force) and use it to update the B matrix in
   the MPC's linear model at runtime. This makes the controller
   self-correcting as battery voltage sags, props wear, and ESC
   characteristics vary motor to motor. Phases:
   - Bidirectional DShot capture (timer input capture + DMA)
   - eRPM → thrust estimation
   - Online B-matrix update in the MPC solve loop
   - Disturbance observer for unmodelled dynamics

### OSD / VTX

- **Analog OSD (AT7456E, onboard):** SPI character-overlay chip, register-
  compatible with the MAX7456. Verified from ArduPilot docs. Driver would
  follow the MAX7456 protocol; datasheet is the AT7456E.
- **Digital OSD:** MSP DisplayPort over UART4 (already labelled for this).
- **VTX control:** SmartAudio or IRC Tramp over a spare UART.

### Blackbox logging

Record ~200 Hz flight data to onboard flash. Key fields: IMU raw+fused,
RC input, control demands, motor outputs, MPC solve time, GPS fix
quality. Format could be Betaflight-compatible or custom.

### Current sensing

Battery voltage and motor current feedback to the FC and pilot.
The board has current-sensing hardware; a driver is needed.

### Configuration interface

- **MSP over USB** — Betaflight configurator compatibility.
- **MAVLink over UART** — Mission Planner / QGC.
- **Parameter storage in flash** — persist tuning across reboots.
