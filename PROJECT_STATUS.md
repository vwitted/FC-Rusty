# FC-Rusty — Project Status

A Rust flight controller targeting the **GEPRC TAKER H743 BT** (STM32H743VIT6,
Cortex-M7F @ 480 MHz). The north-star is **stable, high-authority
attitude control via MPC**; everything else — estimation, sensors,
arming, comms — exists to feed or protect that loop.

Status is a snapshot, not a log. Update this document whenever a
material hardware or design change lands (see `CLAUDE.md`).

---

## Current Hardware — GEPRC TAKER H743 BT

### Peripherals

| Peripheral               | Role                               | Status                                        |
|--------------------------|------------------------------------|-----------------------------------------------|
| USART6 TX (PC6)          | defmt logger (115200)              | ✅ verified                                    |
| USART2 RX (PA3)          | CRSF RC receiver (416666)          | ✅ verified — 6 channels parsing               |
| UART4 (PA0/PA1)          | GPS (NMEA, 9600)                   | ✅ verified — 3D fix, home latches             |
| SPI1 + PA4               | MPU6000 IMU1 (onboard)             | ✅ verified — 8 kHz reads, MEKF fusing         |
| SPI2 + PB12              | ICM-42688P IMU2 (onboard)          | ✅ verified — 8 kHz reads, averaged with IMU1  |
| I2C1 (PB8/PB9)           | SPL06 barometer (onboard)          | ✅ proper driver — 128 Hz, correct calibration |
| I2C1 (PB8/PB9)           | LIS2MDL magnetometer (breakout)    | ✅ wired in — 100 Hz, fused into MEKF for yaw  |
| TIM3 (PB0/PB1/PB5/PB4)   | DShot600, motors M1–M4             | ✅ verified — all four motors spin on arm      |
| PD10                     | Status LED (active low)            | ✅ heartbeat blink task                        |
| USB-C                    | DFU flashing                       | ✅ verified — no SWD on this board             |
| USART1 (PA9/PA10)        | DisplayPort / VTX                  | ⚪ not wired                                   |
| UART8 (PE0/PE1)          | ESC telemetry                      | ⚪ not wired                                   |
| USART3 (PB10/PB11)       | Bluetooth module (internal)        | ⚪ not wired                                   |
| UART7 (PE7/PE8)          | General purpose                    | ⚪ available                                   |
| SPI4 + PE4               | AT7456E OSD (MAX7456-compatible)   | ⚪ no driver yet                               |

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
- **GEPRC TAKER H743 BT**: Current board. Migrated from DAKEFPV H743 to utilize its standard JST connector layout. Features an STM32H743 with an MPU6000 + ICM-42688P dual IMU setup, SPL06 baro, and up to 9 PWM/DShot outputs. Peripherals were relocated (e.g. TIM3 for DShot, UART4 for GPS, USART2 for RC) to match hardware defaults.

---

## What's Verified on Hardware

(The specifics here are largely validated on older hardware, but the overall functionality has now been implemented and enhanced for the STM32H743).

- **Attitude**: MPU6000 and ICM-42688P dual IMUs read at 8 kHz via SPI,
  averaged in body-frame NED for √2 noise reduction, then passed
  through a software 2nd-order Butterworth LPF chain (150 Hz gyro,
  25 Hz accel; module `imu_filter.rs`) before being signalled to the
  MEKF. On-chip filters are left at their wide defaults: the MPU6000's
  DLPF can't be enabled without losing the 8 kHz ODR, and applying it
  only to the ICM leaves the averaged signal with mismatched bands —
  one software LPF stage after averaging gives both sensors identical
  treatment at 8 kHz full rate. The 25 Hz accel cutoff is also our
  anti-alias filter for the 100 Hz MEKF gravity-update decimation
  (~ −12 dB at the 50 Hz Nyquist edge). MEKF fusing accel (100 Hz
  update) and gyro (8 kHz predict). Gyro bias bounded to 0.3–0.5 dps;
  innovation gate rejects ~0% at rest and ~25% under aggressive
  motion. Sensor frames mapped to NED. **Yaw is observable**: LIS2MDL magnetometer
  on I2C1 (shared with SPL06) at 100 Hz feeds `AttitudeMekf::update_mag`
  with a 3-vector body-frame measurement model and innovation gate.
  Reference field is auto-seeded from the first reading after the
  accel seed (so boot heading defines yaw = 0); true-north requires a
  future `set_mag_reference` call from a GPS/declination source.
  Hard-iron / soft-iron calibration is still on the post-Alpha
  backlog — for now we rely on `OFF_CANC` and unit-vector update
  semantics to absorb modest residuals.
- **Position**: 6-state linear PosKF (pn, pe, pz, vn, ve, vz) running
  at 100 Hz predict. Baro and GPS fuse as independent measurement
  paths — pilot decides what to fly with via the soft arm gate
  (`baro_ready || gps_home_latched`).
  - **GPS position**: home latches on the **centroid of a stable
    window** of quality-gated fixes. Quality gate is `FIX3D &&
    sats ≥ 7 && HDOP < 2.0` (tightened from the Alpha-era 5 / 3.5 to
    match the post-Alpha target). On top of that, a two-stage
    stability test prevents anchoring the home origin against early
    acquisition wander:
    - *Fast path* — ≥ 30 s of samples with spread ≤ 10 m → latch.
      Handles the "receiver already locked at power-on" case so
      we don't pay 60 s for a clean fix.
    - *Slow path* — ≥ 60 s of samples with spread ≤ 100 m → latch.
      Wandering acquisitions fail the fast test but eventually pass
      this one once the wander rolls out of the sliding window.
    Home is latched at the centroid of the stable window, not the
    latest sample. Once latched the stability gate is no longer
    consulted — every quality-passing GGA fuses. Subsequent good
    fixes fuse as local NED. σ_gps_h = 2 m, σ_gps_v = 5 m.
    Per-sentence signal storms are deduped at the consumer
    (`pos_kf_task`) so the same GGA isn't fused 3–5× per cycle
    (which over-shrinks P_pp and dominates the filter with stale
    noise). Diagnostic now reports `Nsig / Npos / Nvel` per second.
  - **GPS velocity** (NMEA RMC ground speed × course) fuses
    independently of home latching at σ = 0.3 m/s, via the new
    `PosKf::update_gps_velocity(vn, ve)` method. This caps DR
    position drift from O(t²) to O(t) and means PosHold without GPS
    home is best-effort but useful for tens of seconds rather than
    diverging immediately. Below 0.3 m/s ground speed the path
    fuses (0, 0) since RMC course is undefined at low speeds —
    actively damps drift while stationary.
  - **Baro** is *not* fused before arm. On the Disarmed→Armed event
    the `ARM_LATCH` signal makes pos_kf_task latch `p_ref = current
    pressure` and zero the KF's vertical state, so altitude reads
    ~0 at arm. From arm forward baro fuses every sample at σ_baro =
    0.3 m, which dominates GPS altitude over the short term.
  - **Readiness flags**: `altitude_ready` (= `baro_calibrated ||
    home_latched`, gates AltHold and PosHold) and `home_latched`
    (gates GpsRescue / GpsHome / RTH).
  - Outdoor verification 2026-04-20 (under the prior GPS-anchored
    design): clean ready transition, baro 26 reads/s with 0 errors,
    post-home-latch IMU-only drift (~1500 m) corrected in one GPS
    tick. Re-verification of the new pilot-discretion design plus
    velocity fusion is on the post-Alpha checklist.

- **Failsafe descent**: RC loss never auto-disarms. The control loop
  picks one of three failsafe modes based on what's still alive:
  - `GpsRescue` (existing): home_latched → climb to safe alt + RTH
    - auto-land at home.
  - `FailsafeLand` (new): altitude_ready, no home → closed-loop
    descent at 0.7 m/s, level attitude, auto-disarm when
    `altitude_up < 0.3 m`.
  - `FailsafeBlind` (new): no altitude, no home → open-loop throttle
    at 90 % of hover, level attitude. **No auto-disarm** — without
    altitude data there's no safe stop criterion. Descent runs until
    pilot regains RC or battery cuts. Impact-signature disarm is a
    Beta backlog item.

  IMU loss is the only mid-flight auto-disarm path: there's no safe
  recovery from losing attitude.
- **Comms**: CRSF RC (6 channels), NMEA GPS, defmt over USART3.
- **Control loop**: Asynchronous dual-loop architecture communicating via lock-free `Watch` channel:
  - **Outer Loop (100 Hz)**: `navigation_task` handles Attitude MPC, altitude hold, position hold, and RC processing.
  - **Inner Loop (8 kHz)**: `control_loop` executes the rate PID and DShot output, fully synchronized to the MEKF gyro predicts without arbitrary timers.

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

- [x] GPS thresholds tightened to 7 sats / HDOP < 2.0. Implemented
  2026-05-14 together with a two-stage stability-windowed home latch
  (30 s/10 m fast path, 60 s/100 m slow path, anchored at the
  centroid) and consumer-side dedupe so per-sentence NMEA signal
  storms can't re-fuse the same GGA into the KF. See "What's
  Verified → GPS position" for the full design.
- [x] Re-enable arming on baro only, but if GPS fix is available set home co-ords. Implemented 2026-05-08: arming gate is now soft (`baro_ready || gps_home_latched`); baro p_ref latches on the Disarmed→Armed transition via `ARM_LATCH` signal; `PosEstimate` exposes split `altitude_ready` / `home_latched` flags. `arming::ArmingStateMachine.require_gps` renamed to `require_altitude_ref`.
- [x] Assign CRSF channels for user-initiated GPS Rescue, pos-hold and alt-hold functionality. (Implemented: CH5 for mode, CH6 for RTH trigger)
- [ ] ESC Bidirectional Dshot functionality
- [ ] revert throttle changes implemented for bench motor testing (posssibly this is done by adding a stick scaling factor in the mixer)

### Items for Beta build

- **Impact-signature disarm.** Currently `FailsafeBlind` (RC lost,
  no altitude reference) has no auto-disarm — the descent runs
  until pilot recovery or battery cut. With ICM accel data we can
  detect a hard-landing impulse (peak accel magnitude ≫ 1 g for
  short duration) and disarm on that signature. Bonus feature:
  generalise to a non-failsafe "crash detected → cut motors" gate
  for any flight mode, which would save props on a botched landing.
  - Design: rolling-window peak detection on body-frame accel
    magnitude, threshold ≈ 4–5 g for ≥10 ms. Disable while throttle
    is high (avoid thrust-axis noise during aggressive manoeuvres).
  - **Schedule: with ICM telemetry features in Beta.**

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

- **Magnetometer calibration.** The LIS2MDL is now fused into the
  MEKF (3-vector update at 100 Hz, gated on innovation magnitude) and
  the chip's own `OFF_CANC` handles small hard-iron offsets, but full
  sphere-fit hard-iron + soft-iron calibration is still pending. The
  fusion currently auto-seeds the nav-frame reference from the first
  reading, which makes boot heading the zero-yaw datum — fine for
  drift correction but not for true-north heading. Calibration mode
  needed: ~1 minute where the pilot moves the drone through full
  rotation on all three axes, collecting min/max field readings, FC
  detects when coverage is sufficient and persists the result; then
  inject local declination + sphere-fit centre to lock yaw to
  true-north. No implementation yet.

- ~~**SPL06 barometer driver.**~~ Done. `drivers/baro.rs` now has a
  proper `Spl06` struct reading calibration from the correct register
  start (0x18, not the DPS310's 0x10). Running at 128 Hz / 1× OSR;
  baro task ticks at 125 Hz. DPS310 code retained for reference.

- **Position hold.** `control/position.rs` is written and unit-tested but
  not yet wired into the control loop. Gates on reliable baro + GPS
  fusion and sufficient PID tuning in flight.

- **ISM6HG256X breakout (Beta).** STMicro 6-axis IMU (LGA-14) — driver
  `drivers/ism6hg256x.rs` is written, compiles, and is intentionally
  unreferenced from `main.rs` until the breakout is built and wired.
  Configured for ±16 g / ±4000 dps / 7.68 kHz ODR / high-perf mode,
  push-pull pulsed DRDY on INT1 — same control-loop shape as the
  ICM-42688P so it can substitute or supplement. ±4000 dps (vs the
  ICM's ±2000 max) chosen because the gyro is already noise-floor-
  limited at ±2000, so the wider FS is strictly more saturation
  headroom for crash/recovery scenarios. Fancy features (high-g 256 g
  channel, FSM, MLC, SFLP, OIS, EIS, sensor hub, FIFO) deliberately
  left out.
  - **Bring-up gotchas** (worth re-reading before flashing):
    - Output registers are **little-endian** (L then H) — opposite of
      the ICM-42688P's big-endian. Don't blindly cross-reference
      decoding logic between the two drivers.
    - `WHO_AM_I` returns `0x73` (vs `0x47` on the ICM); the init path
      will hard-fail with `WhoAmIMismatch` if the wrong driver is
      pointed at the wrong chip.
    - `Orientation` parameter must match how the breakout is soldered
      into the airframe — `Identity` is the safe default for bench work
      but won't be right in flight.
    - Temperature uses the standard ST formula `raw/256 + 25` (1 LSB =
      1/256 °C). Diagnostic only, not on the control path.
    - Chip max ODR is 7.68 kHz (not 8 kHz like the ICM); decimation
      math in any task that fuses both must account for that.
    - Gyro LSB is 140 mdps (not 70 mdps as at ±2000 dps); any code
      cross-checking against ICM samples needs to scale, not compare
      raw counts.

- **LIS2MDL magnetometer.** Wired in 2026-05-13. STMicro 3-axis mag
  (LGA-12) over I2C1 at fixed addr `0x1E`, sharing the bus with the
  SPL06 baro and owned by the same `baro_task` so no bus arbitration
  is needed. Configured for 100 Hz continuous HR mode with
  COMP_TEMP_EN, OFF_CANC, BDU and the digital LPF (BW = 25 Hz) all on.
  Polled at 125 Hz alongside the baro; fresh samples are published on
  `MAG_DATA` and consumed by `mekf_task` via `try_take`. The mekf
  calls `update_mag` with a unit-vector 3-axis measurement model that
  mirrors the accel update structure (body-frame δθ Jacobian,
  innovation-gated at ~30° deviation). A mag-init failure leaves the
  MEKF running without yaw fusion exactly as before. Outstanding:
  hard/soft-iron sphere-fit calibration and declination-based
  true-north reference — see "Magnetometer calibration" backlog.

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
| Arming FSM       | `control/arming.rs`      | Pre-arm (thr/lvl/imu/rc/altitude_ref), soft baro-or-GPS gate, failsafe-on-RC-loss (never disarms; control loop chooses descent), IMU-loss-only auto-disarm |
| PosKF            | `estimation.rs`          | 6-state linear KF; GPS position + GPS velocity + baro + IMU predict; stability-windowed home latch |
| MEKF             | `attitude_mekf.rs`       | Quaternion MEKF with gyro-bias state, 8 kHz predict, 100 Hz accel + 100 Hz mag updates |
| IMU LPF          | `imu_filter.rs`          | 2nd-order Butterworth biquad bank applied to fused dual-IMU stream; 150 Hz gyro / 25 Hz accel default |
| CRSF parser      | `drivers/crsf.rs`        | Byte streaming, 11-bit unpack, link stats, CRC8              |
| NMEA parser      | `drivers/nmea.rs`        | GGA/RMC/GSA/VTG, 3D fix detection, checksum                  |
| UBX parser       | `drivers/ubx.rs`         | u-blox binary protocol — written, not yet active             |
| WT901B parser    | `drivers/wt901b.rs`      | All packet types; `ImuData` type still used internally        |
| SPL06 driver     | `drivers/baro.rs`        | 128 Hz / 1× OSR, correct cal from 0x18; DPS310 retained     |
| ISM6HG256X driver | `drivers/ism6hg256x.rs` | ±16 g / ±4000 dps / 7.68 kHz, SPI; written for Beta breakout, unreferenced |
| MPU6000 driver   | `drivers/mpu6000.rs`     | ±16 g / ±2000 dps / 8 kHz, SPI; written for GEPRC TAKER H743 (IMU1)      |
| ICM42688P driver | `drivers/icm42688.rs`    | ±16 g / ±2000 dps / 8 kHz, SPI; written for H743 boards (IMU1 &2 on DAKEFPV and IMU2 on GEPRC TAKER H743)      |
| LIS2MDL driver   | `drivers/lis2mdl.rs`     | 3-axis mag, I2C addr 0x1E, 100 Hz HR + LPF + OFF_CANC; wired into baro_task + fused in MEKF |
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
