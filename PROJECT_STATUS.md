# Rust Flight Controller — Project Status & Roadmap

## Hardware Bring-Up: DONE (2025-09-03)

Firmware boots on a bare STM32F407VET6 dev board via probe-rs / ST-Link,
streams defmt logs over RTT, and runs the full 200 Hz control loop without
peripherals attached. Measured numbers from the first successful flash:

| Metric                       | Value          | Notes                          |
|------------------------------|----------------|--------------------------------|
| Flash footprint (text+ro+data) | ~58 KB       | 11.2% of 512 KB                |
| RAM (bss+uninit)             | ~6.6 KB        | 5% of 128 KB                   |
| Control loop avg             | 61 µs          | 1.2% of 5 ms budget            |
| Control loop max             | 61 µs          | Zero overruns over 28s         |
| Tasks spawned                | RC, IMU, GPS, control | all within 100 µs of boot |

**Required file layout** (all three were wrong initially and needed fixing):
- `build.rs` must be at repo root, NOT `src/build.rs` — otherwise `-Tdefmt.x`
  is never passed to the linker and probe-rs errors about defmt symbols.
- `.cargo/config.toml` (hidden dir), NOT `config.toml` at repo root — cargo
  only reads the hidden-dir variant.
- `~/.cargo/config.toml` must NOT contain `[build] target = ...` — that
  poisons every cargo build on the machine, including host/sim builds.

**Telemetry label gotcha:** the 2 Hz log line reports `thrust_cmd=` (the
altitude controller's internal thrust command), NOT raw RC stick throttle.
While disarmed, `thrust_cmd` is held at `hover_throttle` (currently 0.294),
so seeing `thrust_cmd=29%` with `armed=false` and no RC receiver attached
is **expected behavior**, not a bug. A separate `stick_thr=` field shows
the raw RC input for clarity.

**Failsafe path verified (in code, not yet on hardware):** `arming.rs`
disarms within 500 ms of RC loss or 50 ms of IMU loss, and re-arming
requires an OFF→ON switch edge (see `test_no_rearm_after_failsafe`).
When disarmed, all four motors get `DshotFrame::disarmed()` regardless
of internal controller state.

## What We Have

### Architecture
- **Target platform:** STM32F407(VET6) (Cortex-M4F, 168 MHz, 192 kB SRAM, 512 KB flash)
- **Framework:** Embassy async executor
- **Control strategy:** Cascaded MPC (50 Hz) + PID (200 Hz), with position PD (5 Hz) outer loop
- **State estimation:** 6-state linear Kalman filter (position + velocity, NED)
- **IMU:** WitMotion WT901B (onboard Kalman filter, accel/gyro/mag/baro/quaternion) — *not yet physically connected*
- **RC protocol:** CRSF (ExpressLRS / TBS Crossfire) — *not yet physically connected*
- **ESC protocol:** DShot600 — *not yet physically connected*
- **GPS:** NMEA over UART6 — *not yet physically connected*
- **Task model:** async tasks communicating via lock-free Signals

### Implemented Modules (all tested, no_std compatible)

| Module | File | Tests | Description |
|--------|------|-------|-------------|
| **Control** | | | |
| Rate PID | `control/pid.rs` | 9 | 3-axis rate controller, derivative-on-measurement, D-term LPF, integral anti-windup |
| Attitude MPC | `control/mpc.rs` | — | 6-state [roll,pitch,yaw,p,q,r] MPC via tinympc-rs, 50 Hz, first-order rate lag model |
| Altitude hold | `control/altitude.rs` | 4 | PID altitude controller, hover-throttle feedforward, integral anti-windup |
| Position PD | `control/position.rs` | 7 | Horizontal position controller, world→body frame rotation, tilt-limited output |
| Mixer | `control/mixer.rs` | 7 | Quad-X mixer with airmode and no-airmode paths, phantom-thrust prevention |
| Arming FSM | `control/arming.rs` | 12 | Pre-arm checks, failsafe (RC/IMU loss), re-arm lockout after failsafe |
| **Estimation** | | | |
| Position KF | `estimation.rs` | 4 | 6-state linear KF [px,py,pz,vx,vy,vz], CWNA process noise, GPS+baro updates |
| **Drivers** | | | |
| CRSF parser | `drivers/crsf.rs` | 5 | Streaming byte parser, 11-bit channel unpacking, link statistics, CRC8 |
| WT901B parser | `drivers/wt901b.rs` | 8 | All packet types (accel, gyro, angle, mag, baro, quaternion), config commands |
| NMEA parser | `drivers/nmea.rs` | 17 | GGA/RMC/GSA/VTG sentence parsing, 3D fix detection, checksum validation |
| DShot encoder | `drivers/dshot.rs` | 6 | Frame encoding, CRC, DMA buffer generation, speed/timing calculations |
| **Simulation** | | | |
| Physics sim | `sim/sim.rs` | 5 | 6DOF rigid body, first-order motor lag (τ=30ms), NED frame, ground collision |
| Sensor sim | `sim/sensors.rs` | 4 | GPS (10 Hz, configurable noise), baro (50 Hz, noise + OU drift), xorshift64 PRNG |
| **Firmware** | | | |
| Main loop | `main.rs` | — | Embassy task spawning, control loop, arming logic |
| CRSF UART task | `rc_task.rs` | — | Embassy async task, DMA reads, Signal publishing, failsafe timer |
| TinyMPC solver | `tinympc-rs/` | — | ADMM-based MPC solver, no_std, const-generic dimensions |

**Total: 70 passing tests on host** (`cargo test --no-default-features --target x86_64-unknown-linux-gnu`)

### Simulation Examples

All run on host with `cargo run --example <name> --no-default-features`:

| Example | Description | Status |
|---------|-------------|--------|
| `sim_hover` | PID-only hover at 5m | Stable, ±0.02m altitude |
| `sim_mpc_hover` | MPC+PID hover at 5m | Stable, MPC converges in 3-5 iterations |
| `sim_kf_hover` | Full stack with noisy GPS+baro feeding KF, altitude loop on estimate | KF altitude within ~30mm of truth |
| `sim_gps_rescue` | GPS rescue: fly from (20,10) to home (0,0) at 5m altitude | **SUCCESS** — arrives within 0.23m of home |

### Control Cascade (proven in sim)

```
PosKf (6-state) ← GPS (10 Hz, noisy) + baro (50 Hz, noisy+drift)
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

### Key Design Documents
- `ARCHITECTURE.md` — module structure, task model, data flow diagrams, data types

---

## What's Done (Software)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1 | PID controller | **DONE** | 3-axis, derivative-on-measurement, D-term LPF (τ=8ms), integral anti-windup |
| 2 | MPC integration | **DONE** | 6-state attitude model, tinympc-rs ADMM solver, warm-started, 50 Hz |
| 3 | Altitude controller | **DONE** | PID with hover feedforward, integral anti-windup |
| 4 | Position controller | **DONE** | PD with yaw-aware body-frame rotation, tilt limiting |
| 5 | State estimator (KF) | **DONE** | 6-state linear KF, GPS+baro fusion, CWNA process noise |
| 6 | GPS rescue (sim) | **DONE** | Full cascade proven: 22m → 0.23m in 30s with noisy sensors |
| 7 | Arming FSM | **DONE** | Pre-arm checks, RC/IMU failsafe, re-arm lockout |
| 8 | Embassy project setup | **DONE** | Flashes and runs on STM32F407VET6 via probe-rs |
| 9 | NMEA GPS parser | **DONE** | GGA/RMC/GSA/VTG, 3D fix detection, checksum validation |
| 10 | Sensor sim | **DONE** | GPS noise, baro noise+drift, xorshift64 PRNG (no_std) |

## What's Next (Hardware Integration — Required to Fly)

### 1. DShot Hardware Driver
Wire the DShot encoder to actual STM32 timer + DMA peripherals.
Configure TIM1 (or similar) in PWM mode, set ARR for bit period,
DMA feeds CCR values from the buffers we already generate. This is
the most hardware-specific piece remaining.

### 2. Pin Assignment & Board Definition
Resolve the actual USART/timer/pin mapping for the target board.
Key constraints: 4 timer channels for DShot that don't conflict with
3-4 USARTs for CRSF, IMU, GPS, and telemetry.

### 3. Peripheral Bring-Up
Connect and verify each peripheral individually:
- IMU (WT901B) — UART, verify packet rates and data quality
- RC receiver (CRSF) — UART, verify channel data and link quality
- GPS (NMEA) — UART, verify fix quality and update rate
- ESCs (DShot) — timer+DMA, verify motor response

### 4. Closed-Loop on Hardware
Wire the full control loop with real sensors and actuators.
Start with rate-PID-only hover (no MPC) to validate gains
transfer from sim to hardware. Then enable MPC outer loop.

---

## Non-Essential Features (Post-First-Flight)

### 7. VTX Control & OSD

**VTX (Video Transmitter) control** uses either SmartAudio (UART,
typically 4800 baud half-duplex) or IRC Tramp (UART, 9600 baud).
Both let the FC change VTX channel, power level, and pit mode.
This needs one additional UART (Tx only for SmartAudio, Tx/Rx for
Tramp). Low priority — you can configure VTX manually via its
button until this is implemented.

**OSD (On-Screen Display)** is more interesting. Most FPV systems
now use one of two approaches:

- **Analog OSD** (MAX7456 chip): SPI-driven character overlay on
  the analog video signal. The FC writes characters to a grid.
  Betaflight's OSD runs on this. Requires an SPI bus and the
  MAX7456 on the FC board.

- **Digital OSD** (DJI / HDZero / Walksnail): the FC sends OSD
  data over a UART using the MSP DisplayPort protocol. The VTX
  renders it digitally. This is just a UART stream — much simpler
  from the FC side. Uses the same MSP protocol as Betaflight
  configurator.

For a Rust FC, MSP DisplayPort over UART to a digital VTX system
is the path of least resistance. It's essentially: format strings
with flight data (battery voltage, altitude, flight mode, RSSI,
GPS coords) into an MSP frame and send it at ~30 Hz.

**Implementation outline:**
```
comms/
├── msp.rs            # MSP protocol framing (request/response)
├── osd.rs            # OSD layout engine (character grid)
└── vtx_control.rs    # SmartAudio or Tramp protocol
```

### 8. ESC Telemetry & Motor Feedback

**Bidirectional DShot** returns eRPM data from each ESC on the same
signal wire. After sending a DShot frame, the FC tristates the pin
and captures the ESC's response (GCR-encoded, ~30 µs switchover).
This halves the DShot update rate but gives real-time motor speed.

**What Betaflight does with it:**
- **RPM-based notch filtering:** calculates motor frequencies and
  their harmonics, places dynamic notch filters on the gyro signal
  at exactly those frequencies. Hugely effective at removing motor
  vibration. This is the primary use case.
- **Motor health monitoring:** detects failed motors or lost props
  by comparing commanded vs actual RPM.
- **Extended DShot Telemetry (EDT):** interleaves temperature,
  voltage, and current data in the eRPM frames.

**What we could do beyond Betaflight — MPC-aware motor feedback:**

The MPC controller works with an abstract model of the quadrotor
(the A, B matrices). One key assumption is that commanded thrust
maps linearly to actual thrust. In reality, the mapping is:

```
  command → ESC → motor RPM → thrust (∝ RPM²) → vehicle response
```

With eRPM telemetry, we can close a tighter loop:

1. **Model correction:** compare predicted RPM (from the model)
   with measured RPM. The residual tells you about unmodelled
   disturbances — wind gusts, prop damage, battery voltage sag.
   Feed this back as a disturbance estimate in the MPC.

2. **Adaptive thrust mapping:** build a runtime lookup table of
   (DShot command → measured RPM → estimated thrust). This
   linearises the actuator response, which directly improves MPC
   performance since the model becomes more accurate.

3. **Motor failure detection:** if one motor's RPM drops to zero
   or diverges wildly from commanded, the MPC could potentially
   reconfigure the mixer in real-time to redistribute thrust
   across remaining motors. (This is an active research topic —
   the MPC's constraint handling makes it theoretically possible.)

**Implementation would be phased:**
- Phase 1: Bidirectional DShot capture (timer input capture + DMA)
- Phase 2: RPM filtering (port the notch filter concept)
- Phase 3: RPM → thrust estimation for model correction
- Phase 4: Feed disturbance estimate into MPC state

### 9. GPS Rescue / Return to Home — DONE (sim-proven)

The full GPS rescue cascade is implemented and proven in simulation:
Position PD (5 Hz) → Attitude MPC (50 Hz) → Rate PID (200 Hz) → Mixer.
State estimation uses a 6-state linear Kalman filter fusing GPS (10 Hz,
σ_h=2m) and baro (50 Hz, σ=0.3m with OU drift).

**Sim result:** quad flies from (20, 10) m to home (0, 0) in 30s,
arriving within 0.23m. Altitude holds at 5±0.3m throughout. Yaw stable.

**Remaining for hardware:**
- Wire the NMEA GPS parser (`drivers/nmea.rs`) to the UART6 task
- Feed real GPS fixes into the Kalman filter
- Tune position gains for real GPS noise characteristics
- Add GPS rescue state machine (climb → cruise → loiter) triggered
  by RC link loss — currently the sim just flies direct to home

### 10. Blackbox Logging

Record flight data to onboard flash or SD card for post-flight
analysis. Essential for tuning PID gains, MPC weights, and
diagnosing issues. Format could be Betaflight-compatible (for
use with existing tools like Blackbox Explorer) or custom.

Key data to log at ~200 Hz:
- IMU raw + fused state
- RC input
- Control demand (pre-mixer)
- Motor outputs
- MPC solve time and convergence status
- GPS position (when available)

### 11. Configuration Interface

Some way to change parameters without recompiling. Options:
- **MSP over USB:** compatible with Betaflight configurator
  (ambitious but very useful)
- **MAVLink over UART:** compatible with Mission Planner / QGC
- **Simple CLI over USB serial:** defmt-based, minimal effort
- **Parameter storage in flash:** persist tuning across reboots

---

## F722 Platform — Opportunities for the NMPC

Once the port to the SpeedyBee F7V3 (STM32F722RET6) is complete, the
extra horsepower over the F405 opens up several directions worth
exploring. **None of these are required for first flight** — they're
parked here so we don't lose sight of them while getting the basic
port working.

### Raw headroom on F722 vs F405
- **Clock:** 216 MHz vs 168 MHz (~29% faster)
- **RAM:** 256 KB contiguous vs 128 KB + 64 KB split (+CCM)
- **FPU:** FPv5-SP vs FPv4-SP — better single-precision throughput
  per cycle on the same instructions
- **ITCM (16 KB) + DTCM (64 KB):** zero-wait-state tightly-coupled
  memories for hot code/data. F4 has CCM but no ITCM.
- **ART accelerator:** instruction prefetch/cache on flash reads,
  effectively hides flash wait states at 216 MHz

### Ideas to explore

1. **Put the MPC solver hot loop in ITCM.** TinyMPC's ADMM iteration
   is the tightest inner loop we have; running it from zero-wait
   ITCM should measurably cut solve time. Cheap win, low risk.

2. **Push MPC rate from 50 Hz → 100 Hz** (or even 200 Hz, matching
   the PID rate loop). Reduces attitude-tracking lag. Gated on MPC
   solve time — need to measure first. If we free up enough time
   with (1), this becomes free.

3. **Longer MPC horizon.** Currently 6-state attitude with a short
   horizon. More RAM + faster solve means we can look further ahead,
   which helps on aggressive manoeuvres where the short horizon
   causes the controller to "see" constraints too late.

4. **Higher-dim state — true NMPC.** Currently the attitude MPC is
   linear (6-state, linearised rate dynamics). The F722 could
   plausibly run a **12-state nonlinear MPC** (attitude + position
   + velocity + rates) with a real NMPC scheme: SQP, multiple
   shooting, or real-time iteration (RTI). This is the biggest
   payoff — a genuine NMPC controller — but also the biggest piece
   of work. Would need a solver that handles nonlinear dynamics
   (acados-style), not tinympc.

5. **Unified state estimator on firmware.** Currently the Kalman
   filter only runs in sim (`sim_kf_hover`); on hardware we lean
   on the WT901B's onboard fusion for attitude and do no position
   estimation. A proper EKF fusing WT901B attitude + GPS + baro
   in firmware would give us body-frame velocity estimates the
   NMPC could use directly.

6. **Bidirectional DShot + RPM feedback into the MPC model.**
   Already listed under "ESC Telemetry" in the non-essentials
   section — worth flagging here because on the F722 we'd have the
   cycles to actually close the loop: use measured RPM to correct
   the thrust-mapping term in the MPC model at runtime (adaptive
   B matrix).

### Order of attack (when the time comes)
(1) and (2) are cheap and self-contained — measure first, tune
second. (5) is independent and pairs well with any NMPC work.
(4) is the headline feature and should be gated on (1)+(2)
showing us the solve-time budget we have to play with. (3) and (6)
slot in wherever they're convenient.

---

## Priority Order

```
DONE (software validated in sim):
  ✓ PID controller
  ✓ MPC integration (tinympc-rs)
  ✓ Altitude controller
  ✓ Position controller
  ✓ State estimator (Kalman filter)
  ✓ GPS rescue (sim-proven)
  ✓ Arming state machine
  ✓ Embassy project setup
  ✓ NMEA GPS parser

Next (hardware integration):
  [ ] DShot hardware driver
  [ ] Pin assignment & board definition
  [ ] Peripheral bring-up (IMU, RC, GPS, ESC)
  [ ] Closed-loop hover on hardware

Later (features):
  [ ] ESC telemetry (RPM filtering)
  [ ] Blackbox logging
  [ ] VTX/OSD
  [ ] Configuration interface
  [ ] MPC-aware motor feedback
```
