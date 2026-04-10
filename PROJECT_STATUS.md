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
- **Target platform:** STM32F407(VET6) (Cortex-M4F, 168 MHz, 192 kB SRAM, 512 Kb flash)
- **Framework:** Embassy async executor
- **Control strategy:** TinyMPC outer loop (50 Hz) + PID inner loop (200 Hz)
- **IMU:** WitMotion WT901B (onboard Kalman filter, accel/gyro/mag/baro/quaternion) — *not yet physically connected*
- **RC protocol:** CRSF (ExpressLRS / TBS Crossfire) — *not yet physically connected*
- **ESC protocol:** DShot600 — *not yet physically connected*
- **GPS:** NMEA over UART6 — *not yet physically connected*
- **Task model:** async tasks communicating via lock-free Signals

### Implemented Modules (all tested, no_std compatible)

| Module | File | Tests | Description |
|--------|------|-------|-------------|
| CRSF parser | `drivers/crsf.rs` | 5 | Streaming byte parser, 11-bit channel unpacking, link statistics, CRC8 validation |
| CRSF UART task | `rc_task.rs` | — | Embassy async task, DMA reads, Signal publishing, failsafe timer |
| WT901B parser | `drivers/wt901b.rs` | 8 | All packet types (accel, gyro, angle, mag, baro, quaternion), configuration commands |
| DShot encoder | `drivers/dshot.rs` | 6 | Frame encoding, CRC, DMA buffer generation, speed/timing calculations |
| Mixer | `control/mixer.rs` | 3 | Generic N-motor mixer with const mix matrices, quad-X preset |
| Main loop sketch | `main.rs` | — | Task spawning, control loop structure, arming logic, placeholder P-controller |
| TinyMPC skeleton | `tinympc-rs/` | — | Separate crate, compiles for thumbv7em-none-eabihf, demonstrates algorithm structure |

**Total: 52 passing tests on host** (`cargo test --no-default-features --target x86_64-unknown-linux-gnu`)

### Key Design Documents
- `ARCHITECTURE.md` — module structure, task model, data flow diagrams, data types

---

## What's Next (Core — Required to Fly)

### 1. PID Controller (`control/pid.rs`)
A proper PID with derivative-on-measurement, integral windup limits,
and configurable gains. Runs at IMU rate (200 Hz) as the inner loop.
Straightforward to implement — well-trodden ground.
//Initial work done 

### 2. DShot Hardware Driver
Wire the DShot encoder to actual STM32 timer + DMA peripherals.
Configure TIM1 (or similar) in PWM mode, set ARR for bit period,
DMA feeds CCR values from the buffers we already generate. This is
the most hardware-specific piece remaining.

### 3. Pin Assignment & Board Definition
Resolve the actual USART/timer/pin mapping for the target board.
Spend time with the STM32F405 alternate function table. Key
constraints: 4 timer channels for DShot that don't conflict with
3-4 USARTs for CRSF, IMU, GPS, and telemetry.

### 4. MPC Integration (`control/mpc.rs`)
Wrap `tinympc-rs` (peterkrull's crate) as a cargo dependency.
Define the linearised quadrotor model (A, B matrices) and cost
weights (Q, R) in `config/vehicle.rs`. Run at 50 Hz, feeding
rate setpoints to the PID inner loop.

### 5. Arming State Machine
Proper pre-arm checks: throttle low, attitude level within
tolerance, RC link active, optionally GPS lock. Arm/disarm via
switch or stick gesture. Must be robust — accidental arming is
the most dangerous failure mode.

### 6. Embassy Project Setup — DONE
Cargo.toml wires embassy-stm32/executor/time/sync, defmt-rtt, and
panic-probe. `memory.x` at repo root defines F407VET6's 512K/128K.
`.cargo/config.toml` sets the thumbv7em target and a `probe-rs run`
runner. `build.rs` at repo root emits `-Tlink.x` and `-Tdefmt.x`.
Verified: `cargo run --release --bin fc-firmware` flashes and runs.

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

### 9. GPS Rescue / Return to Home

This is where the MPC approach really shines compared to
traditional cascaded PID. GPS rescue with PID requires multiple
layered controllers (position → velocity → attitude → rate) each
with separate tuning. MPC handles the entire trajectory as one
optimisation problem.

**The challenge:** GPS updates at 5-10 Hz with ~2m accuracy.
The controller needs to handle the slow, noisy position updates
alongside the fast, accurate IMU data. This is fundamentally a
state estimation problem as much as a control problem.

**Implementation outline:**

```
Phase 1 — GPS Driver & State Estimator
├── drivers/ublox.rs        # UBX binary protocol parser
├── state/gps_ekf.rs        # Extended Kalman Filter fusing:
│                            #   IMU (200 Hz) + GPS (10 Hz) + Baro
│                            #   Outputs: position, velocity, attitude
│                            #   (replaces WT901B's onboard filter
│                            #    for position states)
└── state/types.rs           # Full 12-state vector:
                             #   [x, y, z, vx, vy, vz,
                             #    roll, pitch, yaw, p, q, r]

Phase 2 — Position Control via MPC
├── control/mpc.rs           # Extend MPC to full 12-state model
│                            #   A, B matrices now include position
│                            #   dynamics, not just attitude
├── control/setpoint.rs      # GPS waypoint → reference trajectory
│                            #   Smooth trajectory generation
│                            #   between current position and target
└── control/mode.rs          # New modes: PosHold, ReturnToHome,
                             #   Waypoint following

Phase 3 — GPS Rescue State Machine
├── control/gps_rescue.rs    # Triggered by RC link loss
│                            #   1. Climb to safe altitude
│                            #   2. Orient toward home point
│                            #   3. Fly toward home at safe speed
│                            #   4. Descend and land (or loiter)
│                            #   All via MPC reference trajectory
│                            #   generation — the rescue IS just a
│                            #   sequence of waypoints fed to the
│                            #   same controller that does normal
│                            #   position hold.
```

**Why MPC makes GPS rescue more elegant:**

With PID, GPS rescue requires hand-tuned logic for each phase
(climb rate, cruise speed, deceleration profile, landing
detection). Each transition is a heuristic.

With MPC, the rescue becomes: "generate a reference trajectory
from current position to home at safe altitude, with velocity
constraints." The optimiser figures out how to get there while
respecting all constraints simultaneously. Phase transitions
are just waypoints in the trajectory. If wind pushes you off
course, the MPC re-plans automatically at 50 Hz.

The hard part isn't the controller — it's the state estimator.
Fusing 10 Hz GPS with 200 Hz IMU while handling GPS dropouts,
multipath, and the ~2m noise floor is where the real engineering
is. An EKF that trusts GPS when it's good and falls back to
IMU-only dead reckoning when it's bad is essential.

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

## Priority Order

```
Must-have (to fly at all):
  [1] PID controller
  [2] DShot hardware driver
  [3] Pin assignment
  [4] Embassy project setup
  [5] Arming state machine

Should-have (to fly well):
  [4] MPC integration (tinympc-rs)
  [8] ESC telemetry (RPM filtering)
  [10] Blackbox logging

Nice-to-have (features):
  [7] VTX/OSD
  [9] GPS rescue
  [11] Configuration interface
  [8.3-4] MPC-aware motor feedback
```
