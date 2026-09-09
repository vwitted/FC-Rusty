# CLAUDE.md — standing instructions for this repo

This file is loaded automatically into every Claude Code session
working on FC-Rusty. Keep this - and other repo docs - short, concise and reasonably formal, not in a breezy or idiomatic style.  Conversational
state belongs in Claude's memory, not here.

## Environment
This machine is Windows. Use PowerShell-compatible commands, not Git Bash heredocs or POSIX globs. When writing multi-line files, use the Write tool instead of `cat <<EOF`. Quote and escape Windows paths (`C:\Users\...`) explicitly, and never rely on shell glob expansion for `--exclude`/upload path arguments.

## Conventions
 Always state and verify conventions, including both firmware-based conventions (like idle high vs. low), and physics/conceptual conventions (body vs. world frame) before implementing dev work. 

## Verification
Do not claim a change works until it has been verified by actually running it (compile, execute the endpoint, hit the deployed URL, or check the build log). If verification is not possible, say so explicitly rather than asserting success.

## Assertions
When making assertions - especially bold assertions or that existing code/logic/theory is erroneous - verify and surface  in the conversation only with evidence and specific codebase references. 
---

## What this project is

FC-Rusty is a Rust flight controller for the **DAKEFPV H743**
(STM32H743VIT6, dual gyro) built on Embassy async.

North-star: **high-authority attitude control via MPC**. Subordinate
every other decision (estimation, sensors, arming, comms) to the
stability and authority of the inner loop. See `PROJECT_STATUS.md`
for the full state snapshot.

Control cascade: Position PD (5 Hz) → Attitude MPC (100 Hz) →
Rate PID (8 kHz) → DShot. PosKF 6-state (GPS + baro + IMU predict)
at 100 Hz; MEKF attitude at 8 kHz.

---

## Durable rules

### `PROJECT_STATUS.md` / `ARCHITECTURE.md` are a journal, not a spec

Keep logging material changes there when they land — verified
peripheral, commit that changes control behaviour, killed sensor,
new backlog item. The running record is useful.

But **never infer current behaviour from informational docs/logging items** Parts are
stale, and parts were aspirational or never true (this has already
misled an outside reviewer into "fixing" bugs that didn't exist). 
Be skeptical of stale code comments that have pre-date code changes in the file. 
Verify behaviour against the code. This file (CLAUDE.md) is the only doc kept short, curated, and trustworthy.

**Comments:** keep the *why*, but only write *facts* — rationale, hazards, conventions,
current pin/sensor mappings. Keep concise and reasonably formal, not breezy anecdotal style.  Delete or fix comments that assert a
stale fact (retired boards like the F407, removed sensors like the
WT901B, wrong loop rates). Don't blanket-strip comments; the *why*
is what prevents regressions.

### Hardware-safety rules (non-negotiable)

- **I2C bus-recovery bitbang MUST use `OutputOpenDrain`, never
  `Output`.** A push-pull output fighting a clock-stretching slave
  short-circuits the MCU's PMOS through the slave's NMOS. This
  killed the onboard DPS310 on 2026-04-20. Never `Output::new` on
  an I2C pin.
  
### DShot

- **DShot is bit-banged, not timer output-compare.** BF resolves
  `dshot_bitbang = AUTO` to bit-banging on everything after F4, so the
  timer-DMA path (`pwm_output_dshot_hal.c`) is not the reference for this
  board — `dshot_bitbang.c` is. A week was lost in July 2026 porting the
  wrong one.

### Git / destructive operations

- Default to creating new commits, not amending.
- Do not push to `main` without explicit instruction.
- Do not run `--force`, `reset --hard`, branch deletion, or any
  other destructive git operation without asking first, even if it
  looks like an obvious unblock.

---

## Build / run / test

```
# Embedded build (default features = firmware)
cargo build --release

# Flash to DAKEFPV H743 via USB DFU (hold boot button, plug USB)
./scripts/flash-dfu.sh

The `firmware` feature gates all Embassy/Cortex-M/defmt deps.
Disable it (`--no-default-features`) when building on the host.

details for unit tests and sim tests can be found at HOST_RUNNABLES.md when needed. 
```

## Plant calibration and the flight log

see TUNING_DETAILS.md for information when pertinant to the task. 

## Where things live

- `src/main.rs` — Embassy task spawning, control loop, arming logic.
- `src/control/` — PID, MPC, altitude, position, mixer, arming FSM.
- `src/estimation.rs` — 6-state PosKF.
- `src/attitude_mekf.rs` — quaternion MEKF (gyro-bias state).
- `src/drivers/` — ICM-42688P, DPS310, CRSF, NMEA, WT901B (fallback),
  DShot. Magnetometers: `lis2mdl.rs`, `qmc5883l.rs`, `hmc5883l.rs`,
  `ist8310.rs` share `mag.rs`; the SE100 GPS has carried all three of the
  latter across revisions (V2 = IST8310) and they differ in address,
  register map, axis order, endianness and handedness, so `Compass::probe`
  in main.rs scans the bus and identifies by ID register, never by guess. The DShot driver is `dshot_bitbang.rs` (TIM1-paced DMA to
  GPIOA BSRR/IDR) with `dshot_bb_frame.rs` building the BSRR words and
  `dshot_bb_decode.rs` decoding the bidir reply; `dshot_frame.rs` is
  the shared 16-bit frame encoder.
- `src/sim/` — host-side 6DOF physics + sensor models.
- `src/control/tinympc-rs/` — no_std MPC solver (vendored).
- `examples/sim_*.rs` — host sim harnesses.
- `ARCHITECTURE.md` — module structure, task model, data flow.
- `PROJECT_STATUS.md` — current state, next steps, post-Alpha ideas.
- `docs/` — append-only session logs for open investigations
  (e.g. `motor-bringup-log.md`). Not authoritative; useful context
  when picking up a stalled bring-up thread.
