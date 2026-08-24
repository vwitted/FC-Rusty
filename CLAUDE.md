# CLAUDE.md — standing instructions for this repo

This file is loaded automatically into every Claude Code session
working on FC-Rusty. Keep it short and durable — conversational
state belongs in Claude's memory, not here.

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

But **never infer current behaviour from these docs.** Parts are
stale, and parts were aspirational or never true (this has already
misled an outside reviewer into "fixing" bugs that didn't exist).
Verify behaviour against the code. This file (CLAUDE.md) is the only
doc kept short, curated, and trustworthy.

**Comments:** keep the *why* — rationale, hazards, conventions,
current pin/sensor mappings. Delete or fix comments that assert a
stale *fact* (retired boards like the F407, removed sensors like the
WT901B, wrong loop rates). Don't blanket-strip comments; the *why*
is what prevents regressions.

### Hardware-safety rules (non-negotiable)

- **I2C bus-recovery bitbang MUST use `OutputOpenDrain`, never
  `Output`.** A push-pull output fighting a clock-stretching slave
  short-circuits the MCU's PMOS through the slave's NMOS. This
  killed the onboard DPS310 on 2026-04-20. Never `Output::new` on
  an I2C pin.

- **The ESC 'V' pad is Vbat (11–25 V LiPo), not 5 V.** Bridging it
  to the FC's 5 V rail has already destroyed a previous dev board,
  the GPS, and the ST-Link. If you're writing code or docs that
  touch ESC wiring, carry this warning forward.

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

# Host unit tests (strips firmware feature so cortex-m isn't pulled in).
# The tests run as a native binary, so they need the host triple, not the
# thumbv7em default from .cargo/config.toml. These wrappers read it from
# `rustc -vV`, so the same command works on Debian and on Windows.
scripts/test-host.sh                    # Debian, or Git Bash on Windows
scripts\test-host.cmd                   # Windows PowerShell / cmd
scripts/test-host.sh persist::record    # filter to one module or test

# Simulation examples. Examples are HOST binaries, so they need the host
# triple for the same reason the tests do -- .cargo/config.toml pins
# [build] target = thumbv7em, and without --target you get
# "can't find crate for `std`".
TRIPLE=$(rustc -vV | sed -n 's/^host: //p')
cargo run --release --example sim_mpc_hover --no-default-features --target $TRIPLE
cargo run --release --example sim_sweep    --no-default-features --target $TRIPLE
cargo run --release --example sim_sweep    --no-default-features --target $TRIPLE -- --csv

# sim_sweep runs the full cascade across a grid of sensor degradations
# (gyro noise, bias, vibration, intermittency, motor asymmetry), 8 seeds per
# case, and reports att_rms / att_max / alt_rms / air% / failures. The whole
# sweep is well under a second -- it is the host sim, not the emulator, and
# is the right tool for degradation statistics. See src/sim/degrade.rs.
```

The `firmware` feature gates all Embassy/Cortex-M/defmt deps.
Disable it (`--no-default-features`) when building on the host.

---

## Where things live

- `src/main.rs` — Embassy task spawning, control loop, arming logic.
- `src/control/` — PID, MPC, altitude, position, mixer, arming FSM.
- `src/estimation.rs` — 6-state PosKF.
- `src/attitude_mekf.rs` — quaternion MEKF (gyro-bias state).
- `src/drivers/` — ICM-42688P, DPS310, CRSF, NMEA, WT901B (fallback),
  DShot. The DShot driver is `dshot_bitbang.rs` (TIM1-paced DMA to
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
