# CLAUDE.md — standing instructions for this repo

This file is loaded automatically into every Claude Code session
working on FC-Rusty. Keep it short and durable — conversational
state belongs in Claude's memory, not here.

---

## What this project is

FC-Rusty is a Rust flight controller for the **Radiolink F722**
(STM32F722RET6, Cortex-M7F @ 216 MHz) built on Embassy async.

North-star: **high-authority attitude control via MPC**. Subordinate
every other decision (estimation, sensors, arming, comms) to the
stability and authority of the inner loop. See `PROJECT_STATUS.md`
for the full state snapshot.

Control cascade: Position PD (5 Hz) → Attitude MPC (50 Hz) →
Rate PID (200 Hz) → DShot. PosKF 6-state (GPS + baro + IMU predict)
at 100 Hz; MEKF attitude at 8 kHz.

---

## Durable rules

### Keep `PROJECT_STATUS.md` in sync

**Update `PROJECT_STATUS.md` whenever a material hardware or design
change lands** — verified peripheral, commit that changes control
behaviour, killed sensor, new backlog item. If you're writing a
commit message that would change what "What's Verified" or
"Code-Done but Unflashed" or "Backlog" should say, update the doc
in the same commit.

A stale status doc is worse than no status doc — do not let it
drift again.

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

# Flash to Radiolink F722 via USB DFU (hold boot button, plug USB)
./scripts/flash-dfu.sh

# Host unit tests (strips firmware feature so cortex-m isn't pulled in)
cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu

# Simulation examples
cargo run --example sim_mpc_hover --no-default-features
cargo run --example sim_gps_rescue --no-default-features
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
  DShot.
- `src/sim/` — host-side 6DOF physics + sensor models.
- `src/control/tinympc-rs/` — no_std MPC solver (vendored).
- `examples/sim_*.rs` — host sim harnesses.
- `ARCHITECTURE.md` — module structure, task model, data flow.
- `PROJECT_STATUS.md` — current state, next steps, post-Alpha ideas.
- `docs/` — append-only session logs for open investigations
  (e.g. `motor-bringup-log.md`). Not authoritative; useful context
  when picking up a stalled bring-up thread.
