---
name: renode-tests
description: Run, interpret or extend the Renode test suite in renode/ (renode/run-tests.sh, run-tests.cmd, the .resc checks in renode/checks/, esc_model.py, the platform overlay and models in renode/platforms/). Use when running the emulator checks, diagnosing a failing check, or adding a new one.
---

# Renode test suite

```
scripts/stage-firmware.sh             # build both ELFs into target/renode/ first
renode/run-tests.sh --quick           # fast tier only, ~45 s — run this per commit
renode/run-tests.sh                   # everything, ~5 min
renode/run-tests.sh boot              # only matching checks
```

Use `renode\run-tests.cmd` from PowerShell. Harness layout, @-path rules and
the platform overlay: `renode/README.md`.

Two tiers. The slow checks wait out the motor-test firmware's 2.5 s countdown
before it touches a register; that countdown is a REMOVE-PROPS safety feature
and is deliberately **not** made skippable by a build flag. Raising
`cpu PerformanceInMips` only bought ~17%, so the cost is real execution, not
throttling.

Renode's Monitor has no assertion language, so each `.resc` prints its
findings between `RESULT-BEGIN`/`RESULT-END` and `run-tests.sh` does the
checking. Non-zero exit on failure. The runner `cd`s to the repo root, so
every path in a check is repo-relative (`@renode/...`, `@target/renode/...`).

## fast tier (~45 s; boot-smoke and logger-uart run the flight ELF for 50 ms)

- **boot-smoke** — the flight firmware gets through H743 bring-up.
  Asserts defmt bytes leave USART6, *not* merely that the PC is in flash: the
  characteristic failure is spinning forever on PWR ACTVOSRDY inside
  `embassy_stm32::init()`, which is also "in flash". Verified to fail against
  a platform without the `pwr` Python peripheral, where it reports the stall
  and 2765 PWR warnings.
- **logger-uart** — reads USART6 back and asserts `BRR = 0x412` and
  `CR1 = UE|TE`. `logger.rs` hard-codes APB2 = 120 MHz to compute the baud
  divisor and nothing enforces it, so retuning the clock tree leaves the
  logger emitting bytes at the wrong rate: defmt output turns to garbage with
  no error anywhere. Cheap guard on an expensive-to-debug failure.
- **flight-bringup** (~15 s, flight ELF for 150 ms) — the only check on
  `renode/platforms/local-overlay.resc` (DMA that clears `EN` on completion,
  real SPI1) plus `local-imu.resc` (fixed-value ICM-42688-P on SPI1 and SPI4,
  level rest). Decodes USART6 with `defmt-print` and asserts both IMUs
  initialise, `MEKF seeded: euler=[0,0,…]`, and `Control loop starting`.
  Exists because boot-smoke passes while `main()` is parked forever on a DMA
  await: the executor's WFE loop is in flash. On stock the firmware prints
  four lines and stops in `Icm42688::new`. The level seed guards the
  chip->body mounting convention: a flipped chip Z was verified to seed at
  180 deg and fail. Kept at 150 ms because the 8 kHz chain costs ~2.4 min of
  host time per virtual second. The UART file backend needs an absolute
  path, so the runner passes `$uart` via `-e` and includes the .resc from
  there.
- **sim-baseline** (~10 s) — the only check that is not Renode. Runs the
  HOST sim (`sim_sweep --csv`) and diffs 96 rows against
  `tests/sim-baseline.csv` using `tests/check-sim-baseline.py`.

  It is a regression check, not a demonstration: the value is which rows
  MOVED, not the table itself. Float columns compare at 0.1% relative plus a
  1e-4 absolute floor (the sweep is bit-identical run-to-run on one host, but
  this project builds on Windows and Debian and libm differs in the last
  ulp); failure counts compare exactly, because 0/8 becoming 1/8 is a real
  change however small the floats moved.

  Sensitive: changing `motor_tau` by 17% moves 252 values and flips whole
  axes between 8/8 crash and 0/8. That is the plant sitting near a stability
  boundary, not the check being twitchy.

  When a change is intended: `BLESS=1 renode/run-tests.sh sim-baseline`, in
  the same commit, so the moved numbers are reviewable next to the cause.
  Skips cleanly if `cargo` is not on PATH.

## slow tier (~4 min, needs the motor-test ELF and its countdown)

- **dshot-gpio** (~2.5 min) — runs the motor-test firmware through
  `DshotBitbang::new` and reads GPIOA back, asserting PA0–PA3 are OUTPUT /
  PUSH_PULL / LOW_SPEED / PULL_UP. This is real validation of load-bearing
  pin setup that is otherwise only checkable by eye or on a bench. Slow
  because the firmware's 2.5 s bench countdown runs first.

- **esc-model** (~100 s) — `renode/checks/esc_model.py` simulates an ESC: it
  decodes DShot off the BSRR write stream, drives a first-order motor model,
  and builds bidirectional telemetry. `renode/checks/esc-sim.resc` hooks
  GPIOA and the model decodes what the firmware actually put on the pads —
  currently a CRC-valid zero-throttle arming frame on all four motors. Run
  the model's own checks with `python renode/checks/esc_model.py` (instant,
  no Renode): 15 checks covering decode, CRC rejection, silence, spin-up and
  a telemetry round-trip through a mirror of the firmware's decoder.

Both slow checks run on the stock platform, not the overlay.

## Running one check

Each check has a launcher carrying its own notes on what it asserts and
what it needs:

    scripts\test-boot-smoke.cmd
    scripts\test-logger-uart.cmd
    scripts\test-flight-bringup.cmd
    scripts\test-sim-baseline.cmd
    scripts\test-dshot-gpio.cmd
    scripts\test-esc-model.cmd

Each has a `.sh` twin for Git Bash. They are thin: each calls
`scripts/renode-check.{sh,cmd} <name>`, which hands off to
`renode/run-tests.{sh,cmd}`, so there is one copy of the logic.

## Adding a check

1. `renode/checks/<name>.resc`, with repo-relative @-paths and a
   `RESULT-BEGIN`/`RESULT-END` block (or a file backend, as flight-bringup).
2. A block in `renode/run-tests.sh`: `matches <name> [slow]`, `want` for any
   ELF, then `ok`/`bad`/`check`.
3. `scripts/test-<name>.{sh,cmd}`, and the name in `renode-check`'s usage.
4. Prove it can fail: run it once against a deliberately broken input.

## Staging the ELFs

Five checks need an ELF staged in `target/renode/`: `fc-firmware` (flight,
for boot-smoke, logger-uart and flight-bringup) and `fc-firmware-motortest`
(built with `--features motor-test`, for dshot-gpio and esc-model). The
runner says so if one is MISSING.

It cannot tell you one is STALE, which is the failure that actually
happens: a stale ELF passes, having reported on the wrong firmware. On
2026-09-08 the staged motor-test build was two weeks old and both DShot
checks had been green against a binary predating the plant-capture work.

`scripts/stage-firmware.sh` builds both and copies them. The copies exist
because both variants write the same cargo output path, so a later
motor-test build would otherwise replace the flight ELF under test.
