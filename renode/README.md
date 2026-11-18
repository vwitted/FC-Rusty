# renode/ — emulation harness for FC-Rusty

Runs the firmware ELF on an emulated STM32H743 in
[Renode](https://renode.io): boot and clock-tree regression, peripheral
bring-up, defmt over USART6, register-sequence assertions, GDB on port 3333.
Directions and limits: `docs/renode-sim-directions.md`.

```
renode/
  run-tests.sh / .cmd    the check runner (.cmd finds Git Bash and hands off)
  checks/                one .resc per check, plus esc_model.py
  platforms/             local fixes and models layered on the stock platform
  flight.resc            interactive session: flight ELF, IMUs, GDB on 3333
```

Every path is relative to the **repo root**, which `run-tests.sh` makes the
CWD. Staged ELFs live in `target/renode/` (gitignored with `target/`).

## Running

```
scripts/stage-firmware.sh              build both variants into target/renode/
renode/run-tests.sh --quick            fast tier, ~45 s: run per commit
renode/run-tests.sh                    everything, ~5 min
renode/run-tests.sh boot               only checks whose name matches
BLESS=1 renode/run-tests.sh sim-baseline   accept intended sim changes
scripts/test-<check>.sh                one check, with its own notes
renode --console renode/flight.resc    interactive, from the repo root
```

Each `.sh` has a `.cmd` twin. On Windows use the `.cmd` or Git Bash: `bash`
on PATH under PowerShell is WSL, which has neither Renode nor a Rust
toolchain. The runner uses `$HOME/renode-current/renode.exe`; override with
`RENODE=`. It needs a Renode nightly from 2026-07-28 or later (the stock
platform uses `Analog.STM32H7_ADC`) and `defmt-print` for flight-bringup.

Stage before the ELF-dependent checks. **A stale ELF does not fail**, it
reports on the wrong firmware: on 2026-09-08 the staged motor-test build was
two weeks old, so both DShot checks had been passing against a binary that
predated the plant-capture work.

## Scripting Renode: three traps

- **Always pass `--console`.** Without it the Monitor is hidden and command
  errors vanish: a failed `LoadPlatformDescription` prints nothing and the
  machine simply never boots.
- **Always cap output** (`| head -c 400000`). An unbounded run stuck in a
  boot loop wrote 881 MB in four minutes.
- **UART output is defmt-framed binary.** Decode with
  `defmt-print -e <elf>`; the analyzer's text rendering is lossy.

## How @-paths resolve

Renode resolves `@`-paths against the directory holding the marker file
`.renode-root` (the install), then falls back to the CWD. So:

- `@platforms/cpus/stm32h743.repl` always means the **stock** file in the
  install. A local file of that name cannot shadow it; an earlier local fork
  was silently ignored for a session for exactly this reason.
- `@renode/...` and `@target/renode/...` exist only in the repo, so they fall
  back to the CWD. That is why the runner `cd`s to the repo root.
- `cp -r install/*` does not copy `.renode-root`. If Renode reports
  `Could not find '.renode-root'`, fix the install first.
- Neither `$CWD` nor `$ORIGIN` expands inside an @-path. A check that needs
  an absolute host path (the UART file backend) receives it as a variable
  via `-e`, which runs after any script argument.

## Platform

Stock `@platforms/cpus/stm32h743.repl` (a 4-line overlay on upstream
`stm32h7.repl`) already models what this board needs to boot: `pwr`/`fmc` as
Python peripherals (without them `embassy_stm32::init()` stalls on the clock
tree), the GPIO-timer AF maps, and the RCC aliases. Local additions:

| File | Effect |
|---|---|
| `platforms/local-overlay.resc` | Applies the next two. Include after the stock platform. |
| `platforms/STM32DMA_ClearsEnable.cs`, `local-dma.repl` | DMA1/DMA2 clear `SxCR.EN` when a non-circular transfer completes. |
| `platforms/local-spi1.repl` | SPI1 from a Tag to `STM32H7_SPI`, RX DMA request 37. |
| `platforms/local-imu.resc` | Optional, after the overlay: fixed-value ICM-42688-P on SPI1 and SPI4. |
| `platforms/ICM42688P.cs`, `local-icm42688.repl` | That model and its bus and chip-select wiring. |

**DMA.** Upstream `STM32DMA` sets TCIF when NDT reaches 0 but never clears
`EN`, and embassy-stm32's `Transfer::is_running()` polls `EN`. Every
DMA-backed await therefore stayed Pending after its data moved; on stock the
flight firmware parks in `Icm42688::new` and prints four defmt lines. The
class is `sealed`, hence a runtime-compiled copy (`include @file.cs`). A typed
peripheral cannot be redeclared, so the overlay unregisters the stock
instances first, and registers the replacements without an RCC alias, which
the RCC still holds for the old ones: DMA clock gating is unmodelled.

**SPI1.** Under the stock Tag, `SPI_SR` reads 0 and TXC never sets, so SPI1
needs both the model and the DMA fix. `STM32H7_SPI` exposes `IRQ` and
`DMARecieve`, so the interrupt must be named (`IRQ -> nvic@35`); a bare
`-> nvic@35` is `Error E15: Ambiguous choice of default interrupt`.

**IMU.** Values are set in the CHIP frame from the Monitor
(`sysbus.spi1.icm1 AngularRateZ 10.0`); the firmware applies the mounting
rotation. The default, chip `(0, 0, +1) g`, is level rest for both
mountings. Transactions are framed on chip-select (PA4, PB1): `STM32H7_SPI`
calls `FinishTransmission()` only when TSIZE > 0, and embassy writes 0.

## Checks

Two tiers, 8 checks. Five need a staged ELF: flight for boot-smoke,
logger-uart and flight-bringup; motor-test for dshot-gpio and esc-model. For
what each asserts and why, and for editing the suite, see the
`renode-tests` skill (`.claude/skills/renode-tests/SKILL.md`).

boot-smoke passes even when `main()` is parked forever on an await, because
the executor's WFE loop is in flash. flight-bringup is the check that
catches that.

`sim-baseline` runs the HOST sim, not Renode, diffing `sim_sweep --csv`
against `tests/sim-baseline.csv`. It lives in this runner because the runner
is the per-commit gate. When a change to the sim or the controllers is
intended, re-bless in the SAME commit so the moved numbers are reviewable
beside the change.

## What it can and cannot test

**DShot is emitted, but untimed and only once.** Renode's DMA runs a
memory-to-peripheral transfer the moment the stream is enabled rather than
waiting for the TIM1 request (Renode's STM32H7 timers do not raise DMA
requests; renode/renode #618). The *order* of the 51 BSRR words is correct,
so a CRC-valid frame decodes on all four motors; bit order, bidir inversion,
per-pin muxing and hold states are checkable from order alone, which is why
`checks/esc_model.py` never uses timestamps. The *intervals* are
meaningless: bit width, DShot600 vs 300, guard times and the RX window
cannot be checked. Only one frame appears and the firmware parks on the TX
transfer; the missing `EN` clear is the likely cause, but the DShot checks
still run on stock and have not been re-measured on the overlay.

**Timing generally.** The DWT cycle counter (0xE0001000) is unmodelled, so
DWT-based figures read 0. Once the 8 kHz chain runs, one virtual second
costs ~2.4 min of host time.

Good targets: boot and clock regression, sensor bring-up, UART protocols
(CRSF on UART5, GPS on USART1), register-sequence assertions via hooks.
DShot frame logic is covered on the host instead (`scripts/test-host.sh`);
`dshot_bitbang.rs` itself is reachable by neither.
