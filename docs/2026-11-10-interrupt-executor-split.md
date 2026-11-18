# Interrupt-executor split (draft, 2026-11-10)

Draft only. Nothing in `src/` has been changed. Not compiled, not bench-tested.

## Problem

`main.rs` has one executor (`#[embassy_executor::main]`, line 440). Every
task is cooperative on it, so anything that computes without an `.await`
stalls everything else:

- `mpc.solve` at `main.rs:3005` stalls `control_loop` (`main.rs:3120`),
  `mekf_task` (1456) and `dual_icm_read_task` (1206) for the whole solve.
  The log line at `main.rs:3077` already prints `mpc_max`; divide by 125 us
  for the number of gyro samples lost per solve.
- `RAW_IMU` and `IMU_DATA` are latest-value `Signal`s (191, 196), so the
  lost samples are silent. The MEKF integrates a gyro sample as if it
  covered one period when it covered several.
- Flash writes from `persist_task` do the same (your comment at 3162).

## Stage 1: minimal split (about 15 lines)

Move the three 8 kHz tasks onto an `InterruptExecutor`. Everything else
stays where it is. All shared statics are already
`CriticalSectionRawMutex` (lines 191-353), so no sync changes are needed.

```rust
// main.rs, near line 44
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};

static EXECUTOR_FAST: InterruptExecutor = InterruptExecutor::new();

// Any vector the board never uses. DCMI is not pinned out on this FC.
#[interrupt]
unsafe fn DCMI() {
    unsafe { EXECUTOR_FAST.on_interrupt() }
}
```

In `main`, after `embassy_stm32::init`:

```rust
interrupt::DCMI.set_priority(Priority::P6);
let fast = EXECUTOR_FAST.start(interrupt::DCMI);
```

Then:

| Line | Now | Becomes |
|------|-----|---------|
| 613  | `spawner.spawn(dual_icm_read_task(imu1, imu2))` | `fast.spawn(...)` |
| 617  | `spawner.spawn(single_icm_read_task(imu1))` | `fast.spawn(...)` |
| 622  | `spawner.spawn(mekf_task())` | `fast.spawn(...)` |
| 3120 | `async fn control_loop(..) -> !` | add `#[embassy_executor::task]` |
| 696  | `control_loop(dshot).await;` | `fast.spawn(control_loop(dshot)).unwrap();` then park `main` (`core::future::pending::<()>().await`) |

`icm_monitor_task`, `navigation_task`, `pos_kf_task`, GPS, baro, RC,
persist and blink stay on the thread executor.

### Things that will bite

1. **Priority ordering.** The fast executor's vector must be numerically
   higher (less urgent) than every interrupt its tasks wait on: the SPI1
   and SPI4 DMA channels, DMA2_CH2 for DShot, and the TIM5 time driver.
   Check what `board_config()` (line 365) leaves `dma_interrupt_priority`
   at; if those are at P6 or lower the fast tasks deadlock on their own
   wakeups.
2. **`Send`.** `InterruptExecutor::start` returns a `SendSpawner`, so task
   arguments must be `Send`. `Icm42688<'static>` and `DshotBitbang`
   (`dshot_bitbang.rs:224`) should be, since they only hold `Peri` handles,
   but the compiler will say so if not.
3. **The logger defeats the split.** `logger.rs:93` takes a
   `critical_section` and busy-waits on USART6 TXE. With
   `critical-section-single-core` that masks ALL interrupts, including the
   fast executor. A `defmt::info!` from the navigation task still stalls
   the rate loop for the length of the line (about 87 us per byte, per
   your own comment at 3233). Either cut flight-path logging to the 1 Hz
   summaries or move the UART drain out of the critical section into a
   ring buffer emptied by a thread-mode task.
4. **Flash.** Check whether embassy-stm32's H7 blocking flash write and
   sector erase run inside a critical section. If they do, blackbox
   writes still stall the fast loop and the erase stalls it for the whole
   erase time.
5. **`mekf_task` logs and does the 100 Hz accel/mag updates.** In stage 1
   those now run at high priority. That is no worse than today, but it is
   the reason for stage 2.

## Stage 2: take the MEKF out of the rate path

Today `control_loop` waits on `IMU_DATA`, which `mekf_task` publishes, so
the rate loop cannot run without the estimator (this is also why the
gyro-sign scenarios in `sim/REPORT.md` are unrecoverable).

- Fast executor: IMU read, filter, rate PID, DShot. The rate loop reads
  `RAW_IMU` directly and subtracts a gyro bias the MEKF publishes at low
  rate (three atomics or a `Watch`).
- The read task also accumulates delta-angle over N samples and hands the
  MEKF one integrated increment per N. The MEKF then runs at 1 kHz or
  less on the thread executor and can be preempted or delayed without
  losing rotation.

If any future move to INDI this is the structure it needs anyway: gyro, eRPM and motor command all
in one fast task with matched filters.

## Separate finding: the 8 kHz ticker is not 8 kHz

`Cargo.toml` sets `tick-hz-32_768` for embassy-time, and
`dual_icm_read_task` uses `Ticker::every(Duration::from_micros(125))`
(`main.rs:1214`). 125 us is 4.096 ticks, so the period is a whole 4 or 5
ticks: about 8192 Hz or 6554 Hz, never 8000. The comment at `main.rs:449`
notes this for dt measurement, but the sampling itself has the same
problem, and `ImuFilterParams` designs its biquads for 8000 Hz.

Check the `MEKF {} samples/s` line (`main.rs:1800`) for the real figure.
Fix: `tick-hz-1_000_000` (TIM5 is 32-bit, so this is safe), or better,
clock the read from the ICM's own DRDY. This board maps no EXTI pin for
it, which is an argument for the ISM6HG256X breakout on INT1.

## Verification

1. Bench, props off: `control loop N Hz measured` (`main.rs:3305`) should
   read the same with the MPC enabled and disabled.
2. Add a dropped-sample counter: compare `ICM_SAMPLES` with the number of
   iterations `control_loop` and `mekf_task` ran in the same second.
3. Toggle a spare GPIO in `control_loop` and scope it with the ZT-703S
   while arming and while the blackbox is writing.

## Review and implementation (2026-11-18)

Claims were checked against the code and the locked crate sources
(embassy-executor 0.9.1, embassy-stm32 0.4.0, embassy-time 0.5.1).
Branch `interrupt-executor-split`.

### Corrections to the draft

- **Ticker rate.** `Duration::from_micros` rounds up (`div_ceil`), so
  125 us is always 5 ticks: a steady 6553.6 Hz, not an alternation of 4
  and 5 ticks.
- **MEKF dt.** `mekf_task` measures dt from DWT (clamped to 2 ms), so it
  does not integrate a stale sample as one period. It holds the latest
  rate over the real interval. Rotation in the skipped samples is lost
  (zero-order hold), and so is any interval beyond the 2 ms clamp.
- **Priority deadlock (item 1).** DMA interrupts default to P0
  (`dma_interrupt_priority`) and the TIM5 time driver never sets a
  priority, so it is also P0. P6 is safe. A task that awaits cannot
  deadlock on a lower-priority wakeup in any case. The risk applies only
  to busy-waits.
- **Flash (item 4).** Confirmed. `write_chunk_with_critical_section` and
  `erase_sector_with_critical_section` (`flash/common.rs`) mask all
  interrupts. Erase happens only while disarmed or in the dump build.
  Blackbox word writes while armed still stall the fast executor.

### Implemented

- Stage 1 as drafted. The DCMI vector runs at P6, is started before the
  IMU spawn and is gated to the flight build. Both read tasks, `mekf_task`
  and `control_loop` are spawned on it, and `main` parks. No
  `ThreadModeRawMutex`, `NoopRawMutex` or `RefCell` is reachable from the
  moved tasks. The compiler accepted the `Send` bounds.
- Logger (item 3). Frames are encoded into an 8 KB ring inside the
  critical section and drained by the USART6 TXE interrupt at P15. It
  falls back to the old blocking path when the ring is full, so the dump
  CSV loses no bytes. It also blocks when interrupts were masked before
  the log call (panic-probe disables them) and in `flush()`.
- `tick-hz-8_000`, in its own commit, not the drafted 1 MHz. embassy-stm32's
  time driver treats TIM5 as a 16-bit counter with a 2^15-tick period
  (`calc_now`). If interrupts stay masked for longer than about one
  period, `now()` can step backward, and `Instant::duration_since`
  panics on its `checked_sub`. A flash sector erase (the disarmed mag-cal
  save, persist-selftest, the blackbox dump) masks interrupts for the
  whole erase. The period is 33 ms at 1 MHz, 1.0 s at 32 768 Hz (already
  marginal, before this change) and 4.1 s at 8 kHz. The H743 erase time
  was not checked against the datasheet. At 8 kHz, 125 us is exactly one
  tick, but `Instant` resolves only 125 us, so the navigation-loop and
  MPC timings moved to DWT. Short waits round up to one tick: the I2C
  bus-recovery half-period goes from 30.5 us to 125 us.
  The motor-test loop rate also changes. `LOOP_KHZ` of 2, 4 and 8 are
  now exact; other values round up.
- `IMU_SEQ`, a monotonic sample counter. `control_loop` reports
  `control loop N Hz measured, IMU N Hz, missed N` once per second, armed
  or not, and `mekf_task` reports `(missed N)`.

### Not implemented

- Stage 2. It changes the rate loop's gyro source and the MEKF's input
  (delta-angle), and needs sim work and a sim-baseline re-bless. It is a
  design change, not a fix.

### Verification status

- All feature builds compile: default, motor-test, mpc-bench, blackbox,
  blackbox-dump, persist-selftest, dshot-rx-probe.
- 462 host tests pass, and the Renode suite passes 10/10.
- Under Renode the flight build stops at IMU init with or without this
  change (SPI1), so the fast executor never runs there. Decoded USART6
  output is the same four lines before and after, which confirms the
  interrupt drain delivers whole frames.
- Unverified until bench-tested: the timing benefit, and whether one
  IMU-to-DShot chain fits in 125 us at the now-true 8 kHz. If it does
  not, `missed` will be nonzero with the MPC disabled.
