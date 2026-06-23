# Mag-Cal LED Feedback — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the spin-cal lifecycle a no-radio, eyes-on signal on the onboard LED (PD10) — accelerating blink while calibrating, blackout at completion, burst on anchor, held slow-flash on fault.

**Architecture:** A pure host-tested `led_on(phase, elapsed_ms)` pattern function in `control/cal_led.rs`; `mekf_task` publishes the current `CalLed` phase to a `Watch`; the existing `blink_task` reads it each 25 ms tick and drives PD10.

**Tech Stack:** Rust (`no_std` firmware + host `std` tests), `embassy` `Watch`/`Ticker`, STM32H743.

**Spec:** `docs/superpowers/specs/2026-06-23-cal-led-feedback-design.md`

## Global Constraints

- Host tests: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`.
- Firmware build: `cargo build --release` (0 errors).
- `cal_led` is pure `no_std`; declared in **both** `src/lib.rs` and `src/main.rs` `control` modules (like `mag_cal`).
- PD10 is **active-low**: LED on → `set_low()`, off → `set_high()`.
- LED is advisory only — never affects control, arming, or the cal result.

---

## Verified codebase facts (do not re-derive)

- `blink_task` (`src/main.rs`, ~`:600`): currently
  ```rust
  #[embassy_executor::task]
  async fn blink_task(mut led: embassy_stm32::gpio::Output<'static>) {
      loop {
          led.set_low(); // Turn LED ON
          embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
          led.set_high(); // Turn LED OFF
          embassy_time::Timer::after(embassy_time::Duration::from_millis(900)).await;
      }
  }
  ```
  Spawned from `main` as `spawner.spawn(blink_task(led)).unwrap();` with `led = Output::new(p.PD10, Level::High, Speed::Low)`.
- `Watch` pattern in this crate: `static OUTER_CMD: Watch<CriticalSectionRawMutex, OuterLoopCommand, 2> = Watch::new();`, used as `OUTER_CMD.sender().send(x)` and `let mut r = OUTER_CMD.receiver().unwrap(); r.try_get()`.
- `src/main.rs:48`: `use embassy_time::{Duration, Instant, Ticker};` (all available).
- `mekf_task` (from sub-project B) lifecycle points where publishes go: `CalCommand::Start` arm, `CalCommand::Abort` arm, the ~500 ms `calibrator.progress()` log, the `calibrator.is_complete()` → `result()` `Some`/`None` branches, and the `anchor_heading` call. Existing import line: `use control::mag_cal::{CalCommand, MagCalibrator, DECLINATION_DEG};`.
- `mag_cal` module is declared in both crates' `control` mod — follow the same for `cal_led`.

---

## File structure

- Create `src/control/cal_led.rs` — pure `CalLed` enum + `led_on`. Host-tested.
- Modify `src/lib.rs` + `src/main.rs` — declare `control::cal_led`.
- Modify `src/main.rs` — `CAL_LED` watch static, `blink_task` renderer, `mekf_task` publishes.

---

### Task 1: `cal_led` pattern logic (pure, host-tested)

**Files:**
- Create: `src/control/cal_led.rs`
- Modify: `src/lib.rs`, `src/main.rs` (module declarations)

**Interfaces:**
- Produces: `pub enum CalLed { Idle, Calibrating(u8), AwaitingLevel, Saved, Fault }` (`#[derive(Clone, Copy)]`); `pub fn led_on(phase: CalLed, elapsed_ms: u32) -> bool`.

- [ ] **Step 1: Declare the module in both crates**

In `src/lib.rs`, inside `pub mod control { … }`, add `pub mod cal_led;` (after `pub mod arming;`, before `pub mod mag_cal;`). In `src/main.rs`, inside `mod control { … }`, add `pub mod cal_led;` in the same position.

- [ ] **Step 2: Write the file with tests**

Create `src/control/cal_led.rs`:

```rust
//! Onboard-LED (PD10) pattern for the magnetometer-cal lifecycle. Pure
//! no_std, host-tested; the renderer (`blink_task`) and publisher
//! (`mekf_task`) live in main.rs.
//! Spec: docs/superpowers/specs/2026-06-23-cal-led-feedback-design.md

/// Cal-feedback LED phase, published by the MEKF task and rendered by the
/// blink task. `Calibrating` carries coverage percent (0..=100).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalLed {
    Idle,
    Calibrating(u8),
    AwaitingLevel,
    Saved,
    Fault,
}

/// Whether the LED should be lit, given the phase and milliseconds since
/// the renderer last saw a *variant* change (progress updates within
/// `Calibrating` keep the same clock, so the blink stays smooth).
pub fn led_on(phase: CalLed, elapsed_ms: u32) -> bool {
    match phase {
        // 1 Hz heartbeat: 100 ms on / 900 ms off.
        CalLed::Idle => elapsed_ms % 1000 < 100,
        // Accelerating: faster + higher duty as coverage fills.
        // period 600→200 ms, duty 40→95 % as p 0→100. Near-solid at 100%.
        CalLed::Calibrating(p) => {
            let p = (p as u32).min(100);
            let period = 600 - 4 * p; // ms
            let duty_pct = 40 + (55 * p) / 100;
            let on_ms = period * duty_pct / 100;
            elapsed_ms % period < on_ms
        }
        // Coverage complete: hard OFF until the craft is held level.
        CalLed::AwaitingLevel => false,
        // Triple-burst (100 ms on/off ×3) then resume heartbeat.
        CalLed::Saved => {
            if elapsed_ms < 600 {
                (elapsed_ms / 100) % 2 == 0
            } else {
                elapsed_ms % 1000 < 100
            }
        }
        // Held fault: 5 s on / 5 s off until the pilot reverts AUX4.
        CalLed::Fault => elapsed_ms % 10000 < 5000,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_is_short_heartbeat() {
        assert!(led_on(CalLed::Idle, 50));
        assert!(!led_on(CalLed::Idle, 500));
    }

    #[test]
    fn calibrating_full_is_near_solid() {
        // p=100 → 200 ms period, 190 ms on.
        assert!(led_on(CalLed::Calibrating(100), 0));
        assert!(led_on(CalLed::Calibrating(100), 180));
        assert!(!led_on(CalLed::Calibrating(100), 195));
    }

    #[test]
    fn calibrating_empty_is_slower() {
        // p=0 → 600 ms period, 240 ms on.
        assert!(led_on(CalLed::Calibrating(0), 0));
        assert!(!led_on(CalLed::Calibrating(0), 300));
    }

    #[test]
    fn awaiting_level_is_off() {
        for t in [0u32, 200, 1000, 5000] {
            assert!(!led_on(CalLed::AwaitingLevel, t), "t={}", t);
        }
    }

    #[test]
    fn saved_bursts_then_heartbeats() {
        assert!(led_on(CalLed::Saved, 0)); // burst on
        assert!(!led_on(CalLed::Saved, 150)); // burst off
        assert!(led_on(CalLed::Saved, 250)); // burst on
        assert!(led_on(CalLed::Saved, 1000)); // heartbeat resumed (on)
    }

    #[test]
    fn fault_is_slow_5s() {
        assert!(led_on(CalLed::Fault, 1000));
        assert!(!led_on(CalLed::Fault, 6000));
    }
}
```

- [ ] **Step 3: Run tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu control::cal_led`
Expected: PASS (6 tests). (Implementation is alongside the tests; to honour RED, temporarily change `CalLed::AwaitingLevel => false` to `=> true` and confirm `awaiting_level_is_off` FAILS, then restore `false`.)

- [ ] **Step 4: Firmware build**

Run: `cargo build --release`
Expected: 0 errors. New `dead_code` warnings for `CalLed`/`led_on` (not wired until Tasks 2–3) are acceptable.

- [ ] **Step 5: Commit**

```bash
git add src/control/cal_led.rs src/lib.rs src/main.rs
git commit -m "cal-led: LED pattern logic (pure)"
```

---

### Task 2: `CAL_LED` watch + `blink_task` renderer

**Files:**
- Modify: `src/main.rs` (import, static, `blink_task`)

**Interfaces:**
- Consumes: `control::cal_led::{CalLed, led_on}` (Task 1).
- Produces: `static CAL_LED: Watch<CriticalSectionRawMutex, CalLed, 2>` — sent to by Task 3.

- [ ] **Step 1: Import `cal_led` and declare the watch**

Add to the `use control::…` block in `src/main.rs` (next to the `mag_cal` import):

```rust
use control::cal_led::{led_on, CalLed};
```

Near the other signal statics (after the `STORED_CAL` line added in sub-project B), add:

```rust
/// Cal-feedback LED phase: mekf task → blink task. Watch so the renderer
/// can poll the current phase every tick without consuming it.
static CAL_LED: Watch<CriticalSectionRawMutex, CalLed, 2> = Watch::new();
```

- [ ] **Step 2: Rework `blink_task` to render the phase**

Replace the whole `blink_task` body with:

```rust
#[embassy_executor::task]
async fn blink_task(mut led: embassy_stm32::gpio::Output<'static>) {
    let mut rx = CAL_LED.receiver().unwrap();
    let mut phase = CalLed::Idle;
    let mut phase_start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_millis(25));
    loop {
        if let Some(new_phase) = rx.try_get() {
            // Reset the pattern clock only on a *variant* change, so
            // Calibrating(p) progress updates don't restart the blink.
            if core::mem::discriminant(&new_phase) != core::mem::discriminant(&phase) {
                phase_start = Instant::now();
            }
            phase = new_phase;
        }
        let elapsed = phase_start.elapsed().as_millis() as u32;
        if led_on(phase, elapsed) {
            led.set_low(); // active-low: ON
        } else {
            led.set_high(); // OFF
        }
        ticker.next().await;
    }
}
```

- [ ] **Step 3: Build**

Run: `cargo build --release`
Expected: 0 errors. `led_on`/`CalLed` dead-code warnings clear (now used by `blink_task`). `CAL_LED` has no sender yet (Task 3) — reading a never-sent `Watch` returns `None`, so the LED renders the default `Idle` heartbeat: **no visible behaviour change vs. today**.

- [ ] **Step 4: Commit**

```bash
git add src/main.rs
git commit -m "cal-led: CAL_LED watch + blink_task phase renderer"
```

---

### Task 3: `mekf_task` publishes the phase

**Files:**
- Modify: `src/main.rs` (`mekf_task`)

**Interfaces:**
- Consumes: `static CAL_LED` (Task 2), `CalLed` (Task 1).

- [ ] **Step 1: Grab the watch sender before the loop**

In `mekf_task`, next to the cal state added in sub-project B (after `let mut last_cal_log = Instant::now();`), add:

```rust
    let cal_led_tx = CAL_LED.sender();
```

- [ ] **Step 2: Publish on Start / Abort**

In the `CAL_CONTROL.try_take()` match arms, add a publish to each:

```rust
                CalCommand::Start => {
                    calibrator.reset();
                    cal_active = true;
                    cal_led_tx.send(CalLed::Calibrating(0));
                    defmt::info!("MEKF cal: started — rotate the craft through all axes");
                }
                CalCommand::Abort => {
                    if cal_active {
                        defmt::info!("MEKF cal: aborted");
                    }
                    cal_active = false;
                    cal_led_tx.send(CalLed::Idle);
                }
```

- [ ] **Step 3: Publish progress, completion, fault, and anchor**

In the mag block, add the progress publish in the 500 ms log branch, and phase publishes in the completion + anchor branches:

```rust
            if cal_active {
                calibrator.feed(ut);
                if last_cal_log.elapsed().as_millis() >= 500 {
                    cal_led_tx.send(CalLed::Calibrating(calibrator.progress()));
                    defmt::info!("MEKF cal: coverage {}%", calibrator.progress());
                    last_cal_log = Instant::now();
                }
                if calibrator.is_complete() {
                    match calibrator.result() {
                        Some(off) => {
                            mekf.set_hard_iron(off);
                            cal_active = false;
                            anchor_pending = true;
                            cal_led_tx.send(CalLed::AwaitingLevel);
                            let cfg = persist::record::Config {
                                mag_hard_iron_ut: off,
                                declination_rad: DECLINATION_DEG.to_radians(),
                                mag_calibrated: true,
                            };
                            CAL_SAVE.signal(cfg);
                            defmt::info!(
                                "MEKF cal: COMPLETE offset=[{=f32},{=f32},{=f32}] — hold level to anchor",
                                off[0], off[1], off[2],
                            );
                        }
                        None => {
                            cal_active = false;
                            cal_led_tx.send(CalLed::Fault);
                            defmt::error!("MEKF cal: degenerate fit — aborted, keeping prior cal");
                        }
                    }
                }
            } else if !mekf.mag_initialized() {
                if mekf.initialize_mag_from_first(ut) {
                    defmt::info!("MEKF mag reference seeded (relative boot heading)");
                }
            } else {
                if anchor_pending && mag_anchor_ready(&raw) {
                    mekf.anchor_heading(ut, DECLINATION_DEG.to_radians());
                    anchor_pending = false;
                    cal_led_tx.send(CalLed::Saved);
                    defmt::info!(
                        "MEKF anchored to true north: yaw={=f32}deg",
                        mekf.euler()[2] * RAD2DEG,
                    );
                }
                if mekf.update_mag(ut) {
                    mag_applied = mag_applied.wrapping_add(1);
                } else {
                    mag_rejected = mag_rejected.wrapping_add(1);
                }
            }
```

(This is the sub-project B mag block with five `cal_led_tx.send(...)` lines added — Start/Abort in Step 2, and `AwaitingLevel`/`Fault`/`Saved` here. The `Calibrating(progress)` publish rides the existing 500 ms log throttle.)

- [ ] **Step 4: Build + full host suite**

Run: `cargo build --release`
Expected: 0 errors; `CAL_LED`/`CalLed`/`led_on` all now consumed.

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`
Expected: PASS — 154 (sub-project B) + 6 (`cal_led`) = **160**.

- [ ] **Step 5: Commit**

```bash
git add src/main.rs
git commit -m "cal-led: mekf_task publishes cal LED phases"
```

---

### Bench acceptance (user, hardware)

Run a spin-cal (AUX4, disarmed) and watch PD10: heartbeat → **accelerating
blink toward near-solid** as coverage fills → **blackout** at completion →
hold the craft level → **triple-burst** → heartbeat. Revert AUX4 any time
to abort back to heartbeat. (A degenerate `None` fit — rare — shows the
held 5 s on / 5 s off until you revert AUX4.)

---

## Self-review notes

- **Spec coverage:** five phases + patterns → Task 1 `led_on`; watch + renderer (variant-change clock reset, active-low) → Task 2; publishes at every lifecycle transition + throttled progress → Task 3. Anchoring-timing and Abort semantics are unchanged from sub-project B (this plan only adds `cal_led_tx.send` calls).
- **Type consistency:** `CalLed::{Idle, Calibrating(u8), AwaitingLevel, Saved, Fault}` and `led_on(CalLed, u32) -> bool` are used identically in Tasks 1–3; `CAL_LED` is the single `Watch` shared between Task 2 (receiver) and Task 3 (sender).
- **No behaviour change before Task 3:** Task 2 alone renders the default `Idle` heartbeat (the `Watch` has no sender yet), so each task is independently safe.
