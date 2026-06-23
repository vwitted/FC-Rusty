# Mag-Cal LED Feedback (Design)

**Date:** 2026-06-23
**Status:** Approved (design); not yet implemented
**Branch context:** `dakefpv-h743-post-alpha`
**Extends:** sub-project B — mag-cal/yaw fix
(`docs/superpowers/specs/2026-06-22-mag-cal-yaw-fix-design.md`).

---

## Why

The spin-cal lifecycle currently reports only over the USART6 defmt
console — useless at the field without a laptop. The pilot needs a
no-radio, eyes-on signal of cal progress and completion from the onboard
LED (PD10), readable even when the LED is partly obscured inside a frame
(only the USB-port vicinity is exposed).

Design constraint that follows from "obscured LED": signal with **stark
on/off transitions and held terminal states**, not subtle rhythm changes
the pilot might miss while rotating the craft.

(CRSF FC→TX telemetry is the richer long-term channel — see PROJECT_STATUS
"Pilot telemetry". The LED is the right-sized no-radio fallback and stays
even once telemetry exists.)

---

## Behaviour (approved)

| Phase | LED | Held until |
|-------|-----|------------|
| **Idle** | 1 Hz heartbeat (100 ms on / 900 ms off) | — |
| **Calibrating** | accelerating blink; rate **and duty** rise with coverage %, approaching ~100% duty (near-solid) at full coverage | auto |
| **AwaitingLevel** (coverage complete) | **OFF**, held | FC detects level-and-still |
| **Saved** (anchored) | triple-burst → heartbeat | auto (self-times) |
| **Fault** (degenerate fit) | **5 s on / 5 s off**, held | pilot reverts AUX4 (→ Idle) |

Rationale: accelerating-to-near-solid then a **stark blackout** is the most
visible "done" cue for an obscured LED. Each terminal state holds until the
pilot acts (hold level for success; revert AUX4 for failure), so it can't
be missed by looking away.

### Lifecycle mapping (to the existing sub-project B state)

- `Start` (AUX4 rising, disarmed) → **Calibrating**.
- coverage `is_complete()` + `result()=Some` → offset set, **flash write
  happens here** (unchanged), → **AwaitingLevel**.
- coverage complete + `result()=None` → **Fault**.
- `mag_anchor_ready` (≈1 g, <5°/s) while AwaitingLevel → `anchor_heading`,
  → **Saved** (then heartbeat).
- `Abort` (AUX4 falling edge, or arming) from **any** phase → **Idle**.
  This is the user-driven abort: a deliberate revert while only partly
  calibrated safely discards the in-progress cal and resumes heartbeat.
  `Abort` is also what clears **Fault**. `Abort` does not touch the
  separate `anchor_pending` flag, so reverting AUX4 *after* completion is
  harmless — anchoring still happens at the next level moment (just without
  the burst if you've already reverted).

### Anchoring timing (clarification, no behaviour change)

Anchoring is **not** next-boot-only. After a fresh spin-cal it anchors the
same session as soon as the craft is held level (this drives the Saved
burst). It also re-anchors at every boot's first level-still moment when a
stored cal exists (only the offset persists; attitude resets each boot).
"Next boot" is purely the fallback if the pilot never holds level after the
spin.

---

## Architecture

Three units; one new pure module, two wiring points.

| Unit | File | Responsibility | Tested |
|------|------|----------------|--------|
| LED pattern | `src/control/cal_led.rs` (new) | Pure: `CalLed` enum + `led_on(phase, elapsed_ms) -> bool`. | host |
| Renderer | `src/main.rs` `blink_task` | Read the published phase, track elapsed, drive PD10. | bench |
| Publisher | `src/main.rs` `mekf_task` | Publish `CalLed` at each lifecycle transition + throttled progress. | bench |

**Signal:** `static CAL_LED: Watch<CriticalSectionRawMutex, CalLed, 2>`
(retained latest value; `mekf_task` is the sender, `blink_task` the
reader). A `Watch` (not `Signal`) so `blink_task` can poll the current
phase every tick without consuming it.

**`CalLed` enum:** `Idle`, `Calibrating(u8 /*coverage %*/)`, `AwaitingLevel`,
`Saved`, `Fault`. `Copy`. Declared in both crates' `control` module (pure,
like `mag_cal`).

**Pure `led_on(phase, elapsed_ms)`** — `elapsed_ms` is time since the
renderer last saw a *variant* change (progress updates within `Calibrating`
do **not** reset it, so the accelerating blink stays smooth):
- `Idle`: `elapsed % 1000 < 100`.
- `Calibrating(p)`: `period = 600 − 4·p` ms (1.7→5 Hz as p 0→100);
  `duty = 0.40 + 0.55·p/100` (0.40→0.95); on iff `elapsed % period <
  period·duty`. At p=100 → ~190 ms on / 10 ms off (near-solid).
- `AwaitingLevel`: always `false`.
- `Saved`: `elapsed < 600` → triple-burst (`(elapsed/100) % 2 == 0`);
  else heartbeat (same as `Idle`).
- `Fault`: `elapsed % 10000 < 5000`.

**`blink_task`:** fast tick (~25 ms). Hold `current_phase` and a
`phase_start: Instant`; on each tick read `CAL_LED` (default `Idle`), and
if the variant changed (`core::mem::discriminant`) reset `phase_start`.
Compute `elapsed`, call `led_on`, drive PD10 (active-low: on → `set_low`).

**`mekf_task` publishes** via `CAL_LED.sender()`: `Calibrating(0)` on
`Start`; `Calibrating(progress)` at the existing ~2 Hz cal-log cadence;
`AwaitingLevel` on completion-Some; `Fault` on completion-None; `Saved` on
`anchor_heading`; `Idle` on `Abort`. (Internal `cal_active`/`anchor_pending`
logic is unchanged; this only adds publishes.)

---

## Error handling / safety

- LED is purely advisory; it never affects control, arming, or the cal
  result. A missed/garbled phase just shows a stale pattern until the next
  publish.
- No new task, no blocking, no DShot coupling. `blink_task` already owns
  PD10; the 25 ms tick is negligible.
- Disarmed-only context is unchanged (cal is disarmed-only); the LED simply
  overrides the heartbeat during the cal lifecycle.

## Testing

**Host (pure `led_on`):** Idle on@50/off@500; `Calibrating(100)` near-solid
(on@0, off@195); `Calibrating(0)` slower (on@0, off@300); `AwaitingLevel`
false for several t; `Saved` burst (on@0, off@150, on@250) then heartbeat
@700; `Fault` on@1000 / off@6000.

**Bench:** run a spin-cal and watch PD10 — accelerating blink → near-solid →
blackout at completion → (hold level) triple-burst → heartbeat; force a
degenerate fit (e.g. minimal rotation that trips coverage but not a clean
sphere is unlikely — instead verify the slow 5 s/5 s flash holds until AUX4
revert if a `None` fit ever occurs).

## Out of scope (YAGNI)

- CRSF telemetry (separate post-Alpha subsystem).
- Indicating "already calibrated" at boot, battery state, or any non-cal
  status on this LED.
- Buzzer (no pad wired on this board).
