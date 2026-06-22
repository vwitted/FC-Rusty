# Magnetometer Calibration + True-North Yaw Fix (Design)

**Date:** 2026-06-22
**Status:** Approved (design); not yet implemented
**Branch context:** `dakefpv-h743-post-alpha`
**Depends on:** sub-project A — `persist` flash config store
(`docs/superpowers/specs/2026-06-22-persist-flash-config-design.md`).

---

## Why

The attitude MEKF makes yaw observable from the magnetometer, but two
defects make absolute heading wrong:

1. **No hard-iron correction.** `update_mag` normalises the *raw* field,
   so a hard-iron offset shifts the vector before normalisation and leaks
   a heading-dependent error straight into yaw. (The header comment
   claiming hard-iron "doesn't leak" is wrong for an offset.)
2. **Arbitrary boot heading.** `initialize_mag_from_first` seeds the mag
   reference as "the field in nav at boot, assuming yaw = 0", so the
   filter holds yaw self-consistent to an *arbitrary* boot heading, not
   true north. The one true-north signal available (GPS course) only
   feeds the *position* KF today, never attitude. Result: NED-frame
   commands (GPS rescue / position hold) act on a wrong yaw → flyaway.

This is the root cause behind the independent-review "yaw frame mismatch"
finding. The fix: calibrate the hard-iron offset, anchor the MEKF to true
north (magnetic heading + declination), and fuse GPS course-over-ground
as a gated yaw measurement for in-flight robustness.

### North-star alignment

Yaw authority is part of inner-loop integrity. Correct absolute heading is
a prerequisite for the position cascade (MPC → rate PID) to command the
right body axes. Calibration runs disarmed and never touches the control
path; a flash fault or failed cal degrades to today's relative-heading
behaviour, never to a control hazard.

---

## Decisions (locked during brainstorming)

- **Build all of B at once** — calibration + anchor + GPS-COG fusion in one
  spec (not split).
- **Hard-iron only** (offset). No soft-iron / ellipsoid scale.
- **Completion by bin coverage**, not a fixed timer.
- **Declination is a compile-time `const`** (`DECLINATION_DEG`, default
  `0.3`). The persisted `Config.declination_rad` records the value at
  cal-save for traceability, but the live anchor always uses the const —
  one source of truth.
- **COG trusted only when** `groundspeed > V_MIN` **and** pilot
  forward-stick **and** a good 3D fix (a multirotor's COG equals heading
  only in deliberate forward flight).
- **Trigger: AUX4 = RC channel index 7** (the first free channel; index 4 =
  arm, 5 = flight mode, 6 = GPS home). Disarmed-only.

---

## Components

| Unit | File | Responsibility | Tested |
|------|------|----------------|--------|
| Calibrator | `src/control/mag_cal.rs` (new) | Pure: accumulate raw mag → sphere-fit offset + bin-coverage completion. | host |
| MEKF additions | `src/attitude_mekf.rs` (extend) | `set_hard_iron`, `anchor_heading`, `update_yaw_reference`. | host |
| Orchestration | `src/main.rs` (wire) | AUX lifecycle, COG-trust gate, persist save, safety. | bench |
| Declination | `const DECLINATION_DEG` in `mag_cal.rs` | Single source of truth (default 0.3°). | — |

The calibrator and all three MEKF methods are pure and host-tested; only
`main.rs` wiring is firmware/bench.

---

## 1. Calibrator — `mag_cal.rs` (pure)

`MagCalibrator`: feed raw mag samples; track offset estimate and sphere
coverage independently.

**Offset — least-squares sphere fit.** Hard-iron shifts the field sphere
off origin. For each sample accumulate the normal equations of the linear
system `[2x 2y 2z 1]·[cx cy cz d]ᵀ = x²+y²+z²` (a 4×4 `AᵀA` and 4×1
`Aᵀb`). At completion solve the 4×4 (nalgebra, vendored) → centre
`(cx,cy,cz)` is the hard-iron offset. More robust than min/max-midpoint
(a single spike corrupts min/max). Ill-conditioned 4×4 → `None`.

**Completion — bin coverage.** Tessellate direction-from-centre into
**8 azimuth sectors × 3 elevation bands = 24 bins**; set a bit per bin
hit. Complete when `popcount ≥ COVERAGE_BINS_REQUIRED` (20/24) **and**
`samples ≥ MIN_SAMPLES` **and** each-axis span `≥ MIN_SPAN_UT` (rejects a
"didn't rotate" false-complete). Centre for binning is the running
per-axis min/max midpoint (rough but fine for a coverage gate; the final
offset comes from the sphere fit, not the midpoint).

**Interface:**
```rust
pub struct MagCalibrator { /* accumulators, min/max, 24-bit coverage mask, count */ }
impl MagCalibrator {
    pub fn new() -> Self;
    pub fn reset(&mut self);
    pub fn feed(&mut self, sample: [f32; 3]);
    pub fn progress(&self) -> u8;          // 0..=100, popcount/24
    pub fn is_complete(&self) -> bool;
    pub fn result(&self) -> Option<[f32; 3]>; // sphere-fit offset, None if degenerate
}
```

**Tuning consts (in `mag_cal.rs`):** `COVERAGE_BINS_REQUIRED = 20`,
`MIN_SAMPLES = 400`, `MIN_SPAN_UT = 20.0`, `DECLINATION_DEG = 0.3`.

**Host tests:** synthetic offset sphere → centre within tol; full spread →
completes; horizontal-only ring → never completes; zero rotation → never
completes (span gate); single repeated point → `result()` None.

---

## 2. MEKF additions — `attitude_mekf.rs` (pure math)

**2a — Hard-iron application.** `set_hard_iron(&mut self, [f32;3])` stores
the offset (default zero). `update_mag` subtracts it before normalising:
`m = raw − offset; z = m/‖m‖`. Uncalibrated (zero offset) is identical to
today. *Test:* known offset + synthetic field → corrected direction
matches truth across headings.

**2b — True-north anchor.**
`anchor_heading(&mut self, mag_body_corrected, declination_rad) -> bool`,
called when level-and-still (accel ≈ 1 g, low gyro):

1. roll/pitch from the current (accel-converged) quaternion; build a
   yaw-zeroed quat `q0 = euler_to_quat(roll, pitch, 0)`.
2. `m0 = R(q0)·m_body` (corrected field).
3. `ψ_mag = −atan2(m0.E, m0.N)` (sign matches this codebase's NED +
   `quat_to_euler` yaw, guarded by a round-trip test).
4. `ψ_true = ψ_mag + declination_rad`.
5. `q = euler_to_quat(roll, pitch, ψ_true)`, then seed `mag_ref` from the
   corrected body field through `q` (reuse `initialize_mag_from_first`
   logic).

`mag_ref` ends self-consistent with the measured field (no accel/mag
fight) and yaw = 0 ⇒ true north. *Test:* known true heading + synthetic
field → `euler().yaw` matches; swept.

**2c — COG scalar yaw update.**
`update_yaw_reference(&mut self, yaw_meas, sigma_yaw) -> bool`:

- `y = wrap(yaw_meas − yaw(q))` to [−π, π].
- yaw-rotation axis in body = body projection of world-down =
  `r_bn_row_z(q)` ⇒ `H = [r_bn_row_z(q)ᵀ | 0₃]` (1×6).
- scalar `S = hᵀ P_θθ h + sigma_yaw²`; `K = P Hᵀ / S` (6×1); apply
  `δx = K·y`; standard P update + symmetry. No matrix inverse.
- `sigma_yaw` generous (~15°): COG≈heading is approximate even forward-gated.

*Test:* drift yaw +30°, feed COG at truth → yaw walks back; level accel
stream undisturbed; `H`-vector known-answer guard.

Both the `ψ_mag` sign (2b) and the `H` vector (2c) get explicit
known-answer tests — the classic places yaw code goes silently wrong.

---

## 3. Orchestration — `main.rs`

All tasks run on one cooperative thread-mode executor.

**Tasks:**
- `navigation_task` (existing): AUX4 lifecycle + COG-trust gate.
- `mekf_task` (existing): owns `MagCalibrator` + MEKF; applies result; fuses COG.
- `persist_task` (new, tiny): owns the `Flash` handle; writes on save.

**Signals** (`Signal<CriticalSectionRawMutex, _>`):

| Signal | Dir | Payload |
|--------|-----|---------|
| `CAL_CONTROL` | nav → mekf | `Start` / `Abort` |
| `CAL_SAVE` | mekf → persist | `persist::record::Config` |
| `YAW_COG` | nav → mekf | trusted true heading (rad) |
| `STORED_CAL` | main → mekf | boot offset from persisted `Config` |

**Cal lifecycle:**
1. Disarmed + AUX4 rising edge → nav `Start`. mekf resets calibrator,
   feeds every raw mag sample, logs coverage % ~2 Hz (LED fast-blink).
2. `is_complete()` → mekf solves offset, `set_hard_iron`, and at the next
   level-and-still window `anchor_heading(DECLINATION)`. Signals `CAL_SAVE`.
3. `persist_task` writes flash (disarmed). Logs `CAL SAVED`.
4. AUX4 low / arming / degenerate fit → `Abort`: keep prior offset, no
   persist.

**Boot:** `main` reads `Config` (sub-project A), signals `STORED_CAL`;
mekf `set_hard_iron`s the stored offset and anchors at the first
level-and-still reading. Uncalibrated `Config` ⇒ offset zero ⇒ today's
relative-boot-heading behaviour, unchanged.

**COG path:** nav computes a trusted heading only when
`groundspeed > V_MIN` **and** forward-stick beyond a deadband **and** a
good 3D fix, then signals `YAW_COG`. mekf `try_take`s it (fuses once per
fresh value — GPS-rate, not 8 kHz).

**COG-gate consts (in `main.rs` near the nav task):** `V_MIN = 2.0` m/s,
`FWD_STICK_MIN = 0.3` (normalised pitch-forward), `SIGMA_YAW_COG = 0.26`
rad (~15°).

---

## Error handling

- **Degenerate sphere fit** (`result()` None) → abort, keep prior offset,
  log error, do not persist.
- **Never reaches level-and-still after coverage** → offset is applied
  live, but anchoring waits; persist still saves the offset (anchor is
  recomputed at boot/next level window from the stored offset).
- **Flash write failure** → logged; the in-RAM offset already applied, so
  the current session keeps the cal even if the save failed.
- **Uncalibrated / blank flash** → `Config::default()` → zero offset →
  unchanged behaviour.

## Safety (non-negotiable)

- Calibration is **disarmed-only**; the cal path never touches
  DShot/mixer/arming. Arming mid-cal aborts cal and flies normally.
- The flash erase (H7 128 KB sector, up to ~1–2 s) stalls the single
  executor, but runs **only while disarmed, on the ground, motors off** —
  a brief all-task freeze there is harmless. Verify no watchdog trips
  across the write (feed/suspend it around the call if one exists).
- A failed cal or flash fault never persists garbage and never escalates
  beyond "uncalibrated".

---

## Testing

**Host (pure):**
- Calibrator: sphere-fit recovery, coverage completion, span gate,
  degenerate → None.
- MEKF 2a: offset no longer leaks into corrected direction.
- MEKF 2b: anchor → yaw matches known true heading (swept); `ψ_mag` sign
  guard.
- MEKF 2c: drifted yaw + COG → yaw corrects; level accel undisturbed;
  `H`-vector guard.
- Persist: A's non-default round-trip already covers offset + flag.

**Firmware / bench (acceptance gates):**
1. Spin-cal ritual: AUX4 (disarmed) → coverage % climbs while rotating →
   `CAL COMPLETE` → `CAL SAVED`; power-cycle → boot logs
   `mag_calibrated=true` + offset; yaw reads true heading vs a phone
   compass.
2. COG refine: forward pass → yaw converges toward COG; hover / sideways →
   yaw undisturbed.
3. Regression: uncalibrated (erased) board flies exactly as today.

---

## Out of scope (YAGNI)

- Soft-iron / ellipsoid calibration.
- Runtime-settable declination (compile-time const for now).
- Persisted append-log / wear-levelling (inherited from sub-project A).
- Auto-recalibration or temperature compensation.
