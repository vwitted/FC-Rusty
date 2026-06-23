# Mag Calibration + True-North Yaw Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix absolute yaw by calibrating the magnetometer hard-iron offset, anchoring the MEKF to true north (magnetic heading + declination), and fusing GPS course-over-ground as a gated yaw measurement.

**Architecture:** A pure host-tested `MagCalibrator` (sphere-fit offset + bin-coverage completion) and three pure MEKF methods (`set_hard_iron`, `anchor_heading`, `update_yaw_reference`), wired together in `main.rs`: an AUX4-triggered cal lifecycle in `mekf_task`, a forward-flight COG gate in `navigation_task`, and a new `persist_task` that writes the result to flash (sub-project A) while disarmed.

**Tech Stack:** Rust (`no_std` firmware + host `std` tests), `nalgebra` (vendored, `libm`), `embassy` tasks/signals, STM32H743.

**Spec:** `docs/superpowers/specs/2026-06-22-mag-cal-yaw-fix-design.md`
**Depends on:** sub-project A (`src/persist/`) — already implemented.

---

## Verified codebase facts (do not re-derive)

- `mekf_task` (`src/main.rs:886`): owns `AttitudeMekf`; loop waits on `RAW_IMU`, predicts, runs accel update on decimation, and consumes `MAG_DATA.try_take()` (mag block ~`:955`). `raw.accel_g()`, `raw.gyro_dps()` available; `mag.ut()` is the body field in µT.
- `navigation_task` (`src/main.rs:1783`): has `last_rc` (`rc_task::RC_CHANNELS`), `last_gps` (`GPS_DATA`), and `armed`. Channel map: `[0..3]` sticks (roll/pitch/throttle/yaw), `[4]` arm, `[5]` mode, `[6]` GPS home, `[7]` **free → AUX4**.
- `RcChannels::to_us(raw)->u16`, `to_normalised(raw)->f32` (−1..1), `to_unit(raw)->f32`. `GpsData::has_3d_fix()`, `.ground_speed_ms`, `.course_deg`.
- Signal pattern: `static NAME: Signal<CriticalSectionRawMutex, T> = Signal::new();` near `:194`. `persist::record::Config` is `Copy`.
- Sub-project A: `persist::flash::{driver, read, write}`, `persist::record::Config { mag_hard_iron_ut:[f32;3], declination_rad:f32, mag_calibrated:bool }`. Boot read already at the top of the flight `main` (the `let mut cfg_flash = …; let config = …` block).
- MEKF helpers already present in `attitude_mekf.rs`: `quat_mul`, `quat_normalize`, `quat_to_euler`, `euler_to_quat`, `r_bn_mul`, `r_bn_row_z`, `skew`, `set_mag_reference`, `initialize_mag_from_first`. `MekfParams` default. `self.p` is `SMatrix<f32,6,6>`; `self.q:[f32;4]`; `self.bias:Vector3<f32>`.

---

## File structure

- Create `src/control/mag_cal.rs` — pure calibrator + `CalCommand` + `DECLINATION_DEG`. Host-tested.
- Modify `src/lib.rs` + `src/main.rs` — declare `control::mag_cal`.
- Modify `src/attitude_mekf.rs` — add `hard_iron` field + `set_hard_iron`, `anchor_heading`, `update_yaw_reference`; subtract offset in `update_mag` and `initialize_mag_from_first`. Host-tested.
- Modify `src/main.rs` — signals, `persist_task`, `mekf_task` cal lifecycle, `navigation_task` COG gate.

---

### Task 1: `MagCalibrator` (pure, host-tested)

**Files:**
- Create: `src/control/mag_cal.rs`
- Modify: `src/lib.rs` (declare module), `src/main.rs` (declare module)

- [ ] **Step 1: Declare the module in both crates**

In `src/lib.rs`, inside `pub mod control { … }`, add `pub mod mag_cal;` (keep alphabetical-ish, after `mixer`). In `src/main.rs`, inside `mod control { … }` (around `:67`), add `pub mod mag_cal;`.

- [ ] **Step 2: Write the failing tests + types (stubbed fit)**

Create `src/control/mag_cal.rs`:

```rust
//! Magnetometer hard-iron calibration: online least-squares sphere fit
//! for the offset + bin-coverage completion. Pure no_std, host-tested.
//! Spec: docs/superpowers/specs/2026-06-22-mag-cal-yaw-fix-design.md

use nalgebra::{Matrix4, Vector4};

/// Magnetic declination at the flight location, degrees east-positive.
/// Single source of truth for the true-north anchor. Edit + rebuild to
/// change location.
pub const DECLINATION_DEG: f32 = 0.3;

/// Coverage bins required (of 24 = 8 azimuth × 3 elevation).
const COVERAGE_BINS_REQUIRED: u32 = 20;
/// Minimum total samples before completion.
const MIN_SAMPLES: u32 = 400;
/// Minimum per-axis field span (µT) — rejects a "didn't rotate" finish.
const MIN_SPAN_UT: f32 = 20.0;

/// Cal lifecycle command (nav task → mekf task).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalCommand {
    Start,
    Abort,
}

/// Online hard-iron sphere-fit + coverage tracker.
pub struct MagCalibrator {
    ata: Matrix4<f32>,
    atb: Vector4<f32>,
    min: [f32; 3],
    max: [f32; 3],
    coverage: u32, // 24-bit mask
    count: u32,
}

impl Default for MagCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MagCalibrator {
    pub fn new() -> Self {
        Self {
            ata: Matrix4::zeros(),
            atb: Vector4::zeros(),
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            coverage: 0,
            count: 0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one raw mag sample (any consistent unit; µT here).
    pub fn feed(&mut self, s: [f32; 3]) {
        let (x, y, z) = (s[0], s[1], s[2]);
        let a = Vector4::new(2.0 * x, 2.0 * y, 2.0 * z, 1.0);
        let b = x * x + y * y + z * z;
        self.ata += a * a.transpose();
        self.atb += a * b;
        for i in 0..3 {
            if s[i] < self.min[i] {
                self.min[i] = s[i];
            }
            if s[i] > self.max[i] {
                self.max[i] = s[i];
            }
        }
        let cx = 0.5 * (self.min[0] + self.max[0]);
        let cy = 0.5 * (self.min[1] + self.max[1]);
        let cz = 0.5 * (self.min[2] + self.max[2]);
        let (dx, dy, dz) = (x - cx, y - cy, z - cz);
        let horiz = libm::sqrtf(dx * dx + dy * dy);
        if horiz > 1.0e-3 || libm::fabsf(dz) > 1.0e-3 {
            use core::f32::consts::PI;
            let az = libm::atan2f(dy, dx); // −π..π
            let az_sector = (((az + PI) / (2.0 * PI) * 8.0) as u32).min(7);
            let elev = libm::atan2f(dz, horiz); // −π/2..π/2
            let el_band: u32 = if elev < -0.3 {
                0
            } else if elev > 0.3 {
                2
            } else {
                1
            };
            self.coverage |= 1 << (el_band * 8 + az_sector);
        }
        self.count = self.count.wrapping_add(1);
    }

    pub fn progress(&self) -> u8 {
        ((self.coverage.count_ones() * 100) / 24) as u8
    }

    fn span_ok(&self) -> bool {
        (0..3).all(|i| self.max[i] - self.min[i] >= MIN_SPAN_UT)
    }

    pub fn is_complete(&self) -> bool {
        self.coverage.count_ones() >= COVERAGE_BINS_REQUIRED
            && self.count >= MIN_SAMPLES
            && self.span_ok()
    }

    /// Fitted hard-iron offset (sphere centre). `None` if degenerate.
    pub fn result(&self) -> Option<[f32; 3]> {
        let inv = self.ata.try_inverse()?;
        let p = inv * self.atb;
        if p[0].is_finite() && p[1].is_finite() && p[2].is_finite() {
            Some([p[0], p[1], p[2]])
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate points on a sphere of radius `r` centred at `c`, swept
    /// over a grid of azimuth × elevation, and feed them to the cal.
    fn feed_sphere(cal: &mut MagCalibrator, c: [f32; 3], r: f32, n_az: u32, n_el: u32) {
        use core::f32::consts::PI;
        for ia in 0..n_az {
            let az = (ia as f32) / (n_az as f32) * 2.0 * PI;
            for ie in 0..=n_el {
                let el = -PI / 2.0 + (ie as f32) / (n_el as f32) * PI;
                let x = c[0] + r * libm::cosf(el) * libm::cosf(az);
                let y = c[1] + r * libm::cosf(el) * libm::sinf(az);
                let z = c[2] + r * libm::sinf(el);
                cal.feed([x, y, z]);
            }
        }
    }

    #[test]
    fn recovers_sphere_centre() {
        let mut cal = MagCalibrator::new();
        let c = [12.0, -5.0, 30.0];
        feed_sphere(&mut cal, c, 45.0, 36, 12);
        let off = cal.result().expect("fit");
        for i in 0..3 {
            assert!((off[i] - c[i]).abs() < 0.5, "axis {} got {}", i, off[i]);
        }
    }

    #[test]
    fn full_spread_completes() {
        let mut cal = MagCalibrator::new();
        feed_sphere(&mut cal, [0.0, 0.0, 0.0], 45.0, 36, 12);
        assert!(cal.is_complete());
        assert_eq!(cal.progress(), 100);
    }

    #[test]
    fn horizontal_ring_does_not_complete() {
        // A flat spin (elevation ~0) hits only the middle band — no
        // up/down coverage, so completion must not trigger.
        use core::f32::consts::PI;
        let mut cal = MagCalibrator::new();
        for ia in 0..120 {
            let az = (ia as f32) / 120.0 * 2.0 * PI;
            cal.feed([45.0 * libm::cosf(az), 45.0 * libm::sinf(az), 0.0]);
        }
        assert!(!cal.is_complete());
    }

    #[test]
    fn no_rotation_does_not_complete() {
        let mut cal = MagCalibrator::new();
        for _ in 0..1000 {
            cal.feed([40.0, 0.0, 0.0]);
        }
        assert!(!cal.is_complete()); // span gate fails
    }

    #[test]
    fn degenerate_input_returns_none() {
        let mut cal = MagCalibrator::new();
        cal.feed([1.0, 2.0, 3.0]); // single point → singular AᵀA
        assert!(cal.result().is_none());
    }
}
```

- [ ] **Step 3: Run tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu control::mag_cal`
Expected: PASS (5 tests). (The implementation is written alongside the tests here; to honour RED, temporarily change `MIN_SPAN_UT` to `1000.0` and confirm `full_spread_completes` FAILS, then restore `20.0`.)

- [ ] **Step 4: Confirm firmware still builds**

Run: `cargo build --release`
Expected: 0 errors. New `dead_code` warnings for `MagCalibrator`/`CalCommand` (not wired until Task 5–6) are acceptable.

- [ ] **Step 5: Commit**

```bash
git add src/control/mag_cal.rs src/lib.rs src/main.rs
git commit -m "mag-cal: MagCalibrator sphere-fit + bin-coverage (pure)"
```

---

### Task 2: MEKF hard-iron offset

**Files:**
- Modify: `src/attitude_mekf.rs`

- [ ] **Step 1: Write the failing test**

In `attitude_mekf.rs` `mod tests`, add (mirrors `mag_update_corrects_yaw_drift` but every reading carries a hard-iron offset that `set_hard_iron` must cancel):

```rust
    #[test]
    fn hard_iron_offset_does_not_bias_yaw() {
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.initialize_from_accel([0.0, 0.0, -1.0]);

        let offset = [8.0_f32, -4.0, 3.0];
        m.set_hard_iron(offset);

        // True field at boot heading; raw reading = true + offset.
        let true_body = [0.5_f32, 0.0, 0.866];
        let raw = [
            true_body[0] + offset[0],
            true_body[1] + offset[1],
            true_body[2] + offset[2],
        ];
        // Seed reference from the raw reading (cal subtracts internally).
        assert!(m.initialize_mag_from_first(raw));

        // Pretend the filter drifted +30° yaw; feed the same raw reading.
        let yaw_drift = 30.0_f32.to_radians();
        m.q = euler_to_quat(0.0, 0.0, yaw_drift);

        for _ in 0..200 {
            m.predict([0.0, 0.0, 0.0], 1.0 / 8000.0);
            m.update_accel([0.0, 0.0, -1.0]);
            m.update_mag(raw);
        }
        let y = m.euler()[2];
        assert!(y.abs() < 5.0_f32.to_radians(), "residual yaw = {} deg", y.to_degrees());
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu hard_iron_offset_does_not_bias_yaw`
Expected: FAIL to compile — `set_hard_iron` does not exist.

- [ ] **Step 3: Implement**

Add the field to the struct (after `mag_ref_set: bool,`):

```rust
    /// Hard-iron offset subtracted from every raw mag reading (same unit
    /// as the input, µT). Zero until calibrated.
    hard_iron: Vector3<f32>,
```

Initialise it in `new()` (in the `Self { … }` literal): `hard_iron: Vector3::zeros(),`.

Add the setter (anywhere in the `impl`):

```rust
    /// Set the hard-iron offset (sensor native frame, µT). Applied to all
    /// subsequent mag reads (`update_mag`, `initialize_mag_from_first`,
    /// `anchor_heading`).
    pub fn set_hard_iron(&mut self, offset: [f32; 3]) {
        self.hard_iron = Vector3::new(offset[0], offset[1], offset[2]);
    }
```

In `update_mag`, change the first line that builds `m` from `mag_body` to subtract the offset:

```rust
        let m = Vector3::new(
            mag_body[0] - self.hard_iron[0],
            mag_body[1] - self.hard_iron[1],
            mag_body[2] - self.hard_iron[2],
        );
```

In `initialize_mag_from_first`, change the `v_body` construction likewise:

```rust
        let v_body = Vector3::new(
            mag_body[0] - self.hard_iron[0],
            mag_body[1] - self.hard_iron[1],
            mag_body[2] - self.hard_iron[2],
        );
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu attitude_mekf`
Expected: PASS — the new test plus all existing MEKF tests (offset defaults to zero, so prior behaviour is unchanged).

- [ ] **Step 5: Commit**

```bash
git add src/attitude_mekf.rs
git commit -m "mekf: subtract hard-iron offset before mag fusion"
```

---

### Task 3: MEKF true-north anchor

**Files:**
- Modify: `src/attitude_mekf.rs`

- [ ] **Step 1: Write the failing test**

In `mod tests`:

```rust
    #[test]
    fn anchor_sets_true_heading() {
        // Nav field (NED): inclination 60°, declination 0 for the test.
        let f_nav = Vector3::new(0.5_f32, 0.0, 0.866);
        for &heading_deg in &[0.0_f32, 45.0, 90.0, 180.0, -120.0] {
            let psi = heading_deg.to_radians();
            // Body field a craft at this heading (level) would read.
            let q_true = euler_to_quat(0.0, 0.0, psi);
            let body = r_nb_mul(&q_true, &f_nav);

            let mut m = AttitudeMekf::new(MekfParams::default());
            m.initialize_from_accel([0.0, 0.0, -1.0]); // level, yaw 0
            assert!(m.anchor_heading([body[0], body[1], body[2]], 0.0));

            let y = m.euler()[2];
            let err = (y - psi).sin().abs(); // wrap-safe small-angle check
            assert!(err < 1.0e-2, "heading {} got {} deg", heading_deg, y.to_degrees());
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu anchor_sets_true_heading`
Expected: FAIL to compile — `anchor_heading` does not exist.

- [ ] **Step 3: Implement**

Add to the `impl AttitudeMekf` block:

```rust
    /// Anchor yaw to true north from a (raw) mag reading taken while
    /// level-and-still. Computes the tilt-compensated magnetic heading,
    /// adds declination, rebuilds the quaternion at that true heading, and
    /// reseeds `mag_ref` self-consistently with the measured field. The
    /// hard-iron offset is applied internally. Returns false on a
    /// zero-magnitude reading.
    pub fn anchor_heading(&mut self, mag_body: [f32; 3], declination_rad: f32) -> bool {
        let m = Vector3::new(
            mag_body[0] - self.hard_iron[0],
            mag_body[1] - self.hard_iron[1],
            mag_body[2] - self.hard_iron[2],
        );
        if m.norm() < 1e-6 {
            return false;
        }
        let e = quat_to_euler(&self.q);
        let (roll, pitch) = (e[0], e[1]);
        // Rotate the corrected body field into a yaw-zeroed nav frame.
        let q0 = euler_to_quat(roll, pitch, 0.0);
        let m0 = r_bn_mul(&q0, &m);
        // Magnetic heading (sign matches NED + quat_to_euler yaw).
        let psi_mag = -libm::atan2f(m0[1], m0[0]);
        let psi_true = psi_mag + declination_rad;
        // Rebuild q at the true heading, reseed the reference.
        self.q = euler_to_quat(roll, pitch, psi_true);
        let v_nav = r_bn_mul(&self.q, &m);
        self.set_mag_reference([v_nav[0], v_nav[1], v_nav[2]])
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu attitude_mekf`
Expected: PASS (existing `r_nb_mul` helper is used by the test; it already exists in the module).

- [ ] **Step 5: Commit**

```bash
git add src/attitude_mekf.rs
git commit -m "mekf: anchor_heading — true-north yaw from level mag + declination"
```

---

### Task 4: MEKF COG scalar yaw update

**Files:**
- Modify: `src/attitude_mekf.rs`

- [ ] **Step 1: Write the failing test**

In `mod tests`:

```rust
    #[test]
    fn yaw_reference_corrects_drift() {
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.initialize_from_accel([0.0, 0.0, -1.0]);
        // Drift +30° yaw the filter doesn't know about.
        m.q = euler_to_quat(0.0, 0.0, 30.0_f32.to_radians());

        for _ in 0..50 {
            m.predict([0.0, 0.0, 0.0], 1.0 / 8000.0);
            m.update_accel([0.0, 0.0, -1.0]);
            m.update_yaw_reference(0.0, 0.26); // ~15° sigma, measured truth = 0
        }
        let [r, p, y] = m.euler();
        assert!(y.abs() < 10.0_f32.to_radians(), "residual yaw = {} deg", y.to_degrees());
        // Roll/pitch must not be disturbed by the yaw update.
        assert!(r.abs() < 2.0_f32.to_radians(), "roll = {} deg", r.to_degrees());
        assert!(p.abs() < 2.0_f32.to_radians(), "pitch = {} deg", p.to_degrees());
    }

    #[test]
    fn yaw_reference_wraps_innovation() {
        let mut m = AttitudeMekf::new(MekfParams::default());
        m.initialize_from_accel([0.0, 0.0, -1.0]);
        m.q = euler_to_quat(0.0, 0.0, 179.0_f32.to_radians());
        // Measure −179°: true error is +2°, not −358°.
        for _ in 0..50 {
            m.predict([0.0, 0.0, 0.0], 1.0 / 8000.0);
            m.update_yaw_reference((-179.0_f32).to_radians(), 0.26);
        }
        let y = m.euler()[2].to_degrees();
        assert!(y > 178.0 || y < -178.0, "yaw drifted the long way: {}", y);
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu yaw_reference`
Expected: FAIL to compile — `update_yaw_reference` does not exist.

- [ ] **Step 3: Implement**

Add to the `impl AttitudeMekf` block:

```rust
    /// Scalar yaw measurement update (e.g. GPS course-over-ground used as
    /// a heading reference). `yaw_meas` and the internal yaw are both
    /// "angle from north, clockwise positive"; the innovation is wrapped
    /// to [−π, π]. `sigma_yaw` (rad) should be generous — COG ≈ heading
    /// only approximately. Returns false if the innovation covariance is
    /// non-positive.
    pub fn update_yaw_reference(&mut self, yaw_meas: f32, sigma_yaw: f32) -> bool {
        use core::f32::consts::PI;
        let yaw_est = quat_to_euler(&self.q)[2];
        let mut y = yaw_meas - yaw_est;
        while y > PI {
            y -= 2.0 * PI;
        }
        while y < -PI {
            y += 2.0 * PI;
        }

        // H = [hᵀ | 0], h = body projection of world-down = r_bn_row_z(q).
        let h = r_bn_row_z(&self.q);
        let p_tt = self.p.fixed_view::<3, 3>(0, 0).into_owned();
        let p_tb = self.p.fixed_view::<3, 3>(0, 3).into_owned();

        let s = h.dot(&(p_tt * h)) + sigma_yaw * sigma_yaw;
        if !(s > 0.0) {
            return false;
        }
        // K = P Hᵀ / s  (6×1): top = P_tt·h, bottom = P_btᵀ·h.
        let k_top = (p_tt * h) / s;
        let k_bot = (p_tb.transpose() * h) / s;

        let d_theta = k_top * y;
        let d_bias = k_bot * y;

        let dq = [1.0, d_theta[0] * 0.5, d_theta[1] * 0.5, d_theta[2] * 0.5];
        self.q = quat_mul(self.q, dq);
        quat_normalize(&mut self.q);
        self.bias += d_bias;

        // P ← (I − K H) P, KH = [[k_top·hᵀ, 0], [k_bot·hᵀ, 0]].
        let kh_tt = k_top * h.transpose();
        let kh_bt = k_bot * h.transpose();
        let mut kh = SMatrix::<f32, 6, 6>::zeros();
        for i in 0..3 {
            for j in 0..3 {
                kh[(i, j)] = kh_tt[(i, j)];
                kh[(3 + i, j)] = kh_bt[(i, j)];
            }
        }
        let i6 = SMatrix::<f32, 6, 6>::identity();
        self.p = (i6 - kh) * self.p;
        let pt = self.p.transpose();
        self.p = (self.p + pt) * 0.5;
        true
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu attitude_mekf`
Expected: PASS (all MEKF tests).

- [ ] **Step 5: Commit**

```bash
git add src/attitude_mekf.rs
git commit -m "mekf: update_yaw_reference — scalar COG yaw measurement"
```

---

### Task 5: Signals + `persist_task` + boot wiring

**Files:**
- Modify: `src/main.rs`

- [ ] **Step 1: Add imports + signal statics**

In `src/main.rs`, extend the `control::mag_cal` import (add near the other `use control::…` lines around `:90`):

```rust
use control::mag_cal::{CalCommand, MagCalibrator, DECLINATION_DEG};
```

Near the other signal statics (after `ARM_LATCH`, ~`:319`), add:

```rust
/// Magnetometer-cal lifecycle: navigation task → mekf task.
static CAL_CONTROL: Signal<CriticalSectionRawMutex, CalCommand> = Signal::new();
/// Completed cal to persist (disarmed): mekf task → persist task.
static CAL_SAVE: Signal<CriticalSectionRawMutex, persist::record::Config> = Signal::new();
/// Trusted true heading from GPS COG (rad): navigation task → mekf task.
static YAW_COG: Signal<CriticalSectionRawMutex, f32> = Signal::new();
/// Boot-loaded calibration: main → mekf task.
static STORED_CAL: Signal<CriticalSectionRawMutex, persist::record::Config> = Signal::new();
```

- [ ] **Step 2: Publish the boot config and hand the flash handle to a task**

In the flight `main`, find the boot-read block (`let mut cfg_flash = persist::flash::driver(p.FLASH); let config = …; … let _ = &config;`). Replace the trailing `let _ = &config;` line with:

```rust
    // Hand the boot calibration to the MEKF and the flash handle to the
    // persist task (which writes future cals while disarmed).
    STORED_CAL.signal(config);
    spawner.spawn(persist_task(cfg_flash)).unwrap();
```

(The `#[cfg(feature = "persist-selftest")]` block, if present, still runs before this and only borrows `cfg_flash`; leave it in place.)

- [ ] **Step 3: Add the persist task**

Near the other `#[embassy_executor::task]` fns (e.g. after `gps_task`), add:

```rust
// Owns the flash handle; writes a completed magnetometer calibration to
// the persist store. Triggered only by cal completion, which is
// disarmed-only — so the multi-second sector erase happens on the ground.
#[embassy_executor::task]
async fn persist_task(mut flash: embassy_stm32::flash::Flash<'static, embassy_stm32::flash::Blocking>) {
    loop {
        let cfg = CAL_SAVE.wait().await;
        match persist::flash::write(&mut flash, &cfg) {
            Ok(()) => defmt::info!("persist: CAL SAVED to flash"),
            Err(e) => defmt::error!("persist: cal save failed {:?}", e),
        }
    }
}
```

- [ ] **Step 4: Build**

Run: `cargo build --release`
Expected: 0 errors. `write` is now used (persist `write` dead-code warning clears). `MagCalibrator`/`update_yaw_reference`/`anchor_heading`/`CalCommand` may still warn until Task 6–7.

- [ ] **Step 5: Commit**

```bash
git add src/main.rs
git commit -m "mag-cal: cal signals + persist_task + boot cal handoff"
```

---

### Task 6: `mekf_task` cal lifecycle + anchor + COG fusion

**Files:**
- Modify: `src/main.rs` (`mekf_task`, ~`:886`)

- [ ] **Step 1: Add a level-and-still helper**

Just above `async fn mekf_task()`, add:

```rust
// True when the board is near-stationary and ~1 g — a safe moment to
// anchor yaw (attitude/roll/pitch are trustworthy and the field is steady).
fn mag_anchor_ready(raw: &RawImu) -> bool {
    let a = raw.accel_g();
    let amag = libm::sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    let g = raw.gyro_dps();
    let gmag = libm::sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    (amag - 1.0).abs() < 0.1 && gmag < 5.0
}
```

- [ ] **Step 2: Add cal state before the loop**

In `mekf_task`, after `let mut last_mag_ut: [f32; 3] = [0.0; 3];` (~`:917`), add:

```rust
    const SIGMA_YAW_COG: f32 = 0.26; // ~15°, generous: COG ≈ heading only
    let mut calibrator = MagCalibrator::new();
    let mut cal_active = false;
    let mut anchor_pending = false;
    let mut last_cal_log = Instant::now();
```

- [ ] **Step 3: Handle boot cal + cal control at the top of the loop**

Inside the loop, right after `let raw = RAW_IMU.wait().await;` (~`:920`), add:

```rust
        // Apply a boot-loaded calibration once it arrives.
        if let Some(cfg) = STORED_CAL.try_take() {
            if cfg.mag_calibrated {
                mekf.set_hard_iron(cfg.mag_hard_iron_ut);
                anchor_pending = true;
                defmt::info!(
                    "MEKF loaded stored cal: offset=[{=f32},{=f32},{=f32}]",
                    cfg.mag_hard_iron_ut[0], cfg.mag_hard_iron_ut[1], cfg.mag_hard_iron_ut[2],
                );
            }
        }
        // Cal start/abort from the navigation task.
        if let Some(cmd) = CAL_CONTROL.try_take() {
            match cmd {
                CalCommand::Start => {
                    calibrator.reset();
                    cal_active = true;
                    defmt::info!("MEKF cal: started — rotate the craft through all axes");
                }
                CalCommand::Abort => {
                    if cal_active {
                        defmt::info!("MEKF cal: aborted");
                    }
                    cal_active = false;
                }
            }
        }
        // Fuse a fresh trusted COG heading, if any.
        if let Some(yaw_cog) = YAW_COG.try_take() {
            mekf.update_yaw_reference(yaw_cog, SIGMA_YAW_COG);
        }
```

- [ ] **Step 4: Branch the mag block on cal state**

Replace the existing mag block (the `if let Some(mag) = MAG_DATA.try_take() { … }` around `:955`) with:

```rust
        if let Some(mag) = MAG_DATA.try_take() {
            let ut = mag.ut();
            last_mag_ut = ut;
            if cal_active {
                // Collect raw samples; don't fuse mag while rotating.
                calibrator.feed(ut);
                if last_cal_log.elapsed().as_millis() >= 500 {
                    defmt::info!("MEKF cal: coverage {}%", calibrator.progress());
                    last_cal_log = Instant::now();
                }
                if calibrator.is_complete() {
                    match calibrator.result() {
                        Some(off) => {
                            mekf.set_hard_iron(off);
                            cal_active = false;
                            anchor_pending = true;
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
        }
```

- [ ] **Step 5: Build + regression-test**

Run: `cargo build --release`
Expected: 0 errors. `MagCalibrator`/`CalCommand`/MEKF-method dead-code warnings clear (now all used). `CAL_CONTROL`/`YAW_COG` are signalled in Task 7 — until then they're written-by-task-6-read-only; no warning (statics aren't dead-code-checked the same way), but if a warning appears it clears in Task 7.

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu attitude_mekf control::mag_cal`
Expected: PASS (unchanged — pure modules).

- [ ] **Step 6: Commit**

```bash
git add src/main.rs
git commit -m "mekf-task: AUX cal lifecycle, true-north anchor, COG yaw fusion"
```

---

### Task 7: `navigation_task` AUX4 trigger + COG-trust gate

**Files:**
- Modify: `src/main.rs` (`navigation_task`, ~`:1783`)

- [ ] **Step 1: Add gate constants + edge-tracking state**

Inside `navigation_task`, near the top (with the other `let mut` state before the loop), add:

```rust
    // GPS-COG yaw gating: only trust course when genuinely flying forward.
    const V_MIN_COG: f32 = 2.0; // m/s
    const FWD_STICK_MIN: f32 = 0.3; // normalised pitch-forward
    let mut cal_sw_prev = false;
    let mut armed_prev_cal = false;
```

- [ ] **Step 2: Drive the cal switch + COG gate inside the loop**

In the navigation loop, after `armed` is computed and `last_rc` / `last_gps` are refreshed (after the arming/mode block, before the command is sent — a safe spot is just after the flight-mode selection around `:1957`), add:

```rust
        // ---- Magnetometer cal trigger (AUX4 = channel index 7) ----
        // Disarmed-only. Rising edge starts; falling edge or a fresh arm
        // aborts. The cal itself runs in the MEKF task.
        let cal_sw = last_rc.channels[7] > 1500;
        if cal_sw && !cal_sw_prev && !armed {
            CAL_CONTROL.signal(CalCommand::Start);
        } else if (!cal_sw && cal_sw_prev) || (cal_sw && armed && !armed_prev_cal) {
            CAL_CONTROL.signal(CalCommand::Abort);
        }
        cal_sw_prev = cal_sw;
        armed_prev_cal = armed;

        // ---- GPS-COG yaw reference (gated) ----
        // COG equals heading only in deliberate forward flight, so require
        // armed + good 3D fix + above V_MIN + forward pitch stick. The
        // MEKF fuses it as a generous-sigma scalar yaw update.
        // NOTE: confirm the forward-stick sign on the bench (channels[1]
        // forward should be positive here); flip if your TX is reversed.
        let fwd_stick = RcChannels::to_normalised(last_rc.channels[1]);
        if armed
            && last_gps.has_3d_fix()
            && last_gps.ground_speed_ms > V_MIN_COG
            && fwd_stick > FWD_STICK_MIN
        {
            YAW_COG.signal(last_gps.course_deg.to_radians());
        }
```

- [ ] **Step 2a: Verify `last_gps` is in scope**

Run: `cd "/home/phil/Documents/claude code/FC-Rusty" && grep -n "let last_gps\|let mut last_gps\|last_gps =" src/main.rs`
Expected: a binding inside `navigation_task` (the loop already does `if let Some(gps) = GPS_DATA.try_take() { last_gps = gps; }`). If the variable is named differently, use that name.

- [ ] **Step 3: Build**

Run: `cargo build --release`
Expected: 0 errors. `CAL_CONTROL`/`YAW_COG` now both signalled and consumed. Warning count at or below the pre-task-1 baseline plus any intentional cal-related items; no new dead-code from this feature.

- [ ] **Step 4: Full host suite**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`
Expected: PASS — 145 (sub-project A) + 5 (mag_cal) + 3 (MEKF: hard-iron, anchor, two yaw-ref) = **153** (count may differ by the exact number of new MEKF tests; all green).

- [ ] **Step 5: Commit**

```bash
git add src/main.rs
git commit -m "nav-task: AUX4 cal trigger + forward-flight GPS-COG yaw gate"
```

---

### Bench acceptance (user, hardware)

1. **Spin-cal ritual** (disarmed): flip AUX4 high → console logs `cal: started`, then `coverage N%` climbing as you rotate the craft through all axes → `COMPLETE` → hold level → `anchored to true north` → `CAL SAVED`. Power-cycle → boot logs `mag_calibrated=true` + offset and `MEKF loaded stored cal`. Check yaw reads true heading vs a phone compass.
2. **COG refine**: arm, fly a forward pass above 2 m/s → yaw converges toward GPS course; hover / sideways translation → yaw undisturbed. (If yaw runs *away* on forward flight, the forward-stick sign in Task 7 Step 2 is reversed — flip the comparison.)
3. **Regression**: an uncalibrated board (erased CONFIG sector) flies exactly as before — relative boot-heading mag behaviour, no anchor.

---

## Self-review notes

- **Spec coverage:** calibrator (sphere-fit + coverage) → Task 1; hard-iron apply → Task 2; anchor → Task 3; COG scalar update → Task 4; persist save + boot load → Task 5; cal lifecycle + anchor-on-level + COG fuse → Task 6; AUX4 trigger + COG gate → Task 7. Declination const → Task 1. Safety (disarmed-only, no motor path, degenerate→keep prior) → Tasks 6–7.
- **Type consistency:** `MagCalibrator` (`new`/`reset`/`feed`/`progress`/`is_complete`/`result`), `CalCommand::{Start,Abort}`, `set_hard_iron`/`anchor_heading`/`update_yaw_reference`, `persist::record::Config` fields, and the four signals are used identically across tasks.
- **Sign risks flagged for bench:** `ψ_mag` (guarded by `anchor_sets_true_heading`), COG innovation wrap (guarded by `yaw_reference_wraps_innovation`), and the forward-stick sign (explicitly called out in Task 7 + bench step 2).
