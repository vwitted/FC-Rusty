# `persist` Flash Config Store — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a versioned, CRC-checked flash config store (sub-project A) so a small `Config` struct survives power cycles, as the foundation for the magnetometer-calibration / yaw fix (sub-project B).

**Architecture:** Two units. A pure, host-tested `record` module owns the on-flash byte layout (magic + version + len + payload + CRC-32). A firmware-only `flash` module wraps `embassy_stm32::flash::Flash` over a 128 KB sector reserved in `memory.x` at `0x081E0000` (bank 2), exposing `read()`/`write()`. Writes happen only while disarmed; a flash fault degrades to "uncalibrated", never a control hazard.

**Tech Stack:** Rust (`no_std` firmware + host `std` tests), `embassy-stm32` 0.4 blocking Flash HAL, STM32H743 (128 KB erase, 32-byte write word).

**Spec:** `docs/superpowers/specs/2026-06-22-persist-flash-config-design.md`

---

## Verified toolchain facts (do not re-derive)

- `embassy_stm32::flash`: `Flash::new_blocking(p: Peri<'d, FLASH>) -> Flash<'d, Blocking>`. `Blocking` is a marker exported from `embassy_stm32::flash`.
- Methods (offsets are **flash-relative**, not absolute):
  `blocking_read(&mut self, offset: u32, &mut [u8]) -> Result<(), Error>`,
  `blocking_write(&mut self, offset: u32, &[u8]) -> Result<(), Error>`,
  `blocking_erase(&mut self, from: u32, to: u32) -> Result<(), Error>` (erases sectors covering `[from, to)`).
- H743 generated consts: `FLASH_BASE = 0x0800_0000`, `FLASH_SIZE = 0x20_0000` (2 MB), `WRITE_SIZE = 32`, `MAX_ERASE_SIZE = 131072` (128 KB). embassy validates against the **full 2 MB**, so writing at offset `0x1E_0000` is allowed regardless of the shrunk linker `FLASH` region — exactly what we want.
- CONFIG sector: base `0x081E_0000` → flash offset `0x1E_0000`; erase with `blocking_erase(0x1E_0000, 0x20_0000)`.
- Module pattern in this crate: `lib.rs` and `main.rs` are separate crates that each declare modules over the same files (e.g. `src/control/arm_origin.rs`). Pure modules go in both; firmware-only (`embassy`) modules go only in `main.rs`'s tree (like `drivers::dshot_hw`).

---

## File structure

- Create `src/persist/record.rs` — pure: `Config`, constants, CRC-32, `encode`/`decode`. Compiled in both crates; host-tested.
- Create `src/persist/flash.rs` — firmware-only: region constants, `PersistError`, `read`/`write`. Compiled only in the binary.
- Modify `src/lib.rs` — declare `pub mod persist { pub mod record; }`.
- Modify `src/main.rs` — declare `mod persist { pub mod record; pub mod flash; }`; add the bench self-test (Task 5).
- Modify `memory.x` — shrink `FLASH` to 1920K, add `CONFIG` region.
- Modify `Cargo.toml` — add a `persist-selftest` feature (Task 5).

---

### Task 1: CRC-32 (pure, host-tested)

**Files:**
- Create: `src/persist/record.rs`
- Test: same file, `#[cfg(test)] mod tests`

- [ ] **Step 1: Write the failing test**

Create `src/persist/record.rs` with only:

```rust
//! On-flash config record: layout, CRC, (de)serialisation. Pure no_std,
//! host-tested. The firmware flash wrapper lives in `flash.rs`.

/// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF).
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_known_vector() {
        // Standard CRC-32 check value for the ASCII string "123456789".
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu crc32_known_vector`
Expected: FAIL to **compile** — `persist::record` is not declared yet (`lib.rs` doesn't know the module). That compile failure is the RED state.

- [ ] **Step 3: Declare the module so the test can run**

In `src/lib.rs`, immediately after the `motor_test` block (around line 18, before `pub mod control {`), add:

```rust
// Versioned flash config store. `record` is pure (host-tested); the
// firmware flash wrapper lives in main.rs's module tree (needs embassy).
pub mod persist {
    pub mod record;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu crc32_known_vector`
Expected: PASS (1 test).

- [ ] **Step 5: Commit**

```bash
git add src/persist/record.rs src/lib.rs
git commit -m "persist: CRC-32 for the flash config record"
```

---

### Task 2: `Config` + `encode`/`decode` (pure, host-tested)

**Files:**
- Modify: `src/persist/record.rs`
- Test: same file

- [ ] **Step 1: Write the failing tests**

In `src/persist/record.rs`, add above the `#[cfg(test)]` module (keep `crc32` as-is):

```rust
/// Identifies a valid record. ASCII "FCR1".
pub const MAGIC: u32 = 0x4643_5231;
/// Payload schema version. Bump when `Config` grows.
pub const VERSION: u16 = 1;
/// Serialised payload length: 3*f32 + f32 + bool = 17 bytes.
pub const PAYLOAD_LEN: usize = 17;
/// Whole record, padded to the H7 32-byte flash word.
pub const RECORD_LEN: usize = 32;

/// Persisted configuration (v1). Defaults are the safe "uncalibrated"
/// state — identical in effect to having no stored record at all.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Magnetometer hard-iron offset, sensor native frame, µT.
    pub mag_hard_iron_ut: [f32; 3],
    /// Magnetic declination, radians, east-positive. 0.0 = none.
    pub declination_rad: f32,
    /// True once a real calibration has been written (vs. defaults).
    pub mag_calibrated: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mag_hard_iron_ut: [0.0; 3],
            declination_rad: 0.0,
            mag_calibrated: false,
        }
    }
}

/// Serialise + CRC into a flash-word-aligned record.
///
/// Layout: magic[0..4] | version[4..6] | len[6..8] | payload[8..25] |
/// zero-pad[25..28] | crc32[28..32]. CRC covers bytes [0..28].
pub fn encode(cfg: &Config) -> [u8; RECORD_LEN] {
    let mut b = [0u8; RECORD_LEN];
    b[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    b[4..6].copy_from_slice(&VERSION.to_le_bytes());
    b[6..8].copy_from_slice(&(PAYLOAD_LEN as u16).to_le_bytes());
    b[8..12].copy_from_slice(&cfg.mag_hard_iron_ut[0].to_le_bytes());
    b[12..16].copy_from_slice(&cfg.mag_hard_iron_ut[1].to_le_bytes());
    b[16..20].copy_from_slice(&cfg.mag_hard_iron_ut[2].to_le_bytes());
    b[20..24].copy_from_slice(&cfg.declination_rad.to_le_bytes());
    b[24] = cfg.mag_calibrated as u8;
    // [25..28] stay zero (deterministic padding, covered by the CRC).
    let crc = crc32(&b[0..28]);
    b[28..32].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Validate and parse a record. Returns `None` on any mismatch (caller
/// uses `Config::default()`), including a blank (all-0xFF) sector.
pub fn decode(bytes: &[u8]) -> Option<Config> {
    if bytes.len() < RECORD_LEN {
        return None;
    }
    let b = &bytes[0..RECORD_LEN];
    if u32::from_le_bytes(b[0..4].try_into().ok()?) != MAGIC {
        return None;
    }
    if u16::from_le_bytes(b[4..6].try_into().ok()?) != VERSION {
        return None;
    }
    if u16::from_le_bytes(b[6..8].try_into().ok()?) as usize != PAYLOAD_LEN {
        return None;
    }
    let crc = u32::from_le_bytes(b[28..32].try_into().ok()?);
    if crc != crc32(&b[0..28]) {
        return None;
    }
    Some(Config {
        mag_hard_iron_ut: [
            f32::from_le_bytes(b[8..12].try_into().ok()?),
            f32::from_le_bytes(b[12..16].try_into().ok()?),
            f32::from_le_bytes(b[16..20].try_into().ok()?),
        ],
        declination_rad: f32::from_le_bytes(b[20..24].try_into().ok()?),
        mag_calibrated: b[24] != 0,
    })
}
```

Then add these tests inside the existing `mod tests`:

```rust
    #[test]
    fn record_len_is_flash_word_aligned() {
        assert_eq!(RECORD_LEN % 32, 0);
    }

    #[test]
    fn default_is_uncalibrated() {
        let c = Config::default();
        assert!(!c.mag_calibrated);
        assert_eq!(c.mag_hard_iron_ut, [0.0; 3]);
        assert_eq!(c.declination_rad, 0.0);
    }

    #[test]
    fn encode_decode_round_trips() {
        let c = Config {
            mag_hard_iron_ut: [12.5, -3.25, 40.0],
            declination_rad: 0.0052,
            mag_calibrated: true,
        };
        let bytes = encode(&c);
        assert_eq!(decode(&bytes), Some(c));
    }

    #[test]
    fn decode_rejects_bad_magic() {
        let mut bytes = encode(&Config::default());
        bytes[0] ^= 0xFF;
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_wrong_version() {
        let mut bytes = encode(&Config::default());
        bytes[4] = 0xFE; // corrupt version field
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_bad_crc() {
        let mut bytes = encode(&Config::default());
        bytes[24] ^= 0x01; // flip a payload bit; CRC no longer matches
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_blank_sector() {
        // Erased H7 flash reads as all 0xFF.
        let bytes = [0xFFu8; RECORD_LEN];
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_truncated_input() {
        let bytes = encode(&Config::default());
        assert_eq!(decode(&bytes[0..RECORD_LEN - 1]), None);
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu persist::record`
Expected: the seven new tests are present; they PASS already because Step 1 added the implementation alongside them. To honour RED→GREEN, temporarily stub `encode` to `[0u8; RECORD_LEN]` and `decode` to `None`, run, and confirm `encode_decode_round_trips`, `decode_rejects_bad_magic`, etc. FAIL — then restore the real bodies above.

- [ ] **Step 3: Restore the real implementations** (already written above).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu persist::record`
Expected: PASS (8 tests total in `persist::record`).

- [ ] **Step 5: Commit**

```bash
git add src/persist/record.rs
git commit -m "persist: Config struct + versioned encode/decode"
```

---

### Task 3: Reserve the flash sector + wire the firmware module

**Files:**
- Modify: `memory.x`
- Modify: `src/main.rs:53-83` (module declarations region)
- Create: `src/persist/flash.rs`

- [ ] **Step 1: Reserve the CONFIG sector in `memory.x`**

Replace the `MEMORY { ... }` block in `memory.x` with (only `FLASH` length changes and `CONFIG` is added; keep the existing comment header):

```
MEMORY
{
  /* STM32H743VI — 2048 KB flash, 1024 KB RAM. Last 128 KB sector
   * (bank 2, 0x081E0000) is reserved for the persist config store;
   * FLASH is shrunk to 1920K so the firmware image can't overlap it. */
  FLASH  (rx) : ORIGIN = 0x08000000, LENGTH = 1920K
  CONFIG (r)  : ORIGIN = 0x081E0000, LENGTH = 128K
  RAM    (rwx): ORIGIN = 0x24000000, LENGTH = 512K
  DTCM   (rwx): ORIGIN = 0x20000000, LENGTH = 128K
  ITCM   (rx) : ORIGIN = 0x00000000, LENGTH = 64K
}
```

- [ ] **Step 2: Create the firmware flash wrapper**

Create `src/persist/flash.rs`:

```rust
//! Firmware-only flash access for the config record (sub-project A).
//!
//! Wraps the embassy blocking Flash HAL over the reserved CONFIG sector.
//! `read()` runs once at boot; `write()` runs only while disarmed
//! (erase/write must never happen in the control path). A flash fault
//! degrades to "uncalibrated", never a control hazard.

use embassy_stm32::Peri;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::peripherals::FLASH;

use super::record::{self, Config, RECORD_LEN};

/// Flash-relative offset of the reserved CONFIG sector
/// (0x081E0000 − FLASH_BASE 0x08000000).
pub const CONFIG_OFFSET: u32 = 0x1E_0000;
/// H743 sector size (erase granularity).
pub const CONFIG_SECTOR_LEN: u32 = 128 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistError {
    Erase,
    Write,
}

/// Construct the blocking flash driver. Call once; the caller owns the
/// `FLASH` peripheral.
pub fn driver(flash: Peri<'static, FLASH>) -> Flash<'static, Blocking> {
    Flash::new_blocking(flash)
}

/// Read and validate the stored config. `None` ⇒ use `Config::default()`.
pub fn read(flash: &mut Flash<'static, Blocking>) -> Option<Config> {
    let mut buf = [0u8; RECORD_LEN];
    flash.blocking_read(CONFIG_OFFSET, &mut buf).ok()?;
    record::decode(&buf)
}

/// Erase the sector and write one record. Disarmed-only (caller enforces).
pub fn write(flash: &mut Flash<'static, Blocking>, cfg: &Config) -> Result<(), PersistError> {
    let bytes = record::encode(cfg);
    flash
        .blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + CONFIG_SECTOR_LEN)
        .map_err(|_| PersistError::Erase)?;
    flash
        .blocking_write(CONFIG_OFFSET, &bytes)
        .map_err(|_| PersistError::Write)?;
    Ok(())
}
```

- [ ] **Step 3: Declare the firmware module in `main.rs`**

In `src/main.rs`, immediately after the `motor_test` block (after line 83), add:

```rust
mod persist {
    pub mod record;
    pub mod flash;
}
```

- [ ] **Step 4: Verify both builds**

Run: `cargo build --release`
Expected: 0 errors. Warning count unchanged from baseline (92) — `persist::flash` is declared but not yet called, so expect possibly a couple of `dead_code` warnings on `read`/`write`/`driver`; that is acceptable here and resolved by Task 5's self-test. If the count is otherwise unchanged, proceed.

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu persist::record`
Expected: PASS (8 tests) — the host build does not see `flash.rs` (it's only in `main.rs`'s tree), so `memory.x`/embassy changes don't affect it.

- [ ] **Step 5: Commit**

```bash
git add memory.x src/persist/flash.rs src/main.rs
git commit -m "persist: reserve CONFIG sector + firmware flash read/write wrapper"
```

---

### Task 4: Boot-time read into a published default (wire-in, no behaviour change)

This proves `read()` is callable from real boot and that an uncalibrated board behaves exactly as before. No control behaviour changes.

**Files:**
- Modify: `src/main.rs` (the flight `main`, the `#[cfg(not(feature = "motor-test"))]` entry point)

- [ ] **Step 1: Read config at boot and log it**

Find the flight `main` (the `embassy_stm32::init` call in the non-motor-test entry point). Immediately after `let p = embassy_stm32::init(board_config());`, add:

```rust
    // Load persisted config (sub-project A). None ⇒ uncalibrated defaults,
    // identical to prior behaviour. Read once here, before control loops.
    let mut cfg_flash = persist::flash::driver(p.FLASH);
    let config = persist::flash::read(&mut cfg_flash).unwrap_or_default();
    defmt::info!(
        "persist: mag_calibrated={} decl={=f32}rad hard_iron=[{=f32},{=f32},{=f32}]",
        config.mag_calibrated,
        config.declination_rad,
        config.mag_hard_iron_ut[0],
        config.mag_hard_iron_ut[1],
        config.mag_hard_iron_ut[2],
    );
    let _ = &config; // consumed by sub-project B; bound now to prove the boot read
```

Note: `p.FLASH` is moved here. Confirm nothing else in `main` consumes `p.FLASH` (grep below). If the logger is initialised after `init`, place this block after `logger::init_usart6()` so the line is visible.

- [ ] **Step 2: Verify nothing else claims `p.FLASH`**

Run: `cd "/home/phil/Documents/claude code/FC-Rusty" && grep -n "p.FLASH" src/main.rs`
Expected: exactly one match (the line you just added).

- [ ] **Step 3: Build**

Run: `cargo build --release`
Expected: 0 errors. `read`/`driver` `dead_code` warnings from Task 3 are now gone; `write` may still warn until Task 5. Net warnings ≈ baseline (92) ± the single remaining `write` dead_code.

- [ ] **Step 4: Commit**

```bash
git add src/main.rs
git commit -m "persist: read config at boot into uncalibrated default"
```

---

### Task 5: Bench self-test (the hardware acceptance gate)

A feature-gated, two-boot self-test that proves erase/write/read across a real power cycle. Throwaway scaffolding kept behind a feature so it never ships in a flight build.

**Files:**
- Modify: `Cargo.toml` (add `persist-selftest` feature)
- Modify: `src/main.rs` (gated self-test block)

- [ ] **Step 1: Add the feature**

In `Cargo.toml`, after the `motor-test = []` line, add:

```toml
# Bench-only self-test for the persist flash store (sub-project A).
# On boot: logs the pre-write read, then writes a known marker Config.
# Power-cycle and re-flash-free reboot should then read the marker back,
# proving persistence. Never enable for a flight build.
persist-selftest = []
```

- [ ] **Step 2: Add the gated self-test after the boot read**

In `src/main.rs`, immediately after the `let _ = &config;` line from Task 4, add:

```rust
    #[cfg(feature = "persist-selftest")]
    {
        // Two-boot protocol:
        //   Boot 1 (blank sector): `config` read above is the default
        //   (mag_calibrated=false). We then write the marker below.
        //   Boot 2 (after power-cycle): the read above returns the marker
        //   (mag_calibrated=true, decl=0.1234), proving persistence.
        if config.mag_calibrated && (config.declination_rad - 0.1234).abs() < 1e-4 {
            defmt::info!("persist-selftest: PASS — marker survived reboot");
        } else {
            defmt::warn!("persist-selftest: no marker yet — writing it now, power-cycle to verify");
            let marker = persist::record::Config {
                mag_hard_iron_ut: [1.0, 2.0, 3.0],
                declination_rad: 0.1234,
                mag_calibrated: true,
            };
            match persist::flash::write(&mut cfg_flash, &marker) {
                Ok(()) => defmt::info!("persist-selftest: marker written OK"),
                Err(e) => defmt::error!("persist-selftest: write failed {:?}", e),
            }
        }
    }
```

- [ ] **Step 3: Verify the flight build is untouched and the self-test build compiles**

Run: `cargo build --release`
Expected: 0 errors. The `write` `dead_code` warning may persist in the plain flight build (it is only *called* under the feature) — acceptable for sub-project A; it is consumed by sub-project B. Net warnings ≈ baseline.

Run: `cargo build --release --features persist-selftest`
Expected: 0 errors, 0 new warnings beyond baseline.

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`
Expected: full host suite PASS (136 prior + 9 new persist tests = 145).

- [ ] **Step 4: Commit**

```bash
git add Cargo.toml src/main.rs
git commit -m "persist: bench self-test behind persist-selftest feature"
```

- [ ] **Step 5: Bench acceptance (user, hardware — props irrelevant, no motors involved)**

1. `cargo build --release --features persist-selftest` then flash via DFU.
2. Watch USART6 defmt (PC6, 115200): first boot should log `persist: mag_calibrated=false …` then `persist-selftest: no marker yet — writing it now…` and `marker written OK`.
3. **Power-cycle** the board (no reflash).
4. Second boot should log `persist: mag_calibrated=true decl=0.1234 …` and `persist-selftest: PASS — marker survived reboot`.

That PASS line across a power cycle is the acceptance gate for sub-project A. After it passes, sub-project B can begin; the `persist-selftest` feature and its block can stay (harmless, gated) or be removed in B's first commit.

---

## Self-review notes

- **Spec coverage:** A1 sector reservation → Task 3 Step 1; A2 record format → Task 2; A3 erase-then-write → Task 3 (`flash.rs write`); A4 safety (disarmed-only, boot read, no panic) → `flash.rs` doc + Task 4 boot read + Task 5 gating; A5 host tests → Tasks 1–2, bench gate → Task 5 Step 5. CRC-32 → Task 1.
- **Type consistency:** `Config`, `RECORD_LEN`, `PAYLOAD_LEN`, `encode`, `decode`, `crc32`, `read`, `write`, `driver`, `PersistError`, `CONFIG_OFFSET`, `CONFIG_SECTOR_LEN` are used identically everywhere they appear.
- **No persisted soft-iron / append-log** — deliberately out of scope (YAGNI), per spec.
