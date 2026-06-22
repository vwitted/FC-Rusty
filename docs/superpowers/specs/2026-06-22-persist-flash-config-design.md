# `persist` — Versioned Flash Config Store (Design)

**Date:** 2026-06-22
**Status:** Approved (design); not yet implemented
**Branch context:** `dakefpv-h743-post-alpha`

---

## Why

The flight controller has no non-volatile storage. Every tunable that
should survive a power cycle (starting with magnetometer hard-iron
calibration and magnetic declination) currently can't persist.

This is the **first flash-writing code in the repo**. It is the riskiest
part of the yaw-fix work (erase/write semantics, linker-region
reservation, instruction-fetch stalls during erase), so it is built and
bench-verified **in isolation** before anything depends on it.

This spec covers **only** the persistence layer (sub-project A). The
magnetometer calibration + yaw fix (sub-project B) is its first consumer
and gets its own spec once this is proven on hardware.

### North-star alignment

Subordinate to inner-loop authority: the persistence layer never runs in
the 8 kHz control path and never touches it. It reads once at boot and
writes only while disarmed. A flash fault degrades to "uncalibrated"
(today's behaviour), never to a control hazard.

---

## Scope

**In scope**
- A reserved 128 KB flash region for config, carved out in `memory.x`.
- A versioned, CRC-checked record format (pure, host-tested).
- A firmware-only erase/write/read API over that region.
- Bench round-trip verification (write → power-cycle → read back).

**Out of scope (sub-project B and beyond)**
- Magnetometer calibration math, MEKF anchoring, GPS-COG fusion, AUX
  orchestration. (Sketched at the end for context only.)
- An append-log / wear-levelling scheme. v1 erases then writes one
  record; cal saves are rare, so wear is a non-issue. Noted as a future
  optimisation, deliberately not built now (YAGNI).
- A general key-value store. v1 persists a single fixed `Config` struct.

---

## Architecture

Two clearly separated units:

### 1. `config` record (pure, host-testable) — `src/persist/record.rs`

Owns the on-disk layout and (de)serialisation. No hardware, no `embassy`.
Fully unit-tested on the host.

Record layout (little-endian, packed):

| Field   | Type      | Notes                                            |
|---------|-----------|--------------------------------------------------|
| magic   | `u32`     | `0x46435231` = ASCII `"FCR1"`. Identifies a record.|
| version | `u16`     | Payload schema version. Bump when `Config` grows.|
| len     | `u16`     | Payload length in bytes. Guards truncation.      |
| payload | `[u8; N]` | The serialised `Config`.                          |
| crc32   | `u32`     | CRC-32 over `magic .. payload` (everything before crc). |

`Config` (v1 payload — the fields sub-project B will fill in later):

```rust
pub struct Config {
    /// Magnetometer hard-iron offset, sensor native frame, µT.
    pub mag_hard_iron_ut: [f32; 3],
    /// Magnetic declination, radians, east-positive. 0.0 = none.
    pub declination_rad: f32,
    /// True once a real calibration has been written (vs. defaults).
    pub mag_calibrated: bool,
}
```

API of the record unit:

- `Config::default()` — all zeros, `mag_calibrated = false` (safe
  "uncalibrated" state).
- `fn encode(cfg: &Config) -> [u8; RECORD_LEN]` — pack + CRC. `RECORD_LEN`
  is a multiple of 32 (H7 flash word), zero-padded.
- `fn decode(bytes: &[u8]) -> Option<Config>` — validate magic, version,
  len, and CRC; return `None` on any mismatch (caller treats `None` as
  "use defaults"). Decode of a never-written (all-`0xFF`) sector returns
  `None`.

CRC-32: a small `no_std` implementation (IEEE polynomial, table-free or a
const table). Pure function, host-tested against known vectors.

### 2. `flash` access (firmware-only) — `src/persist/flash.rs`

Thin wrapper over `embassy_stm32::flash::Flash` (blocking API) bound to the
reserved `CONFIG` region. Compiled only under the `firmware` feature.

API:

- `fn read(flash: &mut Flash) -> Option<Config>` — read `RECORD_LEN` bytes
  from the region base, hand to `record::decode`. Called once at boot.
- `fn write(flash: &mut Flash, cfg: &Config) -> Result<(), PersistError>` —
  `record::encode`, blocking-erase the sector, blocking-write the record.
  **Disarmed-only** (caller enforces; see Safety).

`PersistError` covers HAL erase/write failure and "not aligned"
defensive checks. No `panic!` on the flash path.

### Region reservation — `memory.x`

```
MEMORY
{
  FLASH  (rx) : ORIGIN = 0x08000000, LENGTH = 1920K
  CONFIG (r)  : ORIGIN = 0x081E0000, LENGTH = 128K
  RAM    (rwx): ORIGIN = 0x24000000, LENGTH = 512K
  DTCM   (rwx): ORIGIN = 0x20000000, LENGTH = 128K
  ITCM   (rx) : ORIGIN = 0x00000000, LENGTH = 64K
}
```

- `FLASH` shrinks 2048K → 1920K (15 × 128 KB sectors). The loaded firmware
  image is well under 1 MB, so it stays clear of the reserved sector.
- `CONFIG` is the last 128 KB sector (`0x081E0000`–`0x081FFFFF`), bank 2.
- The region is declared so the linker reserves it; the firmware does not
  place any section there. The address is also referenced from
  `flash.rs` (region base + length constants kept in one place).
- DFU programming writes only the firmware image (FLASH region), so a
  reflash leaves `CONFIG` intact.

---

## Data flow

**Boot:** `main` opens the `Flash` HAL once, calls `persist::read()`. On
`Some(cfg)` the values are published for consumers (sub-project B). On
`None`, `Config::default()` is used — identical to today's uncalibrated
behaviour.

**Save (sub-project B trigger, disarmed):** the orchestrator builds a
`Config`, calls `persist::write()`. Erase + write of one 32-byte-aligned
record. Returns `Ok` / logs `Err` via defmt. Never invoked while armed.

---

## Error handling

- **Corrupt / blank / wrong-version record** → `decode` returns `None` →
  defaults. The vehicle is simply "uncalibrated"; no crash, no hazard.
- **Erase/write HAL failure** → `write` returns `PersistError`, logged;
  the in-RAM config (already applied by B) is unaffected, so the current
  session still benefits from the calibration even if the save failed.
- **CRC mismatch on read-back** (optional verify-after-write) → log and
  report; do not retry-loop on the flash.

## Safety rules (non-negotiable)

- **Writes only while disarmed.** Flash erase/write stalls instruction
  fetch on the H7; doing it mid-flight is unacceptable. The caller gates
  on disarmed state; `write` is never reachable from the 8 kHz loop.
- **Reads only at boot**, before the control loops are hot.
- A flash fault must never escalate beyond "uncalibrated". No `panic!` on
  the persistence path.

---

## Testing

**Host (pure, `cargo test --lib --no-default-features`):**
- CRC-32 against known test vectors.
- `encode` then `decode` round-trips a `Config` exactly.
- `decode` rejects: bad magic, wrong version, bad CRC, bad length,
  all-`0xFF` (blank sector), truncated input.
- `RECORD_LEN` is a multiple of 32 (H7 flash word alignment).
- `Config::default()` is the "uncalibrated" safe state.

**Bench (firmware, the acceptance gate for sub-project A):**
- A temporary debug path writes a known non-default `Config`, then
  power-cycle, then `read()` returns exactly that struct (logged over
  USART6 defmt). Confirms region reservation, erase, write, and read-back
  on real hardware before sub-project B is started.

---

## Sub-project B sketch (context only — not built here)

For continuity if context is lost. B consumes `persist`:

1. **Hard-iron calibrator** (pure, host-tested): feed raw mag samples;
   least-squares sphere-fit for the offset; **bin-coverage** completion
   (tessellate the direction sphere, require N bins hit + per-axis span +
   min samples) — *not* a fixed timer. Reports progress + completion.
2. **MEKF integration**: subtract the offset before the existing mag
   update; add an absolute-anchor that makes yaw read true heading using
   `declination_rad`; fall back to today's relative boot-heading when
   uncalibrated.
3. **GPS-COG refine**: fuse COG as a **gated yaw pseudo-measurement in the
   MEKF** (active only above a forward-speed threshold, since a quad's COG
   equals heading only in forward flight). Recommended over an ad-hoc
   reference nudge.
4. **AUX orchestration**: a dedicated disarmed-only AUX channel starts the
   spin-cal; on coverage-complete it anchors and calls `persist::write()`.
   Arming aborts cal; the cal path never touches motors.

B gets its own brainstorm + spec once A is bench-proven.
