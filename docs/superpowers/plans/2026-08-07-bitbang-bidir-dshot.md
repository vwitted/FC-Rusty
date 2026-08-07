# Bit-banged Bidirectional DShot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Working bidirectional DShot (motors spin + eRPM telemetry decodes) on the DAKEFPV H743, using GPIO bit-banging instead of timer output-compare.

**Architecture:** A pacer timer generates DMA requests at a fixed rate; DMA writes 32-bit words to `GPIOA->BSRR` to emit the waveform and reads `GPIOA->IDR` to sample the ESC's reply. The timer never drives a pin, and the pins are never in alternate-function mode — they are plain GPIO, output for TX and input-with-pull-up for RX. This is the architecture Betaflight uses on H7 (`dshot_bitbang = AUTO` resolves to bitbang on everything after F4), confirmed on this board.

**Tech Stack:** Rust, Embassy (`embassy-stm32`), STM32H743VIT6, `defmt` logging, host unit tests via `--no-default-features`.

## Why this replaces the timer-DMA driver

`src/drivers/dshot_hw.rs` ports Betaflight's `pwm_output_dshot_hal.c`. Hardware confirmation on 2026-08-07 (`dshot_bitbang = AUTO`, `motor_pwm_protocol = DSHOT300`) established that BF does not run that path on H7 — it bit-bangs. The 2026-07-26 session fixed three real defects in the timer path and still could not make bidir work, because the reference implementation being matched was never the code producing the working waveform.

Two failure modes from that session are structurally absent here:

- **Stale compare register.** The compare unit never drives the pad, so its contents cannot assert the line.
- **Polarity inversion.** Inverting the protocol is swapping which half of `BSRR` you write — there is no `CCER.CCxP`.

One is *acknowledged rather than eliminated*, and this matters. BF's own comment on the 3 hold states at the end of each frame:

> Avoid CRC errors in the case of bi-directional d-shot. CRC errors can occur if the output is transitioned to an input before the signal has been sampled by the ESC... **On some MCUs it's observed that the voltage momentarily drops low on transition to input.**

That is the glitch the 2026-07-26 session chased. It is a known MCU behaviour. The fix is not to remove it but to ensure the ESC has already sampled the last bit before the transition — hence `MOTOR_DSHOT_BIT_HOLD_STATES = 3`. **Do not "optimise away" the hold states.**

## Global Constraints

- Protocol: **DShot300** by default (matches the confirmed-working BF config on this board; more timing margin than DShot600 for bit-banging).
- Pacer timer: **TIM1** (verified unclaimed — `main.rs` claims only `p.TIM2`).
- DMA stream: **DMA2_CH2** (verified free — `main.rs` claims DMA1_CH0/1/2/3/4/5/7 and DMA2_CH0/1/3/6).
- All four motors are **PA0–PA3, one port (GPIOA)** → one buffer, one DMA stream, one pacer.
- DMA buffers must be **32-byte cache-line aligned** and in a DMA-reachable region. Do not rely on D-cache being disabled.
- **`src/drivers/dshot_hw.rs` is not modified by any task in this plan.** It stays working and remains what `main.rs` uses until Task 6.
- Host tests must pass at every commit: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`
- Firmware must build at every commit: `cargo build --release` and `cargo build --release --features motor-test`
- Pure modules (`dshot_bb_frame.rs`, `dshot_bb_decode.rs`) must contain **no** `embassy_stm32` or `cortex_m` references, so they compile and test on the host.

## Derived timing (DShot300, TIM1 at 240 MHz)

BF's `bbTimebaseSetup()`:

```c
outputFreq = getDshotBaseFrequency(protocol);          // symbol rate × 3 states
outputARR  = timerclock / outputFreq - 1;
inputFreq  = outputFreq * 5 * 2 * OVER_SAMPLE / 24;    // = outputFreq × 5/4
inputARR   = timerclock / inputFreq - 1;
```

| Quantity | Value | Derivation |
|---|---|---|
| Symbol (bit) rate | 300 kHz | DShot300 |
| States per symbol | 3 | initial-assert / data / deassert |
| TX pacer rate | 900 kHz | 300 kHz × 3 |
| `outputARR` | 265 | 240e6 / 900e3 = 266 (integer div), −1. Actual 902.3 kHz, +0.25% |
| GCR response bit rate | 375 kHz | 300 kHz × 5/4 (bidir telemetry runs at 5/4 the DShot rate) |
| Oversample | 3 | `DSHOT_BITBANG_TELEMETRY_OVER_SAMPLE` |
| RX pacer rate | 1.125 MHz | 375 kHz × 3 |
| `inputARR` | 212 | 240e6 / 1.125e6 = 213 (integer div), −1. Actual 1.1268 MHz, +0.16% |
| TX buffer | 51 × u32 | (16 bits × 3 states) + 3 hold states |
| RX buffer | 140 × u16 | `DSHOT_BB_PORT_IP_BUF_LENGTH`; 140 / 1.125 MHz ≈ 124 µs window |

Both frequency errors are far inside DShot tolerance.

## File Structure

| File | Responsibility |
|---|---|
| `src/drivers/dshot_bb_frame.rs` | **Create.** Pure. 16-bit DShot frame → 51-word `BSRR` buffer. No hardware. |
| `src/drivers/dshot_bb_decode.rs` | **Create.** Pure. Oversampled `IDR` samples → GCR → eRPM. No hardware. |
| `src/drivers/dshot_bitbang.rs` | **Create.** Hardware: pacer timer, DMA, GPIO direction switching. |
| `src/lib.rs:38-42` | **Modify.** Declare the two *pure* modules inside `pub mod drivers { … }` so host tests compile them. |
| `src/main.rs:53-66` | **Modify.** Declare all three new modules inside `mod drivers { … }` so the firmware compiles them. |
| `src/motor_test.rs` | **Modify.** `DRIVER=bitbang\|timer` selection, default `timer`. |
| `build.rs` | **Modify.** Add `DRIVER` to `rerun-if-env-changed`. |
| `src/main.rs` | **Modify (Task 6 only).** Cutover, gated on Task 5 passing on hardware. |

---

### Task 1: BSRR frame builder (pure)

Transliteration of BF's `bbOutputDataInit` / `bbOutputDataSet` / `bbOutputDataClear` (`src/platform/STM32/dshot_bitbang.c:140-200`).

`BSRR` semantics: writing bit *n* (low half) **sets** pin *n* high; writing bit *n+16* (high half) **resets** it low. So "assert the active level" means the low half when non-inverted and the high half when inverted — that swap is the entire bidirectional inversion.

**Files:**
- Create: `src/drivers/dshot_bb_frame.rs`
- Modify: `src/lib.rs:38-42`, `src/main.rs:53-66`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `pub const BB_BUF_LEN: usize = 51`
  - `pub fn output_data_init(buf: &mut [u32; BB_BUF_LEN], port_mask: u16, inverted: bool)`
  - `pub fn output_data_clear(buf: &mut [u32; BB_BUF_LEN])`
  - `pub fn output_data_set(buf: &mut [u32; BB_BUF_LEN], pin: u8, value: u16, inverted: bool)`

- [ ] **Step 1: Write the failing tests**

Create `src/drivers/dshot_bb_frame.rs` containing only the test module for now:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    const M: u16 = 0b1111; // PA0..PA3

    #[test]
    fn init_noninverted_asserts_high_then_low() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, false);
        // Non-inverted: assert = drive HIGH = BSRR low half.
        assert_eq!(buf[0], M as u32, "state 0 of bit 0 should set pins high");
        assert_eq!(buf[1], 0, "middle state starts as no-change");
        assert_eq!(buf[2], (M as u32) << 16, "state 2 should reset pins low");
    }

    #[test]
    fn init_inverted_swaps_bsrr_halves() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        // Inverted: assert = drive LOW = BSRR high half.
        assert_eq!(buf[0], (M as u32) << 16);
        assert_eq!(buf[1], 0);
        assert_eq!(buf[2], M as u32);
    }

    #[test]
    fn init_writes_all_sixteen_symbols() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        for i in 0..16 {
            assert_eq!(buf[i * 3], (M as u32) << 16, "symbol {} state 0", i);
            assert_eq!(buf[i * 3 + 2], M as u32, "symbol {} state 2", i);
        }
    }

    #[test]
    fn init_hold_states_deassert_then_hold() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        // Hold states let the ESC sample the last bit before the pin
        // becomes an input. First deasserts, other two change nothing.
        assert_eq!(buf[48], M as u32, "hold state 0 deasserts");
        assert_eq!(buf[49], 0, "hold state 1 is no-change");
        assert_eq!(buf[50], 0, "hold state 2 is no-change");
    }

    #[test]
    fn set_all_ones_leaves_middles_untouched() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 0, 0xFFFF, true);
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 0, "a '1' bit never deasserts early");
        }
    }

    #[test]
    fn set_all_zeros_deasserts_every_middle() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 2, 0x0000, true);
        // Inverted: deassert = drive HIGH = BSRR low half = 1 << pin.
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 1 << 2, "a '0' bit deasserts at 1/3");
        }
    }

    #[test]
    fn set_is_msb_first() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 1, 0x8000, true); // only the MSB is 1
        assert_eq!(buf[1], 0, "bit 0 (MSB) is 1 → middle untouched");
        for i in 1..16 {
            assert_eq!(buf[i * 3 + 1], 1 << 1, "bits 1..15 are 0 → deassert");
        }
    }

    #[test]
    fn two_pins_share_one_buffer_without_interfering() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 0, 0x0000, true); // all zeros on PA0
        output_data_set(&mut buf, 3, 0xFFFF, true); // all ones  on PA3
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 1 << 0, "only PA0 deasserts early");
        }
    }

    #[test]
    fn clear_resets_only_middles() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 0, 0x0000, true);
        output_data_clear(&mut buf);
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 0, "middle cleared");
            assert_eq!(buf[i * 3], (M as u32) << 16, "state 0 preserved");
            assert_eq!(buf[i * 3 + 2], M as u32, "state 2 preserved");
        }
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu dshot_bb_frame`
Expected: FAIL — `cannot find function output_data_init in this scope` (and the module isn't declared yet).

- [ ] **Step 3: Declare the module in BOTH places**

There is no `src/drivers/mod.rs` in this repo. Driver modules are declared
inline in two separate lists, and a pure module must appear in both:

`src/lib.rs:38-42` — the host-testable subset, which is what
`cargo test --lib --no-default-features` compiles. Add in alphabetical order:

```rust
pub mod drivers {
    pub mod dshot_bb_frame;
    pub mod dshot_frame;
    pub mod dshot_telemetry;
    pub mod nmea;
}
```

`src/main.rs:53-66` — the firmware set. Add `pub mod dshot_bb_frame;` in
alphabetical order alongside the existing entries.

Omitting the `lib.rs` entry is the failure mode to avoid: the firmware would
still build, but the host tests would never compile the module and would
silently pass without running.

- [ ] **Step 4: Write the implementation**

Prepend to `src/drivers/dshot_bb_frame.rs`, above the test module:

```rust
// dshot_bb_frame.rs — DShot frame → GPIO BSRR pattern, for bit-banged output.
//
// Transliteration of BF's bbOutputDataInit/bbOutputDataSet/bbOutputDataClear
// (src/platform/STM32/dshot_bitbang.c:140-200).
//
// Each DShot bit is three pacer states:
//
//   state 0  assert the active level      (all pins on the port)
//   state 1  per-pin: deassert iff the bit is 0
//   state 2  deassert                     (all pins on the port)
//
// so a '0' is asserted for 1/3 of the bit and a '1' for 2/3.
//
// BSRR: writing bit n sets pin n HIGH, writing bit n+16 resets it LOW. The
// entire bidirectional inversion is therefore swapping which half we use —
// there is no timer polarity involved, because no timer drives the pin.
//
// The three trailing hold states are NOT padding. BF's comment: a CRC error
// occurs if the line is switched to an input before the ESC has sampled the
// last bit, because on some MCUs the pad momentarily drops low on that
// transition. The hold states buy the ESC that sampling time. Do not remove
// them.

/// Pacer states per DShot symbol (BF: `MOTOR_DSHOT_STATE_PER_SYMBOL`).
pub const STATES_PER_SYMBOL: usize = 3;
/// Bits in a DShot frame (BF: `MOTOR_DSHOT_FRAME_BITS`).
pub const FRAME_BITS: usize = 16;
/// Trailing states that hold the line at idle (BF: `MOTOR_DSHOT_BIT_HOLD_STATES`).
pub const HOLD_STATES: usize = 3;
/// BSRR words per frame (BF: `MOTOR_DSHOT_BUF_LENGTH`) = 51.
pub const BB_BUF_LEN: usize = FRAME_BITS * STATES_PER_SYMBOL + HOLD_STATES;

/// (assert, deassert) BSRR masks for the whole port.
fn masks(port_mask: u16, inverted: bool) -> (u32, u32) {
    let pm = port_mask as u32;
    if inverted {
        (pm << 16, pm) // assert = drive LOW, deassert = drive HIGH
    } else {
        (pm, pm << 16) // assert = drive HIGH, deassert = drive LOW
    }
}

/// Lay down the port-wide skeleton: assert at each symbol start, deassert at
/// each symbol end, plus the trailing hold states. Call once per frame before
/// `output_data_set`.
pub fn output_data_init(buf: &mut [u32; BB_BUF_LEN], port_mask: u16, inverted: bool) {
    let (assert, deassert) = masks(port_mask, inverted);

    for symbol in 0..FRAME_BITS {
        buf[symbol * STATES_PER_SYMBOL] |= assert;
        buf[symbol * STATES_PER_SYMBOL + 1] = 0;
        buf[symbol * STATES_PER_SYMBOL + 2] |= deassert;
    }

    let hold = FRAME_BITS * STATES_PER_SYMBOL;
    buf[hold] |= deassert;
    buf[hold + 1] = 0;
    buf[hold + 2] = 0;
}

/// Reset the per-pin middle states to "no change", leaving the port-wide
/// skeleton intact. Call between frames instead of re-running `init`.
pub fn output_data_clear(buf: &mut [u32; BB_BUF_LEN]) {
    for symbol in 0..FRAME_BITS {
        buf[symbol * STATES_PER_SYMBOL + 1] = 0;
    }
}

/// Write one motor's 16-bit frame into the shared buffer, MSB first. A `0`
/// bit deasserts at 1/3 of the symbol; a `1` bit leaves the middle state
/// untouched so the assert runs to 2/3.
pub fn output_data_set(buf: &mut [u32; BB_BUF_LEN], pin: u8, value: u16, inverted: bool) {
    let middle_bit: u32 = if inverted {
        1 << pin // deassert = drive HIGH = BSRR low half
    } else {
        1 << (pin as u32 + 16) // deassert = drive LOW = BSRR high half
    };

    let mut v = value;
    for symbol in 0..FRAME_BITS {
        if v & 0x8000 == 0 {
            buf[symbol * STATES_PER_SYMBOL + 1] |= middle_bit;
        }
        v <<= 1;
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu dshot_bb_frame`
Expected: PASS, 9 tests.

- [ ] **Step 6: Verify the firmware still builds**

Run: `cargo build --release && cargo build --release --features motor-test`
Expected: both succeed.

- [ ] **Step 7: Commit**

```bash
git add src/drivers/dshot_bb_frame.rs src/lib.rs src/main.rs
git commit -m "dshot-bb: BSRR frame builder for bit-banged DShot

Transliteration of BF's bbOutputDataInit/Set/Clear. Three pacer states per
symbol; bidirectional inversion is swapping which half of BSRR is written,
so no timer polarity is involved. The three trailing hold states are load
bearing — they give the ESC time to sample the last bit before the pin
becomes an input, which BF documents as a CRC-error source on some MCUs."
```

---

### Task 2: GCR sample decoder (pure)

Turns oversampled `IDR` samples into an eRPM period. The GCR run-length reconstruction mirrors the existing `decode_telemetry` in `dshot_hw.rs:719-791` (each run of *n* bit-times emits a `1` followed by *n−1* zeros, which performs the transition-decoding inline), so the quintet table and CRC carry over unchanged.

**Files:**
- Create: `src/drivers/dshot_bb_decode.rs`
- Modify: `src/lib.rs:38-42`, `src/main.rs:53-66`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `pub const OVERSAMPLE: usize = 3`
  - `pub const RX_BUF_LEN: usize = 140`
  - `pub enum BbTelemetry { Erpm { period_us: u32 }, NoSignal, InvalidGcr, InvalidCrc }`
  - `pub fn decode(samples: &[u16], pin: u8) -> BbTelemetry`

- [ ] **Step 1: Write the failing tests**

Create `src/drivers/dshot_bb_decode.rs` with only the test module:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    /// Build a sample buffer the way the ESC would drive the line: idle high,
    /// then 21 GCR bits at `OVERSAMPLE` samples each, then idle high again.
    fn samples_from_gcr(gcr: u32, pin: u8) -> [u16; RX_BUF_LEN] {
        let mut buf = [0xFFFFu16; RX_BUF_LEN];
        let mut idx = 8; // leading idle
        for bit in (0..GCR_BITS).rev() {
            let level = (gcr >> bit) & 1;
            for _ in 0..OVERSAMPLE {
                if idx >= RX_BUF_LEN {
                    return buf;
                }
                if level == 1 {
                    buf[idx] |= 1 << pin;
                } else {
                    buf[idx] &= !(1 << pin);
                }
                idx += 1;
            }
        }
        buf
    }

    /// Encode a 16-bit BLHeli telemetry word into its 21-bit GCR line coding,
    /// i.e. the inverse of what `decode` undoes.
    fn gcr_from_payload(decoded: u16) -> u32 {
        const GCR_ENCODE: [u32; 16] = [
            0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
            0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
        ];
        let mut quintets: u32 = 0;
        for nibble in (0..4).rev() {
            let v = (decoded >> (nibble * 4)) & 0xF;
            quintets = (quintets << 5) | GCR_ENCODE[v as usize];
        }
        // Line coding: value ^ (value >> 1), transmitted as 21 bits.
        let mut out: u32 = 0;
        let mut prev = 0u32;
        for bit in (0..20).rev() {
            let b = (quintets >> bit) & 1;
            prev ^= b;
            out = (out << 1) | prev;
        }
        (out & 0x0F_FFFF) | (1 << 20)
    }

    /// A telemetry word with a correct BLHeli checksum for the given payload.
    fn word_with_crc(payload12: u16) -> u16 {
        let mut csum = payload12 ^ (payload12 >> 4) ^ (payload12 >> 8);
        csum = !csum & 0xF;
        (payload12 << 4) | csum
    }

    #[test]
    fn empty_line_reports_no_signal() {
        let buf = [0xFFFFu16; RX_BUF_LEN]; // never goes low
        assert_eq!(decode(&buf, 0), BbTelemetry::NoSignal);
    }

    #[test]
    fn round_trips_a_known_erpm_period() {
        // exponent 0, mantissa 100 → period_us = 100
        let word = word_with_crc(100);
        let buf = samples_from_gcr(gcr_from_payload(word), 0);
        assert_eq!(decode(&buf, 0), BbTelemetry::Erpm { period_us: 100 });
    }

    #[test]
    fn applies_the_exponent() {
        // exponent 2, mantissa 50 → period_us = 50 << 2 = 200
        let word = word_with_crc((2 << 9) | 50);
        let buf = samples_from_gcr(gcr_from_payload(word), 0);
        assert_eq!(decode(&buf, 0), BbTelemetry::Erpm { period_us: 200 });
    }

    #[test]
    fn decodes_on_a_pin_other_than_zero() {
        let word = word_with_crc(100);
        let buf = samples_from_gcr(gcr_from_payload(word), 3);
        assert_eq!(decode(&buf, 3), BbTelemetry::Erpm { period_us: 100 });
        // Pin 1 saw only idle-high in that buffer.
        assert_eq!(decode(&buf, 1), BbTelemetry::NoSignal);
    }

    #[test]
    fn rejects_a_corrupted_checksum() {
        let word = word_with_crc(100) ^ 0x1; // break the CRC nibble
        let buf = samples_from_gcr(gcr_from_payload(word), 0);
        assert_eq!(decode(&buf, 0), BbTelemetry::InvalidCrc);
    }

    #[test]
    fn all_ones_payload_means_not_spinning() {
        let word = word_with_crc(0x0FFF);
        let buf = samples_from_gcr(gcr_from_payload(word), 0);
        assert_eq!(decode(&buf, 0), BbTelemetry::Erpm { period_us: 0 });
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu dshot_bb_decode`
Expected: FAIL to compile — `cannot find function decode in this scope`, plus the same for `RX_BUF_LEN`, `OVERSAMPLE`, `GCR_BITS` and `BbTelemetry`, none of which exist yet. A compile failure is the correct red state here.

- [ ] **Step 3: Declare the module in BOTH places**

Pure module, so it goes in both lists (see Task 1 Step 3):
`pub mod dshot_bb_decode;` in `src/lib.rs`'s `pub mod drivers { … }` block
AND in `src/main.rs`'s `mod drivers { … }` block, alphabetically in each.

- [ ] **Step 4: Write the implementation**

Prepend to `src/drivers/dshot_bb_decode.rs`:

```rust
// dshot_bb_decode.rs — bidirectional DShot telemetry decode from oversampled
// GPIO samples.
//
// The ESC replies at 5/4 the DShot bit rate, 21 GCR bits, line-coded so that
// a transition represents a 1. We sample the port at OVERSAMPLE × that bit
// rate, so a run of N samples at one level is round(N / OVERSAMPLE) bit
// times.
//
// Reconstruction mirrors `dshot_hw::decode_telemetry`: each run of n bit
// times emits a `1` followed by n-1 zeros, which performs the transition
// decode inline — so the quintet table and CRC below are the same ones the
// timer-DMA driver uses.

/// Samples per GCR bit (BF: `DSHOT_BITBANG_TELEMETRY_OVER_SAMPLE`).
pub const OVERSAMPLE: usize = 3;
/// Samples captured per response window (BF: `DSHOT_BB_PORT_IP_BUF_LENGTH`).
pub const RX_BUF_LEN: usize = 140;
/// GCR bits in one reply.
pub const GCR_BITS: usize = 21;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "firmware", derive(defmt::Format))]
pub enum BbTelemetry {
    Erpm { period_us: u32 },
    NoSignal,
    InvalidGcr,
    InvalidCrc,
}

/// Decode one motor's reply out of a port-wide sample buffer.
pub fn decode(samples: &[u16], pin: u8) -> BbTelemetry {
    let bit = |s: u16| -> u32 { ((s >> pin) & 1) as u32 };

    // The line idles high; the reply begins at the first falling edge.
    let Some(start) = samples.iter().position(|&s| bit(s) == 0) else {
        return BbTelemetry::NoSignal;
    };

    // Walk runs of constant level, converting each to a bit count.
    let mut value: u32 = 0;
    let mut bits: u32 = 0;
    let mut run_level = 0u32;
    let mut run_len = 0usize;

    for &s in &samples[start..] {
        let lvl = bit(s);
        if lvl == run_level {
            run_len += 1;
            continue;
        }
        let n = bit_times(run_len);
        if n == 0 || bits + n > GCR_BITS as u32 {
            return BbTelemetry::InvalidGcr;
        }
        value <<= n;
        value |= 1 << (n - 1);
        bits += n;
        run_level = lvl;
        run_len = 1;
    }

    // Pad the tail out to 21 bits, as the trailing idle carries no edge.
    if bits < GCR_BITS as u32 {
        let n = GCR_BITS as u32 - bits;
        value <<= n;
        value |= 1 << (n - 1);
        bits += n;
    }
    if bits != GCR_BITS as u32 {
        return BbTelemetry::InvalidGcr;
    }

    // 5-to-4 GCR quintet decode (BF's table).
    const GCR_DECODE: [u32; 32] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0,
    ];
    let s0 = GCR_DECODE[(value & 0x1F) as usize];
    let s1 = GCR_DECODE[((value >> 5) & 0x1F) as usize];
    let s2 = GCR_DECODE[((value >> 10) & 0x1F) as usize];
    let s3 = GCR_DECODE[((value >> 15) & 0x1F) as usize];
    let decoded = s0 | (s1 << 4) | (s2 << 8) | (s3 << 12);

    // BLHeli checksum: the low nibble of the folded XOR must be 0xF.
    let mut csum = decoded ^ (decoded >> 8);
    csum ^= csum >> 4;
    if (csum & 0xF) != 0xF {
        return BbTelemetry::InvalidCrc;
    }

    let payload = (decoded >> 4) & 0xFFF;
    if payload == 0x0FFF {
        return BbTelemetry::Erpm { period_us: 0 }; // not spinning
    }
    let exponent = (payload >> 9) & 0x7;
    let mantissa = payload & 0x1FF;
    BbTelemetry::Erpm { period_us: mantissa << exponent }
}

/// Samples → bit times, rounding to nearest.
fn bit_times(run_len: usize) -> u32 {
    ((run_len + OVERSAMPLE / 2) / OVERSAMPLE) as u32
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu dshot_bb_decode`
Expected: PASS, 6 tests.

If `round_trips_a_known_erpm_period` fails, the disagreement is between the test's `gcr_from_payload` encoder and the decoder's run-length reconstruction — print `value` and the four quintets and compare against the `GCR_DECODE` table before changing either side. Do **not** "fix" it by relaxing the CRC check.

- [ ] **Step 6: Commit**

```bash
git add src/drivers/dshot_bb_decode.rs src/lib.rs src/main.rs
git commit -m "dshot-bb: GCR telemetry decode from oversampled GPIO samples

Run-length reconstruction mirrors dshot_hw::decode_telemetry, so the quintet
table and BLHeli checksum are shared logic. Tested by round-tripping known
eRPM payloads through a GCR encoder in the test module — the first telemetry
path in this project verifiable without a flash."
```

---

### Task 3: Bit-banged transmit on hardware

Gets a waveform out. Non-inverted only — bidirectional comes in Task 4, so this task is provable against known-good behaviour (motors spin exactly as they do with the timer driver).

**Files:**
- Create: `src/drivers/dshot_bitbang.rs`
- Modify: `src/main.rs:53-66`, `src/motor_test.rs`, `build.rs`

**Interfaces:**
- Consumes: `dshot_bb_frame::{BB_BUF_LEN, output_data_init, output_data_clear, output_data_set}`
- Produces:
  - `pub struct DshotBitbang<'d>`
  - `pub fn new(_tim1: Peri<'d, TIM1>, dma: Peri<'d, DMA2_CH2>, _pa0: Peri<'d, PA0>, _pa1: Peri<'d, PA1>, _pa2: Peri<'d, PA2>, _pa3: Peri<'d, PA3>, bidir: bool) -> Self` — pins are taken by value to claim the peripherals even though the driver addresses GPIOA through the PAC
  - `pub async fn send(&mut self, frames: [DshotFrame; 4])` — reads `frames[i].raw` (`pub raw: u16` in `dshot_frame.rs:74`)

- [ ] **Step 1: Verify the peripheral bindings compile**

Before writing the driver, confirm how Embassy exposes TIM1 update-event DMA. Write a throwaway file `/tmp/bb_probe.rs` contents into `src/drivers/dshot_bitbang.rs` temporarily:

```rust
use embassy_stm32::peripherals::{DMA2_CH2, TIM1};
use embassy_stm32::timer::UpDma;

#[allow(dead_code)]
fn probe(dma: &embassy_stm32::Peri<'static, DMA2_CH2>) -> u8 {
    <DMA2_CH2 as UpDma<TIM1>>::request(dma)
}
```

Run: `cargo build --release --features motor-test`

If `UpDma` does not exist or `DMA2_CH2` does not implement it for `TIM1`, run:
`grep -rn "trait UpDma" ~/.cargo/registry/src/*/embassy-stm32-*/src/timer/mod.rs`
and adjust to the trait Embassy actually provides. **Record the working form in a comment** — this is the binding every later step depends on. Delete the probe once confirmed.

- [ ] **Step 2: Write the driver**

Create `src/drivers/dshot_bitbang.rs`:

```rust
// dshot_bitbang.rs — bit-banged DShot for the DAKEFPVH743.
//
// Reference: betaflight/src/platform/STM32/dshot_bitbang.c
//
// TIM1 is a pacer only: it drives no pin, it just generates a DMA request
// every state period. DMA writes 32-bit BSRR words to GPIOA, producing the
// waveform directly. The motor pins are plain GPIO throughout — never in
// alternate-function mode — which is why none of the compare-register or
// AF-handover failure modes of dshot_hw.rs exist here.
//
// M1..M4 are PA0..PA3, one port, so all four motors share one buffer and one
// DMA stream. Per-pin data lives in the middle state of each symbol.
//
// Timing (DShot300, TIM1 at 240 MHz): 3 states per symbol → 900 kHz pacer,
// ARR = 240e6/900e3 - 1 = 265.

use embassy_stm32::Peri;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{DMA2_CH2, PA0, PA1, PA2, PA3, TIM1};

use super::dshot_bb_frame::{BB_BUF_LEN, output_data_clear, output_data_init, output_data_set};
use super::dshot_frame::DshotFrame;

/// PA0..PA3 = M1..M4.
const PORT_MASK: u16 = 0b1111;
const MOTOR_PINS: [u8; 4] = [0, 1, 2, 3];

/// TIM1 counter period for the transmit pacer. 240 MHz / 900 kHz - 1.
const TX_ARR: u32 = 265;

/// Cache-line-aligned TX buffer. H7 DMA requires 32-byte alignment for clean
/// cache maintenance; do not assume D-cache is disabled.
#[repr(C, align(32))]
struct TxBuf([u32; BB_BUF_LEN]);

static mut TX_BUF: TxBuf = TxBuf([0; BB_BUF_LEN]);

pub struct DshotBitbang<'d> {
    dma: Peri<'d, DMA2_CH2>,
    bidir: bool,
    frame_count: u32,
}

impl<'d> DshotBitbang<'d> {
    pub fn new(
        _tim1: Peri<'d, TIM1>,
        dma: Peri<'d, DMA2_CH2>,
        _pa0: Peri<'d, PA0>,
        _pa1: Peri<'d, PA1>,
        _pa2: Peri<'d, PA2>,
        _pa3: Peri<'d, PA3>,
        bidir: bool,
    ) -> Self {
        // GPIO: plain push-pull output, low slew. ArduPilot notes bidir DShot
        // needs push-pull and below-MID2 slew to avoid noise on the
        // output→input transition; BF uses GPIO_SPEED_FREQ_LOW.
        pac::GPIOA.pupdr().modify(|w| {
            for p in MOTOR_PINS {
                w.set_pupdr(p as usize, pac::gpio::vals::Pupdr::PULLUP);
            }
        });
        pac::GPIOA.otyper().modify(|w| {
            for p in MOTOR_PINS {
                w.set_ot(p as usize, pac::gpio::vals::Ot::PUSHPULL);
            }
        });
        pac::GPIOA.ospeedr().modify(|w| {
            for p in MOTOR_PINS {
                w.set_ospeedr(p as usize, pac::gpio::vals::Ospeedr::LOWSPEED);
            }
        });
        // Idle level before the pins become outputs: bidir idles HIGH.
        set_idle_level(bidir);
        pac::GPIOA.moder().modify(|w| {
            for p in MOTOR_PINS {
                w.set_moder(p as usize, pac::gpio::vals::Moder::OUTPUT);
            }
        });

        // TIM1 as pacer: no output, update event only.
        pac::RCC.apb2enr().modify(|w| w.set_tim1en(true));
        pac::TIM1.cr1().write(|_| {}); // counter disabled while configuring
        pac::TIM1.psc().write_value(0);
        pac::TIM1.arr().write_value(TX_ARR);
        pac::TIM1.egr().write(|w| w.set_ug(true)); // load PSC/ARR
        pac::TIM1.cr1().modify(|w| w.set_cen(true));

        defmt::info!(
            "DShot bitbang init: TIM1 pacer ARR={=u32} bidir={=bool} port_mask={=u16:04b}",
            TX_ARR,
            bidir,
            PORT_MASK,
        );

        Self { dma, bidir, frame_count: 0 }
    }

    /// Emit one frame on all four motors.
    pub async fn send(&mut self, frames: [DshotFrame; 4]) {
        use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
        use embassy_stm32::timer::UpDma;

        // SAFETY: single owner; the DMA transfer is awaited to completion
        // before this function returns, so no aliasing outlives the borrow.
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF.0) };

        if self.frame_count == 0 {
            output_data_init(buf, PORT_MASK, self.bidir);
        } else {
            output_data_clear(buf);
        }
        for (i, pin) in MOTOR_PINS.iter().enumerate() {
            output_data_set(buf, *pin, frames[i].raw, self.bidir);
        }

        let mut opts = TransferOptions::default();
        opts.fifo_threshold = Some(FifoThreshold::Quarter);
        opts.mburst = Burst::Single;
        opts.pburst = Burst::Single;

        let bsrr = pac::GPIOA.bsrr().as_ptr() as *mut u32;

        unsafe {
            // Pacer first, DMA stream armed last — BF's ordering.
            pac::TIM1.cnt().write_value(0);
            pac::TIM1.dier().modify(|w| w.set_ude(true));

            let req = <DMA2_CH2 as UpDma<TIM1>>::request(&self.dma);
            let t = Transfer::new_write(self.dma.reborrow(), req, &buf[..], bsrr, opts);
            t.await;

            pac::TIM1.dier().modify(|w| w.set_ude(false));
        }

        self.frame_count = self.frame_count.wrapping_add(1);
    }
}

/// Drive the four motor pins to their idle level via BSRR.
fn set_idle_level(bidir: bool) {
    pac::GPIOA.bsrr().write(|w| {
        for p in MOTOR_PINS {
            if bidir {
                w.set_bs(p as usize, true); // bidir idles HIGH
            } else {
                w.set_br(p as usize, true); // plain DShot idles LOW
            }
        }
    });
}
```

- [ ] **Step 3: Declare the module and add driver selection**

In `src/main.rs`'s `mod drivers { … }` block only — **not** `lib.rs`. This
module uses `embassy_stm32`, so putting it in `lib.rs` would break the
host test build:

```rust
pub mod dshot_bitbang;
```

In `src/motor_test.rs`, add to the config struct and parser (mirroring the existing `bidir` field exactly):

```rust
/// Which DShot driver to exercise: false = timer-DMA (dshot_hw), true = bit-bang.
pub use_bitbang: bool,
```

```rust
const DEFAULT_USE_BITBANG: bool = false;
```

In `parse_config`, alongside the `bidir` match:

```rust
let use_bitbang = match driver.map(|s| s.trim()) {
    Some("bitbang") => true,
    Some("timer") => false,
    _ => DEFAULT_USE_BITBANG,
};
```

Add `driver: Option<&str>` as a `parse_config` parameter, pass `option_env!("DRIVER")` from `resolve_config`, and add the compile-time validator beside `env_bidir_ok`:

```rust
/// Build-time validation for DRIVER: unset, blank, "timer" or "bitbang" only.
const fn env_driver_ok(v: Option<&str>) -> bool {
    let Some(s) = v else { return true };
    let b = s.as_bytes();
    let (i, j) = trimmed(b);
    if i == j {
        return true;
    }
    let n = j - i;
    if n == 5 {
        return b[i] == b't' && b[i+1] == b'i' && b[i+2] == b'm' && b[i+3] == b'e' && b[i+4] == b'r';
    }
    if n == 7 {
        return b[i] == b'b' && b[i+1] == b'i' && b[i+2] == b't' && b[i+3] == b'b'
            && b[i+4] == b'a' && b[i+5] == b'n' && b[i+6] == b'g';
    }
    false
}
```

and register it in the `const _: ()` block:

```rust
assert!(env_driver_ok(option_env!("DRIVER")), "DRIVER must be `timer` or `bitbang`");
```

In `build.rs`, add `"DRIVER"` to the `emit_motor_test_env_deps` list.

- [ ] **Step 4: Branch the motor-test run loop**

In `motor_test::run`, replace the single `DshotQuad` construction with a branch on `cfg.use_bitbang`. Keep both arms structurally identical (arming stream, then drive loop) so the comparison is honest:

```rust
if cfg.use_bitbang {
    let mut dshot = crate::drivers::dshot_bitbang::DshotBitbang::new(
        p.TIM1, p.DMA2_CH2, p.PA0, p.PA1, p.PA2, p.PA3, cfg.bidir,
    );
    defmt::info!("motor-test: arming ESCs (zero throttle, 3s) [bitbang]");
    for _ in 0..(cfg.loop_khz as u32 * 3000) {
        dshot.send(stop).await;
        ticker.next().await;
    }
    defmt::info!("motor-test: driving motors [bitbang]");
    loop {
        dshot.send(frames).await;
        ticker.next().await;
    }
} else {
    // existing DshotQuad path, unchanged
}
```

- [ ] **Step 5: Write host tests for the new config parsing**

In `motor_test`'s test module:

```rust
#[test]
fn driver_defaults_to_timer() {
    let c = parse_config([None; 4], None, None, None);
    assert!(!c.use_bitbang);
}

#[test]
fn driver_selects_bitbang() {
    let c = parse_config([None; 4], None, None, Some("bitbang"));
    assert!(c.use_bitbang);
}

#[test]
fn driver_garbage_falls_back_to_timer() {
    let c = parse_config([None; 4], None, None, Some("nonsense"));
    assert!(!c.use_bitbang);
}

#[test]
fn env_driver_ok_accepts_only_known_drivers() {
    assert!(env_driver_ok(None));
    assert!(env_driver_ok(Some("")));
    assert!(env_driver_ok(Some(" timer ")));
    assert!(env_driver_ok(Some("bitbang")));
    assert!(!env_driver_ok(Some("bit-bang")));
    assert!(!env_driver_ok(Some("dma")));
}
```

Update the existing `parse_config` call sites in the test module to pass the new fourth argument.

- [ ] **Step 6: Run the tests and build both firmwares**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`
Expected: PASS, including the 4 new tests.

Run: `cargo build --release && DRIVER=bitbang cargo build --release --features motor-test`
Expected: both succeed.

- [ ] **Step 7: Bench-verify transmit**

```bash
DRIVER=bitbang BIDIR=0 LOOP_KHZ=2 ./scripts/flash-motor-test.sh
```

Expected: motors spin at 5%, exactly as they do with `DRIVER=timer BIDIR=0`. On the scope, M1 shows a DShot300 frame — 16 bits at 3.33 µs, `0` bits high for ~1/3 and `1` bits for ~2/3, idle low.

If nothing comes out, check in this order: TIM1 clock enabled (`RCC.APB2ENR.TIM1EN`), `CR1.CEN` set, `DIER.UDE` set, DMA stream request number correct (Step 1), buffer address DMA-reachable from DMA2.

**Do not proceed to Task 4 until motors spin here.** This task exists to prove the pacer, DMA path, and buffer encoding in isolation, with bidirectional out of the picture.

- [ ] **Step 8: Commit**

```bash
git add src/drivers/dshot_bitbang.rs src/main.rs src/motor_test.rs build.rs
git commit -m "dshot-bb: bit-banged transmit on TIM1 pacer + DMA2_CH2

TIM1 drives no pin; it paces DMA writes of BSRR words to GPIOA. Motor pins
stay plain GPIO throughout. Selected with DRIVER=bitbang; defaults to the
existing timer driver, which is untouched.

Verified on hardware: motors spin at DRIVER=bitbang BIDIR=0."
```

---

### Task 4: Inverted transmit and telemetry capture

Adds the bidirectional half: inverted output, then the direction switch to input and the sample capture. Decoding is deliberately *not* wired up yet — this task proves samples arrive.

**Files:**
- Modify: `src/drivers/dshot_bitbang.rs`

**Interfaces:**
- Consumes: `dshot_bb_decode::{OVERSAMPLE, RX_BUF_LEN}`
- Produces: `pub async fn send_and_receive(&mut self, frames: [DshotFrame; 4]) -> [u16; RX_BUF_LEN]`

- [ ] **Step 1: Add the RX buffer and input timing**

In `src/drivers/dshot_bitbang.rs`:

```rust
use super::dshot_bb_decode::RX_BUF_LEN;

/// TIM1 counter period for the receive pacer. The reply runs at 5/4 the DShot
/// bit rate and we oversample 3×, so 300 kHz × 5/4 × 3 = 1.125 MHz.
/// 240 MHz / 1.125 MHz - 1 = 212. (BF derives this as
/// `outputFreq * 5 * 2 * OVER_SAMPLE / 24`, which is `outputFreq × 5/4`.)
const RX_ARR: u32 = 212;

#[repr(C, align(32))]
struct RxBuf([u16; RX_BUF_LEN]);

static mut RX_BUF: RxBuf = RxBuf([0; RX_BUF_LEN]);
```

- [ ] **Step 2: Add the direction switch and capture**

Append to `impl DshotBitbang`:

```rust
    /// Send one frame, then release the line and sample the ESC's reply.
    /// Returns the raw port samples; decoding is the caller's job.
    pub async fn send_and_receive(&mut self, frames: [DshotFrame; 4]) -> [u16; RX_BUF_LEN] {
        use embassy_stm32::dma::{Burst, FifoThreshold, Transfer, TransferOptions};
        use embassy_stm32::timer::UpDma;

        self.send(frames).await;

        // Release the line. The three hold states at the end of the frame have
        // already given the ESC time to sample the last bit, so this
        // transition is safe here and only here.
        pac::GPIOA.moder().modify(|w| {
            for p in MOTOR_PINS {
                w.set_moder(p as usize, pac::gpio::vals::Moder::INPUT);
            }
        });

        // SAFETY: as for TX — awaited to completion before returning.
        let rx = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF.0) };
        rx.fill(0);

        let mut opts = TransferOptions::default();
        opts.fifo_threshold = Some(FifoThreshold::Quarter);
        opts.mburst = Burst::Single;
        opts.pburst = Burst::Single;

        let idr = pac::GPIOA.idr().as_ptr() as *const u16;

        unsafe {
            pac::TIM1.arr().write_value(RX_ARR);
            pac::TIM1.cnt().write_value(0);
            pac::TIM1.egr().write(|w| w.set_ug(true));
            pac::TIM1.dier().modify(|w| w.set_ude(true));

            let req = <DMA2_CH2 as UpDma<TIM1>>::request(&self.dma);
            let t = Transfer::new_read(self.dma.reborrow(), req, idr, &mut rx[..], opts);
            t.await;

            pac::TIM1.dier().modify(|w| w.set_ude(false));
            pac::TIM1.arr().write_value(TX_ARR);
            pac::TIM1.egr().write(|w| w.set_ug(true));
        }

        // Back to driving the line at its idle level.
        set_idle_level(self.bidir);
        pac::GPIOA.moder().modify(|w| {
            for p in MOTOR_PINS {
                w.set_moder(p as usize, pac::gpio::vals::Moder::OUTPUT);
            }
        });

        *rx
    }
```

- [ ] **Step 3: Add a one-shot capture dump**

So the bench run reports what arrived rather than requiring a scope. Add to `send_and_receive`, just before the return, and add a `const RX_PROBE_FRAME: u32 = 100;` beside `TX_ARR`:

```rust
        if self.frame_count == RX_PROBE_FRAME {
            let m1_low = rx.iter().filter(|&&s| s & 1 == 0).count();
            let transitions = rx.windows(2).filter(|w| (w[0] ^ w[1]) & 1 != 0).count();
            defmt::info!(
                "bitbang RX probe: {=usize} of {=usize} samples low on M1, {=usize} transitions",
                m1_low, RX_BUF_LEN, transitions,
            );
            defmt::info!(
                "  first 16 samples (M1 bit): {=u16:016b}",
                rx.iter().take(16).enumerate()
                    .fold(0u16, |acc, (i, &s)| acc | (((s & 1) as u16) << i)),
            );
        }
```

- [ ] **Step 4: Call it from motor-test in bidir mode**

In the bitbang arm of `motor_test::run`, use `send_and_receive` when `cfg.bidir` is set and `send` otherwise.

- [ ] **Step 5: Build**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu && DRIVER=bitbang cargo build --release --features motor-test`
Expected: tests pass, firmware builds.

- [ ] **Step 6: Bench-verify inverted transmit and capture**

```bash
DRIVER=bitbang BIDIR=1 LOOP_KHZ=2 ./scripts/flash-motor-test.sh
```

Expected on the scope: idle **high**, frames inverted (active-low pulses), `0` bits low for ~1/3 and `1` bits for ~2/3, and **no stray pulse in the idle period**.

Expected in the log: the `bitbang RX probe` line showing a non-zero transition count. Zero transitions with all samples high means the ESC is not replying; zero transitions with all samples low means the line is being held down — check the `MODER` switch actually took effect and the pull-up is configured.

**The success criterion for this task is transitions in the capture, not decoded eRPM.** If the ESC replies, Task 5 is decode wiring. If it does not, the problem is upstream and no decoder will help.

- [ ] **Step 7: Commit**

```bash
git add src/drivers/dshot_bitbang.rs src/motor_test.rs
git commit -m "dshot-bb: inverted transmit, direction switch and reply capture

Inversion is the BSRR half-swap in dshot_bb_frame; there is no polarity
register involved. Pacer ARR switches between 265 (TX, 900 kHz) and 212
(RX, 1.125 MHz = reply rate 5/4 × 300 kHz, oversampled 3×).

Capture only — decoding lands in the next commit. One-shot probe reports
sample transitions so the bench can tell 'ESC silent' from 'line stuck'."
```

---

### Task 5: Wire up decoding

**Files:**
- Modify: `src/drivers/dshot_bitbang.rs`, `src/motor_test.rs`

**Interfaces:**
- Consumes: `dshot_bb_decode::{decode, BbTelemetry}`
- Produces: `pub async fn send_and_decode(&mut self, frames: [DshotFrame; 4]) -> [BbTelemetry; 4]`

- [ ] **Step 1: Add the decode wrapper**

```rust
    /// Send one frame and decode all four replies.
    pub async fn send_and_decode(&mut self, frames: [DshotFrame; 4]) -> [BbTelemetry; 4] {
        let rx = self.send_and_receive(frames).await;
        core::array::from_fn(|i| decode(&rx[..], MOTOR_PINS[i]))
    }
```

with `use super::dshot_bb_decode::{BbTelemetry, decode};` at the top.

- [ ] **Step 2: Log decoded telemetry from motor-test**

In the bitbang bidir arm, replace `send_and_receive` with `send_and_decode` and log at ~10 Hz exactly as the timer path does:

```rust
let telem = dshot.send_and_decode(frames).await;
n = n.wrapping_add(1);
if n % log_every == 0 {
    defmt::info!(
        "motor-test RX [bitbang]: M1={=?} M2={=?} M3={=?} M4={=?}",
        telem[0], telem[1], telem[2], telem[3],
    );
}
```

- [ ] **Step 3: Build and test**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu && DRIVER=bitbang cargo build --release --features motor-test`
Expected: pass and build.

- [ ] **Step 4: Bench-verify telemetry**

```bash
DRIVER=bitbang BIDIR=1 LOOP_KHZ=2 ./scripts/flash-motor-test.sh
```

Expected: `motor-test RX [bitbang]: M1=Erpm { period_us: … }` at ~10 Hz, and motors spinning at 5%.

`Erpm { period_us: 0 }` on all four means the ESC is replying "not spinning" — valid frames, so the protocol works and the throttle is the separate question. `InvalidGcr` or `InvalidCrc` means samples arrive but the reconstruction is off: capture the raw samples via the Task 4 probe and check the run lengths against the 3-samples-per-bit expectation before touching the decoder, which is unit-tested.

- [ ] **Step 5: Commit and journal**

```bash
git add src/drivers/dshot_bitbang.rs src/motor_test.rs
git commit -m "dshot-bb: decode telemetry replies to eRPM

Bidirectional DShot working end to end on the bit-banged driver."
```

Then append a `PROJECT_STATUS.md` journal entry recording what worked, the measured reply timing, and any deviation from the values in this plan.

---

### Task 6: Cutover (gated on Task 5 passing on hardware)

**Do not start this task until Task 5 shows decoded eRPM on the bench.**

**Files:**
- Modify: `src/main.rs`, `CLAUDE.md`
- Delete: `src/drivers/dshot_hw.rs`, `src/drivers/dshot_diag.rs`

- [ ] **Step 1: Switch the flight firmware to the bit-banged driver**

In `src/main.rs`, replace the `DshotQuad::new(...)` construction with `DshotBitbang::new(p.TIM1, p.DMA2_CH2, p.PA0, p.PA1, p.PA2, p.PA3, DSHOT_BIDIR)` and update `control_loop`'s parameter type. Set `DSHOT_BIDIR = true`.

- [ ] **Step 2: Verify both protocols on hardware**

Flash the flight firmware and confirm motors arm and spin, then confirm eRPM telemetry appears. This is the last point at which reverting is one commit.

- [ ] **Step 3: Remove the timer-DMA driver**

```bash
git rm src/drivers/dshot_hw.rs src/drivers/dshot_diag.rs
```

Remove their `pub mod` lines from `src/main.rs`'s `mod drivers { … }` block and the `DRIVER` selection from `motor_test.rs` (there is only one driver now). Keep `dshot_frame.rs` — the frame encoder is shared.

- [ ] **Step 4: Correct the documentation**

`CLAUDE.md`'s "Where things live" no longer matches. Update the `src/drivers/` reference. Add a line to the durable rules:

```markdown
- **DShot is bit-banged, not timer output-compare.** BF resolves
  `dshot_bitbang = AUTO` to bit-banging on everything after F4, so the
  timer-DMA path (`pwm_output_dshot_hal.c`) is not the reference for this
  board — `dshot_bitbang.c` is. A week was lost in July 2026 porting the
  wrong one.
```

- [ ] **Step 5: Run everything and commit**

```bash
cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu
cargo build --release && cargo build --release --features motor-test
git add -A
git commit -m "dshot: retire timer-DMA driver in favour of bit-banging

Bidirectional DShot works on the bit-banged driver, so dshot_hw.rs and its
diagnostics are removed. dshot_frame.rs stays — the frame encoder is shared.
CLAUDE.md records why the timer path is not the reference on H7."
```

---

## Risks and open questions

| Risk | Mitigation |
|---|---|
| Embassy's timer-update-DMA binding differs from what Task 3 Step 1 assumes | Step 1 is an explicit probe-and-record before any driver code is written |
| TIM1 needed elsewhere later | TIM3/TIM4/TIM8 are also unclaimed; the pacer choice is one constant and one `Peri` argument |
| D-cache coherency on H7 | Buffers are `#[repr(C, align(32))]`; if corruption appears, add explicit cache maintenance around the transfers rather than disabling the cache |
| DMA2 cannot reach the buffers' memory region | `static mut` lands in `.bss` (AXI SRAM, `0x2400_0000`), which DMA2 can reach on H7. If not, relocate as `dshot_hw.rs` does with fixed SRAM1 addresses |
| The reply never arrives even with a correct waveform | Task 4's success criterion is *transitions captured*, which separates "our TX is wrong" from "our decode is wrong" before any decoder debugging |

## What this plan does not do

- Does not touch `dshot_hw.rs` before Task 6.
- Does not implement DShot150/600 — `TX_ARR`/`RX_ARR` are constants, and adding rates is a follow-up once DShot300 works.
- Does not implement the RPM filter that symonb's repository pairs with this. Telemetry lands in `PROJECT_STATUS` as a post-Alpha item.
