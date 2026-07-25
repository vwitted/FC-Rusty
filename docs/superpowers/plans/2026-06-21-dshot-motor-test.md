# DShot Motor-Test Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A compile-time-gated (`motor-test` cargo feature) firmware mode that drives the DShot driver directly with build-time per-motor throttles, fully decoupled from arming and the flight stack, for bench bring-up / debugging bidir DShot.

**Architecture:** A pure, host-tested config layer parses build-time env vars (`M1_PCT`..`M4_PCT`, `BIDIR`, `LOOP_KHZ`) into a clamped `MotorTestConfig`. A firmware-only `run()` does a props-off countdown then loops, driving the DShot driver at the configured rate and logging bidir telemetry. The clock config is extracted to a shared `board_config()`; two mutually-`cfg`'d `main` functions select flight vs motor-test so neither path fights over peripherals.

**Tech Stack:** Rust, `no_std`, Embassy (async), `embassy-stm32` (STM32H743), defmt logging. Spec: `docs/superpowers/specs/2026-06-21-dshot-motor-test-design.md`.

---

## File Structure

- **`Cargo.toml`** — add `motor-test = []` feature.
- **`src/motor_test.rs`** (new) — config layer (`MotorTestConfig`, `parse_config` [pure, host-tested], `resolve_config` [env wrapper]) + firmware `run()` loop.
- **`src/lib.rs`** — declare the module for host tests: `#[cfg(any(feature = "motor-test", test))] pub mod motor_test;`.
- **`src/main.rs`** — extract `board_config()`; `#[cfg(not(feature = "motor-test"))]` on the existing `main`; add a small `#[cfg(feature = "motor-test")]` `main`; declare the module.

Commands used throughout:

- Host tests: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu motor_test`
- Flight firmware (unchanged): `cargo build --release`
- Motor-test firmware: `cargo build --release --features motor-test` (optionally `M1_PCT=8 LOOP_KHZ=4 BIDIR=1 …`)

---

### Task 1: Config layer (`parse_config`) — pure, host-tested

**Files:**

- Modify: `Cargo.toml` (`[features]`)
- Modify: `src/lib.rs` (module declaration)
- Create: `src/motor_test.rs`

- [ ] **Step 1: Add the cargo feature**

In `Cargo.toml`, directly after the `firmware = [ … ]` block, add:

```toml
# Bench motor-test firmware (see docs/superpowers/specs/2026-06-21-dshot-motor-test-design.md).
# Build with `cargo build --release --features motor-test`.
motor-test = []
```

- [ ] **Step 2: Declare the module for host tests**

In `src/lib.rs`, add at top level (alongside the other `pub mod` declarations):

```rust
#[cfg(any(feature = "motor-test", test))]
pub mod motor_test;
```

- [ ] **Step 3: Create `src/motor_test.rs` with the config types, a STUB `parse_config`, and the tests**

```rust
//! Bench motor-test mode.
//!
//! Drives the DShot driver directly with build-time per-motor throttles,
//! fully decoupled from arming and the flight stack. Compiled only under
//! the `motor-test` cargo feature (and during host tests).
//!
//! Spec: docs/superpowers/specs/2026-06-21-dshot-motor-test-design.md

/// Resolved, clamped motor-test configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorTestConfig {
    /// Per-motor throttle percent, post-clamp. 0 = stopped.
    pub motor_pct: [u8; 4],
    /// Bidirectional DShot.
    pub bidir: bool,
    /// Send-loop frequency in kHz (2..=8).
    pub loop_khz: u8,
}

/// Hard safety ceiling on any motor's throttle. Raising it requires a
/// deliberate source edit + reflash — intentional.
const MAX_PCT: u8 = 25;
const DEFAULT_BIDIR: bool = true;
const DEFAULT_LOOP_KHZ: u8 = 8;
const MIN_LOOP_KHZ: u8 = 2;
const MAX_LOOP_KHZ: u8 = 8;

/// Parse + clamp raw string inputs into a `MotorTestConfig`. Pure (no env
/// IO) so it can be unit-tested with arbitrary inputs.
fn parse_config(
    motor: [Option<&str>; 4],
    bidir: Option<&str>,
    loop_khz: Option<&str>,
) -> MotorTestConfig {
    // STUB — deliberately wrong so the tests fail first (RED).
    let _ = (motor, bidir, loop_khz);
    MotorTestConfig {
        motor_pct: [99, 99, 99, 99],
        bidir: false,
        loop_khz: 0,
    }
}

/// Read the build-time env values and resolve them into a clamped config.
pub fn resolve_config() -> MotorTestConfig {
    parse_config(
        [
            option_env!("M1_PCT"),
            option_env!("M2_PCT"),
            option_env!("M3_PCT"),
            option_env!("M4_PCT"),
        ],
        option_env!("BIDIR"),
        option_env!("LOOP_KHZ"),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_when_all_unset() {
        let c = parse_config([None; 4], None, None);
        assert_eq!(c.motor_pct, [0, 0, 0, 0]);
        assert!(c.bidir);
        assert_eq!(c.loop_khz, 8);
    }

    #[test]
    fn parses_each_motor_independently() {
        let c = parse_config([Some("3"), None, Some("7"), None], None, None);
        assert_eq!(c.motor_pct, [3, 0, 7, 0]);
    }

    #[test]
    fn clamps_motor_to_max_pct() {
        let c = parse_config([Some("90"), Some("25"), Some("26"), Some("0")], None, None);
        assert_eq!(c.motor_pct, [25, 25, 25, 0]);
    }

    #[test]
    fn bidir_explicit_zero_one_else_default() {
        assert!(!parse_config([None; 4], Some("0"), None).bidir);
        assert!(parse_config([None; 4], Some("1"), None).bidir);
        assert!(parse_config([None; 4], Some("yes"), None).bidir); // garbage → default true
    }

    #[test]
    fn loop_khz_clamped_to_range() {
        assert_eq!(parse_config([None; 4], None, Some("1")).loop_khz, 2);
        assert_eq!(parse_config([None; 4], None, Some("9")).loop_khz, 8);
        assert_eq!(parse_config([None; 4], None, Some("4")).loop_khz, 4);
        assert_eq!(parse_config([None; 4], None, Some("x")).loop_khz, 8); // garbage → default
    }
}
```

- [ ] **Step 4: Run the tests to verify they FAIL**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu motor_test`
Expected: FAIL — e.g. `defaults_when_all_unset` panics (`left: [99, 99, 99, 99]`, `right: [0, 0, 0, 0]`).

- [ ] **Step 5: Replace the STUB `parse_config` with the real implementation**

```rust
fn parse_config(
    motor: [Option<&str>; 4],
    bidir: Option<&str>,
    loop_khz: Option<&str>,
) -> MotorTestConfig {
    let motor_pct = core::array::from_fn(|i| {
        motor[i]
            .and_then(|s| s.trim().parse::<u8>().ok())
            .unwrap_or(0)
            .min(MAX_PCT)
    });
    let bidir = match bidir.map(|s| s.trim()) {
        Some("0") => false,
        Some("1") => true,
        _ => DEFAULT_BIDIR,
    };
    let loop_khz = loop_khz
        .and_then(|s| s.trim().parse::<u8>().ok())
        .unwrap_or(DEFAULT_LOOP_KHZ)
        .clamp(MIN_LOOP_KHZ, MAX_LOOP_KHZ);
    MotorTestConfig { motor_pct, bidir, loop_khz }
}
```

- [ ] **Step 6: Run the tests to verify they PASS**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu motor_test`
Expected: PASS — `5 passed`.

- [ ] **Step 7: Commit**

```bash
git add Cargo.toml src/lib.rs src/motor_test.rs
git commit -m "motor-test: build-time config parser (host-tested)"
```

---

### Task 2: Firmware run loop

**Files:**

- Modify: `src/motor_test.rs` (append `run()`)

No host test — `run()` drives hardware and is firmware-only. Verification is a successful feature build; behaviour is bench-checked in Task 4.

- [ ] **Step 1: Append the firmware `run()` to `src/motor_test.rs`** (after `resolve_config`, before the `tests` module)

```rust
/// Bench motor-test entry point. Never returns. Drives the DShot driver
/// directly from the build-time config; no arming, RC, PID, or mixer.
#[cfg(feature = "firmware")]
pub async fn run(p: embassy_stm32::Peripherals) -> ! {
    use crate::drivers::dshot_frame::{DshotFrame, DshotSpeed};
    use crate::drivers::dshot_hw::DshotQuad;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_time::{Duration, Ticker, Timer};

    let cfg = resolve_config();

    defmt::warn!(
        "MOTOR TEST — REMOVE PROPS. Spinning in 5s: M1={}% M2={}% M3={}% M4={}% bidir={} loop={}kHz",
        cfg.motor_pct[0], cfg.motor_pct[1], cfg.motor_pct[2], cfg.motor_pct[3],
        cfg.bidir, cfg.loop_khz,
    );

    // Countdown with LED blink (PD10, active-low on the DAKEFPV) as an
    // "alive" indicator before any frame is sent.
    let mut led = Output::new(p.PD10, Level::High, Speed::Low);
    for s in (1..=5).rev() {
        defmt::info!("motor-test: spinning in {}s", s);
        for _ in 0..5 {
            led.toggle();
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    let mut dshot = DshotQuad::new(
        p.TIM2,
        p.PA0, p.PA1, p.PA2, p.PA3,
        p.DMA1_CH2, p.DMA1_CH3, p.DMA1_CH4, p.DMA1_CH7,
        DshotSpeed::Dshot600,
        cfg.bidir,
    );

    // Per-motor frames are constant for the run; 0% maps to MotorStop.
    let frames: [DshotFrame; 4] = core::array::from_fn(|i| {
        DshotFrame::from_normalised(cfg.motor_pct[i] as f32 / 100.0, cfg.bidir)
    });

    let mut ticker = Ticker::every(Duration::from_micros(1000 / cfg.loop_khz as u64));
    let log_every: u32 = cfg.loop_khz as u32 * 100; // ~10 Hz
    let mut n: u32 = 0;

    defmt::info!("motor-test: driving motors");
    loop {
        let telem = dshot.send_throttles_and_receive(frames).await;
        n = n.wrapping_add(1);
        if cfg.bidir && n % log_every == 0 {
            defmt::info!(
                "motor-test RX: M1={=?} M2={=?} M3={=?} M4={=?}",
                telem[0], telem[1], telem[2], telem[3],
            );
        }
        ticker.next().await;
    }
}
```

> **Note for the implementer:** confirm `send_throttles_and_receive` takes `[DshotFrame; 4]` by value (it does in `main.rs` — `DshotFrame` is `Copy`) and returns a 4-element telemetry array logged with `{=?}`. Match the call shape at `src/main.rs` (the existing armed control loop) if anything differs.

- [ ] **Step 2: Verify it compiles under the feature**

Run: `cargo build --release --features motor-test`
Expected: builds (0 errors). `run()` compiles as part of the **lib** target (the module is declared in `lib.rs` from Task 1, and `firmware` is on by default), so this validates it even though the binary's entry point isn't wired until Task 3 — this build still produces the unchanged *flight* binary.

- [ ] **Step 3: Commit**

```bash
git add src/motor_test.rs
git commit -m "motor-test: firmware run loop (countdown + direct DShot drive)"
```

---

### Task 3: Entry-point wiring

**Files:**

- Modify: `src/main.rs` (extract `board_config`; cfg the existing `main`; add the motor-test `main`; declare the module)

- [ ] **Step 1: Extract the clock config into `board_config()`**

In `src/main.rs`, add this free function immediately **above** `#[embassy_executor::main]` (move the body verbatim from the current lines `318`–`349`):

```rust
/// STM32H743 clock tree for the DAKEFPV (8 MHz HSE → 480 MHz SYSCLK).
/// Shared by the flight and motor-test entry points so the clock config
/// can never drift between them.
fn board_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllDiv, PllPreDiv,
        PllSource, Sysclk, VoltageScale,
    };
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL120,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV20),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config
}
```

- [ ] **Step 2: Point the existing `main` at `board_config()` and gate it off under `motor-test`**

In the existing `main`, delete the inline `use embassy_stm32::rcc::{…};` + the `let mut config = …;` block through `config.rcc.voltage_scale = …;` (the lines now living in `board_config`), and replace `let p = embassy_stm32::init(config);` with:

```rust
    let p = embassy_stm32::init(board_config());
```

Then add the gate directly above its attribute:

```rust
#[cfg(not(feature = "motor-test"))]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(board_config());
    // ... rest of the existing flight main, unchanged ...
}
```

- [ ] **Step 3: Add the motor-test `main` and the module declaration**

Add the module declaration near the other top-level `mod` declarations in `src/main.rs`:

```rust
#[cfg(feature = "motor-test")]
mod motor_test;
```

And add the second entry point (immediately after the flight `main`):

```rust
#[cfg(feature = "motor-test")]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(board_config());

    // D-cache off for DMA coherency (DShot is DMA-driven).
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.SCB.disable_dcache(&mut core.CPUID);

    // defmt over USART6 (PC6), same as the flight path.
    logger::init_usart6();

    motor_test::run(p).await;
}
```

- [ ] **Step 4: Verify both builds compile**

Run: `cargo build --release`
Expected: builds (0 errors); flight firmware unchanged.

Run: `cargo build --release --features motor-test`
Expected: builds (0 errors); produces the motor-test binary.

- [ ] **Step 5: Verify host tests still pass and the flight build is warning-clean**

Run: `cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`
Expected: PASS (includes the 5 `motor_test` tests → 136 total).

Run: `cargo build --release 2>&1 | grep -c '^warning'`
Expected: same count as before this feature (no *new* warnings; compare against `git stash` baseline if unsure).

- [ ] **Step 6: Commit**

```bash
git add src/main.rs
git commit -m "motor-test: feature-gated entry point (shared board_config)"
```

---

### Task 4: Bench verification + docs

**Files:**

- Modify: `PROJECT_STATUS.md` (note the new bench tool, per the journal policy)

- [ ] **Step 1: Bench smoke test (props OFF)**

Flash a single-motor, low-throttle build and observe:

```bash
M1_PCT=6 LOOP_KHZ=8 BIDIR=1 cargo build --release --features motor-test
./scripts/flash-dfu.sh
```

Expected on the defmt console (USART6 / PC6):

- The `MOTOR TEST — REMOVE PROPS …` banner with the resolved values.
- A 5 s countdown (LED blinking).
- `motor-test: driving motors`, then ~10 Hz `motor-test RX:` lines.
- Only M1 spins (~6%); M2–M4 stay stopped.

Then sanity-check a second config (e.g. `M3_PCT=8 LOOP_KHZ=4 BIDIR=0`) spins only M3 and changes the cadence.

- [ ] **Step 2: Note the tool in `PROJECT_STATUS.md`**

Add a short bullet under the appropriate section (e.g. tooling / bench notes):

```markdown
- **Motor-test firmware** (`--features motor-test`): decoupled DShot bench
  driver. Per-motor throttle / bidir / loop-freq via build-time env vars
  (`M1_PCT`..`M4_PCT`, `BIDIR`, `LOOP_KHZ`), 25% hard cap, props-off
  countdown. No arming/RC/flight stack. See
  `docs/superpowers/specs/2026-06-21-dshot-motor-test-design.md`.
```

- [ ] **Step 3: Commit**

```bash
git add PROJECT_STATUS.md
git commit -m "motor-test: document the bench tool in PROJECT_STATUS"
```

---

## Notes / Risks

- **Peripheral ownership:** the two `main`s are mutually exclusive via `cfg`, so `p.TIM2`/`PA0..3`/`DMA1_CH2/3/4/7`/`PD10` are each used in exactly one compiled path — no double-move.
- **`--features motor-test` keeps `firmware` on** (it adds to the default set). `run()` is `cfg(feature = "firmware")`, so it compiles in that build. A nonsensical `--no-default-features --features motor-test` has no `main` and is not a supported invocation.
- **Telemetry call shape:** if `send_throttles_and_receive` / the telemetry tuple differs from what Task 2 assumes, mirror the existing call in `src/main.rs`'s armed control loop.
