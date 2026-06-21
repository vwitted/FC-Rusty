# DShot Motor-Test Mode — Design

**Date:** 2026-06-21
**Status:** Approved (pending spec review)

## Goal

A compile-time-gated, fully decoupled firmware mode that drives the DShot
driver directly with hard-set per-motor throttles, for bench bring-up and
debugging the bidirectional-DShot failure. It bypasses arming, RC, the PID,
the mixer, and every flight/estimation task — the only thing running is a
minimal loop calling the DShot driver.

It is **not** part of the normal firmware: it exists only when the
`motor-test` cargo feature is enabled, and the feature flag is the
deliberate-flash interlock.

## Requirements

- Spin any subset of the four motors at arbitrary, independently-set
  throttles (e.g. only M1 at 10%). A motor set to 0 stays stopped.
- Bidir DShot on/off selectable.
- Loop frequency selectable (to surface rate-dependent DShot behaviour).
- All of the above set at **build time** — no source edits to retune.
- A hard safety cap and a props-off boot countdown, since nothing else
  (arming, RC) guards motor output in this mode.
- Bidir telemetry / eRPM decode logged, since observing it is the point.

## Approach

A cargo feature `motor-test` gates an early branch in `main`, immediately
after `embassy_stm32::init(config)`:

```rust
let p = embassy_stm32::init(config);

#[cfg(feature = "motor-test")]
{
    motor_test::run(p).await; // -> !, never returns; flight tasks never spawn
}

#[cfg(not(feature = "motor-test"))]
{
    // ... existing flight stack: spawn IMU/MEKF/nav/control/GPS/baro ...
}
```

Chosen over a separate `[[bin]]` target so the mode reuses the exact clock
and peripheral initialisation (`config` + `embassy_stm32::init`) rather than
duplicating that boilerplate and risking clock-config drift.

`run` consumes the four DShot peripherals it needs (`p.TIM2`, `p.PA0..PA3`,
`DMA1_CH2/3/4/7`); every other peripheral in `p` is simply left unused.

## Module structure — `src/motor_test.rs`

Split into a pure, host-tested config layer and a firmware run loop.

```rust
pub struct MotorTestConfig {
    pub motor_pct: [u8; 4], // post-clamp, 0..=MAX_PCT; 0 = stopped
    pub bidir: bool,
    pub loop_khz: u8,       // 2..=8
}

/// Compile-time safety ceiling. Raising it requires a deliberate source
/// edit + reflash — that is intentional.
const MAX_PCT: u8 = 25;
const DEFAULT_LOOP_KHZ: u8 = 8;
const DEFAULT_BIDIR: bool = true;

/// Pure parse + clamp. Host-tested. Separates logic from env IO so tests
/// can drive arbitrary inputs.
fn parse_config(
    m: [Option<&str>; 4],
    bidir: Option<&str>,
    loop_khz: Option<&str>,
) -> MotorTestConfig { /* parse ints, default 0 / DEFAULT_*, clamp */ }

/// Reads the baked-in env values and delegates to `parse_config`.
pub fn resolve_config() -> MotorTestConfig {
    parse_config(
        [option_env!("M1_PCT"), option_env!("M2_PCT"),
         option_env!("M3_PCT"), option_env!("M4_PCT")],
        option_env!("BIDIR"),
        option_env!("LOOP_KHZ"),
    )
}

#[cfg(feature = "firmware")]
pub async fn run(p: embassy_stm32::Peripherals) -> ! { /* see Behaviour */ }
```

Module declaration:
- `src/lib.rs`: `#[cfg(any(feature = "motor-test", test))] pub mod motor_test;`
  — so the host tests compile/run under the default `cargo test --lib`
  invocation (the `test` cfg), without dragging the module into normal
  firmware builds (no dead-code warnings).
- `src/main.rs`: `#[cfg(feature = "motor-test")] mod motor_test;`

Under `cargo test --lib --no-default-features` the `firmware`-gated `run`
is not compiled, so only the pure config layer + tests build (no embassy
dependency on the host path).

## Configuration (all build-time)

| Env var | Meaning | Default | Bounds |
|---|---|---|---|
| `M1_PCT`..`M4_PCT` | Per-motor throttle percent | `0` (off) | clamped to `0..=MAX_PCT` (25) |
| `BIDIR` | Bidir DShot (`0`/`1`) | `1` | — |
| `LOOP_KHZ` | Send-loop frequency, kHz | `8` | clamped to `2..=8` (integer ⇒ 1 kHz steps) |

Parsing is at startup (the strings are baked by `option_env!`; integers are
parsed at runtime in `parse_config`). Unparseable or empty → the default.

Example: `M1_PCT=10 LOOP_KHZ=4 BIDIR=1 cargo build --release --features motor-test`
→ spins only M1 at 10%, 4 kHz, bidir on.

## Behaviour (`run`)

1. `let cfg = resolve_config();`
2. defmt warning + props-off banner:
   `⚠ MOTOR TEST — REMOVE PROPS. Spinning in 5s: M1=x% M2=y% M3=z% M4=w% bidir=b loop=k kHz`
   (post-clamp values).
3. ~5 s countdown, LED blinking as an "alive" indicator.
4. `DshotQuad::new(p.TIM2, p.PA0..p.PA3, DMA…, DshotSpeed::Dshot600, cfg.bidir)`.
5. Loop on `Ticker::every(1000 / loop_khz µs)`:
   - Build `[DshotFrame; 4]` via `DshotFrame::from_normalised(pct/100.0, bidir)`
     (0% ⇒ `MotorStop`).
   - `send_throttles_and_receive(frames)`.
   - Log the per-motor telemetry result (`Erpm`/`NoEdge`/`InvalidGcr`/
     `InvalidCrc`) at ~10 Hz, reusing the existing telemetry-log pattern.
   - Never exits.

## Safety

- Every motor clamped to `MAX_PCT` (25%) at config time.
- 5 s props-off countdown before any frame is sent.
- The `motor-test` feature flag — normal builds can never enter this mode.
- No arming, no RC, no failsafe interaction; fully standalone.

## Testing

- **Host unit tests** on `parse_config`:
  - all-unset → all motors 0, bidir default, loop 8 kHz;
  - per-motor values parsed independently;
  - over-cap value (e.g. `90`) clamps to `MAX_PCT`;
  - `LOOP_KHZ` below/above range clamps to `2`/`8`; non-step is moot
    (integer kHz);
  - garbage / empty strings fall back to defaults.
- **Bench verification** (props off): flash with chosen env vars, confirm
  the named motor(s) spin at the expected throttle, the countdown fires,
  and telemetry decodes (or doesn't) as logged.

## Out of scope (YAGNI)

Arming, PID, mixer, IMU/MEKF, navigation, GPS, baro, failsafe, RC input,
and any runtime adjustment of values. All values are fixed at build time.

## Files changed

- `Cargo.toml` — add `[features] motor-test = []`.
- `src/motor_test.rs` — new module (config layer + run loop + tests).
- `src/main.rs` — `#[cfg(feature = "motor-test")] mod motor_test;` + the
  post-init cfg branch.
- `src/lib.rs` — `#[cfg(any(feature = "motor-test", test))] pub mod motor_test;`.
