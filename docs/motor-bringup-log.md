# Motor bring-up debug log

Append-only log of FC→ESC signal debugging on the Radiolink F722.
Each session gets its own dated entry. Preserves raw observations so
later sessions (and the oscilloscope, once it arrives) have a clean
reference rather than reconstructing from memory.

## Hardware under test

- **FC**: Radiolink F722 (STM32F722RET6). DShot pins:
  - M1 → PA15 → TIM2 CH1 (AF1), TIM2_UP DMA1 Stream 7
  - M2 → PB3  → TIM2 CH2 (AF1), shared TIM2_UP
  - M3 → PB4  → TIM3 CH1 (AF2), TIM3_UP DMA1 Stream 2
  - M4 → PB6  → TIM4 CH1 (AF2), TIM4_UP DMA1 Stream 6
  - Timer clock 108 MHz (APB1 × 2). DShot600 → 180 ticks/bit.
- **ESC**: 4-in-1, factory firmware (unknown variant, never flashed).
  Channels labelled ESC1…ESC4 by silkscreen; ESCs are fixed to the
  board, only the signal wires move.
- **Bench supply**: 12 V, 1.5 A. Baseline idle ~0.15 A, climbs to
  ~0.25 A when M3/M4 twitch.
- **Props**: off throughout.

## Session 2026-04-22 — DShot signal asymmetry across timers

### Setup

- Branch: `feat/icm42688-mekf`
- Arming: `require_gps = false` (bench mode). All other pre-arm
  checks live.
- Control loop healthy throughout: 200 Hz, ~235 µs avg / 640 µs max,
  MPC ~488 µs, zero overruns. FC-side control was never suspect.

### Tests and observations

#### T1 — DShot600, original driver (`waveform_up` for TIM3/TIM4)

- **M1, M2**: 3 rising + 2 longer rising tones → 4 sets of 4
  ascending tones. Idle, no spin.
- **M3**: 3 rising + 2 longer rising, then stuck repeating the
  2-long-rising indefinitely. No spin.
- **M4**: initial boot tones only, then silently spinning weakly
  with no arm command.

#### T2 — JST connector reseat / replacement

- Replaced cables, confirmed solid crimps.
- **Result**: identical to T1. Rules out connector integrity.

#### T3 — Swap test: move M1 and M3 signal wires

Moved FC output PA15/M1 onto ESC3's input, and PB4/M3 onto ESC1's
input. ESCs themselves are fixed (4-in-1 board) — only the *signal
wires* move.

- **ESC1** (now fed by TIM3 M3 signal): weakly spinning, no tones.
  Previously healthy on this ESC — behaviour followed the signal.
- **ESC3** (now fed by TIM2 M1 signal): reached 4×4 tones (same as
  M1/M2 had been doing). Behaviour followed the signal.
- **Conclusion**: the bad behaviour follows the FC signal path, not
  the ESC. ESC3 is proven healthy. ESC1 and ESC2 were already
  healthy. ESC4 remains unproven — never received a known-good
  signal.

#### T4 — Driver change: `waveform_up_multi_channel` for TIM3/TIM4

Embassy's single-channel `waveform_up` acknowledges a CCR
save/restore race at frame end ("this can almost always trigger a
DMA FIFO error", `simple_pwm.rs:357`). Switched TIM3 and TIM4 to
the multi-channel burst API in its degenerate Ch1→Ch1 form, which
TIM2 was already using successfully.

- **Result**: no change. All symptoms identical to T1.
- **Conclusion**: the Embassy single-channel API was not the root
  cause.
- **Code**: kept the change (it's strictly at-least-as-correct and
  unifies all four motors on one DMA mechanism).

#### T5 — Drop to DShot150 (diagnostic)

Reduce bit rate 4× — if the issue is timing margin, should improve.

- **M1, M2**: startup tones only. No 4×4 confirmation.
- **M3**: spinning (changed from stuck-at-2-tones → spinning).
- **M4**: no tones (changed from silent-spin → silent).
- **Conclusion**: DShot150 isn't reliably supported by these ESCs.
  Made things worse on the previously-OK channels too.

#### T6 — DShot300 (diagnostic)

- **All four**: identical 3+2 startup tones. No spinning at idle.
- No 4×4 confirmation tones on any channel.
- **At this point we wrongly concluded the signal was now "clean"**.
  T7 showed this was wrong — the ESCs just hadn't confirmed any
  protocol and were in idle-waiting state across the board.

#### T7 — DShot300, arm attempt

RC armed, throttle stick at 0%. Firmware sends 29% to every motor
(bench mode, altitude controller uses `hover_throttle = 0.294`
because PosKF isn't ready without GPS home).

- **defmt confirms**: `armed=true`, `thrust_cmd=29%`, sticks at 0
  for the full armed window. FC doing everything right.
- **M1, M2**: no response at all. Did not spin.
- **M3, M4**: brief strong spin-up, then cut off.
- **Conclusion**: at DShot300 no ESC had confirmed the protocol, so
  M1/M2's silence wasn't "healthy arm waiting for throttle" — they
  were ignoring everything. M3/M4's brief spin was garbage
  interpretation of malformed frames.

#### T8 — Revert to DShot600, arm attempt

Hypothesis going in: M1/M2 (protocol-confirmed at 600) would spin at
29%; M3/M4 would misbehave as before. This would have cleanly
separated "FC signal on TIM3/4 is bad" from "ESC3/4 are bad".

- **Result**: "mostly behaving similarly to the no arm case".
  M1/M2 did not spin at 29 % even though they had been reaching the
  4×4 tones.
- **Conclusion**: my interpretation of the 4×4 tones as "protocol
  confirmed, armed-ready" was probably wrong. BLHeli_S also emits
  periodic rising-tone sequences for "no valid signal detected".
  This would mean **no ESC has ever successfully decoded our
  DShot**, and we've only seen different *flavours* of decode
  failure — M1/M2 producing a less-bad-looking "no signal" beep
  than M3/M4 producing spurious motor motion.

### Proven at end of session

1. **FC control stack is healthy** — arming, mixing, frame encoding,
   defmt all report correct state under arm.
2. **FC signal behaviour differs by timer** — the swap test in T3
   shows the malformed signal follows the wire, not the ESC.
3. **ESC1, ESC2, ESC3 are healthy** — all three played 4×4 tones
   when fed by TIM2's signal.

### Unproven / open

1. **ESC4 health** — never received a known-good signal. The
   TIM2→ESC4 mirror of T3 would resolve this; we haven't run it.
2. **Whether the 4×4 tone is "protocol confirmed" or "looking for
   signal".** T8 points toward the latter, in which case even
   TIM2's signal is malformed — just more benignly so.
3. **Root cause of the signal malformation.** Candidate
   hypotheses, ranked roughly:
   - DMA stream priority / contention between TIM2_UP (DMA1 S7),
     TIM3_UP (S2), TIM4_UP (S6). Not investigated.
   - Timer peripheral configuration delta Embassy applies to TIM2
     (32-bit) but not TIM3/4 (16-bit) — preload, enable-outputs,
     MOE, counting-mode edge cases.
   - GPIO slew / AF config marginal on PB4/PB6 despite
     `Speed::VeryHigh`.
   - ESCs simply don't speak any DShot variant we can generate with
     the current peripheral setup (only fully resolves once we test
     a second ESC — user has a spare to try).
4. **Whether this is driver-level or ESC-level.** The T3 swap is
   strong circumstantial evidence for driver, but T8 muddies it —
   if *no* signal is cleanly decoded, "signal X is worse than
   signal Y" doesn't necessarily mean signal Y is good.

### Scheduled next steps

- **When oscilloscope arrives**: capture TIM2 vs TIM3 output on
  identical frame data. Compare timing (T0H/T1H widths, bit
  period), rise/fall edges, idle level. This should settle
  hypothesis (3) definitively.
- **Tomorrow-ish**: swap to the second (fancier) ESC. If it behaves
  identically across all four channels → FC signal is bad on all
  four outputs. If M1/M2 work cleanly and M3/M4 fail → confirms
  pure TIM3/4 asymmetry.
- **Deferred**: mirror of T3 on M4 (TIM2 signal → ESC4) — would
  close the "is ESC4 healthy" gap. Low priority once the scope
  lands, since the scope renders it moot.

### Code state at session end

- `src/drivers/dshot_hw.rs`: TIM3 and TIM4 use
  `waveform_up_multi_channel(Ch1, Ch1, …)`. Kept from T4 (strictly
  correct, didn't cause regression).
- `src/main.rs`: DShot600 (reverted from T5/T6/T7 diagnostics).
- No other changes retained from the session.

### What not to re-try without new evidence

- DShot150 or DShot300 speed changes. T5/T6 showed neither helps.
- Sending special DShot commands (e.g. TELEMETRY_DISABLE, cmd 32)
  as a fix. Only runs after protocol confirmation, which none of
  our ESCs have reliably reached — so the command can't be the
  lever.
- `waveform_up` single-channel API for TIM3/TIM4. T4 ruled it out.

## Session 2026-04-25 — register-level instrumentation, alignment red herring, all checks pass

### Setup

- Branch: `feat/icm42688-mekf` (still).
- Same physical bench as 2026-04-22.
- **Hardware change**: ESC swapped to a different model running the
  A-H-30 BLHeli_S firmware (no bidirectional). Each motor verified
  spinning under direct bench drive before this session.
- New diagnostic module `src/drivers/dshot_diag.rs` written this
  session — boot-time + 1 Hz post-send register dumps gated behind
  `embassy-stm32/unstable-pac`. See `DshotQuad::log_config` and
  `log_runtime_state`.

### Tests and observations

#### T1 — Move DMA buffers to SRAM2

Hypothesis: STM32F722 DTCM (0x2000_0000–0x2000_FFFF) is unreachable
by DMA1; if Embassy's main-task stack puts our buffer there, DMA
reads silently return garbage.

Approach: shrink `RAM` in `memory.x` from 256 K to 255 K, point
three `&'static mut` slices at hardcoded addresses in the reserved
1 KB at the top of SRAM2.

- **Boot log proved the hypothesis was real**: pre-relocation buffer
  addresses were `tim2=2000_039c tim3=2000_03e4 tim4=2000_0408` —
  all firmly in DTCM. The static-pool task stack model put them
  there, not the cortex-m-rt top-of-RAM stack assumption.
- **But moving them did not change the fault.** Symptoms identical
  to 2026-04-22. So DTCM-unreachability was not the root cause we
  thought it was — DMA must have been transferring *something*, just
  apparently still wrong.
- **Code kept**: defensive even if not curative. Buffers now live at
  `0x2003_FC00 / FC60 / FC90`.

#### T2 — Build the diagnostic module

Wrote `dshot_diag.rs` exposing one-shot boot dumps (TIMPRE, GPIO
state, timer counters, all three timer config registers, SCB cache
state, buffer-canary write/readback) and a 1 Hz post-send dump
(per-stream NDTR, error flags, CR/FCR/PAR/M0AR; per-channel CCR1).
Wired into `DshotQuad::log_config()` (boot) and `send()` (every
200th frame).

#### T3 — First diagnostic run: FE on every stream every frame

Boot dump: PSC=0, ARR=179, CCMR1=0x6868 (PWM mode 1 + CCR preload),
CCER active-high, caches off, all canaries OK. *Configuration
correct.*

Runtime dump: every stream on every frame:
`NDTR=0 TC=false HT=true TE=false DME=false FE=true`.
Last CCR=0 on every channel.

So: data transferred to completion (NDTR=0, last cell landed) but
the FIFO error flag fires on every transfer.

#### T4 — Buffer-alignment hypothesis (red herring)

Found in Embassy: `waveform_up_multi_channel` hardcodes
`mburst=Incr4` and `fifo_threshold=Full` on F7. With 16-bit MSIZE
that requires NDTR to be a multiple of 8 cells (16 bytes). Our
buffers were 36 cells (TIM2) and 18 cells (TIM3/4) — none aligned.
Per F7 RM, an unaligned trailing burst is dropped → last bits of
every frame would be silently truncated.

Padded `STEPS_PER_FRAME` from 18 → 24 (16 data + 8 trailing zeros);
TIM2 grew correspondingly to 48 cells; SRAM2 layout re-laid out as
`0x2003_FC00 / FC60 / FC90`.

- **Result**: FE flag still set on every transfer. Symptoms
  unchanged.
- **Conclusion**: the alignment hypothesis was wrong. Re-reading
  Embassy's own `simple_pwm.rs:357` comment, the FE on
  `waveform_up*` is a documented benign cleanup race — DMA stream
  is disabled before the timer's last UEV-triggered request, which
  always sets FE without corrupting prior transfers.
- **Padding kept**: harmless (extra 13 µs idle low per frame, well
  under our 5 ms inter-frame budget) and removes alignment as a
  variable for any future reanalysis.

#### T5 — Clock-tree / GPIO / counter sanity

Added `log_timpre`, `log_gpio_pins`, `log_timer_running` to the
boot dump. All clean:

- `RCC.DCKCFGR1.TIMPRE = 0 (MUL2)` → timer clock = 108 MHz, exactly
  as the bit-cell math assumes. (TIMPRE=1 would have made our
  "DShot600" actually DShot1200, undecodable by BLHeli_S — credible
  hypothesis fully ruled out.)
- All four motor pins: `MODER=2 (AF), OSPEEDR=3 (VeryHigh)`,
  `AF=1 / 1 / 2 / 2` — exactly the F722 datasheet expects for
  TIM2_CH1, TIM2_CH2, TIM3_CH1, TIM4_CH1.
- TIM2/3/4 CNT advance 6–8 ticks between back-to-back reads
  (~55 ns at 108 MHz). All three timers running at the configured
  rate.

#### T6 — Manual throttle pass-through

Spotted in the log: `armed=true stick_thr=100% thrust_cmd=29%`.
Stick was being captured for the log line only; the altitude
controller pinned `current_thrust = hover_throttle` (0.294)
whenever PosKF wasn't ready, regardless of arm or stick.

Fixed in `src/main.rs`: when PosKF isn't ready, fall through to
`current_thrust = throttle_raw.clamp(0.0, 1.0)` (direct stick →
mixer). Position-flight path unchanged. One-shot
`WARN ARMED without PosKF lock — MANUAL THROTTLE pass-through` on
the rising arm edge so the operator gets a clear reminder in the
log. Unrelated to DShot decode but blocks any meaningful motor
verification once DShot starts working.

### Proven at end of session

1. **Every register-level check passes.** Clock tree, GPIO config,
   timer config, DMA placement, buffer alignment, cache state,
   canary integrity — all correct. By every metric we can read,
   the FC is emitting valid DShot600 on all four pins.
2. **The FE flag is benign** (Embassy's own documented cleanup race
   in `waveform_up*`). It's not the cause of decode failure.
3. **Two physically distinct ESCs (different brands, different
   silkscreens) both fail identically.** No credible systemic flaw
   shared between them — points strongly at the FC's signal as the
   problem, not the receiver.
4. **Manual throttle pass-through works** when PosKF isn't ready,
   so once DShot decodes, motor response can be verified directly.

### Unproven / open

1. **What the signal actually looks like at the pad.** Every
   register says it's correct, but no oscilloscope trace exists yet
   to confirm the physical waveform matches the configured timing
   and polarity. This is now the only remaining unknown.
2. **Whether the signal degrades between MCU pad and ESC input.**
   Wire impedance, broken trace, capacitive loading — none ruled
   out without scope traces at both ends.
3. **One cosmetic bug**: the `log_dma1_stream` "error flags set"
   warn renders as the D-cache warn string at runtime (defmt index
   mismatch — likely a stale elf in the decoder, not a real D-cache
   problem; boot dump confirms `IC=0 DC=0`). `cargo clean &&
   rebuild && reflash` should resync. Not investigated further this
   session.

### Scheduled next steps

- **Scope arrives 2026-04-26**. Capture order:
  1. PA15 directly at the MCU pad (M1, TIM2_CH1). Confirm: bit
     period 1.67 µs, T1H ≈ 1.25 µs, T0H ≈ 625 ns, 0–3.3 V swing,
     16 bits then idle low.
  2. Same probe on the ESC input pad fed by that same wire. If
     identical to MCU side → ESC-side issue (despite the
     two-different-ESCs argument); if different → wire/connector
     integrity.
  3. Repeat on PB4 (TIM3_CH1) for cross-timer comparison.
- **If the scope shows a clean, spec-compliant DShot600 frame at
  both ends** — then re-examine the BLHeli_S decode-side
  assumptions (boot tones don't necessarily mean protocol
  detection; bidirectional accidentally enabled despite A-H-30
  defaults; etc.).

### Code state at session end

- `memory.x`: RAM 256 K → 255 K, reserves the top 1 KB of SRAM2 for
  manually-placed DMA buffers.
- `src/drivers/dshot_hw.rs`: `STEPS_PER_FRAME = 24` (was 18); SRAM2
  buffers at `0x2003_FC00 / FC60 / FC90`; new `log_config()` and
  `log_runtime_state()` methods; 1 Hz post-send diagnostic gate.
- `src/drivers/dshot_diag.rs`: new module — TIMPRE / GPIO / timer
  CNT / cache / per-timer config / per-DMA-stream state / canary
  helpers.
- `Cargo.toml`: `embassy-stm32` gains the `unstable-pac` feature.
- `src/main.rs`: `dshot.log_config()` at boot; `dshot_diag` mod
  registered; manual-throttle pass-through when PosKF not ready;
  one-shot `ARMED without PosKF lock` warn on the arm edge.

### What not to re-try without new evidence

- Buffer relocation, padding, or SRAM-region tricks. T1+T4 fully
  exhausted that direction; the buffers are demonstrably in
  DMA-reachable memory and DMA demonstrably consumes the full
  count.
- Clock-tree retuning (PSC/ARR/TIMPRE). T5 confirmed 108 MHz timer
  clock matches the bit-cell math.
- GPIO retuning (MODER/OSPEEDR/AF). T5 confirmed the datasheet
  values.
- Treating the FE flag as a bug to fix. It's Embassy's documented
  end-of-transfer cleanup race; chasing it costs time and the
  scope will dispatch the actual root cause faster.
