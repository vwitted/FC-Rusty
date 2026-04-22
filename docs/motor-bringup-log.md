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
