
## Journal — 2026-07-26: bidirectional DShot bring-up - RESOLVED

Long bench session on the DAKEFPV H743 chasing why bidirectional DShot
does not work while plain DShot does. **Not resolved.** Bidir still fails:
the ESC does not accept frames (motors do not spin) and no telemetry is
ever decoded (`NoEdge` on every frame). Plain DShot is unaffected and
works end to end, including in the flight firmware.

### Fixed and confirmed on hardware

Three real defects, all in the bidir-only RX→TX path (`dshot_hw.rs`):

1. **Stale compare register** left the line asserted for ~8 µs before
   every frame, swallowing bit 0's falling edge — the edge BLHeli syncs
   on. The old guard wrote `CCRn` with `OCxPE=1`, so the zero landed in
   the preload register and never reached the active one.
2. **The update event needed `CCxE=1`** to take effect. Isolated with a
   bench probe: writing `CCR1` alone changed nothing, a single `EGR.UG`
   released all four pads at once.
3. **A GPIO glitch guard in the output direction that BF does not have.**
   BF's H7 guard exists only in `pwmDshotSetDirectionInput`;
   `pwmDshotSetDirectionOutput` touches no GPIO. Ours was symmetric.

Also: the direction switch back to output now happens immediately before
transmit (as BF does from `pwmTelemetryDecode`) rather than when the
response window closes, and the transmit path follows BF's register
order with the DMA streams armed last. That shortened the stuck-LOW
first bit from ~8 µs to 3.4 µs — it moved the symptom, not the cause.

The transmit reorder sits in the path **shared with non-bidir**, so it
was re-verified on hardware afterwards: plain DShot still spins the
motors normally. No regression to the working protocol.

### The open problem

The transmit-setup pad trace localises it exactly:

    after switch=0000 | after ARR/CNT=0000 | after CCxDE=0000
    | after DMA armed=1111 | after frame=1111

The line is LOW from the moment the direction switch returns, and only
goes HIGH once the DMA writes a cell value. The idle probe narrows it
further: `OCM=FORCE_INACTIVE` gives idle-high, but PWM mode 1 with what
should be `CCR=0` gives an **active** output. So the active compare
register still holds an RX capture value, and with `ARR` at `0xFFFFFFFF`
the `CNT < CCR` condition stays true for a long time.

**Unexplained:** why the active compare register cannot be cleared by
writing it with preload disabled — which is all BF's `LL_TIM_OC_Init`
does, and BF works on this exact board and ESC.

### 2026-08-02 — the reference was the wrong file

`dshot_bitbang = AUTO` (the BF default) resolves to **bit-banging** on
H7. Verified verbatim in `src/platform/common/stm32/dshot_bitbang_shared.c`:

    bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
    {
    #if defined(STM32F4) || defined(APM32F4)
        return useDshotBitbang == ON ||
            (useDshotBitbang == AUTO && useDshotTelemetry
             && motorProtocol != PROSHOT1000);
    #else
        return useDshotBitbang == ON ||
            (useDshotBitbang == AUTO && motorProtocol != PROSHOT1000);
    #endif
    }

H7 takes the `#else` branch: AUTO means bitbang for any protocol except
ProShot1000, regardless of whether telemetry is enabled. Only F4
additionally requires `useDshotTelemetry`.

So the working Betaflight on this board is **bit-banging**, for both
bidir and plain DShot, and `pwm_output_dshot_hal.c` — the file this
driver claims to port and which the 2026-07-26 session transliterated
against — is not the code producing that waveform.

**This voids the open question above** rather than answering it. "Why
can we not clear the active compare register when BF's `LL_TIM_OC_Init`
can?" assumed BF does so successfully on this hardware. It does not do
it at all. There was no working counter-example.

It does not prove the timer-DMA path cannot work on H7 — BF still ships
it for when bitbang is off — only that we have no evidence it does, and
that BF defaults away from it on every family after F4.

### Next steps

- **Reference capture.** Flash BF (known working bidir on this hardware)
  and capture one full frame period at ~10 µs/div. Gives the ESC's real
  reply timing and edge spacing — which calibrates `DEADTIME_US` and the
  GCR decoder — and settles whether BF's idle line is clean.
- **Finish the port properly.** The header claims a direct port of BF's
  H7 driver; it is not. The DMA lifecycle is the substituted piece: BF
  tears down and reconfigures the stream *inside* the direction switches
  with an explicit `Direction` field, while we construct and drop Embassy
  `Transfer` objects per frame. Needing an `EGR.UG` that BF does not need
  is itself evidence the port diverges structurally.
- **Confirm `UDE` vs `CCxDE`** for H7 specifically. Our port assumes
  per-channel compare DMA; one BF source read suggested update-event DMA
  ("exactly one transfer per TIM cycle"), but that fetch mixed in H5/N6
  detail and was not confirmed.
- **Recalibrate the RX self-test** before trusting it. It reports 2 of 8
  self-driven edges captured, but the capture timestamps show ~145 ticks
  between them rather than the ~24 expected, so the pulse generator's
  timing is wrong, not necessarily the capture path. Register dump
  confirms the RX config is correct (`CCS=1 ICPSC=0`, both-edge `CCER`).

### Bench tooling added

- Build stamp (`<epoch>-<sha>[-dirty]`) logged at DShot init and echoed
  by the flash scripts with the binary's SHA-256, so "is this the
  firmware I just built" is answerable.
- `rerun-if-env-changed` for the motor-test env vars. Without it,
  `LOOP_KHZ=2 ./scripts/flash-motor-test.sh` recompiled nothing and
  flashed the previous config — very likely the cause of the
  "motor bidir setting not responding to code changes" note from
  2026-07-25.
- `DEADTIME_US=<n>` build-time override. Moving it moves the direction
  switch, which is how the stray pulse was pinned to our code rather
  than the ESC.
- Idle probe, transmit-setup pad trace, and RX loopback self-test, all
  gated to single frames inside the MotorStop arming window.

### 2026-08-08 — bidirectional DShot works on the bit-banged driver

Bidirectional DShot now works on the bit-banged driver
(`src/drivers/dshot_bitbang.rs` + `src/drivers/dshot_bb_decode.rs`). The
older timer-DMA driver (`dshot_hw.rs`), the subject of the investigation
above, never achieved it and remains unfixed; plain (non-bidir) DShot
does work there.

The rewrite happened because of the 2026-08-02 finding above: Betaflight
resolves `dshot_bitbang = AUTO` to bit-banging on everything after F4,
including H7, so the whole timer-DMA port was built against the wrong
reference. In the bitbang design the timer is only a pacer; DMA writes
BSRR words to GPIOA to produce the waveform and reads IDR with 3×
oversampling to capture the reply.

Measured/derived timing actually in the code: TX pacer ARR=265 (240 MHz
/ 266 = 902 kHz state rate, 3 states per bit = 3.325 µs/bit = DShot300,
+0.25% fast). RX pacer ARR=212 (1.125 MHz = reply rate 5/4 × 300 kHz,
oversampled 3×). One frame is 51 states = 56.5 µs; the RX window is 140
samples = 124 µs.

Bench result 2026-08-08 with `BIDIR=1 LOOP_KHZ=2` (at the time this also
took `DRIVER=bitbang`; the timer driver was retired later the same day and
that variable no longer exists): motors
ran and reply data resembling eRPM appeared on the scope after the
frame. Caveat: one of the four motors was physically unsoldered on the
bench rig, so this was a 3-of-4 result, not a clean sweep. Decoded
telemetry has **not** yet been confirmed in the logs — that remains the
pending bench gate. This entry adds the decode wrapper
(`DshotBitbang::send_and_decode`, commit `6f5cb61`) and wires it into the
bitbang drive loop in `motor_test.rs`, logging
`motor-test RX [bitbang]: M1=… M2=… M3=… M4=…` at ~10 Hz; the arming
loop still calls the undecoded `send_and_receive`, mirroring the timer
path's arming loop, which also doesn't decode or log. Bench-verifying
that log line is still open.

Three defects were found in the implementation plan document itself
during execution, worth recording as a caution about that document: a
GCR test-encoder helper that drove the line HIGH at frame start when a
real ESC pulls it LOW; PAC type errors (TIM1 needs `ArrCore`/`CntCore`
newtypes, TIM2 does not); and `Transfer::new_read` requiring
`peri_addr: *mut W`, not `*const W`.

One code-review finding was raised as Critical and then downgraded to
Minor after the bench disproved it: a predicted stray pulse in the idle
gap from the input→output MODER switch. It did not appear. The reviewer
had cited the Betaflight hold-states rationale as its mechanism, but
that rationale is explicitly about the transition *to* an input, which
the hold states already cover — not the transition back to output. That
distinction has now misled one reviewer; worth checking carefully before
citing it again.

Still outstanding: the driver is bench-only (`motor-test` feature), not
yet wired into the flight path — that's Task 6.

### 2026-08-08 (later) — cutover: timer-DMA DShot retired

The bit-banged driver replaced `dshot_hw.rs` on the flight path.
`dshot_hw.rs` and `dshot_diag.rs` are deleted; `dshot_frame.rs` stays
(shared encoder). Work moved to branch `dakefpv-h743-bitbang-dshot`;
the pre-cutover tree is preserved on `archive/dakefpv-h743-timer-dma-dshot`
and, more portably, at commit `bbf2d2b`.

Justification for deleting rather than keeping a fallback: the bitbang
driver covers *both* modes. Plain DShot (`BIDIR=0`) was the Task 3 bench
gate and motors spun; bidirectional was verified with decoded eRPM on
three channels. So the timer driver was redundant, not a safety net.

Bench state at cutover: M1/M2/M3 return stable eRPM ~3900–4100 µs at 5%
throttle (≈15,200 eRPM ≈ 2,170 mechanical RPM on a 14-pole motor). M4
returns `NoSignal` — its line never goes low in any of the 140 samples,
across every probe burst. Since all four pins are read from one `IDR`
word in a single DMA transfer with `MODER` written for all four together,
there is no per-channel code path that could single out M4, so this is
an off-chip fault. Two candidates were considered and both weakened:
ESC-side bidir config (there is none — ESCs auto-detect the inverted
signalling) and a broken signal wire (M4 spins at the correct frequency,
so TX arrives intact). The asymmetry worth probing is that the MCU drives
push-pull both ways while the ESC only pulls *low* against our internal
pull-up, so a degraded path can pass TX and still fail RX. Unresolved;
scope the M4 pad during the receive window.

**Open, and load-bearing for flight:** `control_loop` hardcodes
`dt = 0.000125` (8 kHz), but a bidirectional DShot300 frame is ~181 µs
(TX 56.5 + RX 124.2) against a 125 µs period. The loop cannot hold 8 kHz;
because `IMU_DATA` is a latest-value `Signal` it free-runs at ~5.5 kHz and
drops gyro samples rather than lagging. The rate PID then sees
non-uniformly sampled gyro *and* a `dt` wrong by ~45%. Fix under
consideration is DShot600 (`TX_ARR` 265→132, `RX_ARR` 212→106, total
~91 µs) plus a measured rather than hardcoded `dt`. **Do not read
anything into 8 kHz inner-loop behaviour until this is settled.**

A code review of the cutover also caught that the bench RX probe had
followed the driver into the armed flight loop. It emits eight
`defmt::info!` lines, and `logger::putc` busy-waits on USART6 TXE at
115200 baud inside a global `critical_section` — milliseconds of
interrupts-off, repeating, in flight. Now gated behind the `motor-test`
feature. The 10 Hz telemetry log in `control_loop` has the same blocking
property and predates the cutover; it should get the same treatment.

### 2026-08-08 (later still) — DShot600, measured `dt`, telemetry decoder retired

Three follow-ups after the cutover, all bench-driven:

**DShot600.** At DShot300 a bidirectional frame was ~181 µs against the
8 kHz loop's 125 µs period, so the loop could not hold its rate. `TX_ARR`
265→132 and `RX_ARR` 212→106 give ~91 µs. ESCs auto-detect the bit rate,
so no ESC-side change was needed. Note the ESC turnaround is a *fixed*
delay, so halving the sample period doubles the share of the RX buffer it
consumes: the first falling edge moved from sample 26 to 51, both ≈23 µs.

**Measured `dt`.** `control_loop` hardcoded `dt = 0.000125` while awaiting
a DShot frame each iteration, and `mekf_task` measured its predict step
with `Instant::now()`. Both are now DWT cycle counts. embassy-time is
configured `tick-hz-32_768` — one tick is 30.5 µs, so a true 125 µs
interval read as 122 or 153 µs, which is worse than the constant it would
have replaced. `CORE_HZ` must track `board_config`: the M7 core and DWT
run at SYSCLK (480 MHz), not the 240 MHz AHB.

**`dshot_telemetry.rs` deleted.** It was the decode half of the retired
timer-DMA driver and had no callers. Its wire-format documentation moved
into `dshot_bb_decode.rs`, and its external reference vector (uf-dshot,
raw 0x15EA6F → 0xB83F) became a test there — the first fixed vector in
that module's suite, which until now was self-consistent only. Note that
vector anchors the *quintet table* alone: 0xB83F is not a CRC-valid frame,
so it cannot be pushed through `decode` end to end.

**M4 telemetry, still open.** Its ESC drives motors correctly at DShot600
and a scope shows an eRPM response on the wire, but our capture holds only
4–8 samples of noise at wandering positions 71–100, where a healthy reply
occupies ~52 contiguous samples between the leading idle (51) and trailing
idle (37). A truncated reply would fill the rest of the buffer with
alternating runs; this does not. So the reply falls outside the 62.4 µs
window entirely. `RX_SAMPLES` now overrides the window width at build time
to test that (`RX_SAMPLES=400 BIDIR=1 LOOP_KHZ=2`, bench only — 400 samples
is 178 µs and does not fit the flight period). Current intention is to
swap the ESC regardless: one unit of four needing 3× its siblings'
turnaround is not something the flight firmware should be shaped around.
