# esc_model.py -- a simulated ESC + motor for the Renode harness.
#
# Behaviour, deliberately: it decodes DShot off the GPIOA BSRR write stream and
# answers with bidirectional telemetry. If it cannot parse a frame -- bad CRC,
# a partial frame, no traffic at all -- it does NOTHING. A silent model is the
# honest answer to a silent wire, and it keeps "the FC never transmitted"
# distinguishable from "the ESC ignored us" instead of both looking like noise.
#
# Runs under CPython 3 (self-test) and Renode's embedded IronPython 2.7 (hook),
# so: no f-strings, no py3-only syntax.
#
# How the frame is recovered from BSRR writes
# -------------------------------------------
# The firmware paces DMA at 3 states per DShot bit, one 32-bit BSRR word per
# state (see dshot_bb_frame.rs):
#
#   state 0  assert   -- whole port
#   state 1  per-pin: deassert iff that motor's bit is 0
#   state 2  deassert -- whole port
#
# so a bit is simply "was this pin deasserted in state 1". That is recoverable
# from the write ORDER alone; no timing needed. BSRR semantics: bit n sets pin
# n high, bit n+16 drives it low. Bidirectional DShot swaps which half means
# assert, which is the only thing `inverted` changes.

STATES_PER_SYMBOL = 3
FRAME_BITS = 16
OVERSAMPLE = 3          # samples per GCR bit, matching the firmware's RX rate
GCR_BITS = 21

# nibble -> 5-bit GCR quintet (BLHeli). Inverse of the decoder's table in
# dshot_bb_decode.rs, which is what the firmware has to read back.
GCR_ENCODE = [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
              0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F]


def dshot_crc(data12, inverted):
    """4-bit DShot CRC over the 12-bit (11 throttle + 1 telemetry) field."""
    c = (data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0xF
    if inverted:
        c = (~c) & 0xF
    return c


class Motor(object):
    """First-order lag from commanded throttle to shaft speed.

    Not a physical model -- just enough dynamics that eRPM behaves like a real
    one (spins up over ~150 ms, settles, never steps instantly), so a consumer
    of the telemetry sees plausible traffic rather than a constant.
    """

    def __init__(self, kv=1900.0, volts=16.0, pole_pairs=7, tau_s=0.15):
        self.max_rpm = kv * volts
        self.pole_pairs = pole_pairs
        self.tau_s = tau_s
        self.rpm = 0.0
        self.target_rpm = 0.0

    def command(self, throttle_11):
        # DShot: 0 = stop, 1..47 are commands, 48..2047 is the throttle range.
        if throttle_11 < 48:
            self.target_rpm = 0.0
        else:
            frac = (throttle_11 - 48) / float(2047 - 48)
            self.target_rpm = frac * self.max_rpm

    def step(self, dt_s):
        if dt_s <= 0.0:
            return
        alpha = dt_s / (self.tau_s + dt_s)
        self.rpm += (self.target_rpm - self.rpm) * alpha

    def period_us(self):
        """Electrical period in microseconds, as the ESC reports it."""
        erpm = self.rpm * self.pole_pairs
        if erpm < 1.0:
            return None                     # not spinning
        return 60000000.0 / erpm


class Esc(object):
    """One ESC channel: recovers DShot, drives a Motor, emits telemetry."""

    def __init__(self, pin, inverted=True, motor=None):
        self.pin = pin
        self.inverted = inverted
        self.motor = motor if motor is not None else Motor()
        self.frames_ok = 0
        self.frames_bad_crc = 0
        self.throttle = None
        self.reset()

    def reset(self):
        self._level = 1 if self.inverted else 0   # idle level
        self._state = 0
        self._bits = []
        self._deasserted_in_middle = False

    def on_bsrr(self, value):
        """One BSRR write = one pacer state. Returns a throttle when a frame
        completes and passes CRC, otherwise None."""
        set_bit = (value >> self.pin) & 1
        clr_bit = (value >> (self.pin + 16)) & 1
        if set_bit:
            self._level = 1
        elif clr_bit:
            self._level = 0

        if self._state == 1:
            deassert_level = 1 if self.inverted else 0
            self._deasserted_in_middle = (self._level == deassert_level)

        self._state += 1
        if self._state == STATES_PER_SYMBOL:
            self._bits.append(0 if self._deasserted_in_middle else 1)
            self._deasserted_in_middle = False
            self._state = 0
            if len(self._bits) == FRAME_BITS:
                return self._finish_frame()
        return None

    def _finish_frame(self):
        raw = 0
        for b in self._bits:
            raw = (raw << 1) | b
        self._bits = []

        data12 = (raw >> 4) & 0xFFF
        if (raw & 0xF) != dshot_crc(data12, self.inverted):
            self.frames_bad_crc += 1
            return None                     # unparseable -> do nothing
        self.frames_ok += 1
        self.throttle = (data12 >> 1) & 0x7FF
        self.motor.command(self.throttle)
        return self.throttle

    def telemetry_word(self):
        """16-bit reply: 12-bit payload + 4-bit inverted CRC."""
        period = self.motor.period_us()
        payload = 0x0FFF if period is None else encode_period(period)
        csum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0xF
        return ((payload << 4) | ((~csum) & 0xF)) & 0xFFFF

    def telemetry_samples(self, buf_len=140, start=51):
        """The reply as the firmware would sample it, this pin driven low.

        `start` is the ESC guard time in samples; the rig's healthy motors all
        answer at 51, so that is the default.
        """
        return line_encode(gcr_encode(self.telemetry_word()), buf_len,
                           start, self.pin)


def encode_period(period_us):
    """12-bit eRPM payload: 3-bit exponent + 9-bit mantissa, period = m << e."""
    p = int(round(period_us))
    if p < 1:
        p = 1
    for e in range(8):
        if (p >> e) < 512:
            return ((e & 0x7) << 9) | ((p >> e) & 0x1FF)
    return 0x0FFF                            # too slow to represent


def gcr_encode(word16):
    """16-bit value -> 21 line levels (transition coding: a 1 flips the line).

    Bit 20 is the sync marker, and flipping on it is what produces the leading
    falling edge the firmware triggers on.
    """
    gcr20 = 0
    for i in range(4):
        gcr20 |= GCR_ENCODE[(word16 >> (4 * i)) & 0xF] << (5 * i)
    seq = (1 << 20) | gcr20
    levels = []
    level = 1
    for bit in range(GCR_BITS - 1, -1, -1):
        if (seq >> bit) & 1:
            level ^= 1
        levels.append(level)
    return levels


def line_encode(levels, buf_len, start, pin):
    """Levels -> a port sample buffer: idle high, this pin driven low on 0."""
    buf = [0xFFFF] * buf_len
    idx = start
    for lvl in levels:
        for _ in range(OVERSAMPLE):
            if idx >= buf_len:
                return buf
            if lvl == 0:
                buf[idx] &= ~(1 << pin) & 0xFFFF
            idx += 1
    return buf


# ---------------------------------------------------------------------------
# Self-test:  python renode/checks/esc_model.py
# ---------------------------------------------------------------------------

def frame_words(throttle_11, telemetry_bit, inverted, pin, port_mask=0b1111):
    """The BSRR word stream the firmware's DMA would emit for one frame."""
    data12 = ((throttle_11 & 0x7FF) << 1) | (telemetry_bit & 1)
    raw = (data12 << 4) | dshot_crc(data12, inverted)
    if inverted:
        assert_w, deassert_w = port_mask << 16, port_mask
    else:
        assert_w, deassert_w = port_mask, port_mask << 16
    words = []
    for i in range(FRAME_BITS - 1, -1, -1):
        words.append(assert_w)
        if ((raw >> i) & 1) == 0:
            words.append((1 << pin) if inverted else (1 << (pin + 16)))
        else:
            words.append(0)
        words.append(deassert_w)
    return words


def decode_like_firmware(samples, pin):
    """Mirror of dshot_bb_decode::decode, to prove a reply is readable."""
    start = None
    for i, s in enumerate(samples):
        if ((s >> pin) & 1) == 0:
            start = i
            break
    if start is None:
        return None

    value = 0
    bits = 0
    run_level = 0
    run_len = 0
    for s in samples[start:]:
        lvl = (s >> pin) & 1
        if lvl == run_level:
            run_len += 1
            continue
        n = (run_len + OVERSAMPLE // 2) // OVERSAMPLE
        if n == 0 or bits + n > GCR_BITS:
            return None
        value = (value << n) | (1 << (n - 1))
        bits += n
        run_level = lvl
        run_len = 1
    if bits < GCR_BITS:
        n = GCR_BITS - bits
        value = (value << n) | (1 << (n - 1))
        bits += n
    if bits != GCR_BITS:
        return None

    decode = [0] * 32
    for nib, quint in enumerate(GCR_ENCODE):
        decode[quint] = nib
    out = 0
    for i in range(4):
        out |= decode[(value >> (5 * i)) & 0x1F] << (4 * i)
    return out


def _selftest():
    failures = []

    def check(name, got, want):
        if got == want:
            print("    ok    %-34s %s" % (name, got))
        else:
            print("    FAIL  %-34s got %s, want %s" % (name, got, want))
            failures.append(name)

    print("==> esc_model self-test")

    # A well-formed frame is recovered, on every pin.
    for pin in range(4):
        esc = Esc(pin=pin, inverted=True)
        got = None
        for w in frame_words(1200, 0, True, pin):
            r = esc.on_bsrr(w)
            if r is not None:
                got = r
        check("decodes throttle on pin %d" % pin, got, 1200)

    # A corrupted frame is ignored rather than half-believed.
    esc = Esc(pin=0, inverted=True)
    words = frame_words(1200, 0, True, 0)
    words[4] ^= 0x1
    for w in words:
        esc.on_bsrr(w)
    check("bad CRC -> nothing accepted", esc.frames_ok, 0)
    check("bad CRC -> counted", esc.frames_bad_crc, 1)
    check("bad CRC -> no throttle", esc.throttle, None)

    # Silence produces nothing at all -- the wire being idle is not a frame.
    esc = Esc(pin=0, inverted=True)
    for _ in range(500):
        esc.on_bsrr(0)
    check("silent wire -> no frames", esc.frames_ok, 0)
    check("silent wire -> no throttle", esc.throttle, None)

    def settle(throttle):
        e = Esc(pin=0, inverted=True)
        for x in frame_words(throttle, 0, True, 0):
            e.on_bsrr(x)
        for _ in range(200):
            e.motor.step(0.01)
        return e.motor.period_us()

    check("more throttle -> shorter period", settle(600) > settle(1800), True)
    check("idle motor reports not spinning", settle(0), None)

    e = Esc(pin=0, inverted=True)
    for x in frame_words(1500, 0, True, 0):
        e.on_bsrr(x)
    e.motor.step(0.01)
    early = e.motor.rpm
    for _ in range(200):
        e.motor.step(0.01)
    check("spin-up is gradual, not a step", early < e.motor.rpm * 0.5, True)

    # The reply we would put on the wire must be one the firmware can read.
    e = Esc(pin=2, inverted=True)
    for x in frame_words(1500, 0, True, 2):
        e.on_bsrr(x)
    for _ in range(200):
        e.motor.step(0.01)
    samples = e.telemetry_samples()
    check("reply decodes back to its word",
          decode_like_firmware(samples, 2), e.telemetry_word())

    # And it must look like the rig's healthy motors do.
    first_low = None
    for i, s in enumerate(samples):
        if not ((s >> 2) & 1):
            first_low = i
            break
    check("first falling edge at 51", first_low, 51)
    trans = 0
    for i in range(1, len(samples)):
        if ((samples[i - 1] ^ samples[i]) >> 2) & 1:
            trans += 1
    check("transition count in 12..18", 12 <= trans <= 18, True)

    print("")
    if failures:
        print("FAILED: " + ", ".join(failures))
        return 1
    print("all checks passed")
    return 0


def _decode_renode_log(path):
    """Decode a captured Renode run: GPIOA BSRR writes -> per-motor throttle.

    The log comes from renode/checks/esc-sim.resc, which hooks GPIOA writes. Offset
    0x18 is BSRR, which is the entire DShot waveform.
    """
    import re
    words = []
    pat = re.compile(r"GPIOA-WRITE off=0x18 val=0x([0-9a-fA-F]+)")
    fh = open(path)
    try:
        for line in fh:
            m = pat.search(line)
            if m:
                words.append(int(m.group(1), 16))
    finally:
        fh.close()

    print("==> esc-model: decode the emulated DShot wire")
    print("    captured %d BSRR writes" % len(words))
    if not words:
        print("    FAIL  nothing reached the pad")
        return 1

    # The stream opens with set_idle_level() from DshotBitbang::new, which is
    # not part of the DMA buffer. Align on whichever offset yields a valid
    # frame rather than hard-coding an assumption about the prologue.
    failures = []
    for skip in (0, 1, 2):
        results = []
        for pin in range(4):
            esc = Esc(pin=pin, inverted=True)
            got = None
            for w in words[skip:]:
                r = esc.on_bsrr(w)
                if r is not None:
                    got = r
            results.append((got, esc.frames_ok))
        if all(r[1] > 0 for r in results):
            for pin, (thr, ok) in enumerate(results):
                print("    ok    M%d (PA%d) decoded throttle=%-5s frames=%d"
                      % (pin + 1, pin, thr, ok))
            # Arming sends zero throttle; anything else means we mis-decoded
            # or the firmware is further along than expected.
            if all(r[0] == 0 for r in results):
                print("    ok    all four at zero throttle (arming)")
            else:
                print("    note  non-zero throttle: %s"
                      % [r[0] for r in results])
            return 0
        failures.append(skip)
    print("    FAIL  no alignment produced a CRC-valid frame (tried %s)"
          % failures)
    return 1


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2 and sys.argv[1] == "--decode-log":
        sys.exit(_decode_renode_log(sys.argv[2]))
    sys.exit(_selftest())
