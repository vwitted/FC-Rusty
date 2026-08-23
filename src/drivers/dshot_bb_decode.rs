// dshot_bb_decode.rs — bidirectional DShot telemetry decode from oversampled
// GPIO samples.
//
// The ESC replies at 5/4 the DShot bit rate, 21 GCR bits, line-coded so that
// a transition represents a 1. We sample the port at OVERSAMPLE × that bit
// rate, so a run of N samples at one level is round(N / OVERSAMPLE) bit
// times.
//
// Reconstruction works by run length: each run of n bit times emits a `1`
// followed by n-1 zeros, which performs the transition decode inline. The
// quintet table and CRC below are the standard BLHeli ones.
//
// Wire format (BlueJay, BLHeli_S, BLHeli_32 with bidir enabled). One
// telemetry packet follows each command frame:
//
//   1. The ESC waits ~30 µs after the FC's frame ends (guard time), then
//      drives the line LOW to begin the response. Measured on this rig it is
//      ~22.7 µs; an ESC that waits materially longer replies outside the
//      capture window and looks identical to a dead telemetry line — see
//      RX_BUF_LEN.
//   2. 21 bits at 5/4 × the TX bit period (DShot600 → 750 kbps response,
//      1.33 µs per response bit).
//   3. The 20 lowest bits are 4 GCR symbols of 5 bits, decoding to 4 nibbles
//      that form a 16-bit value. The 21st (MSB) bit is a sync marker and is
//      logical only — the line goes LOW there, which is why the frame is
//      found by its first falling edge.
//   4. The 16-bit value is `data_12 << 4 | crc_4`, the CRC being the one's
//      complement (low nibble) of the standard DShot CRC.
//   5. The 12-bit data is either eRPM (3-bit period exponent + 9-bit
//      mantissa; `period_µs = mantissa << exponent`, and
//      `eRPM = 60_000_000 / (period_µs × pole_pairs)`) or extended telemetry
//      (4-bit type + 8-bit value: temperature, voltage, current, debug).
//      Only types with the LSB set are eRPM.
//
//      NOTE: this decoder does not yet discriminate the two — every payload
//      is read as eRPM. Close that before anything in the flight path
//      consumes the value.
//
// References:
//   - Betaflight `src/main/drivers/dshot_bitbang_decode.c`
//   - BLHeli_S source, telemetry transmit routines in `BLHeli_S.asm`

/// Samples per GCR bit (BF: `DSHOT_BITBANG_TELEMETRY_OVER_SAMPLE`).
pub const OVERSAMPLE: usize = 3;
/// Samples captured per response window (BF: `DSHOT_BB_PORT_IP_BUF_LENGTH`).
///
/// 140 samples at the DShot600 receive rate (445.8 ns each) is a 62.4 µs
/// window. A healthy ESC answers at ~22.7 µs and its 21-bit reply occupies
/// ~52 samples, so the default has ample margin and fits the 8 kHz flight
/// loop budget.
///
/// Overridable at build time (`RX_SAMPLES=400 ./scripts/flash-motor-test.sh`)
/// purely as a bench instrument: if an ESC answers *later* than the window,
/// its reply is invisible to us while still plainly present on a scope, and
/// the only way to tell that apart from a dead telemetry line is to widen the
/// window and look. Do not raise this for flight — the window is awaited
/// inside the control loop, so 400 samples is 178 µs and blows the 125 µs
/// period. Pair a raised value with `LOOP_KHZ=2`.
pub const RX_BUF_LEN: usize = parse_rx_samples();

const fn parse_rx_samples() -> usize {
    let Some(s) = option_env!("RX_SAMPLES") else {
        return 140;
    };
    let b = s.as_bytes();
    let mut i = 0;
    let mut v: usize = 0;
    if b.is_empty() {
        return 140;
    }
    while i < b.len() {
        if b[i] < b'0' || b[i] > b'9' {
            return 140;
        }
        v = v * 10 + (b[i] - b'0') as usize;
        i += 1;
    }
    if v < 64 { 140 } else { v }
}
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

    let decoded = gcr_quintets_to_word(value);

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

/// 5-to-4 GCR quintet decode (BF's table): a 21-bit line-decoded word to the
/// 16-bit `data_12 << 4 | crc_4`. Bit 20 is the sync marker and is discarded.
///
/// Split out from `decode` so a fixed external vector can anchor the table
/// without needing a CRC-valid frame — see `decodes_external_reference_vector`.
fn gcr_quintets_to_word(value: u32) -> u32 {
    const GCR_DECODE: [u32; 32] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0,
    ];
    let s0 = GCR_DECODE[(value & 0x1F) as usize];
    let s1 = GCR_DECODE[((value >> 5) & 0x1F) as usize];
    let s2 = GCR_DECODE[((value >> 10) & 0x1F) as usize];
    let s3 = GCR_DECODE[((value >> 15) & 0x1F) as usize];
    s0 | (s1 << 4) | (s2 << 8) | (s3 << 12)
}

/// Samples → bit times, rounding to nearest.
fn bit_times(run_len: usize) -> u32 {
    ((run_len + OVERSAMPLE / 2) / OVERSAMPLE) as u32
}

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

    /// Encode a 16-bit BLHeli telemetry word into the *physical line levels*
    /// an ESC would drive for its 21-bit GCR reply, i.e. the inverse of what
    /// `decode`'s run-length reconstruction undoes.
    ///
    /// `decode` recovers the pre-transition-encoded 21-bit value (sync bit +
    /// 4 GCR quintets) directly from level runs: each run of n bit-times is
    /// a transition (`1`) followed by n-1 flat bit-times (`0`). To build a
    /// sample buffer that round-trips through that, this must run the same
    /// transition encoding *forward*: level flips exactly on a `1` bit,
    /// starting from the idle-high reference (the line's state before the
    /// first edge) — across all 21 bits, sync bit included. An earlier
    /// version of this helper differentially-encoded only the 20 quintet
    /// bits from a `0` reference and then spliced the sync bit on as a bare
    /// `1`, which desynced it from the transition chain and produced a
    /// waveform `decode` could not reconstruct (see task-2-report.md).
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
        // The logical 21-bit value: forced sync bit (always 1) followed by
        // the 4 GCR quintets — exactly what `decode` is expected to recover.
        let logical: u32 = quintets | (1 << 20);

        // Forward transition-encode: level flips iff the logical bit is 1,
        // starting from the idle-high reference (level = 1) that precedes
        // bit 20 on the wire.
        let mut level = 1u32;
        let mut out: u32 = 0;
        for bit in (0..GCR_BITS as u32).rev() {
            let b = (logical >> bit) & 1;
            level ^= b;
            out = (out << 1) | level;
        }
        out
    }

    /// A telemetry word with a correct BLHeli checksum for the given payload.
    fn word_with_crc(payload12: u16) -> u16 {
        let mut csum = payload12 ^ (payload12 >> 4) ^ (payload12 >> 8);
        csum = !csum & 0xF;
        (payload12 << 4) | csum
    }

    /// External fixed vector, not generated by this file's own encoder.
    ///
    /// Every other test here builds samples with `samples_from_gcr` and reads
    /// them back, so encoder and decoder are inverses by construction and a
    /// shared misunderstanding of the encoding would pass silently. This
    /// vector is from uf-dshot, via the retired `dshot_telemetry.rs`, which
    /// decoded it with an independently written GCR table: raw 21-bit
    /// 0x15EA6F → 16-bit 0xB83F.
    ///
    /// It anchors the quintet table only. 0xB83F is NOT a CRC-valid frame
    /// (0xB83F ^ 0x0B83 ^ 0x00B8 = 0xB304, low nibble 4, not 0xF), so it
    /// cannot be pushed through `decode` end to end — the original test
    /// asserted on the symbol decode alone for the same reason.
    #[test]
    fn decodes_external_reference_vector() {
        assert_eq!(gcr_quintets_to_word(0x15EA6F), 0xB83F);
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

    // ---- Field captures from the bench RX probe (2026-08, motor-test) ----
    //
    // Run lengths as printed by the probe, starting from the idle-high line:
    // `hi=1010...` means run 0 is high. M1-M3 decode cleanly; M4 is the
    // motor that never yields eRPM. Kept as a regression fixture so a change
    // to the reconstruction is checked against real wire data, not just
    // synthesised frames.

    /// Rebuild a port sample buffer from a probe run-length list.
    fn samples_from_runs(runs: &[usize], pin: u8) -> [u16; RX_BUF_LEN] {
        let mut buf = [0xFFFFu16; RX_BUF_LEN];
        let mut idx = 0usize;
        let mut high = true;
        for &len in runs {
            for _ in 0..len {
                if idx >= RX_BUF_LEN {
                    return buf;
                }
                if !high {
                    buf[idx] &= !(1u16 << pin);
                }
                idx += 1;
            }
            high = !high;
        }
        buf
    }

    /// Healthy captures: first edge at 51, 14-16 transitions.
    const HEALTHY: [&[usize]; 3] = [
        &[51, 3, 5, 3, 2, 5, 3, 2, 8, 9, 5, 2, 3, 2, 37],   // M1 @ 6.659
        &[51, 3, 5, 2, 3, 5, 3, 2, 3, 2, 2, 6, 2, 2, 6, 2, 41], // M2 @ 6.663
        &[51, 3, 5, 2, 3, 5, 2, 3, 2, 3, 2, 3, 5, 2, 9, 2, 38], // M3 @ 6.668
    ];

    /// M4 captures: first edge late and drifting (71-100), 0-6 transitions.
    const M4_BAD: [&[usize]; 4] = [
        &[71, 3, 2, 2, 62],   // @ 6.673
        &[84, 3, 53],         // @ 12.898
        &[99, 1, 40],         // @ 29.500
        &[140],               // @ 19.124  no falling edge at all
    ];

    #[test]
    fn healthy_field_captures_decode_to_erpm() {
        for (i, runs) in HEALTHY.iter().enumerate() {
            let buf = samples_from_runs(runs, i as u8);
            let got = decode(&buf, i as u8);
            assert!(
                matches!(got, BbTelemetry::Erpm { .. }),
                "M{} healthy capture should decode to Erpm, got {:?}",
                i + 1,
                got
            );
        }
    }

    #[test]
    fn m4_field_captures_never_decode_to_erpm() {
        for (i, runs) in M4_BAD.iter().enumerate() {
            let buf = samples_from_runs(runs, 3);
            let got = decode(&buf, 3);
            assert!(
                !matches!(got, BbTelemetry::Erpm { .. }),
                "M4 capture {} is noise, must not decode as eRPM, got {:?}",
                i,
                got
            );
        }
    }

    /// The decoder must treat every pin identically -- the same wire data on
    /// pin 0 and pin 3 must give the same answer. Rules out an index-boundary
    /// bug as the explanation for one motor behaving differently.
    #[test]
    fn decode_is_identical_across_all_four_pins() {
        for runs in HEALTHY.iter().chain(M4_BAD.iter()) {
            let reference = decode(&samples_from_runs(runs, 0), 0);
            for pin in 1..4u8 {
                let got = decode(&samples_from_runs(runs, pin), pin);
                assert_eq!(got, reference, "pin {} differs from pin 0", pin);
            }
        }
    }

    /// All four motors share one buffer and one window, so a healthy reply on
    /// three pins must not rescue -- or corrupt -- the fourth.
    #[test]
    fn four_pins_in_one_buffer_decode_independently() {
        let mut buf = [0xFFFFu16; RX_BUF_LEN];
        for (pin, runs) in HEALTHY.iter().enumerate() {
            let one = samples_from_runs(runs, pin as u8);
            for (dst, src) in buf.iter_mut().zip(one.iter()) {
                *dst &= *src | !(1u16 << pin);
            }
        }
        let bad = samples_from_runs(M4_BAD[0], 3);
        for (dst, src) in buf.iter_mut().zip(bad.iter()) {
            *dst &= *src | !(1u16 << 3);
        }
        for pin in 0..3u8 {
            assert!(
                matches!(decode(&buf, pin), BbTelemetry::Erpm { .. }),
                "pin {} should still decode in the shared buffer",
                pin
            );
        }
        assert!(!matches!(decode(&buf, 3), BbTelemetry::Erpm { .. }));
    }
}
