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
