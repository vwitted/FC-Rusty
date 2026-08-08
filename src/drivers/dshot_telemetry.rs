// dshot_telemetry.rs — bidir DShot telemetry response decoder.
//
// BLHeli's bidirectional response (BlueJay, BLHeli_S, BLHeli_32 with
// bidir enabled) carries one telemetry packet after each command
// frame. The wire format is:
//
//   1. ESC waits ~30 µs after the FC's frame ends (guard time),
//      then drives the line LOW to begin the response.
//   2. 21 bits at 5/4 × the TX bit period (so DShot600 = 750 kbps
//      response → 1.33 µs per response bit).
//   3. The 20 lowest bits encode 4 GCR symbols (5 bits each) which
//      decode to 4 nibbles forming a 16-bit value. The 21st (MSB)
//      bit is a synchronisation marker.
//   4. The 16-bit value is `data_12 << 4 | crc_4`. CRC is the one's
//      complement (lower 4 bits) of the standard DShot CRC.
//   5. The 12-bit data is either:
//      - **eRPM**: 3-bit period exponent + 9-bit mantissa.
//        period_µs = mantissa << exponent.
//        eRPM = 60_000_000 / (period_µs * pole_pairs).
//      - **Extended telemetry** (temperature / voltage / current /
//        debug fields): 4-bit type + 8-bit value. Only types with
//        the LSB set are eRPM; other even values are non-eRPM fields.
//
// References:
//   - Betaflight `src/main/drivers/dshot_bitbang_decode.c`
//   - BLHeli_S source (esp. `BLHeli_S.asm` telemetry transmit
//     routines)
//
// NOTE: nothing calls this module. It was the decode half of the
// retired timer-DMA driver; the live bidir path is
// `dshot_bb_decode.rs`, which carries its own copy of the GCR table
// and CRC because it reconstructs bits from oversampled IDR runs
// rather than from captured edges. Kept for the prose above and as a
// second, independently-written reference for the same wire format —
// delete it if it ever starts drifting from `dshot_bb_decode.rs`.

/// GCR 5-bit → 4-bit decode table. 0xFF marks invalid symbols
/// (every 5-bit pattern not in the BLHeli encoding alphabet).
const GCR_DECODE_TABLE: [u8; 32] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
];

/// 4-bit → 5-bit GCR encode table. Kept here for test vectors
/// (we never encode at runtime — only the ESC does that).
#[cfg(test)]
const GCR_ENCODE_TABLE: [u8; 16] = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110,
    0b10111, 0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101,
    0b01110, 0b01111,
];

/// Decode the lower 20 bits of `raw_21` as 4 GCR symbols into a
/// 16-bit value. Returns `None` if any symbol isn't in the GCR
/// alphabet. The MSB (bit 20) is the sync marker and is ignored.
pub fn decode_gcr_symbols(raw_21: u32) -> Option<u16> {
    let encoded = raw_21 & 0xF_FFFF;
    let mut result: u16 = 0;
    for i in 0..4 {
        let chunk = ((encoded >> (15 - i * 5)) & 0x1F) as usize;
        let nibble = GCR_DECODE_TABLE[chunk];
        if nibble == 0xFF {
            return None;
        }
        result = (result << 4) | u16::from(nibble);
    }
    Some(result)
}

/// Bidir telemetry CRC: one's complement of the standard DShot CRC
/// (XOR of 3 nibbles of the 12-bit data field), low 4 bits.
fn telemetry_crc(data_12: u16) -> u8 {
    let base = ((data_12 ^ (data_12 >> 4) ^ (data_12 >> 8)) & 0x0F) as u8;
    (!base) & 0x0F
}

/// Result of decoding a single telemetry packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetryFrame {
    /// eRPM period (µs). Zero means "no rotation" or out-of-range
    /// sentinel (0x0FFF maps to 0 for Betaflight compatibility).
    ErpmPeriod(u16),
    /// Extended telemetry: temperature in °C.
    Temperature(u8),
    /// Extended telemetry: voltage in 0.25 V units.
    Voltage(u8),
    /// Extended telemetry: current in 1 A units.
    Current(u8),
    /// Extended telemetry debug fields.
    Debug1(u8),
    Debug2(u8),
    Debug3(u8),
    StateEvent(u8),
    UnknownExtended { type_id: u8, value: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetryError {
    InvalidGcrSymbol,
    InvalidCrc { calculated: u8, packet: u8 },
    InvalidErpmPeriod,
}

const ERPM_OUT_OF_RANGE: u16 = 0x0FFF;

/// Decode a 21-bit captured GCR response into a parsed telemetry
/// frame. This is the full decode pipeline: GCR symbol decode →
/// CRC verify → payload parse.
pub fn decode_response(raw_21: u32) -> Result<TelemetryFrame, TelemetryError> {
    let raw_16 = decode_gcr_symbols(raw_21).ok_or(TelemetryError::InvalidGcrSymbol)?;
    let data_12 = raw_16 >> 4;
    let packet_crc = (raw_16 & 0x0F) as u8;
    let calculated = telemetry_crc(data_12);
    if calculated != packet_crc {
        return Err(TelemetryError::InvalidCrc {
            calculated,
            packet: packet_crc,
        });
    }
    parse_data_12(data_12)
}

fn parse_data_12(data_12: u16) -> Result<TelemetryFrame, TelemetryError> {
    // The Extended Telemetry (EDT) format hijacks the high 4 bits of
    // the 12-bit field as a type code. EDT types with the LSB set are
    // eRPM (back-compat with the original 3-exp + 9-mantissa layout,
    // since eRPM always has at least one of the top bits set
    // depending on the period range, which makes "type even" a clean
    // discriminator).
    let edt_type = ((data_12 >> 8) & 0x0F) as u8;
    let is_erpm = edt_type == 0x00 || (edt_type & 0x01) != 0;

    if is_erpm {
        if data_12 == ERPM_OUT_OF_RANGE {
            return Ok(TelemetryFrame::ErpmPeriod(0));
        }
        let exponent = (data_12 >> 9) & 0b111;
        let mantissa = data_12 & 0x1FF;
        let period = mantissa << exponent;
        if period == 0 {
            return Err(TelemetryError::InvalidErpmPeriod);
        }
        return Ok(TelemetryFrame::ErpmPeriod(period));
    }

    let value = (data_12 & 0xFF) as u8;
    let frame = match edt_type {
        0x02 => TelemetryFrame::Temperature(value),
        0x04 => TelemetryFrame::Voltage(value),
        0x06 => TelemetryFrame::Current(value),
        0x08 => TelemetryFrame::Debug1(value),
        0x0A => TelemetryFrame::Debug2(value),
        0x0C => TelemetryFrame::Debug3(value),
        0x0E => TelemetryFrame::StateEvent(value),
        _ => TelemetryFrame::UnknownExtended {
            type_id: edt_type,
            value,
        },
    };
    Ok(frame)
}

/// Convert an eRPM period (µs) into mechanical RPM, given the
/// motor's pole-pair count (typically 7 for a 12N14P stator).
pub fn period_to_rpm(period_us: u16, pole_pairs: u8) -> u32 {
    if period_us == 0 || pole_pairs == 0 {
        return 0;
    }
    let erpm_x100 = 60_000_000u32 / u32::from(period_us);
    erpm_x100 / u32::from(pole_pairs)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip helper for tests: encode 16-bit payload via the
    /// reference GCR_ENCODE_TABLE.
    #[cfg(test)]
    fn encode_gcr_for_test(payload_16: u16) -> u32 {
        let mut encoded: u32 = 0;
        for i in 0..4 {
            let nibble = ((payload_16 >> (12 - i * 4)) & 0x0F) as usize;
            let chunk = u32::from(GCR_ENCODE_TABLE[nibble]);
            encoded = (encoded << 5) | chunk;
        }
        encoded | (1 << 20)
    }

    #[test]
    fn gcr_decode_reference_vector() {
        // From uf-dshot's reference test vector:
        // raw_21 = 0x15EA6F → 16-bit payload = 0xB83F
        assert_eq!(decode_gcr_symbols(0x15EA6F), Some(0xB83F));
    }

    #[test]
    fn gcr_roundtrip_known_values() {
        for &payload in &[0xB83F, 0xAAAA, 0x5555, 0x1234, 0xFEDC, 0xFFFF, 0x0000] {
            let gcr = encode_gcr_for_test(payload);
            assert_eq!(decode_gcr_symbols(gcr), Some(payload));
        }
    }

    #[test]
    fn gcr_rejects_invalid_symbols() {
        // 0b00000 is not in the GCR alphabet.
        let invalid = encode_gcr_for_test(0xB83F) & !0x1F; // wipe last chunk
        assert_eq!(decode_gcr_symbols(invalid), None);
    }

    #[test]
    fn telemetry_crc_motor_stop_bidir() {
        // For motor-stop bidir TX: data_12 = 0x001, CRC = ~0x1 & 0xF = 0xE.
        // Same CRC formula is used (in the inverted direction) for
        // telemetry responses, so test it here too.
        assert_eq!(telemetry_crc(0x001), 0x0E);
    }

    #[test]
    fn parse_erpm_simple() {
        // data_12 = 0x100 → exp=0, mantissa=0x100=256 → period=256
        let frame = parse_data_12(0x100).unwrap();
        assert_eq!(frame, TelemetryFrame::ErpmPeriod(256));
    }

    #[test]
    fn parse_erpm_with_exponent() {
        // The EDT-type discriminator is the high nibble of data_12.
        // EDT types with LSB clear and non-zero get parsed as
        // extended-telemetry instead of eRPM, so use a mantissa with
        // bit 8 set (≥ 256) so the LSB of the type nibble is 1 and we
        // stay on the eRPM path. exp=2, mantissa=257 → period = 1028.
        let data_12 = (2 << 9) | 257;
        let frame = parse_data_12(data_12).unwrap();
        assert_eq!(frame, TelemetryFrame::ErpmPeriod(1028));
    }

    #[test]
    fn parse_erpm_out_of_range_sentinel() {
        assert_eq!(parse_data_12(0x0FFF).unwrap(), TelemetryFrame::ErpmPeriod(0));
    }

    #[test]
    fn parse_temperature_extended() {
        // type=2 (temperature), value=25 → 0x219
        let frame = parse_data_12(0x0219).unwrap();
        assert_eq!(frame, TelemetryFrame::Temperature(25));
    }

    #[test]
    fn parse_voltage_extended() {
        // type=4 (voltage), value=42 → 0x42A
        let frame = parse_data_12(0x042A).unwrap();
        assert_eq!(frame, TelemetryFrame::Voltage(42));
    }

    #[test]
    fn period_to_rpm_sanity() {
        // 1000 µs period at 7 pole pairs → 60_000 RPM / 7 ≈ 8571
        let rpm = period_to_rpm(1000, 7);
        assert!(rpm > 8400 && rpm < 8700, "got {rpm}");
    }

    #[test]
    fn period_to_rpm_zero_period() {
        assert_eq!(period_to_rpm(0, 7), 0);
    }

    #[test]
    fn decode_response_end_to_end() {
        // Synthetic: data_12=0x123, crc=~(0x1^0x2^0x3) = ~0x0 = 0xF → raw_16 = 0x123F
        let raw_16 = 0x123F;
        let gcr = encode_gcr_for_test(raw_16);
        let frame = decode_response(gcr).unwrap();
        // data_12=0x123: type=0x1 (LSB set) → eRPM. exp=0, mantissa=0x123=291.
        assert_eq!(frame, TelemetryFrame::ErpmPeriod(291));
    }

    #[test]
    fn decode_response_bad_crc() {
        // raw_16 = 0x1230 (data_12=0x123, packet_crc=0x0; calc=0xF)
        let gcr = encode_gcr_for_test(0x1230);
        match decode_response(gcr) {
            Err(TelemetryError::InvalidCrc { calculated, packet }) => {
                assert_eq!(calculated, 0x0F);
                assert_eq!(packet, 0x00);
            }
            other => panic!("expected InvalidCrc, got {:?}", other),
        }
    }
}
