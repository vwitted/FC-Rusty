// crsf.rs — CRSF protocol parser for RC receivers
//
// Parses the TBS Crossfire (CRSF) protocol used by ExpressLRS
// and TBS Crossfire receivers. We only need to receive frames
// from the receiver (Rx → FC direction), not transmit.
//
// Reference: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
//
// The protocol is simple:
//   [sync] [length] [type] [payload...] [crc]
//
// Sync byte is 0xC8 (FC address), length is payload+type+crc size,
// CRC8 covers type+payload using polynomial 0xD5.
// UART: 416666 baud, 8N1, non-inverted, full-duplex.
// Max frame size: 64 bytes total.

/// CRSF sync byte (flight controller device address)
const SYNC_BYTE: u8 = 0xC8;

/// Maximum frame size including sync and length bytes
const MAX_FRAME_SIZE: usize = 64;

/// Minimum valid frame length field value (type + crc, no payload)
const MIN_FRAME_LENGTH: u8 = 2;

/// Maximum valid frame length field value (64 - sync - length)
const MAX_FRAME_LENGTH: u8 = 62;

/// CRSF frame types we care about
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum FrameType {
    RcChannelsPacked = 0x16,
    LinkStatistics = 0x14,
}

/// Parsed RC channel data — 16 channels, each 0-1984
///
/// The raw 11-bit values map as follows:
///   172  = 988us  (minimum stick position)
///   992  = 1500us (centre)
///   1811 = 2012us (maximum stick position)
///
/// For the control loop, you'll probably want to map these
/// to a normalised -1.0 to 1.0 range (or 0.0 to 1.0 for throttle).
#[derive(Debug, Clone, Copy)]
pub struct RcChannels {
    pub channels: [u16; 16],
}

impl RcChannels {
    /// Convert a raw 11-bit CRSF channel value to microseconds
    /// (approximately — CRSF doesn't use us natively, but this
    /// is useful for compatibility with traditional RC ranges)
    pub fn to_us(raw: u16) -> u16 {
        // Linear mapping: 172 → 988us, 1811 → 2012us
        // us = (raw - 172) * 1024 / 1639 + 988
        // Simplified to avoid float:
        ((raw as u32 * 625) / 1000 + 880) as u16
    }

    /// Convert a raw channel value to -1.0..1.0 range
    /// (centred at 992 = 0.0)
    pub fn to_normalised(raw: u16) -> f32 {
        (raw as f32 - 992.0) / 819.5
    }

    /// Convert a raw channel value to 0.0..1.0 range
    /// (useful for throttle)
    pub fn to_unit(raw: u16) -> f32 {
        (raw as f32 - 172.0) / 1639.0
    }
}

/// Link quality and signal strength data
#[derive(Debug, Clone, Copy)]
pub struct LinkStatistics {
    /// Uplink RSSI antenna 1 (dBm, negate to get actual value)
    pub uplink_rssi_ant1: u8,
    /// Uplink RSSI antenna 2 (dBm, negate to get actual value)
    pub uplink_rssi_ant2: u8,
    /// Uplink link quality (0-100%)
    pub uplink_link_quality: u8,
    /// Uplink SNR (dB, signed)
    pub uplink_snr: i8,
    /// Active antenna (0 or 1)
    pub active_antenna: u8,
    /// RF mode (depends on receiver firmware)
    pub rf_mode: u8,
    /// Uplink TX power (index, not dBm directly)
    pub uplink_tx_power: u8,
    /// Downlink RSSI (dBm, negate)
    pub downlink_rssi: u8,
    /// Downlink link quality (0-100%)
    pub downlink_link_quality: u8,
    /// Downlink SNR (dB, signed)
    pub downlink_snr: i8,
}

/// What the parser emits when it successfully decodes a frame
#[derive(Debug)]
pub enum CrsfEvent {
    /// Fresh RC channel data
    Channels(RcChannels),
    /// Fresh link statistics
    Link(LinkStatistics),
}

/// Parser state machine — feed it bytes, get events out.
///
/// This is a streaming parser: you call `push_byte()` for each
/// byte received from the UART, and it returns `Some(event)`
/// when a complete, valid frame has been decoded.
///
/// No allocations, no heap, no dependencies. Just a state machine
/// and a 64-byte buffer.
pub struct CrsfParser {
    /// Receive buffer
    buf: [u8; MAX_FRAME_SIZE],
    /// Current write position in buffer
    pos: usize,
    /// Expected frame length (set after reading length byte)
    expected_len: usize,
    /// Parser state
    state: ParserState,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ParserState {
    /// Waiting for sync byte (0xC8)
    WaitSync,
    /// Got sync, waiting for length byte
    WaitLength,
    /// Reading payload bytes until we have expected_len
    Reading,
}

impl CrsfParser {
    /// Create a new parser, ready to receive bytes.
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME_SIZE],
            pos: 0,
            expected_len: 0,
            state: ParserState::WaitSync,
        }
    }

    /// Feed one byte from the UART into the parser.
    ///
    /// Returns `Some(CrsfEvent)` when a complete valid frame
    /// has been decoded, `None` otherwise.
    ///
    /// Call this from your UART Rx interrupt or async read loop.
    pub fn push_byte(&mut self, byte: u8) -> Option<CrsfEvent> {
        match self.state {
            ParserState::WaitSync => {
                if byte == SYNC_BYTE {
                    self.buf[0] = byte;
                    self.pos = 1;
                    self.state = ParserState::WaitLength;
                }
                // else: discard, keep waiting
                None
            }

            ParserState::WaitLength => {
                if byte < MIN_FRAME_LENGTH || byte > MAX_FRAME_LENGTH {
                    // Invalid length — reset
                    self.reset();
                    return None;
                }

                self.buf[1] = byte;
                self.pos = 2;
                // Total bytes to read after length byte = frame_length
                // We need: type + payload + crc = frame_length bytes
                self.expected_len = byte as usize;
                self.state = ParserState::Reading;
                None
            }

            ParserState::Reading => {
                if self.pos < MAX_FRAME_SIZE {
                    self.buf[self.pos] = byte;
                    self.pos += 1;
                }

                // Check if we've received all expected bytes
                // Total frame = sync(1) + length(1) + expected_len
                if self.pos >= 2 + self.expected_len {
                    let result = self.try_decode();
                    self.reset();
                    return result;
                }

                None
            }
        }
    }

    /// Try to decode the complete frame in the buffer.
    fn try_decode(&self) -> Option<CrsfEvent> {
        // CRC covers type + payload (everything after length, except CRC itself)
        // buf layout: [sync, length, type, payload..., crc]
        let crc_range_start = 2; // type byte
        let crc_range_end = 2 + self.expected_len - 1; // exclude CRC byte
        let received_crc = self.buf[2 + self.expected_len - 1];

        let computed_crc = crc8(&self.buf[crc_range_start..crc_range_end]);

        if computed_crc != received_crc {
            return None; // CRC mismatch, discard
        }

        let frame_type = self.buf[2];
        let payload = &self.buf[3..crc_range_end];

        match frame_type {
            0x16 => self.decode_rc_channels(payload),
            0x14 => self.decode_link_statistics(payload),
            _ => None, // Frame type we don't handle — ignore
        }
    }

    /// Decode RC Channels Packed (type 0x16)
    ///
    /// 16 channels × 11 bits = 176 bits = 22 bytes of payload.
    /// Channels are packed LSB-first in a bitstream.
    fn decode_rc_channels(&self, payload: &[u8]) -> Option<CrsfEvent> {
        if payload.len() < 22 {
            return None;
        }

        let mut channels = [0u16; 16];

        // Unpack 11-bit values from the bitstream.
        // Each channel is 11 bits, packed sequentially LSB-first.
        //
        // Channel 0: bits 0-10
        // Channel 1: bits 11-21
        // etc.
        //
        // This is the same packing as SBUS but with 11-bit values.
        channels[0] = ((payload[0] as u16) | ((payload[1] as u16) << 8)) & 0x07FF;
        channels[1] = (((payload[1] as u16) >> 3) | ((payload[2] as u16) << 5)) & 0x07FF;
        channels[2] = (((payload[2] as u16) >> 6) | ((payload[3] as u16) << 2) | ((payload[4] as u16) << 10)) & 0x07FF;
        channels[3] = (((payload[4] as u16) >> 1) | ((payload[5] as u16) << 7)) & 0x07FF;
        channels[4] = (((payload[5] as u16) >> 4) | ((payload[6] as u16) << 4)) & 0x07FF;
        channels[5] = (((payload[6] as u16) >> 7) | ((payload[7] as u16) << 1) | ((payload[8] as u16) << 9)) & 0x07FF;
        channels[6] = (((payload[8] as u16) >> 2) | ((payload[9] as u16) << 6)) & 0x07FF;
        channels[7] = (((payload[9] as u16) >> 5) | ((payload[10] as u16) << 3)) & 0x07FF;
        channels[8] = ((payload[11] as u16) | ((payload[12] as u16) << 8)) & 0x07FF;
        channels[9] = (((payload[12] as u16) >> 3) | ((payload[13] as u16) << 5)) & 0x07FF;
        channels[10] = (((payload[13] as u16) >> 6) | ((payload[14] as u16) << 2) | ((payload[15] as u16) << 10)) & 0x07FF;
        channels[11] = (((payload[15] as u16) >> 1) | ((payload[16] as u16) << 7)) & 0x07FF;
        channels[12] = (((payload[16] as u16) >> 4) | ((payload[17] as u16) << 4)) & 0x07FF;
        channels[13] = (((payload[17] as u16) >> 7) | ((payload[18] as u16) << 1) | ((payload[19] as u16) << 9)) & 0x07FF;
        channels[14] = (((payload[19] as u16) >> 2) | ((payload[20] as u16) << 6)) & 0x07FF;
        channels[15] = (((payload[20] as u16) >> 5) | ((payload[21] as u16) << 3)) & 0x07FF;

        Some(CrsfEvent::Channels(RcChannels { channels }))
    }

    /// Decode Link Statistics (type 0x14)
    ///
    /// 10 bytes of telemetry about the radio link.
    fn decode_link_statistics(&self, payload: &[u8]) -> Option<CrsfEvent> {
        if payload.len() < 10 {
            return None;
        }

        Some(CrsfEvent::Link(LinkStatistics {
            uplink_rssi_ant1: payload[0],
            uplink_rssi_ant2: payload[1],
            uplink_link_quality: payload[2],
            uplink_snr: payload[3] as i8,
            active_antenna: payload[4],
            rf_mode: payload[5],
            uplink_tx_power: payload[6],
            downlink_rssi: payload[7],
            downlink_link_quality: payload[8],
            downlink_snr: payload[9] as i8,
        }))
    }

    /// Reset the parser to wait for the next sync byte.
    fn reset(&mut self) {
        self.state = ParserState::WaitSync;
        self.pos = 0;
        self.expected_len = 0;
    }
}

// ---- CRC8 implementation ----

/// CRC8 lookup table, polynomial 0xD5
/// From the CRSF spec directly.
const CRC8_TABLE: [u8; 256] = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
];

/// Compute CRC8 over a byte slice using the CRSF polynomial.
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc = CRC8_TABLE[(crc ^ byte) as usize];
    }
    crc
}

// ---- Tests ----

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc8_empty() {
        assert_eq!(crc8(&[]), 0);
    }

    #[test]
    fn test_parser_rejects_bad_sync() {
        let mut parser = CrsfParser::new();
        // Random bytes that aren't sync should produce nothing
        assert!(parser.push_byte(0x00).is_none());
        assert!(parser.push_byte(0xFF).is_none());
        assert!(parser.push_byte(0x42).is_none());
    }

    #[test]
    fn test_parser_rejects_invalid_length() {
        let mut parser = CrsfParser::new();
        parser.push_byte(SYNC_BYTE); // sync
        // Length of 0 is invalid (min is 2)
        assert!(parser.push_byte(0x00).is_none());
        // Parser should have reset, so next sync should work
        assert!(parser.push_byte(SYNC_BYTE).is_none());
    }

    #[test]
    fn test_rc_channels_round_trip() {
        // Build a valid RC channels frame with known values
        let mut payload = [0u8; 22];

        // Pack channel 0 = 992 (centre), rest = 0
        // Channel 0 occupies bits 0-10 of the payload
        payload[0] = (992 & 0xFF) as u8;
        payload[1] = ((992 >> 8) & 0x07) as u8;

        // Build complete frame
        let frame_type = 0x16u8;
        let frame_length = (1 + 22 + 1) as u8; // type + payload + crc

        // CRC covers type + payload
        let mut crc_data = [0u8; 23];
        crc_data[0] = frame_type;
        crc_data[1..23].copy_from_slice(&payload);
        let crc = crc8(&crc_data);

        // Feed into parser
        let mut parser = CrsfParser::new();
        assert!(parser.push_byte(SYNC_BYTE).is_none());
        assert!(parser.push_byte(frame_length).is_none());
        assert!(parser.push_byte(frame_type).is_none());
        for &b in &payload {
            assert!(parser.push_byte(b).is_none());
        }
        // Last byte (CRC) should trigger decode
        let event = parser.push_byte(crc);

        match event {
            Some(CrsfEvent::Channels(ch)) => {
                assert_eq!(ch.channels[0], 992);
                // Other channels should be 0
                assert_eq!(ch.channels[1], 0);
            }
            other => panic!("Expected Channels event, got {:?}", other),
        }
    }

    #[test]
    fn test_normalised_conversion() {
        // Centre should be ~0.0
        let centre = RcChannels::to_normalised(992);
        assert!((centre).abs() < 0.01);

        // Min should be ~ -1.0
        let min = RcChannels::to_normalised(172);
        assert!((min + 1.0).abs() < 0.01);

        // Max should be ~ 1.0
        let max = RcChannels::to_normalised(1811);
        assert!((max - 1.0).abs() < 0.01);
    }
}
