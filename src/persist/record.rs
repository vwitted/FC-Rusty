//! On-flash config record: layout, CRC, (de)serialisation. Pure no_std,
//! host-tested. The firmware flash wrapper lives in `flash.rs`.

/// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF).
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Identifies a valid record. ASCII "FCR1".
pub const MAGIC: u32 = 0x4643_5231;
/// Payload schema version. Bump when `Config` grows.
pub const VERSION: u16 = 1;
/// Serialised payload length: 3*f32 + f32 + bool = 17 bytes.
pub const PAYLOAD_LEN: usize = 17;
/// Whole record, padded to the H7 32-byte flash word.
pub const RECORD_LEN: usize = 32;

/// Persisted configuration (v1). Defaults are the safe "uncalibrated"
/// state — identical in effect to having no stored record at all.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Magnetometer hard-iron offset, sensor native frame, µT.
    pub mag_hard_iron_ut: [f32; 3],
    /// Magnetic declination, radians, east-positive. 0.0 = none.
    pub declination_rad: f32,
    /// True once a real calibration has been written (vs. defaults).
    pub mag_calibrated: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mag_hard_iron_ut: [0.0; 3],
            declination_rad: 0.0,
            mag_calibrated: false,
        }
    }
}

/// Serialise + CRC into a flash-word-aligned record.
///
/// Layout: magic[0..4] | version[4..6] | len[6..8] | payload[8..25] |
/// zero-pad[25..28] | crc32[28..32]. CRC covers bytes [0..28].
pub fn encode(cfg: &Config) -> [u8; RECORD_LEN] {
    let mut b = [0u8; RECORD_LEN];
    b[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    b[4..6].copy_from_slice(&VERSION.to_le_bytes());
    b[6..8].copy_from_slice(&(PAYLOAD_LEN as u16).to_le_bytes());
    b[8..12].copy_from_slice(&cfg.mag_hard_iron_ut[0].to_le_bytes());
    b[12..16].copy_from_slice(&cfg.mag_hard_iron_ut[1].to_le_bytes());
    b[16..20].copy_from_slice(&cfg.mag_hard_iron_ut[2].to_le_bytes());
    b[20..24].copy_from_slice(&cfg.declination_rad.to_le_bytes());
    b[24] = cfg.mag_calibrated as u8;
    // [25..28] stay zero (deterministic padding, covered by the CRC).
    let crc = crc32(&b[0..28]);
    b[28..32].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Validate and parse a record. Returns `None` on any mismatch (caller
/// uses `Config::default()`), including a blank (all-0xFF) sector.
pub fn decode(bytes: &[u8]) -> Option<Config> {
    if bytes.len() < RECORD_LEN {
        return None;
    }
    let b = &bytes[0..RECORD_LEN];
    if u32::from_le_bytes(b[0..4].try_into().ok()?) != MAGIC {
        return None;
    }
    if u16::from_le_bytes(b[4..6].try_into().ok()?) != VERSION {
        return None;
    }
    if u16::from_le_bytes(b[6..8].try_into().ok()?) as usize != PAYLOAD_LEN {
        return None;
    }
    let crc = u32::from_le_bytes(b[28..32].try_into().ok()?);
    if crc != crc32(&b[0..28]) {
        return None;
    }
    Some(Config {
        mag_hard_iron_ut: [
            f32::from_le_bytes(b[8..12].try_into().ok()?),
            f32::from_le_bytes(b[12..16].try_into().ok()?),
            f32::from_le_bytes(b[16..20].try_into().ok()?),
        ],
        declination_rad: f32::from_le_bytes(b[20..24].try_into().ok()?),
        mag_calibrated: b[24] != 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_known_vector() {
        // Standard CRC-32 check value for the ASCII string "123456789".
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn record_len_is_flash_word_aligned() {
        assert_eq!(RECORD_LEN % 32, 0);
    }

    #[test]
    fn default_is_uncalibrated() {
        let c = Config::default();
        assert!(!c.mag_calibrated);
        assert_eq!(c.mag_hard_iron_ut, [0.0; 3]);
        assert_eq!(c.declination_rad, 0.0);
    }

    #[test]
    fn encode_decode_round_trips() {
        let c = Config {
            mag_hard_iron_ut: [12.5, -3.25, 40.0],
            declination_rad: 0.0052,
            mag_calibrated: true,
        };
        let bytes = encode(&c);
        assert_eq!(decode(&bytes), Some(c));
    }

    #[test]
    fn decode_rejects_bad_magic() {
        let mut bytes = encode(&Config::default());
        bytes[0] ^= 0xFF;
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_wrong_version() {
        let mut bytes = encode(&Config::default());
        bytes[4] = 0xFE; // corrupt version field
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_bad_crc() {
        let mut bytes = encode(&Config::default());
        bytes[24] ^= 0x01; // flip a payload bit; CRC no longer matches
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_blank_sector() {
        // Erased H7 flash reads as all 0xFF.
        let bytes = [0xFFu8; RECORD_LEN];
        assert_eq!(decode(&bytes), None);
    }

    #[test]
    fn decode_rejects_truncated_input() {
        let bytes = encode(&Config::default());
        assert_eq!(decode(&bytes[0..RECORD_LEN - 1]), None);
    }
}
