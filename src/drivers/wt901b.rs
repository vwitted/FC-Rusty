// wt901b.rs — WitMotion WT901B IMU protocol parser
//
// Parses the WIT Standard Communication Protocol used by
// WT901B and similar WitMotion IMUs over serial UART.
//
// Protocol: each packet is 11 bytes:
//   [0x55] [TYPE] [D0] [D1] [D2] [D3] [D4] [D5] [D6] [D7] [SUM]
//
// TYPE determines what the 8 data bytes contain.
// Data is little-endian signed 16-bit integers (4 per packet).
// SUM = (0x55 + TYPE + D0..D7) & 0xFF
//
// At 200 Hz with multiple outputs enabled, packets arrive
// interleaved — you might get accel, then gyro, then angle,
// then mag, etc. The driver caches the latest value for each
// type so the consumer always has a complete picture.

/// Sync byte for all WitMotion packets
const SYNC_BYTE: u8 = 0x55;

/// Total packet size including sync and checksum
const PACKET_SIZE: usize = 11;

/// Packet type identifiers
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum PacketType {
    Time = 0x50,
    Acceleration = 0x51,
    AngularVelocity = 0x52,
    Angle = 0x53,
    Magnetic = 0x54,
    Barometer = 0x56,
    Quaternion = 0x59,
}

impl PacketType {
    fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x50 => Some(Self::Time),
            0x51 => Some(Self::Acceleration),
            0x52 => Some(Self::AngularVelocity),
            0x53 => Some(Self::Angle),
            0x54 => Some(Self::Magnetic),
            0x56 => Some(Self::Barometer),
            0x59 => Some(Self::Quaternion),
            _ => None,
        }
    }
}

/// Parsed IMU data — all fields are in SI-ish units.
///
/// Not every field is updated every packet. The `updated`
/// flags tell you what's fresh since the last call to
/// `take_updates()`.
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    // ---- Acceleration (from 0x51) ----
    /// Acceleration in m/s² (x, y, z)
    pub accel: [f32; 3],
    /// Temperature in °C (packed in accel packet)
    pub temperature: f32,

    // ---- Angular velocity (from 0x52) ----
    /// Angular rates in °/s (x, y, z)
    pub gyro: [f32; 3],

    // ---- Euler angles (from 0x53) ----
    /// Roll, Pitch, Yaw in degrees
    pub angle: [f32; 3],

    // ---- Magnetic field (from 0x54) ----
    /// Magnetic field raw values (x, y, z) in LSB
    pub mag: [i16; 3],

    // ---- Barometer (from 0x56) ----
    /// Atmospheric pressure in Pa
    pub pressure: u32,
    /// Barometric altitude in cm
    pub altitude_cm: i32,

    // ---- Quaternion (from 0x59) ----
    /// Quaternion [w, x, y, z] (normalised)
    pub quaternion: [f32; 4],

    /// Bitfield of which data has been updated since last check
    pub updated: u8,
}

// Bit flags for the `updated` field
pub const UPDATED_ACCEL: u8 = 1 << 0;
pub const UPDATED_GYRO: u8 = 1 << 1;
pub const UPDATED_ANGLE: u8 = 1 << 2;
pub const UPDATED_MAG: u8 = 1 << 3;
pub const UPDATED_BARO: u8 = 1 << 4;
pub const UPDATED_QUAT: u8 = 1 << 5;

impl ImuData {
    pub const fn new() -> Self {
        Self {
            accel: [0.0; 3],
            temperature: 0.0,
            gyro: [0.0; 3],
            angle: [0.0; 3],
            mag: [0; 3],
            pressure: 0,
            altitude_cm: 0,
            quaternion: [1.0, 0.0, 0.0, 0.0], // identity
            updated: 0,
        }
    }

    /// Check if a specific data type was updated, then clear the flag.
    pub fn was_updated(&mut self, flag: u8) -> bool {
        let yes = self.updated & flag != 0;
        self.updated &= !flag;
        yes
    }

    /// Clear all update flags.
    pub fn clear_updates(&mut self) {
        self.updated = 0;
    }
}

/// Streaming parser for WT901B packets.
///
/// Same pattern as CrsfParser: feed bytes via `push_byte()`,
/// it updates the internal `ImuData` when valid packets arrive.
pub struct Wt901bParser {
    buf: [u8; PACKET_SIZE],
    pos: usize,
    state: ParserState,
    pub data: ImuData,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ParserState {
    WaitSync,
    WaitType,
    Reading,
}

impl Wt901bParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; PACKET_SIZE],
            pos: 0,
            state: ParserState::WaitSync,
            data: ImuData::new(),
        }
    }

    /// Feed one byte from the UART.
    ///
    /// Returns `Some(PacketType)` when a complete valid packet
    /// has been decoded and `self.data` updated, `None` otherwise.
    pub fn push_byte(&mut self, byte: u8) -> Option<PacketType> {
        match self.state {
            ParserState::WaitSync => {
                if byte == SYNC_BYTE {
                    self.buf[0] = byte;
                    self.pos = 1;
                    self.state = ParserState::WaitType;
                }
                None
            }

            ParserState::WaitType => {
                // Check this is a known type byte
                if PacketType::from_byte(byte).is_some() {
                    self.buf[1] = byte;
                    self.pos = 2;
                    self.state = ParserState::Reading;
                    None
                } else {
                    // Unknown type or this wasn't actually a packet start.
                    // Could be a 0x55 data byte followed by a non-type byte.
                    // Reset, but check if this byte is itself a sync.
                    self.reset();
                    if byte == SYNC_BYTE {
                        self.buf[0] = byte;
                        self.pos = 1;
                        self.state = ParserState::WaitType;
                    }
                    None
                }
            }

            ParserState::Reading => {
                self.buf[self.pos] = byte;
                self.pos += 1;

                if self.pos >= PACKET_SIZE {
                    let result = self.try_decode();
                    self.reset();
                    return result;
                }
                None
            }
        }
    }

    fn try_decode(&mut self) -> Option<PacketType> {
        // Verify checksum: sum of bytes 0..10, truncated to u8
        let computed: u8 = self.buf[..PACKET_SIZE - 1]
            .iter()
            .fold(0u8, |acc, &b| acc.wrapping_add(b));

        if computed != self.buf[PACKET_SIZE - 1] {
            return None;
        }

        let ptype = PacketType::from_byte(self.buf[1])?;
        let d = &self.buf[2..10]; // 8 data bytes

        match ptype {
            PacketType::Acceleration => {
                self.data.accel[0] = raw_i16(d[0], d[1]) as f32 / 32768.0 * 16.0 * 9.8;
                self.data.accel[1] = raw_i16(d[2], d[3]) as f32 / 32768.0 * 16.0 * 9.8;
                self.data.accel[2] = raw_i16(d[4], d[5]) as f32 / 32768.0 * 16.0 * 9.8;
                self.data.temperature = raw_i16(d[6], d[7]) as f32 / 100.0;
                self.data.updated |= UPDATED_ACCEL;
            }

            PacketType::AngularVelocity => {
                self.data.gyro[0] = raw_i16(d[0], d[1]) as f32 / 32768.0 * 2000.0;
                self.data.gyro[1] = raw_i16(d[2], d[3]) as f32 / 32768.0 * 2000.0;
                self.data.gyro[2] = raw_i16(d[4], d[5]) as f32 / 32768.0 * 2000.0;
                // d[6..7] is voltage (non-bluetooth products: invalid)
                self.data.updated |= UPDATED_GYRO;
            }

            PacketType::Angle => {
                self.data.angle[0] = raw_i16(d[0], d[1]) as f32 / 32768.0 * 180.0;
                self.data.angle[1] = raw_i16(d[2], d[3]) as f32 / 32768.0 * 180.0;
                self.data.angle[2] = raw_i16(d[4], d[5]) as f32 / 32768.0 * 180.0;
                // d[6..7] is version number
                self.data.updated |= UPDATED_ANGLE;
            }

            PacketType::Magnetic => {
                self.data.mag[0] = raw_i16(d[0], d[1]);
                self.data.mag[1] = raw_i16(d[2], d[3]);
                self.data.mag[2] = raw_i16(d[4], d[5]);
                // d[6..7] is temperature again
                self.data.updated |= UPDATED_MAG;
            }

            PacketType::Barometer => {
                // Pressure is 32-bit unsigned, little-endian
                self.data.pressure =
                    (d[0] as u32)
                    | ((d[1] as u32) << 8)
                    | ((d[2] as u32) << 16)
                    | ((d[3] as u32) << 24);

                // Altitude is 32-bit signed, little-endian, in cm
                self.data.altitude_cm =
                    (d[4] as i32)
                    | ((d[5] as i32) << 8)
                    | ((d[6] as i32) << 16)
                    | ((d[7] as i32) << 24);

                self.data.updated |= UPDATED_BARO;
            }

            PacketType::Quaternion => {
                // Note: WitMotion order is q0,q1,q2,q3 = w,x,y,z
                self.data.quaternion[0] = raw_i16(d[0], d[1]) as f32 / 32768.0;
                self.data.quaternion[1] = raw_i16(d[2], d[3]) as f32 / 32768.0;
                self.data.quaternion[2] = raw_i16(d[4], d[5]) as f32 / 32768.0;
                self.data.quaternion[3] = raw_i16(d[6], d[7]) as f32 / 32768.0;
                self.data.updated |= UPDATED_QUAT;
            }

            PacketType::Time => {
                // We don't need time for flight control, skip
            }
        }

        Some(ptype)
    }

    fn reset(&mut self) {
        self.state = ParserState::WaitSync;
        self.pos = 0;
    }
}

/// Combine two bytes into a signed 16-bit integer (little-endian).
fn raw_i16(low: u8, high: u8) -> i16 {
    (high as i16) << 8 | (low as i16)
}

// ---- Configuration commands (write to WT901B over UART Tx) ----

/// Build a 5-byte write command for the WT901B.
///
/// Format: [0xFF] [0xAA] [ADDR] [DATA_L] [DATA_H]
///
/// Remember: you must send the unlock command first,
/// then your command, then the save command.
pub fn write_command(addr: u8, data: u16) -> [u8; 5] {
    [0xFF, 0xAA, addr, (data & 0xFF) as u8, (data >> 8) as u8]
}

/// Unlock command — must be sent before any write operations.
pub const UNLOCK: [u8; 5] = [0xFF, 0xAA, 0x69, 0x88, 0xB5];

/// Save command — must be sent after write operations to persist.
pub const SAVE: [u8; 5] = [0xFF, 0xAA, 0x00, 0x00, 0x00];

/// Common configuration helpers
pub mod config {
    use super::write_command;

    /// Set output rate. Common values:
    /// 0x06 = 10Hz, 0x07 = 20Hz, 0x08 = 50Hz,
    /// 0x09 = 100Hz, 0x0B = 200Hz
    pub fn set_output_rate(rate_code: u8) -> [u8; 5] {
        write_command(0x03, rate_code as u16)
    }

    /// Set baud rate. Common values:
    /// 0x02 = 9600, 0x06 = 115200, 0x07 = 230400
    pub fn set_baud_rate(baud_code: u8) -> [u8; 5] {
        write_command(0x04, baud_code as u16)
    }

    /// Set which data packets are output.
    /// Bits: 0=time, 1=acc, 2=gyro, 3=angle, 4=mag,
    ///       5=port, 6=baro, 9=quat
    ///
    /// For flight control you'd want at minimum:
    /// acc + gyro + angle + quat = 0x020E
    /// Add baro: 0x024E
    /// Add mag: 0x025E
    pub fn set_output_content(flags: u16) -> [u8; 5] {
        write_command(0x02, flags)
    }

    /// Set 6-axis mode (gyro-only heading, no mag influence)
    pub fn set_6axis_mode() -> [u8; 5] {
        write_command(0x24, 0x0001)
    }

    /// Set 9-axis mode (mag-fused heading)
    pub fn set_9axis_mode() -> [u8; 5] {
        write_command(0x24, 0x0000)
    }

    /// Set bandwidth. 0x00=256Hz, 0x01=188Hz, 0x02=98Hz,
    /// 0x03=42Hz, 0x04=20Hz
    pub fn set_bandwidth(bw_code: u8) -> [u8; 5] {
        write_command(0x1F, bw_code as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a valid packet from type and 8 data bytes
    fn make_packet(ptype: u8, data: &[u8; 8]) -> [u8; 11] {
        let mut pkt = [0u8; 11];
        pkt[0] = SYNC_BYTE;
        pkt[1] = ptype;
        pkt[2..10].copy_from_slice(data);
        // Checksum
        let sum: u8 = pkt[..10].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        pkt[10] = sum;
        pkt
    }

    #[test]
    fn test_parse_acceleration() {
        let mut parser = Wt901bParser::new();

        // Encode 1g on Z axis: raw = 32768 / 16 = 2048
        // In i16 LE: low = 0x00, high = 0x08
        let data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00];
        let pkt = make_packet(0x51, &data);

        let mut result = None;
        for &b in &pkt {
            if let Some(t) = parser.push_byte(b) {
                result = Some(t);
            }
        }

        assert_eq!(result, Some(PacketType::Acceleration));
        // 2048 / 32768 * 16 * 9.8 ≈ 9.8 m/s²
        assert!((parser.data.accel[2] - 9.8).abs() < 0.1);
        assert!(parser.data.updated & UPDATED_ACCEL != 0);
    }

    #[test]
    fn test_parse_angle() {
        let mut parser = Wt901bParser::new();

        // Encode 45° roll: raw = 45 * 32768 / 180 = 8192
        // 8192 = 0x2000, LE: low=0x00, high=0x20
        let data = [0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let pkt = make_packet(0x53, &data);

        for &b in &pkt {
            parser.push_byte(b);
        }

        // 8192 / 32768 * 180 = 45.0
        assert!((parser.data.angle[0] - 45.0).abs() < 0.1);
    }

    #[test]
    fn test_parse_quaternion() {
        let mut parser = Wt901bParser::new();

        // Identity quaternion: w=1, x=0, y=0, z=0
        // w=1.0 → raw = 32768, but i16 max is 32767
        // WitMotion uses 32768 as the divisor, so w=1.0 → raw=32767
        // Actually let's use w=0.5 → raw=16384=0x4000
        let data = [0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let pkt = make_packet(0x59, &data);

        for &b in &pkt {
            parser.push_byte(b);
        }

        assert!((parser.data.quaternion[0] - 0.5).abs() < 0.01);
        assert!(parser.data.updated & UPDATED_QUAT != 0);
    }

    #[test]
    fn test_bad_checksum_rejected() {
        let mut parser = Wt901bParser::new();

        let mut pkt = make_packet(0x51, &[0; 8]);
        pkt[10] = pkt[10].wrapping_add(1); // corrupt checksum

        let mut got_packet = false;
        for &b in &pkt {
            if parser.push_byte(b).is_some() {
                got_packet = true;
            }
        }
        assert!(!got_packet);
    }

    #[test]
    fn test_barometer_parsing() {
        let mut parser = Wt901bParser::new();

        // Pressure = 101325 Pa = 0x0001_8BCD (little-endian: CD 8B 01 00)
        // Altitude = 500 cm = 0x000001F4 (little-endian: F4 01 00 00)
        let data = [0xCD, 0x8B, 0x01, 0x00, 0xF4, 0x01, 0x00, 0x00];
        let pkt = make_packet(0x56, &data);

        for &b in &pkt {
            parser.push_byte(b);
        }

        assert_eq!(parser.data.pressure, 101325);
        assert_eq!(parser.data.altitude_cm, 500);
    }

    #[test]
    fn test_resyncs_after_garbage() {
        let mut parser = Wt901bParser::new();

        // Feed some garbage first
        for &b in &[0x12, 0x34, 0x56, 0x78, 0x9A] {
            parser.push_byte(b);
        }

        // Then a valid packet
        let data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let pkt = make_packet(0x52, &data);

        let mut got_packet = false;
        for &b in &pkt {
            if parser.push_byte(b).is_some() {
                got_packet = true;
            }
        }
        assert!(got_packet);
    }

    #[test]
    fn test_write_command_format() {
        let cmd = write_command(0x03, 0x000B); // set 200Hz
        assert_eq!(cmd, [0xFF, 0xAA, 0x03, 0x0B, 0x00]);
    }

    #[test]
    fn test_unlock_command() {
        assert_eq!(UNLOCK, [0xFF, 0xAA, 0x69, 0x88, 0xB5]);
    }
}
