// dshot_frame.rs — DShot frame encoder (direct, no uf-dshot).
//
// Ported from the pre-uf-dshot implementation at commit e32a44a's
// `src/drivers/dshot.rs`. Extended with bidirectional support
// (`bidir: bool` parameter): forces the telemetry-request bit to 1
// and inverts the CRC (`~base & 0xF`), matching the BLHeli bidir
// convention and Betaflight's `prepareDshotPacket` at
// `src/main/drivers/dshot.c:111-118`.
//
// Frame structure (16 bits, MSB first):
//
//   [11-bit value] [1-bit telemetry request] [4-bit CRC]
//
// - Value 0          = motor stop
// - Values 1..47     = ESC commands (only honoured when motor stopped)
// - Values 48..2047  = throttle (2000 steps)
//
// CRC = XOR of three 4-bit nibbles of the first 12 bits. In bidir
// mode the CRC is inverted (one's complement, lower 4 bits).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DshotSpeed {
    Dshot150,
    Dshot300,
    Dshot600,
}

impl DshotSpeed {
    pub const fn bitrate_hz(self) -> u32 {
        match self {
            DshotSpeed::Dshot150 => 150_000,
            DshotSpeed::Dshot300 => 300_000,
            DshotSpeed::Dshot600 => 600_000,
        }
    }
}

#[cfg(feature = "firmware")]
impl defmt::Format for DshotSpeed {
    fn format(&self, f: defmt::Formatter) {
        match self {
            DshotSpeed::Dshot150 => defmt::write!(f, "Dshot150"),
            DshotSpeed::Dshot300 => defmt::write!(f, "Dshot300"),
            DshotSpeed::Dshot600 => defmt::write!(f, "Dshot600"),
        }
    }
}

/// DShot command space (raw 11-bit values 0..47 are commands; only
/// honoured by the ESC while the motor is stopped). We only use the
/// few we actually need.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DshotCommand {
    MotorStop = 0,
    Beacon1 = 1,
    Beacon2 = 2,
    Beacon3 = 3,
    Beacon4 = 4,
    Beacon5 = 5,
    EscInfo = 6,
    SpinDirectionNormal = 20,
    SpinDirectionReversed = 21,
}

/// Throttle range applied by the FC side before the DShot offset (+48).
/// `from_normalised(1.0)` maps to throttle raw 1999 → DShot value 2047.
const THROTTLE_MAX_RAW: u16 = 1999;
const DSHOT_THROTTLE_OFFSET: u16 = 48;

/// A raw 16-bit DShot frame, ready to be unpacked MSB-first onto
/// the wire.
#[derive(Debug, Clone, Copy)]
pub struct DshotFrame {
    pub raw: u16,
}

impl DshotFrame {
    /// MotorStop (command 0). In bidir mode this still asserts the
    /// telemetry-request bit and uses an inverted CRC, matching what
    /// BlueJay expects from a bidir-configured ESC.
    pub fn motor_stop(bidir: bool) -> Self {
        Self::encode(DshotCommand::MotorStop as u16, bidir)
    }

    /// Encode an ESC command (1..47). Commands are only honoured by
    /// the ESC while the motor is stopped.
    pub fn command(cmd: DshotCommand, bidir: bool) -> Self {
        Self::encode(cmd as u16, bidir)
    }

    /// Throttle in the FC-side 0..1999 range. Values are clamped
    /// before the +48 DShot offset is applied.
    pub fn throttle(raw: u16, bidir: bool) -> Self {
        let clamped = raw.min(THROTTLE_MAX_RAW);
        Self::encode(clamped + DSHOT_THROTTLE_OFFSET, bidir)
    }

    /// Normalised throttle in 0.0..1.0. Values ≤ 0 produce MotorStop.
    pub fn from_normalised(v: f32, bidir: bool) -> Self {
        if v <= 0.0 {
            return Self::motor_stop(bidir);
        }
        let raw = (v * THROTTLE_MAX_RAW as f32) as u16;
        Self::throttle(raw, bidir)
    }

    /// Build a frame from a raw 11-bit value (0..2047). Useful for
    /// tests and direct register-level work; production callers should
    /// prefer `throttle` / `command` / `motor_stop`.
    pub fn from_raw_value(value_11: u16, bidir: bool) -> Self {
        Self::encode(value_11, bidir)
    }

    fn encode(value_11: u16, bidir: bool) -> Self {
        debug_assert!(value_11 <= 2047, "DShot value field is 11 bits");
        let telemetry_bit: u16 = if bidir { 1 } else { 0 };
        let data_12 = (value_11 << 1) | telemetry_bit;

        let base_crc: u16 = (data_12 ^ (data_12 >> 4) ^ (data_12 >> 8)) & 0x0F;
        let crc: u16 = if bidir { (!base_crc) & 0x0F } else { base_crc };

        let raw = (data_12 << 4) | crc;
        DshotFrame { raw }
    }

    /// The 4-bit CRC nibble actually packed in the frame.
    pub const fn crc(&self) -> u8 {
        (self.raw & 0x0F) as u8
    }

    /// The 12-bit data field (value << 1 | telem).
    pub const fn data_12(&self) -> u16 {
        self.raw >> 4
    }

    /// Unpack the 16 bits in MSB-first transmission order. Index 0 is
    /// the first bit on the wire (= `(raw >> 15) & 1`).
    pub fn bits_msb_first(&self) -> [bool; 16] {
        let mut bits = [false; 16];
        for (i, bit) in bits.iter_mut().enumerate() {
            *bit = ((self.raw >> (15 - i)) & 1) != 0;
        }
        bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motor_stop_standard() {
        // value=0, telem=0, CRC=0 → raw=0x0000
        let f = DshotFrame::motor_stop(false);
        assert_eq!(f.raw, 0x0000);
        assert_eq!(f.crc(), 0);
    }

    #[test]
    fn motor_stop_bidir() {
        // value=0, telem=1 → data_12 = 0x001
        // base_crc = 0x1 ^ 0x0 ^ 0x0 = 0x1
        // bidir crc = ~0x1 & 0xF = 0xE
        // raw = (0x001 << 4) | 0xE = 0x1E
        let f = DshotFrame::motor_stop(true);
        assert_eq!(f.raw, 0x001E);
        assert_eq!(f.crc(), 0xE);
    }

    #[test]
    fn throttle_standard_crc() {
        // raw=200 → +48 = 248 → data_12 = 248 << 1 = 0x1F0
        // base_crc = 0x1F0 ^ 0x01F ^ 0x001 = (0x1F0 ^ 0x01F) = 0x1EF
        //            0x1EF ^ 0x001 = 0x1EE, & 0xF = 0xE
        let f = DshotFrame::throttle(200, false);
        assert_eq!(f.data_12(), 0x1F0);
        assert_eq!(f.crc(), 0xE);
    }

    #[test]
    fn throttle_bidir_crc_is_inverted() {
        // raw=200 → +48 = 248 → data_12 = (248 << 1) | 1 = 0x1F1
        // base_crc = (0x1F1 ^ 0x01F ^ 0x001) & 0xF
        //          = (0x1EE ^ 0x001) & 0xF = 0x1EF & 0xF = 0xF
        // bidir crc = ~0xF & 0xF = 0x0
        let f = DshotFrame::throttle(200, true);
        assert_eq!(f.data_12(), 0x1F1);
        assert_eq!(f.crc(), 0x0);
    }

    #[test]
    fn throttle_clamps_to_max() {
        // raw=10000 → clamp to 1999 → +48 = 2047 → all-ones throttle
        let f = DshotFrame::throttle(10_000, false);
        let value_11 = f.data_12() >> 1;
        assert_eq!(value_11, 2047);
    }

    #[test]
    fn bits_msb_first_starts_with_msb() {
        // raw = 0xABCD = 0b1010_1011_1100_1101
        let f = DshotFrame { raw: 0xABCD };
        let bits = f.bits_msb_first();
        assert_eq!(bits[0], true); // bit 15
        assert_eq!(bits[1], false); // bit 14
        assert_eq!(bits[2], true); // bit 13
        assert_eq!(bits[15], true); // bit 0
    }

    #[test]
    fn from_normalised_zero_is_motor_stop() {
        let f = DshotFrame::from_normalised(0.0, true);
        assert_eq!(f.raw, DshotFrame::motor_stop(true).raw);
    }

    #[test]
    fn from_normalised_negative_is_motor_stop() {
        let f = DshotFrame::from_normalised(-1.0, true);
        assert_eq!(f.raw, DshotFrame::motor_stop(true).raw);
    }

    #[test]
    fn from_normalised_one_is_max_throttle() {
        let f = DshotFrame::from_normalised(1.0, false);
        let value_11 = f.data_12() >> 1;
        assert_eq!(value_11, 2047);
    }

    #[test]
    fn speed_bit_periods() {
        // At 240 MHz timer clock, DShot600 = 400-tick cells
        assert_eq!(240_000_000 / DshotSpeed::Dshot600.bitrate_hz(), 400);
        assert_eq!(240_000_000 / DshotSpeed::Dshot300.bitrate_hz(), 800);
        assert_eq!(240_000_000 / DshotSpeed::Dshot150.bitrate_hz(), 1600);
    }
}
