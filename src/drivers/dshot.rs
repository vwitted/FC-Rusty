// dshot.rs — DShot protocol encoder
//
// DShot is a digital protocol for FC→ESC communication.
// Each frame is 16 bits sent as PWM pulses where the duty
// cycle encodes 1 or 0:
//
//   Bit "1": high for 75% of the bit period
//   Bit "0": high for 37.5% of the bit period
//
// Frame structure (16 bits, MSB first):
//   [11-bit throttle] [1-bit telemetry request] [4-bit CRC]
//
// Throttle range: 48-2047 (0 = disarmed, 1-47 = special commands)
// CRC: XOR of three 4-bit nibbles of the first 12 bits
//
// DShot variants differ only in bit rate:
//   DShot150:  150 kbit/s  →  6.67 µs per bit
//   DShot300:  300 kbit/s  →  3.33 µs per bit
//   DShot600:  600 kbit/s  →  1.67 µs per bit
//
// On STM32, the standard approach is to use a timer in PWM mode
// with DMA feeding the CCR (compare) register to change the
// duty cycle for each bit. The timer auto-reload sets the bit
// period, and the DMA writes 16 compare values (one per bit).

/// DShot speed variants.
#[derive(Debug, Clone, Copy)]
pub enum DshotSpeed {
    /// 150 kbit/s — most tolerant of signal quality
    Dshot150,
    /// 300 kbit/s — good balance, works with bidirectional
    Dshot300,
    /// 600 kbit/s — most common, fast enough for 8kHz PID
    Dshot600,
}

impl DshotSpeed {
    /// Bit rate in bits per second.
    pub const fn bitrate(self) -> u32 {
        match self {
            DshotSpeed::Dshot150 => 150_000,
            DshotSpeed::Dshot300 => 300_000,
            DshotSpeed::Dshot600 => 600_000,
        }
    }

    /// Calculate the timer auto-reload value (bit period in ticks)
    /// for a given timer clock frequency.
    ///
    /// For example, with an 84 MHz timer clock and DShot600:
    ///   bit_period = 84_000_000 / 600_000 = 140 ticks
    pub const fn bit_period_ticks(self, timer_clock_hz: u32) -> u16 {
        (timer_clock_hz / self.bitrate()) as u16
    }

    /// Timer compare value for a "1" bit (75% duty cycle).
    pub const fn t1h_ticks(self, timer_clock_hz: u32) -> u16 {
        // 3/4 of the bit period
        (self.bit_period_ticks(timer_clock_hz) as u32 * 3 / 4) as u16
    }

    /// Timer compare value for a "0" bit (37.5% duty cycle).
    pub const fn t0h_ticks(self, timer_clock_hz: u32) -> u16 {
        // 3/8 of the bit period
        (self.bit_period_ticks(timer_clock_hz) as u32 * 3 / 8) as u16
    }
}

/// A raw DShot frame (16 bits), ready for transmission.
#[derive(Debug, Clone, Copy)]
pub struct DshotFrame {
    /// The 16-bit frame value (MSB first when transmitted)
    pub raw: u16,
}

impl DshotFrame {
    /// Encode a throttle value (0-2047) into a DShot frame.
    ///
    /// Throttle 0 = disarmed/stop.
    /// Throttle 48-2047 = actual motor speed (2000 steps).
    /// Throttle 1-47 = special commands (only when motor stopped).
    ///
    /// `telemetry`: set to true to request telemetry from the ESC.
    /// In bidirectional DShot, this triggers an eRPM response.
    pub fn from_throttle(throttle: u16, telemetry: bool) -> Self {
        debug_assert!(throttle <= 2047, "DShot throttle max is 2047");

        // Build the 12-bit value: [11-bit throttle][1-bit telemetry]
        let value = (throttle << 1) | (telemetry as u16);

        // CRC: XOR the three 4-bit nibbles of the 12-bit value
        let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

        // Full 16-bit frame: [12-bit value][4-bit CRC]
        let raw = (value << 4) | crc;

        DshotFrame { raw }
    }

    /// Encode the disarmed/stop command.
    pub fn disarmed() -> Self {
        Self::from_throttle(0, false)
    }

    /// Convert a normalised throttle (0.0 - 1.0) to a DShot frame.
    ///
    /// Maps 0.0 → throttle 48 (minimum), 1.0 → throttle 2047 (maximum).
    /// Returns disarmed frame if value is <= 0.0.
    pub fn from_normalised(value: f32, telemetry: bool) -> Self {
        if value <= 0.0 {
            return Self::disarmed();
        }
        // Map 0.0..1.0 → 48..2047
        let throttle = (value * 1999.0) as u16 + 48;
        let throttle = throttle.min(2047);
        Self::from_throttle(throttle, telemetry)
    }

    /// Fill a buffer with timer compare values for DMA transmission.
    ///
    /// This is the key function for hardware output. It converts
    /// the 16-bit frame into 16 timer compare values that, when
    /// fed to a PWM timer via DMA, produce the correct DShot
    /// waveform on the output pin.
    ///
    /// The buffer should be 17 or 18 elements long — 16 for the
    /// data bits plus 1-2 zeros to ensure the line goes low after
    /// the last bit (the "reset" period).
    ///
    /// `t1h`: compare value for a "1" bit (from DshotSpeed::t1h_ticks)
    /// `t0h`: compare value for a "0" bit (from DshotSpeed::t0h_ticks)
    pub fn fill_dma_buffer(&self, buf: &mut [u16], t1h: u16, t0h: u16) {
        debug_assert!(buf.len() >= 17, "DMA buffer needs at least 17 entries");

        // MSB first — bit 15 is sent first
        for i in 0..16 {
            let bit = (self.raw >> (15 - i)) & 1;
            buf[i] = if bit == 1 { t1h } else { t0h };
        }

        // Trailing zero(s) to pull the line low after the last bit
        for i in 16..buf.len() {
            buf[i] = 0;
        }
    }
}

/// Special DShot commands (sent as throttle values 1-47, motor must be stopped).
/// These must be sent repeatedly (typically 10 times) to take effect.
#[allow(dead_code)]
pub mod commands {
    pub const MOTOR_STOP: u16 = 0;
    pub const BEACON1: u16 = 1;
    pub const BEACON2: u16 = 2;
    pub const BEACON3: u16 = 3;
    pub const BEACON4: u16 = 4;
    pub const BEACON5: u16 = 5;
    pub const ESC_INFO: u16 = 6;
    pub const SPIN_DIRECTION_NORMAL: u16 = 20;
    pub const SPIN_DIRECTION_REVERSED: u16 = 21;
    pub const SIGNAL_LINE_TELEMETRY_DISABLE: u16 = 32;
    pub const SIGNAL_LINE_TELEMETRY_ENABLE: u16 = 33;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_disarmed_frame() {
        let frame = DshotFrame::disarmed();
        // Throttle 0, telemetry 0 → value = 0, CRC = 0
        assert_eq!(frame.raw, 0x0000);
    }

    #[test]
    fn test_crc_calculation() {
        // Throttle 1046, no telemetry
        // value = 1046 << 1 = 2092 = 0x82C
        // CRC = 0x8 ^ 0x2 ^ 0xC = 0x6 (wait, let me compute properly)
        // value = 0x82C → nibbles: 0x8, 0x2, 0xC
        // CRC = 0xC ^ 0x2 ^ 0x8 = 0x6... wait:
        // value = (1046 << 1) | 0 = 2092
        // 2092 in binary = 0000_1000_0010_1100
        // but only 12 bits: 1000_0010_1100
        // nibbles: 0x8, 0x2, 0xC
        // CRC = (0x82C ^ 0x082 ^ 0x008) & 0xF
        //     = (0x82C ^ 0x082) = 0x8AE, ^ 0x008 = 0x8A6, & 0xF = 0x6
        let frame = DshotFrame::from_throttle(1046, false);
        let crc = frame.raw & 0x0F;
        let value_check = frame.raw >> 4;
        let expected_crc = (value_check ^ (value_check >> 4) ^ (value_check >> 8)) & 0x0F;
        assert_eq!(crc, expected_crc);
    }

    #[test]
    fn test_telemetry_bit() {
        let no_telem = DshotFrame::from_throttle(100, false);
        let with_telem = DshotFrame::from_throttle(100, true);
        // Telemetry bit is bit 4 of the raw frame (bit 0 of the 12-bit value)
        let no_telem_bit = (no_telem.raw >> 4) & 1;
        let with_telem_bit = (with_telem.raw >> 4) & 1;
        assert_eq!(no_telem_bit, 0);
        assert_eq!(with_telem_bit, 1);
    }

    #[test]
    fn test_dma_buffer_generation() {
        let frame = DshotFrame::from_throttle(2047, false);
        // 2047 = all 1s in 11 bits, so most bits should be T1H

        let t1h = 105u16; // Example: 75% of 140
        let t0h = 52u16;  // Example: 37.5% of 140
        let mut buf = [0u16; 18];
        frame.fill_dma_buffer(&mut buf, t1h, t0h);

        // Trailing entries should be 0
        assert_eq!(buf[16], 0);
        assert_eq!(buf[17], 0);

        // All data entries should be either t1h or t0h
        for i in 0..16 {
            assert!(buf[i] == t1h || buf[i] == t0h,
                    "bit {} was {} (expected {} or {})", i, buf[i], t1h, t0h);
        }
    }

    #[test]
    fn test_normalised_range() {
        // 0.0 should be disarmed
        let zero = DshotFrame::from_normalised(0.0, false);
        assert_eq!(zero.raw, 0);

        // Negative should also be disarmed
        let neg = DshotFrame::from_normalised(-0.5, false);
        assert_eq!(neg.raw, 0);

        // 1.0 should produce max throttle (2047)
        let full = DshotFrame::from_normalised(1.0, false);
        let throttle = (full.raw >> 5) & 0x7FF; // extract 11-bit throttle
        assert_eq!(throttle, 2047);

        // Small positive should produce near-minimum throttle
        let min = DshotFrame::from_normalised(0.001, false);
        let throttle = (min.raw >> 5) & 0x7FF;
        assert!(throttle >= 48 && throttle <= 50);
    }

    #[test]
    fn test_timer_calculations() {
        // STM32F405 timer clock: 84 MHz (APB1 timers × 2)
        let clk = 84_000_000u32;

        let period = DshotSpeed::Dshot600.bit_period_ticks(clk);
        assert_eq!(period, 140); // 84MHz / 600kHz = 140

        let t1h = DshotSpeed::Dshot600.t1h_ticks(clk);
        assert_eq!(t1h, 105); // 140 * 3/4 = 105

        let t0h = DshotSpeed::Dshot600.t0h_ticks(clk);
        assert_eq!(t0h, 52); // 140 * 3/8 = 52 (truncated)
    }
}
