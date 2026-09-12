//! Record format for plant-characterisation logs.
//!
//! One sample is what you need to identify the motor/airframe model the
//! sim uses: what each motor was commanded, what it actually did (from
//! bidirectional DShot telemetry), and how the airframe responded.
//!
//! Defined once, here, because there are two sinks and they must not
//! drift apart:
//!
//!   - RAM, for a bench capture that is dumped over defmt after the run.
//!     The logger busy-waits on USART6 TXE inside a critical_section, so
//!     logging a sample as it is taken would stall the DShot loop for
//!     milliseconds; capture has to be a memcpy and the transmission has
//!     to happen when nothing is spinning.
//!   - Flash, for anything involving flight, where the wire is not
//!     available at all. `RECORD_LEN` is 32 bytes for that reason: it is
//!     exactly the H7's flash write word, so a record is one program
//!     operation with no read-modify-write and no straddling.
//!
//! Pure `no_std` and host-tested; nothing here touches hardware.

/// Bytes per record. One H7 flash word.
pub const RECORD_LEN: usize = 32;

/// Fixed-point scale for the normalised motor command.
///
/// The command is 0.0..=1.0 and is stored as an integer because a record
/// that is memcpy-able is a record that can be written straight to flash.
/// 10000 gives 0.01% resolution, far finer than DShot's own 2048 steps.
pub const CMD_SCALE: f32 = 10_000.0;

/// Fixed-point scale for gyro rates: deci-degrees per second.
///
/// i16 at this scale reaches +/-3276 deg/s, which is beyond what the
/// ICM-42688 is configured to report and beyond what the airframe can do.
pub const GYRO_SCALE: f32 = 10.0;

/// Sentinel period meaning "no usable telemetry from this motor in this
/// frame" — either nothing came back, or it came back and failed to
/// decode. Zero is safe as a sentinel because a real eRPM period of zero
/// would be infinite RPM.
pub const NO_TELEMETRY: u16 = 0;

/// One sample of the plant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PlantSample {
    /// Milliseconds since capture started.
    ///
    /// u32, not u16. A bench capture is under four seconds and u16 was
    /// ample for it, but the same record now goes to the flash blackbox
    /// during flight, where 65.5 s is a short hop and a wrapped timestamp
    /// fits a plausible-looking, wrong time constant without ever looking
    /// broken.
    pub t_ms: u32,
    /// Per-motor normalised throttle command, scaled by `CMD_SCALE`.
    pub cmd: [u16; 4],
    /// Per-motor eRPM period in microseconds, straight from the DShot
    /// bidirectional reply. `NO_TELEMETRY` when absent or undecodable.
    pub period_us: [u16; 4],
    /// Body rates, scaled by `GYRO_SCALE`.
    ///
    /// Zero on a bench capture: the motor-test build runs no IMU. That is
    /// deliberate rather than a gap — the bench measures the motor, and
    /// the airframe response needs the aircraft to be free to move, which
    /// is a flight measurement and a flash log.
    pub gyro_dps10: [i16; 3],
}

impl PlantSample {
    /// Serialise into a flash-word-sized record.
    ///
    /// No CRC. A config record gets one because a corrupt config is
    /// silently wrong forever; a corrupt log sample is one point among
    /// thousands and the fit is robust to a few outliers. Spending 4 of
    /// 32 bytes on it would cost real capture time for very little.
    pub fn encode(&self) -> [u8; RECORD_LEN] {
        let mut b = [0u8; RECORD_LEN];
        b[0..4].copy_from_slice(&self.t_ms.to_le_bytes());
        for i in 0..4 {
            b[4 + i * 2..6 + i * 2].copy_from_slice(&self.cmd[i].to_le_bytes());
            b[12 + i * 2..14 + i * 2].copy_from_slice(&self.period_us[i].to_le_bytes());
        }
        for i in 0..3 {
            b[20 + i * 2..22 + i * 2].copy_from_slice(&self.gyro_dps10[i].to_le_bytes());
        }
        // 26..32 reserved. Left ZERO, which matters: erased flash reads as
        // 0xFF, so an all-0xFF record is "never written" and an all-zero
        // tail is "written by a version that did not use these bytes".
        // The blackbox's append-point scan depends on that distinction.
        b
    }

    pub fn decode(b: &[u8; RECORD_LEN]) -> Self {
        let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let i16_at = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Self {
            t_ms: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            cmd: core::array::from_fn(|i| u16_at(4 + i * 2)),
            period_us: core::array::from_fn(|i| u16_at(12 + i * 2)),
            gyro_dps10: core::array::from_fn(|i| i16_at(20 + i * 2)),
        }
    }

    /// Commanded throttle for one motor, 0.0..=1.0.
    pub fn cmd_f32(&self, motor: usize) -> f32 {
        self.cmd[motor] as f32 / CMD_SCALE
    }

    /// Body rate for one axis, deg/s.
    pub fn gyro_dps(&self, axis: usize) -> f32 {
        self.gyro_dps10[axis] as f32 / GYRO_SCALE
    }

    /// Electrical RPM for one motor, or `None` if the frame carried no
    /// usable telemetry.
    ///
    /// The DShot bidirectional reply is the period of one electrical
    /// revolution in microseconds, so eRPM = 60e6 / period.
    pub fn erpm(&self, motor: usize) -> Option<f32> {
        match self.period_us[motor] {
            NO_TELEMETRY => None,
            p => Some(60_000_000.0 / p as f32),
        }
    }

    /// Mechanical RPM, given the motor's pole-pair count.
    ///
    /// A 12N14P outrunner -- which is what a 5-7" quad motor almost always
    /// is -- has 7 pole pairs. Note this scale factor CANCELS in a time
    /// constant, so getting it wrong costs nothing in `motor_tau`; it
    /// only matters for reporting absolute RPM and for thrust curves.
    pub fn rpm(&self, motor: usize, pole_pairs: f32) -> Option<f32> {
        self.erpm(motor).map(|e| e / pole_pairs)
    }
}

/// Prefix on every sample line in a text dump, so the fit tool can find
/// them in a log that also carries ordinary defmt output.
pub const CSV_TAG: &str = "PLANT";

/// Column header matching `write_csv_into`, for a dumped file to be
/// readable by anything else.
pub const CSV_HEADER: &str =
    "tag,t_ms,cmd0,cmd1,cmd2,cmd3,per0,per1,per2,per3,gx,gy,gz";

/// Parse one text line as produced by the firmware dump.
///
/// Tolerant on purpose: a defmt line arrives with a timestamp and level
/// prefix attached, so the tag is searched for rather than required at
/// the start, and anything that is not a sample line returns `None`
/// rather than erroring. A capture is thousands of lines interleaved with
/// ordinary logging and the tool should not care.
pub fn parse_csv_line(line: &str) -> Option<PlantSample> {
    let start = line.find(CSV_TAG)?;
    let mut it = line[start..].split(',');
    if it.next()?.trim() != CSV_TAG {
        return None;
    }
    let t_ms = it.next()?.trim().parse::<u32>().ok()?;
    let mut next_u16 = || -> Option<u16> { it.next()?.trim().parse::<u16>().ok() };
    let mut cmd = [0u16; 4];
    for c in cmd.iter_mut() {
        *c = next_u16()?;
    }
    let mut period_us = [0u16; 4];
    for p in period_us.iter_mut() {
        *p = next_u16()?;
    }
    // Gyro is signed and may legitimately be absent on a bench capture
    // written by an older firmware; missing tail means zeros rather than
    // a rejected line.
    let mut gyro_dps10 = [0i16; 3];
    for g in gyro_dps10.iter_mut() {
        match it.next() {
            Some(tok) => *g = tok.trim().parse::<i16>().ok()?,
            None => break,
        }
    }
    Some(PlantSample { t_ms, cmd, period_us, gyro_dps10 })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> PlantSample {
        PlantSample {
            t_ms: 1234,
            cmd: [1000, 2000, 3000, 4000],
            period_us: [500, 501, 0, 503],
            gyro_dps10: [-100, 250, 0],
        }
    }

    #[test]
    fn a_long_flight_does_not_wrap_the_timestamp() {
        // The reason t_ms is u32. At u16 this would have wrapped after
        // 65.5 s and produced a log that fits a wrong answer silently.
        let ten_minutes_ms = 10 * 60 * 1000u32;
        let s = PlantSample { t_ms: ten_minutes_ms, ..PlantSample::default() };
        assert_eq!(PlantSample::decode(&s.encode()).t_ms, ten_minutes_ms);
    }

    #[test]
    fn record_is_one_flash_word() {
        // Not cosmetic: the H7 programs flash in 32-byte words, so a
        // record of any other size either wastes a word or straddles two,
        // and a straddled record cannot be written atomically.
        assert_eq!(RECORD_LEN, 32);
        assert_eq!(RECORD_LEN % 32, 0);
    }

    #[test]
    fn encode_decode_round_trips() {
        let s = sample();
        assert_eq!(PlantSample::decode(&s.encode()), s);
    }

    #[test]
    fn fields_do_not_overlap() {
        // Set one field at a time and check nothing else moves. Overlapping
        // offsets are the classic hand-rolled-serialisation bug and they
        // corrupt the field written first, which here would be the
        // timestamp -- and a log with wrong timestamps fits a wrong tau
        // without ever looking broken.
        let mut s = PlantSample::default();
        s.t_ms = 0xFFFF_FFFF;
        let d = PlantSample::decode(&s.encode());
        assert_eq!(d.cmd, [0; 4]);
        assert_eq!(d.period_us, [0; 4]);
        assert_eq!(d.gyro_dps10, [0; 3]);

        let mut s = PlantSample::default();
        s.period_us = [0xFFFF; 4];
        let d = PlantSample::decode(&s.encode());
        assert_eq!(d.t_ms, 0);
        assert_eq!(d.cmd, [0; 4]);
        assert_eq!(d.gyro_dps10, [0; 3]);
    }

    #[test]
    fn negative_gyro_survives_the_round_trip() {
        let mut s = PlantSample::default();
        s.gyro_dps10 = [-3000, -1, i16::MIN];
        assert_eq!(PlantSample::decode(&s.encode()).gyro_dps10, s.gyro_dps10);
    }

    #[test]
    fn erpm_matches_the_dshot_definition() {
        // The reply is the period of one electrical revolution in us.
        // 1000 us -> 60000 eRPM; at 7 pole pairs that is ~8571 mechanical
        // RPM, a plausible hover figure.
        let mut s = PlantSample::default();
        s.period_us = [1000, 0, 0, 0];
        assert!((s.erpm(0).unwrap() - 60_000.0).abs() < 1.0);
        assert!((s.rpm(0, 7.0).unwrap() - 8571.4).abs() < 1.0);
    }

    #[test]
    fn zero_period_is_absence_not_infinity() {
        // Reading the sentinel as a period would give infinite RPM and
        // poison every average it touched.
        let s = PlantSample::default();
        assert_eq!(s.erpm(0), None);
        assert_eq!(s.rpm(0, 7.0), None);
    }

    #[test]
    fn scales_convert_back() {
        let mut s = PlantSample::default();
        s.cmd = [5000; 4];
        s.gyro_dps10 = [-1234, 0, 0];
        assert!((s.cmd_f32(0) - 0.5).abs() < 1e-6);
        assert!((s.gyro_dps(0) + 123.4).abs() < 1e-3);
    }

    #[test]
    fn parses_its_own_csv_form() {
        let line = "PLANT,1234,1000,2000,3000,4000,500,501,0,503,-100,250,0";
        assert_eq!(parse_csv_line(line).unwrap(), sample());
    }

    #[test]
    fn parses_out_of_a_defmt_decorated_line() {
        // What actually comes off the wire: defmt-print prepends a
        // timestamp and level. Requiring the tag at column zero would
        // reject every real line.
        let line = "1.234567 INFO  PLANT,1234,1000,2000,3000,4000,500,501,0,503,-100,250,0";
        assert_eq!(parse_csv_line(line).unwrap(), sample());
    }

    #[test]
    fn ignores_lines_that_are_not_samples() {
        for line in [
            "1.0 INFO  motor-test: driving motors",
            "",
            "PLANTAIN,1,2,3",
            "PLANT,not-a-number,1,2,3,4,5,6,7,8",
            "PLANT,1,2",
        ] {
            assert!(parse_csv_line(line).is_none(), "accepted {line:?}");
        }
    }

    #[test]
    fn a_missing_gyro_tail_reads_as_zeros() {
        // Bench captures have no IMU. A line that stops after the periods
        // is complete, not truncated.
        let line = "PLANT,10,1,2,3,4,5,6,7,8";
        let s = parse_csv_line(line).unwrap();
        assert_eq!(s.t_ms, 10);
        assert_eq!(s.period_us, [5, 6, 7, 8]);
        assert_eq!(s.gyro_dps10, [0; 3]);
    }
}
