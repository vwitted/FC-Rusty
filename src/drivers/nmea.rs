// nmea.rs — NMEA 0183 GPS sentence parser
//
// Parses standard NMEA sentences from any GPS module over UART.
// Handles four sentence types useful for flight control:
//
//   GGA — Fix quality, satellite count, altitude, HDOP
//   RMC — Lat/lon, ground speed, course, date/time, fix validity
//   VTG — Course and speed (true/magnetic course, knots + km/h speed)
//   GSA — DOP values (PDOP, HDOP, VDOP) and fix mode (2D/3D)
//
// Together these give everything needed for position hold, GPS
// rescue, geofencing, and fix quality assessment.
//
// The parser accepts any talker ID (GP, GN, GL, etc.) — it matches
// on the sentence type suffix (GGA, RMC, VTG, GSA).
//
// Checksum: XOR of all bytes between '$' and '*' (exclusive).

/// Maximum NMEA sentence length (spec says 82 chars max including $, *, checksum, \r\n)
const MAX_SENTENCE_LEN: usize = 83;

/// GPS fix quality (from GGA sentence).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FixQuality {
    NoFix = 0,
    GpsFix = 1,
    DgpsFix = 2,
    PpsFix = 3,
    RtkFixed = 4,
    RtkFloat = 5,
    Estimated = 6,
}

impl FixQuality {
    fn from_byte(b: u8) -> Self {
        match b {
            b'1' => Self::GpsFix,
            b'2' => Self::DgpsFix,
            b'3' => Self::PpsFix,
            b'4' => Self::RtkFixed,
            b'5' => Self::RtkFloat,
            b'6' => Self::Estimated,
            _ => Self::NoFix,
        }
    }

    pub fn has_fix(self) -> bool {
        (self as u8) >= 1
    }
}

/// Fix mode from GSA sentence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FixMode {
    NoFix = 1,
    Fix2D = 2,
    Fix3D = 3,
}

impl FixMode {
    fn from_byte(b: u8) -> Self {
        match b {
            b'2' => Self::Fix2D,
            b'3' => Self::Fix3D,
            _ => Self::NoFix,
        }
    }
}

/// Parsed GPS data — updated incrementally as sentences arrive.
#[derive(Debug, Clone, Copy)]
pub struct GpsData {
    // ---- Position (from RMC + GGA) ----
    /// Latitude in degrees (positive = North, negative = South)
    pub latitude: f64,
    /// Longitude in degrees (positive = East, negative = West)
    pub longitude: f64,
    /// Altitude above mean sea level in metres (from GGA)
    pub altitude_m: f32,

    // ---- Velocity (from RMC) ----
    /// Ground speed in m/s
    pub ground_speed_ms: f32,
    /// Course over ground in degrees (0-360, true north)
    pub course_deg: f32,

    // ---- Velocity from VTG ----
    /// Course over ground, true north, in degrees (from VTG)
    pub course_true_deg: f32,
    /// Course over ground, magnetic, in degrees (from VTG)
    pub course_magnetic_deg: f32,
    /// Ground speed in km/h (from VTG, complements RMC's knot-derived m/s)
    pub ground_speed_kmh: f32,

    // ---- Fix info (from GGA + GSA) ----
    /// Fix quality (from GGA)
    pub fix: FixQuality,
    /// Fix mode: no fix / 2D / 3D (from GSA)
    pub fix_mode: FixMode,
    /// Number of satellites in use
    pub satellites: u8,
    /// Horizontal dilution of precision (from GGA or GSA)
    pub hdop: f32,
    /// Positional dilution of precision (from GSA)
    pub pdop: f32,
    /// Vertical dilution of precision (from GSA)
    pub vdop: f32,

    // ---- Time (from RMC) ----
    /// UTC time: hours (0-23)
    pub hour: u8,
    /// UTC time: minutes (0-59)
    pub minute: u8,
    /// UTC time: seconds (0-59)
    pub second: u8,

    /// RMC fix valid flag ('A' = active/valid, 'V' = void)
    pub rmc_valid: bool,

    /// Bitfield of which sentences have been received
    pub updated: u8,
}

pub const UPDATED_GGA: u8 = 1 << 0;
pub const UPDATED_RMC: u8 = 1 << 1;
pub const UPDATED_VTG: u8 = 1 << 2;
pub const UPDATED_GSA: u8 = 1 << 3;

impl GpsData {
    pub const fn new() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude_m: 0.0,
            ground_speed_ms: 0.0,
            course_deg: 0.0,
            course_true_deg: 0.0,
            course_magnetic_deg: 0.0,
            ground_speed_kmh: 0.0,
            fix: FixQuality::NoFix,
            fix_mode: FixMode::NoFix,
            satellites: 0,
            hdop: 99.9,
            pdop: 99.9,
            vdop: 99.9,
            hour: 0,
            minute: 0,
            second: 0,
            rmc_valid: false,
            updated: 0,
        }
    }

    /// Returns true if we have a valid fix (GGA reports fix + RMC active).
    pub fn has_fix(&self) -> bool {
        self.fix.has_fix() && self.rmc_valid
    }

    /// Returns true if we have a 3D fix (altitude is valid).
    pub fn has_3d_fix(&self) -> bool {
        self.has_fix() && self.fix_mode == FixMode::Fix3D
    }

    /// Check if a specific sentence was updated, then clear the flag.
    pub fn was_updated(&mut self, flag: u8) -> bool {
        let yes = self.updated & flag != 0;
        self.updated &= !flag;
        yes
    }
}

/// NMEA sentence type we recognised.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SentenceType {
    Gga,
    Rmc,
    Vtg,
    Gsa,
}

/// Streaming NMEA parser.
///
/// Feed bytes via `push_byte()`. When a complete, checksum-valid
/// sentence arrives, it updates `self.data` and returns the type.
pub struct NmeaParser {
    buf: [u8; MAX_SENTENCE_LEN],
    pos: usize,
    state: ParserState,
    pub data: GpsData,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ParserState {
    WaitDollar,
    Reading,
}

impl NmeaParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_SENTENCE_LEN],
            pos: 0,
            state: ParserState::WaitDollar,
            data: GpsData::new(),
        }
    }

    /// Feed one byte from the UART.
    ///
    /// Returns `Some(SentenceType)` when a complete valid sentence
    /// has been decoded and `self.data` updated.
    pub fn push_byte(&mut self, byte: u8) -> Option<SentenceType> {
        match self.state {
            ParserState::WaitDollar => {
                if byte == b'$' {
                    self.buf[0] = byte;
                    self.pos = 1;
                    self.state = ParserState::Reading;
                }
                None
            }
            ParserState::Reading => {
                if self.pos >= MAX_SENTENCE_LEN {
                    // Overflow — discard and resync
                    self.state = ParserState::WaitDollar;
                    return None;
                }

                self.buf[self.pos] = byte;
                self.pos += 1;

                if byte == b'\n' {
                    // End of sentence — try to parse
                    let result = self.try_parse();
                    self.state = ParserState::WaitDollar;
                    result
                } else {
                    None
                }
            }
        }
    }

    /// Try to parse the buffered sentence.
    fn try_parse(&mut self) -> Option<SentenceType> {
        let len = self.pos;

        // Minimum: $XXYYY,...*CC\r\n = at least ~15 chars
        if len < 10 {
            return None;
        }

        // Find '*' for checksum
        let star_pos = self.buf[..len].iter().position(|&b| b == b'*')?;

        // Verify checksum: XOR of bytes between '$' and '*' (exclusive)
        let computed = self.buf[1..star_pos]
            .iter()
            .fold(0u8, |acc, &b| acc ^ b);

        // Parse the two hex digits after '*'
        if star_pos + 2 >= len {
            return None;
        }
        let expected = parse_hex_byte(self.buf[star_pos + 1], self.buf[star_pos + 2])?;

        if computed != expected {
            return None;
        }

        // Identify sentence type from bytes 3..6 (after talker ID)
        // e.g., $GPGGA → buf[3..6] = "GGA"
        if len < 7 {
            return None;
        }

        let type_bytes = &self.buf[3..6];

        if type_bytes == b"GGA" {
            self.parse_gga(star_pos);
            self.data.updated |= UPDATED_GGA;
            Some(SentenceType::Gga)
        } else if type_bytes == b"RMC" {
            self.parse_rmc(star_pos);
            self.data.updated |= UPDATED_RMC;
            Some(SentenceType::Rmc)
        } else if type_bytes == b"VTG" {
            self.parse_vtg(star_pos);
            self.data.updated |= UPDATED_VTG;
            Some(SentenceType::Vtg)
        } else if type_bytes == b"GSA" {
            self.parse_gsa(star_pos);
            self.data.updated |= UPDATED_GSA;
            Some(SentenceType::Gsa)
        } else {
            None // Sentence type we don't handle
        }
    }

    /// Parse GGA sentence fields.
    /// $GPGGA,hhmmss.ss,lat,N,lon,E,fix,sats,hdop,alt,M,geoid,M,,*CC
    fn parse_gga(&mut self, end: usize) {
        let mut fields = FieldIter::new(&self.buf[7..end]); // skip "$GPGGA,"

        // Field 1: time (hhmmss.ss) — parsed in RMC instead
        fields.next();

        // Field 2,3: latitude, N/S
        let lat_raw = fields.next();
        let lat_dir = fields.next();

        // Field 4,5: longitude, E/W
        let lon_raw = fields.next();
        let lon_dir = fields.next();

        // Field 6: fix quality
        if let Some(fix_field) = fields.next() {
            if !fix_field.is_empty() {
                self.data.fix = FixQuality::from_byte(fix_field[0]);
            }
        }

        // Field 7: satellite count
        if let Some(sat_field) = fields.next() {
            self.data.satellites = parse_u8(sat_field);
        }

        // Field 8: HDOP
        if let Some(hdop_field) = fields.next() {
            if let Some(v) = parse_f32(hdop_field) {
                self.data.hdop = v;
            }
        }

        // Field 9: altitude above MSL
        if let Some(alt_field) = fields.next() {
            if let Some(v) = parse_f32(alt_field) {
                self.data.altitude_m = v;
            }
        }

        // Parse lat/lon
        if let (Some(lat_r), Some(lat_d)) = (lat_raw, lat_dir) {
            if let Some(lat) = parse_nmea_coord(lat_r) {
                self.data.latitude = if lat_d == b"S" { -lat } else { lat };
            }
        }
        if let (Some(lon_r), Some(lon_d)) = (lon_raw, lon_dir) {
            if let Some(lon) = parse_nmea_coord(lon_r) {
                self.data.longitude = if lon_d == b"W" { -lon } else { lon };
            }
        }
    }

    /// Parse RMC sentence fields.
    /// $GPRMC,hhmmss.ss,A,lat,N,lon,E,speed,course,ddmmyy,mag,E,mode*CC
    fn parse_rmc(&mut self, end: usize) {
        let mut fields = FieldIter::new(&self.buf[7..end]); // skip "$GPRMC,"

        // Field 1: time (hhmmss.ss)
        if let Some(time_field) = fields.next() {
            parse_time(time_field, &mut self.data);
        }

        // Field 2: status (A=active, V=void)
        if let Some(status_field) = fields.next() {
            self.data.rmc_valid = !status_field.is_empty() && status_field[0] == b'A';
        }

        // Field 3,4: latitude, N/S
        let lat_raw = fields.next();
        let lat_dir = fields.next();

        // Field 5,6: longitude, E/W
        let lon_raw = fields.next();
        let lon_dir = fields.next();

        // Field 7: speed over ground in knots
        if let Some(spd_field) = fields.next() {
            if let Some(knots) = parse_f32(spd_field) {
                self.data.ground_speed_ms = knots * 0.514444; // knots → m/s
            }
        }

        // Field 8: course over ground (degrees true)
        if let Some(crs_field) = fields.next() {
            if let Some(v) = parse_f32(crs_field) {
                self.data.course_deg = v;
            }
        }

        // Parse lat/lon (RMC also carries position)
        if let (Some(lat_r), Some(lat_d)) = (lat_raw, lat_dir) {
            if let Some(lat) = parse_nmea_coord(lat_r) {
                self.data.latitude = if lat_d == b"S" { -lat } else { lat };
            }
        }
        if let (Some(lon_r), Some(lon_d)) = (lon_raw, lon_dir) {
            if let Some(lon) = parse_nmea_coord(lon_r) {
                self.data.longitude = if lon_d == b"W" { -lon } else { lon };
            }
        }
    }

    /// Parse VTG sentence fields.
    /// $GPVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode*CC
    fn parse_vtg(&mut self, end: usize) {
        let mut fields = FieldIter::new(&self.buf[7..end]); // skip "$GPVTG,"

        // Field 1: course over ground, true (degrees)
        if let Some(f) = fields.next() {
            if let Some(v) = parse_f32(f) {
                self.data.course_true_deg = v;
            }
        }

        // Field 2: 'T' (true) — skip
        fields.next();

        // Field 3: course over ground, magnetic (degrees)
        if let Some(f) = fields.next() {
            if let Some(v) = parse_f32(f) {
                self.data.course_magnetic_deg = v;
            }
        }

        // Field 4: 'M' (magnetic) — skip
        fields.next();

        // Field 5: speed over ground in knots
        if let Some(f) = fields.next() {
            if let Some(knots) = parse_f32(f) {
                self.data.ground_speed_ms = knots * 0.514444;
            }
        }

        // Field 6: 'N' (knots) — skip
        fields.next();

        // Field 7: speed over ground in km/h
        if let Some(f) = fields.next() {
            if let Some(v) = parse_f32(f) {
                self.data.ground_speed_kmh = v;
            }
        }
    }

    /// Parse GSA sentence fields.
    /// $GPGSA,mode1,mode2,sv1,sv2,...,sv12,pdop,hdop,vdop*CC
    fn parse_gsa(&mut self, end: usize) {
        let mut fields = FieldIter::new(&self.buf[7..end]); // skip "$GPGSA,"

        // Field 1: selection mode (M=manual, A=automatic) — skip
        fields.next();

        // Field 2: fix mode (1=no fix, 2=2D, 3=3D)
        if let Some(f) = fields.next() {
            if !f.is_empty() {
                self.data.fix_mode = FixMode::from_byte(f[0]);
            }
        }

        // Fields 3-14: satellite PRNs (12 slots) — skip
        for _ in 0..12 {
            fields.next();
        }

        // Field 15: PDOP
        if let Some(f) = fields.next() {
            if let Some(v) = parse_f32(f) {
                self.data.pdop = v;
            }
        }

        // Field 16: HDOP
        if let Some(f) = fields.next() {
            if let Some(v) = parse_f32(f) {
                self.data.hdop = v;
            }
        }

        // Field 17: VDOP
        if let Some(f) = fields.next() {
            if let Some(v) = parse_f32(f) {
                self.data.vdop = v;
            }
        }
    }
}

// ---- Field iterator: splits a byte slice on commas ----

struct FieldIter<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> FieldIter<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }
}

impl<'a> Iterator for FieldIter<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos > self.data.len() {
            return None;
        }
        let start = self.pos;
        // Find next comma or end
        let end = self.data[start..]
            .iter()
            .position(|&b| b == b',')
            .map(|i| start + i)
            .unwrap_or(self.data.len());
        self.pos = end + 1; // skip past comma
        Some(&self.data[start..end])
    }
}

// ---- Parsing helpers ----

/// Parse NMEA coordinate: "ddmm.mmmm" or "dddmm.mmmm" → decimal degrees.
///
/// NMEA encodes lat as ddmm.mmmm and lon as dddmm.mmmm.
/// The first 2 (lat) or 3 (lon) digits are degrees, the rest is minutes.
fn parse_nmea_coord(field: &[u8]) -> Option<f64> {
    if field.is_empty() {
        return None;
    }

    // Find the decimal point to determine degree/minute split
    let dot_pos = field.iter().position(|&b| b == b'.')?;

    // Degrees are everything before (dot_pos - 2)
    if dot_pos < 2 {
        return None;
    }
    let deg_end = dot_pos - 2;

    let degrees = parse_f64(&field[..deg_end])?;
    let minutes = parse_f64(&field[deg_end..])?;

    Some(degrees + minutes / 60.0)
}

/// Parse UTC time from "hhmmss.ss" field.
fn parse_time(field: &[u8], data: &mut GpsData) {
    if field.len() >= 6 {
        data.hour = parse_u8(&field[0..2]);
        data.minute = parse_u8(&field[2..4]);
        data.second = parse_u8(&field[4..6]);
    }
}

/// Parse a u8 from ASCII decimal digits.
fn parse_u8(field: &[u8]) -> u8 {
    let mut val: u8 = 0;
    for &b in field {
        if b.is_ascii_digit() {
            val = val.wrapping_mul(10).wrapping_add(b - b'0');
        }
    }
    val
}

/// Parse an f32 from ASCII (no_std compatible, no allocator).
fn parse_f32(field: &[u8]) -> Option<f32> {
    parse_f64(field).map(|v| v as f32)
}

/// Parse an f64 from ASCII decimal (handles sign, integer, fraction).
fn parse_f64(field: &[u8]) -> Option<f64> {
    if field.is_empty() {
        return None;
    }

    let mut i = 0;
    let negative = if field[0] == b'-' {
        i = 1;
        true
    } else {
        false
    };

    let mut int_part: f64 = 0.0;
    while i < field.len() && field[i].is_ascii_digit() {
        int_part = int_part * 10.0 + (field[i] - b'0') as f64;
        i += 1;
    }

    let mut frac_part: f64 = 0.0;
    if i < field.len() && field[i] == b'.' {
        i += 1;
        let mut divisor: f64 = 10.0;
        while i < field.len() && field[i].is_ascii_digit() {
            frac_part += (field[i] - b'0') as f64 / divisor;
            divisor *= 10.0;
            i += 1;
        }
    }

    let val = int_part + frac_part;
    Some(if negative { -val } else { val })
}

/// Parse two hex ASCII characters into a byte.
fn parse_hex_byte(hi: u8, lo: u8) -> Option<u8> {
    let h = hex_digit(hi)?;
    let l = hex_digit(lo)?;
    Some((h << 4) | l)
}

fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed a complete sentence string to the parser and return the result.
    fn feed_sentence(parser: &mut NmeaParser, sentence: &str) -> Option<SentenceType> {
        let mut result = None;
        for &b in sentence.as_bytes() {
            if let Some(t) = parser.push_byte(b) {
                result = Some(t);
            }
        }
        result
    }

    #[test]
    fn test_parse_gga() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*61\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Gga));
        assert_eq!(parser.data.fix, FixQuality::GpsFix);
        assert_eq!(parser.data.satellites, 8);
        assert!((parser.data.hdop - 0.9).abs() < 0.01);
        assert!((parser.data.altitude_m - 545.4).abs() < 0.1);

        // Latitude: 48°07.038' = 48 + 7.038/60 = 48.1173°
        assert!((parser.data.latitude - 48.1173).abs() < 0.001);
        // Longitude: 11°31.000' = 11 + 31.0/60 = 11.5167°
        assert!((parser.data.longitude - 11.51667).abs() < 0.001);

        assert!(parser.data.updated & UPDATED_GGA != 0);
    }

    #[test]
    fn test_parse_rmc() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*52\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Rmc));
        assert!(parser.data.rmc_valid);
        assert_eq!(parser.data.hour, 12);
        assert_eq!(parser.data.minute, 35);
        assert_eq!(parser.data.second, 19);

        // Speed: 22.4 knots = 11.52 m/s
        assert!((parser.data.ground_speed_ms - 11.52).abs() < 0.1);
        // Course: 084.4°
        assert!((parser.data.course_deg - 84.4).abs() < 0.1);

        // Position (same as GGA test)
        assert!((parser.data.latitude - 48.1173).abs() < 0.001);
        assert!((parser.data.longitude - 11.51667).abs() < 0.001);

        assert!(parser.data.updated & UPDATED_RMC != 0);
    }

    #[test]
    fn test_south_west_coordinates() {
        let mut parser = NmeaParser::new();
        // São Paulo, Brazil: ~23.55°S, 46.63°W
        let sentence = "$GPGGA,120000.00,2333.000,S,04638.000,W,1,10,1.0,760.0,M,,,,*3A\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Gga));
        assert!(parser.data.latitude < 0.0, "South should be negative");
        assert!(parser.data.longitude < 0.0, "West should be negative");
        assert!((parser.data.latitude - (-23.55)).abs() < 0.001);
        assert!((parser.data.longitude - (-46.6333)).abs() < 0.001);
    }

    #[test]
    fn test_bad_checksum_rejected() {
        let mut parser = NmeaParser::new();
        // Corrupt checksum (should be 61, we use FF)
        let sentence = "$GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*FF\r\n";
        let result = feed_sentence(&mut parser, sentence);
        assert_eq!(result, None);
    }

    #[test]
    fn test_gn_talker_id() {
        let mut parser = NmeaParser::new();
        // GNSS combined talker ID (multi-constellation)
        let sentence = "$GNGGA,120000.00,5133.000,N,00007.000,W,1,12,0.8,100.0,M,47.0,M,,*69\r\n";
        let result = feed_sentence(&mut parser, sentence);
        assert_eq!(result, Some(SentenceType::Gga));
        assert_eq!(parser.data.satellites, 12);
    }

    #[test]
    fn test_no_fix() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPGGA,120000.00,,,,,0,00,99.9,,,,,,*5C\r\n";
        let result = feed_sentence(&mut parser, sentence);
        assert_eq!(result, Some(SentenceType::Gga));
        assert_eq!(parser.data.fix, FixQuality::NoFix);
        assert_eq!(parser.data.satellites, 0);
        assert!(!parser.data.has_fix());
    }

    #[test]
    fn test_rmc_void() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPRMC,120000.00,V,,,,,,,230394,,,N*71\r\n";
        let result = feed_sentence(&mut parser, sentence);
        assert_eq!(result, Some(SentenceType::Rmc));
        assert!(!parser.data.rmc_valid);
        assert!(!parser.data.has_fix());
    }

    #[test]
    fn test_sequential_sentences() {
        let mut parser = NmeaParser::new();

        // First GGA
        feed_sentence(
            &mut parser,
            "$GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*61\r\n",
        );
        assert_eq!(parser.data.fix, FixQuality::GpsFix);

        // Then RMC
        feed_sentence(
            &mut parser,
            "$GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*52\r\n",
        );
        assert!(parser.data.rmc_valid);
        assert!(parser.data.has_fix());

        // Both flags set
        assert!(parser.data.updated & UPDATED_GGA != 0);
        assert!(parser.data.updated & UPDATED_RMC != 0);
    }

    #[test]
    fn test_interleaved_garbage() {
        let mut parser = NmeaParser::new();

        // Feed some garbage bytes then a valid sentence
        for &b in b"\x00\xFF\x55garbage" {
            assert_eq!(parser.push_byte(b), None);
        }

        let result = feed_sentence(
            &mut parser,
            "$GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*61\r\n",
        );
        assert_eq!(result, Some(SentenceType::Gga));
    }

    #[test]
    fn test_parse_vtg() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A*25\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Vtg));
        assert!((parser.data.course_true_deg - 54.7).abs() < 0.1);
        assert!((parser.data.course_magnetic_deg - 34.4).abs() < 0.1);
        // Speed: 5.5 knots = 2.829 m/s
        assert!((parser.data.ground_speed_ms - 2.829).abs() < 0.1);
        assert!((parser.data.ground_speed_kmh - 10.2).abs() < 0.1);
        assert!(parser.data.updated & UPDATED_VTG != 0);
    }

    #[test]
    fn test_parse_vtg_empty_fields() {
        let mut parser = NmeaParser::new();
        // VTG with empty fields (no fix yet)
        let sentence = "$GPVTG,,T,,M,,N,,K,N*2C\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Vtg));
        // Fields should remain at defaults (0.0) since empty fields aren't parsed
        assert!((parser.data.course_true_deg - 0.0).abs() < 0.01);
        assert!((parser.data.ground_speed_kmh - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_parse_gsa() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPGSA,A,3,04,05,09,12,24,25,,,,,,,2.5,1.3,2.1*3E\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Gsa));
        assert_eq!(parser.data.fix_mode, FixMode::Fix3D);
        assert!((parser.data.pdop - 2.5).abs() < 0.01);
        assert!((parser.data.hdop - 1.3).abs() < 0.01);
        assert!((parser.data.vdop - 2.1).abs() < 0.01);
        assert!(parser.data.updated & UPDATED_GSA != 0);
    }

    #[test]
    fn test_parse_gsa_no_fix() {
        let mut parser = NmeaParser::new();
        let sentence = "$GPGSA,A,1,,,,,,,,,,,,,99.9,99.9,99.9*09\r\n";
        let result = feed_sentence(&mut parser, sentence);

        assert_eq!(result, Some(SentenceType::Gsa));
        assert_eq!(parser.data.fix_mode, FixMode::NoFix);
        assert!((parser.data.pdop - 99.9).abs() < 0.1);
    }

    #[test]
    fn test_3d_fix_requires_gsa() {
        let mut parser = NmeaParser::new();

        // GGA + RMC give has_fix() but not has_3d_fix() without GSA
        feed_sentence(
            &mut parser,
            "$GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*61\r\n",
        );
        feed_sentence(
            &mut parser,
            "$GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*52\r\n",
        );
        assert!(parser.data.has_fix());
        assert!(!parser.data.has_3d_fix()); // no GSA yet

        // GSA with 3D fix completes the picture
        feed_sentence(
            &mut parser,
            "$GPGSA,A,3,04,05,09,12,24,25,,,,,,,2.5,1.3,2.1*3E\r\n",
        );
        assert!(parser.data.has_3d_fix());
    }

    #[test]
    fn test_vtg_updates_speed() {
        let mut parser = NmeaParser::new();

        // RMC sets ground_speed_ms from knots
        feed_sentence(
            &mut parser,
            "$GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*52\r\n",
        );
        let rmc_speed = parser.data.ground_speed_ms;

        // VTG also sets ground_speed_ms (should overwrite with same value)
        feed_sentence(
            &mut parser,
            "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A*25\r\n",
        );
        // VTG had 5.5 knots ≈ 2.83 m/s, different from RMC's 22.4 knots
        assert!((parser.data.ground_speed_ms - 2.829).abs() < 0.1);
        assert!(parser.data.ground_speed_ms != rmc_speed);

        // VTG also provides km/h
        assert!((parser.data.ground_speed_kmh - 10.2).abs() < 0.1);
    }

    #[test]
    fn test_parse_nmea_coord() {
        // 48°07.038' = 48.1173°
        assert!((parse_nmea_coord(b"4807.038").unwrap() - 48.1173).abs() < 0.001);
        // 011°31.000' = 11.5167°
        assert!((parse_nmea_coord(b"01131.000").unwrap() - 11.51667).abs() < 0.001);
        // Empty
        assert!(parse_nmea_coord(b"").is_none());
    }

    #[test]
    fn test_parse_f64() {
        assert!((parse_f64(b"123.456").unwrap() - 123.456).abs() < 0.0001);
        assert!((parse_f64(b"-42.5").unwrap() - (-42.5)).abs() < 0.0001);
        assert!((parse_f64(b"0.9").unwrap() - 0.9).abs() < 0.0001);
        assert!((parse_f64(b"100").unwrap() - 100.0).abs() < 0.0001);
        assert!(parse_f64(b"").is_none());
    }
}
