// ubx.rs — u-blox UBX binary protocol parser
//
// Parses the UBX binary protocol used by u-blox GPS modules
// (NEO-6M, NEO-M8N, NEO-M9N, WalkSnail WS-M181, etc.)
//
// Protocol: each frame is:
//   [0xB5] [0x62] [CLASS] [ID] [LEN_L] [LEN_H] [PAYLOAD...] [CK_A] [CK_B]
//
// Checksum is Fletcher-16 over CLASS + ID + LEN + PAYLOAD.
//
// The key message for flight control is NAV-PVT (0x01 0x07),
// which provides position, velocity, time, fix quality, and
// accuracy estimates in a single 92-byte payload.

/// UBX sync bytes
const SYNC_1: u8 = 0xB5;
const SYNC_2: u8 = 0x62;

/// Maximum payload we'll accept (NAV-PVT is 92 bytes)
const MAX_PAYLOAD: usize = 128;

/// Full frame buffer: class(1) + id(1) + len(2) + payload(max) + ck(2)
const MAX_FRAME: usize = 4 + MAX_PAYLOAD + 2;

/// UBX message class + ID pairs we care about
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MsgType {
    NavPvt,    // 0x01 0x07 — position, velocity, time
}

impl MsgType {
    fn from_class_id(class: u8, id: u8) -> Option<Self> {
        match (class, id) {
            (0x01, 0x07) => Some(Self::NavPvt),
            _ => None,
        }
    }
}

/// GPS fix type from NAV-PVT fixType field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FixType {
    NoFix = 0,
    DeadReckoning = 1,
    Fix2D = 2,
    Fix3D = 3,
    GnssDeadReckoning = 4,
    TimeOnly = 5,
}

impl FixType {
    fn from_byte(b: u8) -> Self {
        match b {
            1 => Self::DeadReckoning,
            2 => Self::Fix2D,
            3 => Self::Fix3D,
            4 => Self::GnssDeadReckoning,
            5 => Self::TimeOnly,
            _ => Self::NoFix,
        }
    }

    pub fn has_fix(self) -> bool {
        matches!(self, Self::Fix2D | Self::Fix3D | Self::GnssDeadReckoning)
    }

    pub fn has_3d_fix(self) -> bool {
        matches!(self, Self::Fix3D | Self::GnssDeadReckoning)
    }
}

/// Parsed GPS data from NAV-PVT.
///
/// All fields populated from a single NAV-PVT message.
#[derive(Debug, Clone, Copy)]
pub struct GpsData {
    // ---- Position ----
    /// Latitude in degrees (positive = north)
    pub latitude: f64,
    /// Longitude in degrees (positive = east)
    pub longitude: f64,
    /// Altitude above mean sea level in metres
    pub altitude_msl_m: f32,
    /// Horizontal accuracy estimate in metres
    pub h_acc_m: f32,
    /// Vertical accuracy estimate in metres
    pub v_acc_m: f32,

    // ---- Velocity (NED frame) ----
    /// North velocity in m/s
    pub vel_n_ms: f32,
    /// East velocity in m/s
    pub vel_e_ms: f32,
    /// Down velocity in m/s (positive = descending)
    pub vel_d_ms: f32,
    /// Ground speed in m/s
    pub ground_speed_ms: f32,
    /// Speed accuracy estimate in m/s
    pub s_acc_ms: f32,

    // ---- Heading ----
    /// Heading of motion in degrees (0-360)
    pub heading_motion_deg: f32,
    /// Heading accuracy estimate in degrees
    pub heading_acc_deg: f32,

    // ---- Fix quality ----
    /// Fix type
    pub fix_type: FixType,
    /// Number of satellites used
    pub satellites: u8,
    /// GNSS fix OK flag (from flags field)
    pub fix_ok: bool,
    /// Position DOP (scaled by 0.01)
    pub pdop: f32,

    // ---- Time ----
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    /// Time validity flags
    pub time_valid: bool,

    /// Set to true when a new NAV-PVT has been decoded
    pub updated: bool,
}

impl GpsData {
    pub const fn new() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude_msl_m: 0.0,
            h_acc_m: 99.0,
            v_acc_m: 99.0,
            vel_n_ms: 0.0,
            vel_e_ms: 0.0,
            vel_d_ms: 0.0,
            ground_speed_ms: 0.0,
            s_acc_ms: 99.0,
            heading_motion_deg: 0.0,
            heading_acc_deg: 180.0,
            fix_type: FixType::NoFix,
            satellites: 0,
            fix_ok: false,
            pdop: 99.0,
            hour: 0,
            minute: 0,
            second: 0,
            time_valid: false,
            updated: false,
        }
    }

    pub fn has_fix(&self) -> bool {
        self.fix_ok && self.fix_type.has_fix()
    }

    pub fn has_3d_fix(&self) -> bool {
        self.fix_ok && self.fix_type.has_3d_fix()
    }
}

/// Streaming parser for UBX frames.
///
/// Same pattern as Wt901bParser and CrsfParser: feed bytes
/// via `push_byte()`, it updates `self.data` when valid
/// NAV-PVT messages arrive.
pub struct UbxParser {
    buf: [u8; MAX_FRAME],
    pos: usize,
    payload_len: u16,
    state: ParserState,
    pub data: GpsData,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ParserState {
    WaitSync1,
    WaitSync2,
    ReadHeader,  // reading class, id, len_l, len_h (4 bytes)
    ReadPayload, // reading payload + 2 checksum bytes
}

impl UbxParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME],
            pos: 0,
            payload_len: 0,
            state: ParserState::WaitSync1,
            data: GpsData::new(),
        }
    }

    /// Feed one byte from the UART.
    ///
    /// Returns `Some(MsgType)` when a complete valid message
    /// has been decoded and `self.data` updated.
    pub fn push_byte(&mut self, byte: u8) -> Option<MsgType> {
        match self.state {
            ParserState::WaitSync1 => {
                if byte == SYNC_1 {
                    self.state = ParserState::WaitSync2;
                }
                None
            }

            ParserState::WaitSync2 => {
                if byte == SYNC_2 {
                    self.pos = 0;
                    self.state = ParserState::ReadHeader;
                } else {
                    self.state = ParserState::WaitSync1;
                    // Check if this byte is itself a sync1
                    if byte == SYNC_1 {
                        self.state = ParserState::WaitSync2;
                    }
                }
                None
            }

            ParserState::ReadHeader => {
                self.buf[self.pos] = byte;
                self.pos += 1;

                if self.pos >= 4 {
                    // We have class, id, len_l, len_h
                    self.payload_len =
                        (self.buf[2] as u16) | ((self.buf[3] as u16) << 8);

                    if self.payload_len as usize > MAX_PAYLOAD {
                        // Too large, skip this frame
                        self.reset();
                        return None;
                    }

                    self.state = ParserState::ReadPayload;
                }
                None
            }

            ParserState::ReadPayload => {
                self.buf[self.pos] = byte;
                self.pos += 1;

                // Total bytes after header: payload + 2 checksum
                let expected = 4 + self.payload_len as usize + 2;
                if self.pos >= expected {
                    let result = self.try_decode();
                    self.reset();
                    return result;
                }
                None
            }
        }
    }

    fn try_decode(&mut self) -> Option<MsgType> {
        let total = 4 + self.payload_len as usize + 2;

        // Verify Fletcher-16 checksum over class + id + len + payload
        let (ck_a, ck_b) = fletcher16(&self.buf[..total - 2]);

        if ck_a != self.buf[total - 2] || ck_b != self.buf[total - 1] {
            return None;
        }

        let class = self.buf[0];
        let id = self.buf[1];
        let msg_type = MsgType::from_class_id(class, id)?;
        let plen = self.payload_len as usize;

        match msg_type {
            MsgType::NavPvt => {
                if plen < 92 {
                    return None;
                }
                self.decode_nav_pvt();
            }
        }

        Some(msg_type)
    }

    /// Decode NAV-PVT payload (92 bytes).
    ///
    /// Reference: u-blox M8/M9/M10 protocol description,
    /// section UBX-NAV-PVT.
    fn decode_nav_pvt(&mut self) {
        let p = &self.buf[4..4 + self.payload_len as usize];
        // Time (offsets 8-10)
        self.data.hour = p[8];
        self.data.minute = p[9];
        self.data.second = p[10];

        // Valid flags (offset 11): bit 0 = validDate, bit 1 = validTime
        self.data.time_valid = (p[11] & 0x03) == 0x03;

        // Fix (offset 20-23)
        self.data.fix_type = FixType::from_byte(p[20]);
        // flags byte (offset 21): bit 0 = gnssFixOK
        self.data.fix_ok = (p[21] & 0x01) != 0;
        self.data.satellites = p[23];

        // Position (offsets 24-44)
        let lon_1e7 = i32_le(p, 24);
        let lat_1e7 = i32_le(p, 28);
        let h_msl_mm = i32_le(p, 36);
        let h_acc_mm = u32_le(p, 40);
        let v_acc_mm = u32_le(p, 44);

        self.data.longitude = lon_1e7 as f64 * 1e-7;
        self.data.latitude = lat_1e7 as f64 * 1e-7;
        self.data.altitude_msl_m = h_msl_mm as f32 * 0.001;
        self.data.h_acc_m = h_acc_mm as f32 * 0.001;
        self.data.v_acc_m = v_acc_mm as f32 * 0.001;

        // Velocity NED (offsets 48-60)
        let vel_n_mms = i32_le(p, 48);
        let vel_e_mms = i32_le(p, 52);
        let vel_d_mms = i32_le(p, 56);
        let g_speed_mms = i32_le(p, 60);
        let s_acc_mms = u32_le(p, 68);

        self.data.vel_n_ms = vel_n_mms as f32 * 0.001;
        self.data.vel_e_ms = vel_e_mms as f32 * 0.001;
        self.data.vel_d_ms = vel_d_mms as f32 * 0.001;
        self.data.ground_speed_ms = g_speed_mms as f32 * 0.001;
        self.data.s_acc_ms = s_acc_mms as f32 * 0.001;

        // Heading (offset 64)
        let head_mot_1e5 = i32_le(p, 64);
        let head_acc_1e5 = u32_le(p, 72);

        self.data.heading_motion_deg = head_mot_1e5 as f32 * 1e-5;
        self.data.heading_acc_deg = head_acc_1e5 as f32 * 1e-5;

        // pDOP (offset 76, scale 0.01)
        let pdop_raw = u16_le(p, 76);
        self.data.pdop = pdop_raw as f32 * 0.01;

        self.data.updated = true;
    }

    fn reset(&mut self) {
        self.state = ParserState::WaitSync1;
        self.pos = 0;
    }
}

/// Fletcher-16 checksum used by UBX protocol.
fn fletcher16(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in data {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Read a little-endian i32 from a byte slice at the given offset.
fn i32_le(buf: &[u8], off: usize) -> i32 {
    i32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

/// Read a little-endian u32 from a byte slice at the given offset.
fn u32_le(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

/// Read a little-endian u16 from a byte slice at the given offset.
fn u16_le(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

// ---- UBX command construction (for configuring the module) ----

/// Build a complete UBX frame with sync, header, payload, and checksum.
/// Returns the number of bytes written into `out`.
pub fn build_frame(out: &mut [u8], class: u8, id: u8, payload: &[u8]) -> usize {
    let len = payload.len() as u16;
    let total = 6 + payload.len() + 2; // sync(2) + header(4) + payload + ck(2)
    assert!(out.len() >= total);

    out[0] = SYNC_1;
    out[1] = SYNC_2;
    out[2] = class;
    out[3] = id;
    out[4] = (len & 0xFF) as u8;
    out[5] = (len >> 8) as u8;
    out[6..6 + payload.len()].copy_from_slice(payload);

    // Checksum over class + id + len + payload
    let (ck_a, ck_b) = fletcher16(&out[2..6 + payload.len()]);
    out[6 + payload.len()] = ck_a;
    out[7 + payload.len()] = ck_b;

    total
}

/// Build a UBX poll request (empty payload) to request a specific message.
pub fn poll_msg(out: &mut [u8], class: u8, id: u8) -> usize {
    build_frame(out, class, id, &[])
}

// ---- Boot-time configuration ----

/// Target UART baud after `configure()` completes.
pub const TARGET_BAUD: u32 = 115_200;

/// Factory-default baud rate for u-blox modules.
pub const FACTORY_BAUD: u32 = 9600;

/// Listen for ~500ms at the current baud and report what was seen.
///
/// Distinguishes three outcomes so the caller can decide what to do:
/// - `Ubx`: a valid UBX frame parsed fully (module already UBX-configured).
/// - `NmeaOrSync`: we saw a UBX sync byte (0xB5) or an NMEA start ($)
///   but didn't complete a UBX frame — the module is alive at this
///   baud but streaming NMEA (factory default on most u-blox units).
/// - `Silent`: no recognisable GPS bytes within the window (wrong
///   baud or module silent / misrouted).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ProbeResult { Ubx, NmeaOrSync, Silent }

async fn probe_for_data(
    rx: &mut embassy_stm32::usart::UartRx<'_, embassy_stm32::mode::Async>,
) -> ProbeResult {
    use embassy_time::{Duration, Instant, with_timeout};

    let mut parser = UbxParser::new();
    let mut buf = [0u8; 64];
    let deadline = Instant::now() + Duration::from_millis(500);
    let mut saw_hint = false;

    while Instant::now() < deadline {
        let timeout = deadline - Instant::now();
        match with_timeout(timeout, rx.read(&mut buf)).await {
            Ok(Ok(())) => {
                for &byte in &buf {
                    if parser.push_byte(byte).is_some() {
                        return ProbeResult::Ubx;
                    }
                    // 0xB5 = UBX sync1, '$' = NMEA sentence start.
                    // Either is strong evidence the module is alive
                    // at this baud; neither is likely to appear in
                    // random noise from a baud mismatch.
                    if byte == SYNC_1 || byte == b'$' {
                        saw_hint = true;
                    }
                }
            }
            _ => break,
        }
    }
    if saw_hint { ProbeResult::NmeaOrSync } else { ProbeResult::Silent }
}

/// Auto-detect the module's current baud and leave the UART at
/// `TARGET_BAUD` if the module is alive.
///
/// - If the module is already at 115200 (persisted from a prior
///   session, or for some units that ship configured higher):
///   returns 115200 immediately.
/// - If at factory 9600: sends CFG-PRT to switch the module to
///   115200, then bumps the UART to match. u-blox modules change
///   baud immediately on CFG-PRT acknowledgement (unlike WT901B
///   which requires a power cycle), so we keep using the new rate
///   this session. We deliberately do NOT issue CFG-CFG save,
///   because on some modules the setting doesn't persist across
///   cold boots reliably — configuring every boot is cheaper than
///   debugging "why is it 9600 again after unplugging".
/// - If no data at either baud: logs a warning and returns 0.
pub async fn configure(
    tx: &mut embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>,
    rx: &mut embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>,
) -> u32 {
    use embassy_time::{Duration, Timer};

    // Give the module time to boot and start streaming after power-on.
    Timer::after(Duration::from_millis(500)).await;

    // ---- Phase 1: try TARGET_BAUD first ----
    tx.set_baudrate(TARGET_BAUD).unwrap();
    rx.set_baudrate(TARGET_BAUD).unwrap();
    Timer::after(Duration::from_millis(50)).await;

    match probe_for_data(rx).await {
        ProbeResult::Ubx => {
            defmt::info!("GPS: detected UBX at {} baud", TARGET_BAUD);
            return TARGET_BAUD;
        }
        ProbeResult::NmeaOrSync => {
            defmt::info!(
                "GPS: alive at {} but emitting NMEA — switching to UBX-only",
                TARGET_BAUD,
            );
            // CFG-PRT carries both baud and outProtoMask; reuse it
            // to force UBX-only output at the same rate.
            let mut frame = [0u8; 28];
            let n = cfg::set_uart_baud(&mut frame, TARGET_BAUD);
            let _ = tx.write(&frame[..n]).await;
            Timer::after(Duration::from_millis(100)).await;
            enable_nav_pvt(tx).await;
            return TARGET_BAUD;
        }
        ProbeResult::Silent => {}
    }

    // ---- Phase 2: fall back to factory 9600 ----
    defmt::info!("GPS: no data at {}, trying {}", TARGET_BAUD, FACTORY_BAUD);
    tx.set_baudrate(FACTORY_BAUD).unwrap();
    rx.set_baudrate(FACTORY_BAUD).unwrap();
    Timer::after(Duration::from_millis(50)).await;

    let probe2 = probe_for_data(rx).await;
    if probe2 == ProbeResult::Silent {
        defmt::warn!("GPS: no data at {} either — check wiring!", FACTORY_BAUD);
        return 0;
    }

    defmt::info!(
        "GPS: detected at {} ({}), switching module to {} UBX-only",
        FACTORY_BAUD,
        if probe2 == ProbeResult::Ubx { "UBX" } else { "NMEA" },
        TARGET_BAUD,
    );

    // Send CFG-PRT to change the module's UART1 baud AND restrict
    // output to UBX (see `set_uart_baud` — outProtoMask = 0x0001).
    // u-blox modules change rate as soon as the ACK is sent, so we
    // flip our own UART immediately after.
    let mut frame = [0u8; 28];
    let n = cfg::set_uart_baud(&mut frame, TARGET_BAUD);
    let _ = tx.write(&frame[..n]).await;

    // Drain the ACK at the old rate, then flip our UART.
    Timer::after(Duration::from_millis(100)).await;
    tx.set_baudrate(TARGET_BAUD).unwrap();
    rx.set_baudrate(TARGET_BAUD).unwrap();
    Timer::after(Duration::from_millis(100)).await;

    // Module is now silent on UBX — factory configs don't enable
    // any UBX messages by default, only NMEA. Enable NAV-PVT (the
    // single message our parser consumes) at 1 Hz.
    enable_nav_pvt(tx).await;

    TARGET_BAUD
}

/// Enable UBX NAV-PVT (0x01 0x07) at 1 Hz on the current UART.
async fn enable_nav_pvt(
    tx: &mut embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>,
) {
    use embassy_time::{Duration, Timer};
    let mut frame = [0u8; 16];
    // rate=1 means "once per nav solution". CFG-RATE default is 1 Hz.
    let n = cfg::set_msg_rate(&mut frame, 0x01, 0x07, 1);
    let _ = tx.write(&frame[..n]).await;
    Timer::after(Duration::from_millis(50)).await;
    defmt::info!("GPS: enabled NAV-PVT at nav-solution rate");
}

// ---- Common configuration commands ----

pub mod cfg {
    use super::build_frame;

    /// Set the navigation solution output rate on a given port.
    ///
    /// CFG-MSG (0x06 0x01): set message rate for a given class/id.
    /// `rate` is messages per navigation solution (1 = every fix).
    pub fn set_msg_rate(out: &mut [u8], class: u8, id: u8, rate: u8) -> usize {
        // Payload: class, id, rate (for current port)
        build_frame(out, 0x06, 0x01, &[class, id, rate])
    }

    /// Set the navigation solution rate.
    ///
    /// CFG-RATE (0x06 0x08): measurement rate in ms, nav rate (cycles),
    /// time reference (0=UTC, 1=GPS).
    pub fn set_nav_rate(out: &mut [u8], meas_rate_ms: u16, nav_rate: u16) -> usize {
        let mut payload = [0u8; 6];
        payload[0] = (meas_rate_ms & 0xFF) as u8;
        payload[1] = (meas_rate_ms >> 8) as u8;
        payload[2] = (nav_rate & 0xFF) as u8;
        payload[3] = (nav_rate >> 8) as u8;
        payload[4] = 1; // timeRef = GPS
        payload[5] = 0;
        build_frame(out, 0x06, 0x08, &payload)
    }

    /// Set UART1 baud rate via CFG-PRT (0x06 0x00).
    pub fn set_uart_baud(out: &mut [u8], baud: u32) -> usize {
        let mut payload = [0u8; 20];
        payload[0] = 1; // portID = UART1
        // bytes 1 = reserved
        // bytes 2-3 = txReady (disabled)
        // bytes 4-7 = mode: 8N1 = 0x000008D0
        payload[4] = 0xD0;
        payload[5] = 0x08;
        payload[6] = 0x00;
        payload[7] = 0x00;
        // bytes 8-11 = baudRate
        payload[8] = (baud & 0xFF) as u8;
        payload[9] = ((baud >> 8) & 0xFF) as u8;
        payload[10] = ((baud >> 16) & 0xFF) as u8;
        payload[11] = ((baud >> 24) & 0xFF) as u8;
        // bytes 12-13 = inProtoMask: UBX only = 0x0001
        payload[12] = 0x01;
        payload[13] = 0x00;
        // bytes 14-15 = outProtoMask: UBX only = 0x0001
        payload[14] = 0x01;
        payload[15] = 0x00;
        // bytes 16-19 = flags, reserved
        build_frame(out, 0x06, 0x00, &payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a complete UBX frame for testing
    fn make_frame(class: u8, id: u8, payload: &[u8]) -> alloc::vec::Vec<u8> {
        let mut buf = alloc::vec![0u8; 8 + payload.len()];
        let n = build_frame(&mut buf, class, id, payload);
        buf.truncate(n);
        buf
    }

    extern crate alloc;

    #[test]
    fn test_parse_nav_pvt() {
        let mut parser = UbxParser::new();

        // Build a NAV-PVT payload (92 bytes)
        let mut payload = [0u8; 92];

        // Time: 14:30:45
        payload[8] = 14;  // hour
        payload[9] = 30;  // minute
        payload[10] = 45; // second
        payload[11] = 0x03; // validDate + validTime

        // Fix: 3D, gnssFixOK, 12 sats
        payload[20] = 3; // fixType = 3D
        payload[21] = 0x01; // flags: gnssFixOK
        payload[23] = 12; // numSV

        // Lon = 11.5° = 115000000 * 1e-7
        // 115000000 = 0x06DA_C200 LE: 0x00, 0xC2, 0xDA, 0x06
        let lon: i32 = 115_000_000;
        payload[24..28].copy_from_slice(&lon.to_le_bytes());

        // Lat = 48.0° = 480000000 * 1e-7
        let lat: i32 = 480_000_000;
        payload[28..32].copy_from_slice(&lat.to_le_bytes());

        // hMSL = 500m = 500000 mm
        let h_msl: i32 = 500_000;
        payload[36..40].copy_from_slice(&h_msl.to_le_bytes());

        // hAcc = 2.5m = 2500 mm
        let h_acc: u32 = 2500;
        payload[40..44].copy_from_slice(&h_acc.to_le_bytes());

        // velN = 5.0 m/s = 5000 mm/s
        let vel_n: i32 = 5000;
        payload[48..52].copy_from_slice(&vel_n.to_le_bytes());

        // velE = -3.0 m/s = -3000 mm/s
        let vel_e: i32 = -3000;
        payload[52..56].copy_from_slice(&vel_e.to_le_bytes());

        // gSpeed = 5831 mm/s (sqrt(5^2 + 3^2) * 1000)
        let g_speed: i32 = 5831;
        payload[60..64].copy_from_slice(&g_speed.to_le_bytes());

        // headMot = 329.04° = 32904000 * 1e-5
        let head_mot: i32 = 32_904_000;
        payload[64..68].copy_from_slice(&head_mot.to_le_bytes());

        // pDOP = 1.5 = 150 raw
        let pdop: u16 = 150;
        payload[76..78].copy_from_slice(&pdop.to_le_bytes());

        let frame = make_frame(0x01, 0x07, &payload);

        let mut result = None;
        for &b in &frame {
            if let Some(t) = parser.push_byte(b) {
                result = Some(t);
            }
        }

        assert_eq!(result, Some(MsgType::NavPvt));
        assert!((parser.data.latitude - 48.0).abs() < 0.0001);
        assert!((parser.data.longitude - 11.5).abs() < 0.0001);
        assert!((parser.data.altitude_msl_m - 500.0).abs() < 0.1);
        assert!((parser.data.h_acc_m - 2.5).abs() < 0.01);
        assert!((parser.data.vel_n_ms - 5.0).abs() < 0.01);
        assert!((parser.data.vel_e_ms - (-3.0)).abs() < 0.01);
        assert!((parser.data.ground_speed_ms - 5.831).abs() < 0.01);
        assert_eq!(parser.data.fix_type, FixType::Fix3D);
        assert!(parser.data.fix_ok);
        assert_eq!(parser.data.satellites, 12);
        assert!((parser.data.pdop - 1.5).abs() < 0.01);
        assert_eq!(parser.data.hour, 14);
        assert_eq!(parser.data.minute, 30);
        assert_eq!(parser.data.second, 45);
        assert!(parser.data.time_valid);
        assert!(parser.data.updated);
    }

    #[test]
    fn test_bad_checksum_rejected() {
        let mut parser = UbxParser::new();

        let mut frame = make_frame(0x01, 0x07, &[0u8; 92]);
        let last = frame.len() - 1;
        frame[last] = frame[last].wrapping_add(1); // corrupt checksum

        let mut got_msg = false;
        for &b in &frame {
            if parser.push_byte(b).is_some() {
                got_msg = true;
            }
        }
        assert!(!got_msg);
    }

    #[test]
    fn test_resyncs_after_garbage() {
        let mut parser = UbxParser::new();

        // Feed garbage
        for &b in &[0x12, 0x34, 0xB5, 0x00, 0x56, 0x78] {
            parser.push_byte(b);
        }

        // Then a valid frame
        let frame = make_frame(0x01, 0x07, &[0u8; 92]);
        let mut got_msg = false;
        for &b in &frame {
            if parser.push_byte(b).is_some() {
                got_msg = true;
            }
        }
        assert!(got_msg);
    }

    #[test]
    fn test_too_large_payload_rejected() {
        let mut parser = UbxParser::new();

        // Craft a frame header claiming 200 bytes payload
        let bytes = [SYNC_1, SYNC_2, 0x01, 0x07, 0xC8, 0x00];
        for &b in &bytes {
            parser.push_byte(b);
        }
        // Parser should have reset
        assert_eq!(parser.state, ParserState::WaitSync1);
    }

    #[test]
    fn test_fletcher16() {
        // Known test vector: class=0x01, id=0x07, len=0, payload=empty
        // ck_a = 0x01 + 0x07 + 0x00 + 0x00 = 0x08
        // ck_b = 0x01 + 0x08 + 0x08 + 0x08 = 0x19
        let data = [0x01, 0x07, 0x00, 0x00];
        let (a, b) = fletcher16(&data);
        assert_eq!(a, 0x08);
        assert_eq!(b, 0x19);
    }

    #[test]
    fn test_build_frame() {
        let mut buf = [0u8; 16];
        let n = build_frame(&mut buf, 0x01, 0x07, &[]);
        assert_eq!(n, 8); // sync(2) + class(1) + id(1) + len(2) + ck(2)
        assert_eq!(buf[0], SYNC_1);
        assert_eq!(buf[1], SYNC_2);
        assert_eq!(buf[2], 0x01);
        assert_eq!(buf[3], 0x07);
        assert_eq!(buf[4], 0x00); // len_l
        assert_eq!(buf[5], 0x00); // len_h
        assert_eq!(buf[6], 0x08); // ck_a
        assert_eq!(buf[7], 0x19); // ck_b
    }
}
