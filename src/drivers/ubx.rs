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

/// Navigation solution period in milliseconds. 100 ms = 10 Hz.
///
/// Chosen to match what gps_accel::GpsAccelEstimator assumes ("~5 fixes
/// at 10 Hz are averaged") and what pos_kf's velocity fusion is tuned
/// for. Raising it costs bandwidth and lowering it costs the whole
/// benefit of the acceleration estimate.
pub const NAV_RATE_MS: u16 = 100;

/// Target UART baud after `configure()` completes.
pub const TARGET_BAUD: u32 = 115_200;

/// Factory-default baud rate for u-blox M8 modules, and what the M8
/// build of the Radiolink SE100 ships at.
pub const FACTORY_BAUD: u32 = 9600;

/// The other factory default worth probing: u-blox M10 modules ship at
/// 38400, and the M10 SE100 is externally indistinguishable from the M8
/// one. Cheaper to try than to open the case.
pub const ALT_FACTORY_BAUD: u32 = 38_400;

/// Candidate baud rates, in probe order.
///
/// 115200 first because that is where a module configured by a previous
/// boot already is, and finding it there costs one window instead of
/// three.
///
/// Then both factory defaults. 9600 is the u-blox M8 default and what
/// the SE100's M8 build ships at; 38400 is the M10 default, and the M10
/// SE100 is externally identical. You cannot tell which you have without
/// opening it, so the firmware asks instead of assuming.
///
/// The list is deliberately wider than the two factory defaults. A
/// framing error means the line is carrying transitions at a rate we are
/// not clocking -- so "errors at every candidate" is evidence the module
/// is on a rate NOT in this list, and the cure is more candidates. These
/// four extras are the rates FPV vendors and configurators actually
/// leave M10 modules on for 10 Hz work.
pub const CANDIDATE_BAUDS: [u32; 6] = [
    TARGET_BAUD,
    FACTORY_BAUD,
    ALT_FACTORY_BAUD,
    230_400,
    57_600,
    460_800,
];

/// How long to listen at each candidate baud.
///
/// Has to exceed one solution period, because an unconfigured receiver
/// emits its whole sentence set in a burst once per second and a shorter
/// window can land entirely in the gap and call a live module silent.
/// 1200 ms covers a 1 Hz burst with margin. A module that IS at the baud
/// being tried usually answers well inside this -- the window only runs
/// to completion when the guess was wrong.
pub const PROBE_WINDOW_MS: u64 = 1200;

/// How long the first pass listens at each candidate baud.
///
/// The full window cannot be spent on every candidate: at 1200 ms a list
/// long enough to actually FIND a misconfigured module costs more boot
/// latency than the GPS is worth (see
/// `worst_case_probe_time_stays_reasonable`). So the sweep is two-pass.
///
/// Pass one is a census, not a search. It does not need to see a whole
/// frame -- it only needs to know whether the line produces clockable
/// bytes at this rate, and a module streaming at 10 Hz answers that in
/// one solution period. 200 ms covers two.
///
/// A 1 Hz burster can sit out a 200 ms census entirely, which is exactly
/// why pass two exists and why it still runs when the census finds
/// nothing at all.
pub const CENSUS_WINDOW_MS: u64 = 200;

/// How many candidates pass two is allowed to spend a full window on.
pub const FULL_WINDOW_ATTEMPTS: usize = 3;

/// What was heard at a given baud.
///
/// The distinction that matters is CHECKSUM-VALID or nothing. An earlier
/// version accepted a single 0xB5 or '$' byte as evidence of life, which
/// does not survive contact with a baud mismatch: a receiver transmitting
/// at 9600 while we listen at 115200 produces a continuous stream of
/// framing garbage, and over a window of several thousand bytes the odds
/// of never seeing one of two specific byte values are negligible. The
/// probe would lock onto the wrong rate almost every time it guessed
/// wrong -- and the more candidate bauds there are, the more chances it
/// gets to be wrong. Both arms below now require a frame whose checksum
/// verifies, which garbage does not produce.
///
/// A fourth outcome used to be folded into `Silent` and should not have
/// been: see `ProbeStats`.
#[cfg(feature = "firmware")]
#[derive(Debug, Clone, Copy, PartialEq)]
enum ProbeResult {
    /// A complete UBX frame parsed and its Fletcher-16 checked out.
    Ubx,
    /// A complete NMEA sentence parsed and its XOR checksum checked out.
    Nmea,
    /// Nothing intelligible within the window.
    Silent,
}

/// What the line actually did during one probe window.
///
/// The point of this type is that "no valid frame" and "nothing on the
/// wire" are completely different faults and the old probe reported both
/// as "nothing at N baud". A dead RX pin, a module on a baud we do not
/// probe, and a module talking to us correctly but losing bytes to
/// buffer turnaround are three different repairs, and they are told
/// apart by bytes-received and error counts -- not by whether a checksum
/// happened to verify.
#[cfg(feature = "firmware")]
#[derive(Debug, Clone, Copy, Default)]
struct ProbeStats {
    bytes: u32,
    framing: u32,
    noise: u32,
    overrun: u32,
    other: u32,
    /// First bytes seen, for the log. At a wrong baud these are the
    /// mis-clocked bits of real characters, and the pattern is often
    /// enough to recognise the true rate by eye.
    head: [u8; 8],
    head_n: usize,
}

#[cfg(feature = "firmware")]
impl ProbeStats {
    fn errors(&self) -> u32 {
        self.framing + self.noise + self.overrun + self.other
    }
}

/// Listen at the current baud for `PROBE_WINDOW_MS` and report both what
/// parsed and what was seen.
///
/// Errors do NOT end the window. The previous version broke out of the
/// loop on the first `Err`, which made every "nothing at N baud" line in
/// the boot log a lie: the flight log showed windows of 356 ms and 106 ms
/// against a nominal 1200 ms, i.e. the probe was abandoning the rate
/// after one framing error rather than listening. A framing error is
/// evidence ABOUT the baud, not grounds to stop collecting evidence --
/// and on the correct baud a single noise glitch would equally have
/// thrown the rate away.
#[cfg(feature = "firmware")]
async fn probe_for_data(
    rx: &mut embassy_stm32::usart::UartRx<'_, embassy_stm32::mode::Async>,
    window_ms: u64,
) -> (ProbeResult, ProbeStats) {
    use embassy_stm32::usart::Error as UartError;
    use embassy_time::{with_timeout, Duration, Instant};

    let mut ubx = UbxParser::new();
    let mut nmea = super::nmea::NmeaParser::new();
    let mut buf = [0u8; 64];
    let mut st = ProbeStats::default();
    let deadline = Instant::now() + Duration::from_millis(window_ms);

    while Instant::now() < deadline {
        let timeout = deadline - Instant::now();
        // read_until_idle, not read: `read` completes only when all 64
        // bytes have arrived, so a burst shorter than the buffer is held
        // until the window expires and is then DISCARDED with the dropped
        // future. Idle-line detection hands back short bursts intact,
        // which is what both parsers need to see a whole frame.
        match with_timeout(timeout, rx.read_until_idle(&mut buf)).await {
            Ok(Ok(n)) => {
                st.bytes += n as u32;
                for &byte in &buf[..n] {
                    if st.head_n < st.head.len() {
                        st.head[st.head_n] = byte;
                        st.head_n += 1;
                    }
                    if ubx.push_byte(byte).is_some() {
                        return (ProbeResult::Ubx, st);
                    }
                    if nmea.push_byte(byte).is_some() {
                        return (ProbeResult::Nmea, st);
                    }
                }
            }
            Ok(Err(e)) => {
                match e {
                    UartError::Framing => st.framing += 1,
                    UartError::Noise => st.noise += 1,
                    UartError::Overrun => st.overrun += 1,
                    _ => st.other += 1,
                }
                // Keep listening. The next read re-arms DMA and clears
                // the stale error flag on the way in.
            }
            // Window expired mid-read.
            Err(_) => break,
        }
    }
    (ProbeResult::Silent, st)
}

/// Find the module, put it on `TARGET_BAUD` speaking UBX only, and set
/// the solution rate.
///
/// Returns the baud the link ends up on, or 0 if nothing answered at any
/// candidate rate -- in which case the caller's NMEA fallback is still
/// live and behaves as it did before.
///
/// Every successful path ends the same way: CFG-PRT to pin baud and
/// UBX-only output, then `enable_nav_pvt`. Including the case where the
/// module was ALREADY at 115200 speaking UBX, which the previous version
/// returned from immediately. That shortcut assumed a module found in
/// UBX had been configured by us, and since CFG-RATE became load-bearing
/// the assumption is no longer safe: a module set up by u-center at its
/// 1 Hz default would have been accepted as correct and would silently
/// starve gps_accel of the rate it needs. These commands are idempotent
/// and cost a few hundred milliseconds once at boot.
///
/// CFG-CFG (save to flash) is deliberately NOT sent: on some modules it
/// does not persist across cold boots reliably, and configuring every
/// boot is cheaper than debugging "why is it 9600 again after
/// unplugging".
#[cfg(feature = "firmware")]
pub async fn configure(
    tx: &mut embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>,
    rx: &mut embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>,
) -> u32 {
    use embassy_time::{Duration, Timer};

    // Give the module time to boot and start streaming after power-on.
    Timer::after(Duration::from_millis(500)).await;

    // ---- Pass one: census every candidate rate ----
    //
    // Cheap windows across the whole list, recording what the line does
    // at each rate rather than only whether a frame parsed. This is the
    // pass that answers "which rate is the module actually on", which a
    // single-pass search cannot afford to ask across a list this long.
    // If a frame happens to parse here, so much the better -- take it.
    let mut census = [ProbeStats::default(); CANDIDATE_BAUDS.len()];
    let mut found_at: Option<(u32, ProbeResult)> = None;

    for (i, &baud) in CANDIDATE_BAUDS.iter().enumerate() {
        tx.set_baudrate(baud).unwrap();
        rx.set_baudrate(baud).unwrap();
        Timer::after(Duration::from_millis(50)).await;

        let (res, st) = probe_for_data(rx, CENSUS_WINDOW_MS).await;
        census[i] = st;
        log_probe(baud, "census", res, &st);
        if res != ProbeResult::Silent {
            found_at = Some((baud, res));
            break;
        }
    }

    // ---- Pass two: full windows, best evidence first ----
    //
    // Ordered by what the census saw. A rate that produced clockable
    // bytes is a better bet than one that produced only framing errors,
    // and both beat a rate that produced nothing -- but "nothing" is not
    // disqualifying, because a 1 Hz burster is silent for most of a
    // 200 ms census. So every candidate stays eligible; the census only
    // decides the ORDER and how many get a full window.
    if found_at.is_none() {
        let mut order: [usize; CANDIDATE_BAUDS.len()] = [0; CANDIDATE_BAUDS.len()];
        for (i, slot) in order.iter_mut().enumerate() {
            *slot = i;
        }
        // Insertion sort by descending evidence; stable, so an all-zero
        // census leaves CANDIDATE_BAUDS order untouched and pass two
        // degrades to exactly the old behaviour.
        for i in 1..order.len() {
            let mut j = i;
            while j > 0 && evidence(&census[order[j]]) > evidence(&census[order[j - 1]]) {
                order.swap(j, j - 1);
                j -= 1;
            }
        }

        for &i in order.iter().take(FULL_WINDOW_ATTEMPTS) {
            let baud = CANDIDATE_BAUDS[i];
            tx.set_baudrate(baud).unwrap();
            rx.set_baudrate(baud).unwrap();
            Timer::after(Duration::from_millis(50)).await;

            let (res, st) = probe_for_data(rx, PROBE_WINDOW_MS).await;
            log_probe(baud, "full", res, &st);
            if res != ProbeResult::Silent {
                found_at = Some((baud, res));
                break;
            }
        }
    }

    let (baud, found) = match found_at {
        Some(v) => v,
        None => {
            defmt::warn!(
                "GPS: no valid frame at any of {} candidate bauds. Read the per-baud lines above: bytes=0 and err=0 everywhere means the RX line never moved (wiring, or the module is not transmitting); err>0 at every rate means it IS transmitting, on a rate not in CANDIDATE_BAUDS.",
                CANDIDATE_BAUDS.len(),
            );
            return 0;
        }
    };

    defmt::info!(
        "GPS: module found at {} baud speaking {}",
        baud,
        if found == ProbeResult::Ubx { "UBX" } else { "NMEA" },
    );

    // CFG-PRT carries baud AND the protocol masks, so this both moves
    // the module to TARGET_BAUD and restricts it to UBX output. Sent
    // even when baud already equals TARGET_BAUD, for the masks.
    //
    // NOTE: CFG-PRT (0x06 0x00) exists only on M8 and earlier. An M10
    // supports exactly five UBX-CFG messages -- CFG-CFG, CFG-RST,
    // CFG-VALDEL, CFG-VALGET, CFG-VALSET -- and NAKs everything else, so
    // on an M10 this and the two commands in `enable_nav_pvt` are all
    // rejected and the module stays on whatever it booted with. See the
    // module comment on `cfg`.
    let mut frame = [0u8; 28];
    let n = cfg::set_uart_baud(&mut frame, TARGET_BAUD);
    let _ = tx.write(&frame[..n]).await;

    // Same request through the modern interface, for M9/M10 parts that
    // do not implement CFG-PRT. VALSET applies atomically, so baud and
    // the protocol masks either all take effect or none do -- the module
    // cannot end up at the new baud while still emitting NMEA.
    //
    // NMEA output off matches what the CFG-PRT masks above ask for. It is
    // safe against losing the NMEA fallback: if this frame is rejected
    // nothing changes, and if it is accepted then NAV-PVT was enabled by
    // the same atomic write.
    let n = cfg::valset(
        &mut frame,
        cfg::LAYER_RAM,
        &[
            (cfg::KEY_UART1_BAUDRATE, TARGET_BAUD as u64),
            (cfg::KEY_UART1_OUTPROT_UBX, 1),
            (cfg::KEY_UART1_OUTPROT_NMEA, 0),
        ],
    );
    if n > 0 {
        let _ = tx.write(&frame[..n]).await;
    }

    // u-blox modules switch as soon as the command is accepted, so
    // let the ACK drain at the OLD rate before following it.
    Timer::after(Duration::from_millis(100)).await;
    if baud != TARGET_BAUD {
        tx.set_baudrate(TARGET_BAUD).unwrap();
        rx.set_baudrate(TARGET_BAUD).unwrap();
        Timer::after(Duration::from_millis(100)).await;
    }

    // Factory configs enable no UBX messages at all, only NMEA, so
    // the module is silent on UBX until this.
    enable_nav_pvt(tx).await;
    TARGET_BAUD
}

/// Rank a census result. Higher is a better bet for a full window.
///
/// Bytes outrank errors because a rate that clocks characters at all is
/// closer to correct than one that only produces framing errors, and
/// both outrank silence. Errors still count for something: they prove
/// the line is alive, which is the difference between "wrong baud" and
/// "nothing connected".
#[cfg(feature = "firmware")]
fn evidence(st: &ProbeStats) -> u32 {
    st.bytes.saturating_mul(4).saturating_add(st.errors())
}

/// One per-baud line, covering both passes.
#[cfg(feature = "firmware")]
fn log_probe(baud: u32, pass: &str, res: ProbeResult, st: &ProbeStats) {
    if res != ProbeResult::Silent {
        return;
    }
    defmt::info!(
        "GPS [{}]: no valid frame at {} baud - {=u32} bytes, {=u32} err ({=u32} framing / {=u32} noise / {=u32} overrun), head {=[u8]:02x}",
        pass,
        baud,
        st.bytes,
        st.errors(),
        st.framing,
        st.noise,
        st.overrun,
        &st.head[..st.head_n],
    );
}

/// Put NAV-PVT on the current UART at `NAV_RATE_MS`.
///
/// Sends the request twice, once through each generation's configuration
/// interface, because the module generation is not known at runtime and
/// each generation ignores the other's messages:
///
///   - M8 and earlier understand CFG-RATE / CFG-MSG;
///   - M9/M10 understand CFG-VALSET and NAK CFG-RATE / CFG-MSG, which
///     are not in their UBX-CFG class at all.
///
/// Both are idempotent and cost a few hundred bytes once at boot. The
/// alternative -- identify the part from UBX-MON-VER first -- needs a
/// reply parser and an ACK/NAK path, and buys nothing: whichever message
/// the module does not recognise it simply rejects.
///
/// This is why an M10 sat at 1 Hz on the bench: only the legacy pair was
/// ever sent, so the solution rate never moved off the module default,
/// and nothing parsed the NAK that said so.
#[cfg(feature = "firmware")]
async fn enable_nav_pvt(
    tx: &mut embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>,
) {
    use embassy_time::{Duration, Timer};
    let mut frame = [0u8; 32];

    // ---- Modern interface (M9/M10) ----
    //
    // One VALSET carrying all three items. VALSET applies atomically, so
    // the solution rate and the message rate cannot land out of step the
    // way two separate legacy commands can.
    //
    // 100 ms with nav_rate = 1 is 10 Hz. One NAV-PVT is 100 bytes, so
    // 10 Hz is 1 kB/s against 11.5 kB/s of a 115200 link -- comfortable.
    // It would NOT fit in the factory 9600.
    let n = cfg::valset(
        &mut frame,
        cfg::LAYER_RAM,
        &[
            (cfg::KEY_RATE_MEAS, NAV_RATE_MS as u64),
            (cfg::KEY_RATE_NAV, 1),
            (cfg::KEY_MSGOUT_NAV_PVT_UART1, 1),
        ],
    );
    if n > 0 {
        let _ = tx.write(&frame[..n]).await;
        Timer::after(Duration::from_millis(50)).await;
    } else {
        defmt::warn!("GPS: CFG-VALSET frame did not build - skipping modern config");
    }

    // ---- Legacy interface (M8 and earlier) ----
    //
    // Solution rate FIRST, then the message rate against it.
    //
    // gps_accel.rs differentiates GPS velocity and is specified against
    // 10 Hz fixes; at 1 Hz the differentiation interval is a full second
    // and the estimate is worthless.
    let n = cfg::set_nav_rate(&mut frame, NAV_RATE_MS, 1);
    let _ = tx.write(&frame[..n]).await;
    Timer::after(Duration::from_millis(50)).await;

    // rate=1 means "once per nav solution", so this now means 10 Hz.
    let n = cfg::set_msg_rate(&mut frame, 0x01, 0x07, 1);
    let _ = tx.write(&frame[..n]).await;
    Timer::after(Duration::from_millis(50)).await;
    defmt::info!(
        "GPS: NAV-PVT requested at {} ms solution rate (VALSET + legacy CFG)",
        NAV_RATE_MS,
    );
}

// ---- Common configuration commands ----

// ---- LEGACY (M8-era) CONFIGURATION INTERFACE ----
//
// Every command in this module is a UBX-CFG-* message from the interface
// u-blox froze at protocol version 23.01. They work on M8 and earlier.
//
// They DO NOT WORK ON M10. The u-blox M10 SPG 5.10 interface description
// (UBX-21035062, protocol 34.10) documents exactly five messages in the
// UBX-CFG class, section 3.10:
//
//     UBX-CFG-CFG    0x06 0x09
//     UBX-CFG-RST    0x06 0x04
//     UBX-CFG-VALDEL 0x06 0x8c
//     UBX-CFG-VALGET 0x06 0x8b
//     UBX-CFG-VALSET 0x06 0x8a
//
// CFG-PRT (0x06 0x00), CFG-MSG (0x06 0x01) and CFG-RATE (0x06 0x08) are
// not among them. They survive in that document only as an appendix
// mapping each legacy field onto a configuration key -- e.g.
// `UBX-CFG-PRT.baudRate` -> `CFG-UART1-BAUDRATE` (0x40520001). An M10
// answers all three with UBX-ACK-NAK and changes nothing.
//
// Consequence for this driver: on an M10 the probe may find the module
// and report success, then fail to move its baud, fail to restrict it to
// UBX, and fail to set the 10 Hz solution rate -- silently, because no
// ACK/NAK is parsed. The module keeps whatever configuration it booted
// with (factory: 38400 baud, NMEA, 1 Hz -- CFG-UART1-BAUDRATE default is
// 38400, Table 84).
//
// Replacing these with CFG-VALSET is the fix; it is not done here yet.
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

    // ---- Modern (M9/M10) configuration interface ----
    //
    // UBX-CFG-VALSET (0x06 0x8a). Payload, section 3.10.5:
    //
    //   byte 0    U1     version = 0x00
    //   byte 1    X1     layers: bit0 RAM, bit1 BBR, bit2 Flash
    //   bytes 2-3 U1[2]  reserved
    //   bytes 4+         key/value pairs, concatenated with NO padding
    //
    // A key is a U4 whose bits 30..28 encode the VALUE width, so the key
    // itself says how many bytes follow it:
    //
    //   0x01 = one bit (stored in one byte)   0x02 = one byte
    //   0x03 = two bytes                      0x04 = four bytes
    //   0x05 = eight bytes
    //
    // `valset` derives the width from that encoding rather than taking a
    // separate length argument, which makes a key/width mismatch
    // unrepresentable. The failure it replaces is a frame the receiver
    // NAKs as a whole, with no indication of which pair was malformed.

    /// RAM layer only.
    ///
    /// Deliberately not Flash: same reasoning as the CFG-CFG note on
    /// `configure`. Configuring every boot is cheaper than debugging a
    /// module that half-remembers a previous firmware's settings.
    pub const LAYER_RAM: u8 = 0x01;

    /// Measurement rate, milliseconds. U2.
    pub const KEY_RATE_MEAS: u32 = 0x3021_0001;
    /// Measurements per navigation solution. U2.
    pub const KEY_RATE_NAV: u32 = 0x3021_0002;
    /// NAV-PVT output rate on UART1, in solutions per message. U1.
    pub const KEY_MSGOUT_NAV_PVT_UART1: u32 = 0x2091_0007;
    /// UART1 baud rate. U4.
    pub const KEY_UART1_BAUDRATE: u32 = 0x4052_0001;
    /// UBX as an output protocol on UART1. L (one bit, one byte stored).
    pub const KEY_UART1_OUTPROT_UBX: u32 = 0x1074_0001;
    /// NMEA as an output protocol on UART1. L. Defaults to 1 (Table 87),
    /// so an unconfigured M10 emits NMEA alongside UBX.
    pub const KEY_UART1_OUTPROT_NMEA: u32 = 0x1074_0002;

    /// Value width in bytes that a key ID declares, from bits 30..28.
    ///
    /// Returns 0 for the reserved size codes. No key this firmware uses
    /// has one, and `valset` refuses to encode such a key rather than
    /// guessing a width for it.
    pub const fn key_value_len(key: u32) -> usize {
        match (key >> 28) & 0x7 {
            0x01 => 1, // one bit, stored in a byte
            0x02 => 1,
            0x03 => 2,
            0x04 => 4,
            0x05 => 8,
            _ => 0,
        }
    }

    /// Build a CFG-VALSET frame from key/value pairs.
    ///
    /// Each value is taken little-endian from the low bytes of the `u64`
    /// and truncated to the width its key declares. Returns 0 if a key
    /// has an unknown width or the frame does not fit, so callers send
    /// nothing rather than something malformed: VALSET applies
    /// atomically, and one bad pair voids the whole message.
    pub fn valset(out: &mut [u8], layers: u8, pairs: &[(u32, u64)]) -> usize {
        // 64 pairs is the documented maximum for one VALSET, and 12 bytes
        // is the widest pair (4-byte key + 8-byte value).
        let mut payload = [0u8; 4 + 64 * 12];
        payload[0] = 0x00; // version
        payload[1] = layers;
        // bytes 2..4 are reserved and already zero.
        let mut n = 4;

        for &(key, value) in pairs {
            let width = key_value_len(key);
            if width == 0 || n + 4 + width > payload.len() {
                return 0;
            }
            payload[n..n + 4].copy_from_slice(&key.to_le_bytes());
            n += 4;
            payload[n..n + width].copy_from_slice(&value.to_le_bytes()[..width]);
            n += width;
        }

        if out.len() < n + 8 {
            return 0;
        }
        build_frame(out, 0x06, 0x8A, &payload[..n])
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
    fn every_candidate_baud_is_distinct_and_starts_at_the_target() {
        // A duplicated entry would waste a full probe window at boot for
        // nothing, and would be invisible -- the loop would just try the
        // same rate twice and report the same result.
        for (i, a) in CANDIDATE_BAUDS.iter().enumerate() {
            for b in CANDIDATE_BAUDS.iter().skip(i + 1) {
                assert_ne!(a, b, "duplicate candidate baud {a}");
            }
        }
        // TARGET_BAUD first: a module already configured by a previous
        // boot is found in one window instead of three.
        assert_eq!(CANDIDATE_BAUDS[0], TARGET_BAUD);
    }

    #[test]
    fn both_factory_defaults_are_probed() {
        // 9600 is the u-blox M8 default, 38400 the M10's. The SE100 ships
        // in both builds and they look identical from outside, so leaving
        // either out means a module that simply never answers.
        assert!(CANDIDATE_BAUDS.contains(&FACTORY_BAUD), "M8 default missing");
        assert!(CANDIDATE_BAUDS.contains(&ALT_FACTORY_BAUD), "M10 default missing");
        assert_eq!(ALT_FACTORY_BAUD, 38_400);
    }

    #[test]
    fn probe_window_outlasts_an_unconfigured_receivers_burst() {
        // An unconfigured module emits its whole NMEA set once per
        // second. A window shorter than that can land entirely in the
        // gap and declare a live module silent -- which then shows up as
        // "the GPS does not work", three candidate rates later.
        assert!(
            PROBE_WINDOW_MS > 1000,
            "{PROBE_WINDOW_MS} ms can miss a 1 Hz burst entirely"
        );
    }

    #[test]
    fn worst_case_probe_time_stays_reasonable() {
        // Nothing connected means every window runs to completion. This
        // is boot latency the user pays before the GPS task starts
        // reading, so it is worth knowing when it grows.
        //
        // Two passes now: a census window at every candidate, then a
        // full window at up to FULL_WINDOW_ATTEMPTS of them. This is what
        // buys a candidate list long enough to find a module that is not
        // on a factory default -- a single-pass sweep at the full window
        // would blow this budget at four candidates.
        let census_ms = CENSUS_WINDOW_MS * CANDIDATE_BAUDS.len() as u64;
        let full_ms = PROBE_WINDOW_MS * FULL_WINDOW_ATTEMPTS as u64;
        let worst_ms = census_ms + full_ms;
        assert!(worst_ms <= 5000, "{worst_ms} ms of probing at boot");
    }

    #[test]
    fn census_pass_is_cheap_enough_to_afford_every_candidate() {
        // The census only has to catch a streaming module, so it can be
        // far shorter than the burst-catching full window. If it ever
        // grows to the point where sweeping the whole list costs as much
        // as a full window, the two-pass split has stopped paying for
        // itself.
        assert!(
            CENSUS_WINDOW_MS * CANDIDATE_BAUDS.len() as u64 <= PROBE_WINDOW_MS * 2,
            "census sweep is no longer cheap relative to a full window",
        );
        assert!(FULL_WINDOW_ATTEMPTS <= CANDIDATE_BAUDS.len());
    }

    #[test]
    fn the_factory_defaults_are_all_still_candidates() {
        // Widening the list must not drop the rates a module is most
        // likely to actually be on. M8 ships at 9600, M10 at 38400, and
        // a module a previous boot configured is at TARGET_BAUD.
        for b in [TARGET_BAUD, FACTORY_BAUD, ALT_FACTORY_BAUD] {
            assert!(CANDIDATE_BAUDS.contains(&b), "{b} baud is no longer probed");
        }
    }


    // ---- CFG-VALSET (M9/M10 configuration interface) ----

    #[test]
    fn key_ids_declare_the_widths_the_datasheet_gives_them() {
        // Bits 30..28 of a key ID are its value width (section 4,
        // "Configuration Key ID"). Every key below was read off the M10
        // SPG 5.10 interface description; if one is mistyped the width
        // almost always changes with it, so this catches transcription
        // errors that would otherwise show up as a NAKed frame.
        assert_eq!(cfg::key_value_len(cfg::KEY_RATE_MEAS), 2, "CFG-RATE-MEAS is U2");
        assert_eq!(cfg::key_value_len(cfg::KEY_RATE_NAV), 2, "CFG-RATE-NAV is U2");
        assert_eq!(
            cfg::key_value_len(cfg::KEY_MSGOUT_NAV_PVT_UART1), 1,
            "CFG-MSGOUT-UBX_NAV_PVT_UART1 is U1",
        );
        assert_eq!(
            cfg::key_value_len(cfg::KEY_UART1_BAUDRATE), 4,
            "CFG-UART1-BAUDRATE is U4",
        );
        // L (single bit) still occupies a whole byte on the wire.
        assert_eq!(cfg::key_value_len(cfg::KEY_UART1_OUTPROT_UBX), 1);
        assert_eq!(cfg::key_value_len(cfg::KEY_UART1_OUTPROT_NMEA), 1);
        // Reserved size codes are refused, not guessed at.
        assert_eq!(cfg::key_value_len(0x0000_0001), 0);
        assert_eq!(cfg::key_value_len(0x6000_0001), 0);
    }

    #[test]
    fn valset_rate_frame_is_byte_exact() {
        // The frame that fixes the 1 Hz M10. Built by hand from section
        // 3.10.5: version, layers, two reserved bytes, then key/value
        // pairs concatenated with NO padding, every field little-endian.
        let mut out = [0u8; 64];
        let n = cfg::valset(
            &mut out,
            cfg::LAYER_RAM,
            &[
                (cfg::KEY_RATE_MEAS, NAV_RATE_MS as u64),
                (cfg::KEY_RATE_NAV, 1),
                (cfg::KEY_MSGOUT_NAV_PVT_UART1, 1),
            ],
        );

        let expect: [u8; 29] = [
            0xB5, 0x62, // sync
            0x06, 0x8A, // CFG-VALSET
            21, 0x00,   // payload length
            0x00,       // version
            0x01,       // layers = RAM
            0x00, 0x00, // reserved
            0x01, 0x00, 0x21, 0x30, 0x64, 0x00, // CFG-RATE-MEAS = 100 ms
            0x02, 0x00, 0x21, 0x30, 0x01, 0x00, // CFG-RATE-NAV = 1
            0x07, 0x00, 0x91, 0x20, 0x01,       // NAV-PVT on UART1 = 1
            0x00, 0x00, // checksum, filled below
        ];
        assert_eq!(n, expect.len(), "frame length");

        let (ck_a, ck_b) = fletcher16(&out[2..n - 2]);
        let mut want = expect;
        want[27] = ck_a;
        want[28] = ck_b;
        assert_eq!(&out[..n], &want[..], "frame bytes");

        // 100 ms measurement rate with 1 measurement per solution is the
        // 10 Hz this whole exercise is for.
        assert_eq!(NAV_RATE_MS, 100);
    }

    #[test]
    fn valset_truncates_each_value_to_its_keys_width() {
        // A U1 key must emit one byte even though the value is a u64.
        // Emitting eight would shift every following pair and void the
        // frame -- and the receiver reports that as a single NAK with no
        // indication of which pair was wrong.
        let mut out = [0u8; 64];
        let n = cfg::valset(&mut out, cfg::LAYER_RAM, &[(cfg::KEY_MSGOUT_NAV_PVT_UART1, 0xFFFF_FFFF_FFFF_FF01)]);
        assert_eq!(n, 8 + 4 + 4 + 1, "header + framing + key + one value byte");
        // Frame is sync(2) + class/id/len(4) + payload; the payload opens
        // with version/layers/reserved(4) then the 4-byte key, so the
        // first value byte lands at index 14.
        assert_eq!(&out[10..14], &cfg::KEY_MSGOUT_NAV_PVT_UART1.to_le_bytes(), "key");
        assert_eq!(out[14], 0x01, "value truncated to its low byte");

        // U4 key, four bytes, little-endian.
        let n = cfg::valset(&mut out, cfg::LAYER_RAM, &[(cfg::KEY_UART1_BAUDRATE, 115_200)]);
        assert_eq!(n, 8 + 4 + 4 + 4);
        assert_eq!(&out[10..14], &cfg::KEY_UART1_BAUDRATE.to_le_bytes(), "key");
        assert_eq!(&out[14..18], &115_200u32.to_le_bytes(), "value");
    }

    #[test]
    fn valset_refuses_rather_than_emitting_a_bad_frame() {
        let mut out = [0u8; 64];
        // Unknown width code.
        assert_eq!(cfg::valset(&mut out, cfg::LAYER_RAM, &[(0x0000_0001, 1)]), 0);
        // Output buffer too small for the frame.
        let mut small = [0u8; 8];
        assert_eq!(cfg::valset(&mut small, cfg::LAYER_RAM, &[(cfg::KEY_RATE_MEAS, 100)]), 0);
    }

    #[test]
    fn valset_frame_is_self_consistent_for_any_pair_list() {
        // UbxParser deliberately signals only for NAV-PVT (see
        // `MsgType`), so it cannot be used to validate an outgoing
        // command frame. Check the two fields a receiver actually
        // rejects on instead: the length must match the payload we
        // wrote, and the checksum must cover class..payload inclusive.
        let cases: [&[(u32, u64)]; 3] = [
            &[(cfg::KEY_RATE_MEAS, 100)],
            &[(cfg::KEY_RATE_MEAS, 100), (cfg::KEY_RATE_NAV, 1)],
            &[
                (cfg::KEY_UART1_BAUDRATE, 115_200),
                (cfg::KEY_UART1_OUTPROT_UBX, 1),
                (cfg::KEY_UART1_OUTPROT_NMEA, 0),
            ],
        ];

        for pairs in cases {
            let mut out = [0u8; 64];
            let n = cfg::valset(&mut out, cfg::LAYER_RAM, pairs);
            assert!(n > 0, "frame failed to build");

            assert_eq!(&out[..2], &[SYNC_1, SYNC_2]);
            assert_eq!(out[2], 0x06, "class");
            assert_eq!(out[3], 0x8A, "id");

            let plen = u16::from_le_bytes([out[4], out[5]]) as usize;
            // 4 bytes of version/layers/reserved, then each pair is its
            // 4-byte key plus the width that key declares.
            let want: usize = 4 + pairs
                .iter()
                .map(|&(k, _)| 4 + cfg::key_value_len(k))
                .sum::<usize>();
            assert_eq!(plen, want, "length field disagrees with the payload");
            assert_eq!(n, 6 + plen + 2, "frame length");

            let (ck_a, ck_b) = fletcher16(&out[2..n - 2]);
            assert_eq!((ck_a, ck_b), (out[n - 2], out[n - 1]), "checksum");
        }
    }

    #[test]
    fn layer_is_ram_only() {
        // Flash would persist across boots, which this driver explicitly
        // does not want: it configures every boot instead, so a module is
        // never half-remembering a previous firmware's settings.
        assert_eq!(cfg::LAYER_RAM, 0x01);
        assert_eq!(cfg::LAYER_RAM & 0x04, 0, "flash layer bit must not be set");
    }

    #[test]
    fn cfg_rate_frame_asks_for_ten_hz() {
        // CFG-RATE (0x06 0x08), payload measRate=100 ms, navRate=1,
        // timeRef=1. This frame is now load-bearing: without it NAV-PVT
        // arrives at the module default of 1 Hz and the GPS-derived
        // acceleration estimate is worthless.
        let mut out = [0u8; 16];
        let n = cfg::set_nav_rate(&mut out, NAV_RATE_MS, 1);
        assert_eq!(n, 14, "6-byte payload plus 8 bytes of framing");
        assert_eq!(&out[..2], &[SYNC_1, SYNC_2]);
        assert_eq!(out[2], 0x06, "class");
        assert_eq!(out[3], 0x08, "id");
        assert_eq!(&out[4..6], &[6, 0], "payload length, little-endian");
        assert_eq!(&out[6..8], &[0x64, 0x00], "measRate = 100 ms = 10 Hz");
        assert_eq!(&out[8..10], &[0x01, 0x00], "navRate = 1 solution");
        // Checksum must cover class..payload inclusive.
        let (a, b) = fletcher16(&out[2..12]);
        assert_eq!((out[12], out[13]), (a, b));
    }

    #[test]
    fn nav_rate_fits_the_configured_link() {
        // One NAV-PVT is 92 payload + 8 framing = 100 bytes. At 10 Hz
        // that is 1000 B/s; 115200 8N1 carries 11520 B/s. If someone
        // raises the rate far enough that this fails, the receiver will
        // silently start dropping frames instead.
        let bytes_per_s = 100.0 * (1000.0 / NAV_RATE_MS as f32);
        let link_bytes_per_s = TARGET_BAUD as f32 / 10.0;
        assert!(
            bytes_per_s < link_bytes_per_s * 0.5,
            "{bytes_per_s} B/s needs more than half of {link_bytes_per_s} B/s"
        );
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
