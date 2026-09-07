// gps_adapt.rs — present a UBX NAV-PVT fix in the shape the fusion code
// already consumes.
//
// Why an adapter rather than a rewrite.
//
// `pos_kf_task` reads `nmea::GpsData`: latitude, longitude, altitude_m,
// fix_mode, satellites, hdop, ground_speed_ms, course_deg. That path is
// the one the position KF, the home latch and the COG yaw gate are all
// written against. Swapping the whole task over to `ubx::GpsData` means
// touching every one of those at once, in code that has never flown, for
// a change whose real payoff is upstream — getting 10 Hz fixes at all.
//
// So: convert at the driver boundary, leave the fusion alone. The
// conversion is lossless for every field that path uses, and every unit
// and sign in it is pinned by a test below, because this is exactly the
// kind of function that goes wrong quietly.
//
// The one thing worth doing afterwards is letting the KF take vel_n_ms
// and vel_e_ms directly. NAV-PVT carries true NED Doppler velocity;
// squashing it to speed-and-course here so that `pos_kf_task` can
// immediately rebuild north/east from it is a round trip that only
// exists to avoid changing two places at once. It also means the
// `ground_speed < 0.3 => (0, 0)` clamp in that task still applies, which
// exists because an NMEA receiver's course report is noise at low speed
// — a UBX velocity is not, and would not need it.

use super::nmea;
use super::ubx;

/// Convert a decoded NAV-PVT into the NMEA-shaped record the fusion
/// path consumes.
pub fn ubx_to_nmea(u: &ubx::GpsData) -> nmea::GpsData {
    let mut g = nmea::GpsData::new();

    g.latitude = u.latitude;
    g.longitude = u.longitude;
    g.altitude_m = u.altitude_msl_m;

    // fix_mode gates fusion, so `fix_ok` has to be honoured here rather
    // than left to the consumer: u-blox reports a stale fix_type with
    // fix_ok clear while the solution is invalid, and the NMEA-shaped
    // record has nowhere else to carry that.
    g.fix_mode = if !u.fix_ok {
        nmea::FixMode::NoFix
    } else {
        match u.fix_type {
            ubx::FixType::Fix3D | ubx::FixType::GnssDeadReckoning => nmea::FixMode::Fix3D,
            ubx::FixType::Fix2D => nmea::FixMode::Fix2D,
            // DeadReckoning and TimeOnly are not position fixes for our
            // purposes. Mapping them to 2D would let the home latch
            // capture a position the receiver is only guessing at.
            _ => nmea::FixMode::NoFix,
        }
    };
    g.fix = if g.fix_mode == nmea::FixMode::NoFix {
        nmea::FixQuality::NoFix
    } else {
        nmea::FixQuality::GpsFix
    };
    g.rmc_valid = g.fix_mode != nmea::FixMode::NoFix;
    g.satellites = u.satellites;

    // NAV-PVT carries pDOP, not hDOP. pDOP >= hDOP always (it is the
    // 3D figure and includes the vertical axis), so using it where the
    // consumer wants hDOP is CONSERVATIVE: the gate rejects marginally
    // more fixes and the measurement covariance is scaled marginally
    // larger. That is the safe direction, and it is why this is an
    // acceptable substitution rather than a bug waiting to be found.
    g.hdop = u.pdop;
    g.pdop = u.pdop;
    g.vdop = u.pdop;

    g.ground_speed_ms = u.ground_speed_ms;
    g.ground_speed_kmh = u.ground_speed_ms * 3.6;

    // Course. NAV-PVT gives heading-of-motion directly, but it is
    // derived from the same velNED, so recomputing from vel_n/vel_e
    // keeps speed and course exactly consistent with each other — and
    // the consumer immediately does the inverse, so any mismatch would
    // show up as a velocity that disagrees with itself.
    //
    // NED, so atan2(east, north), and normalised to 0..360 to match what
    // an NMEA receiver reports.
    g.course_deg = heading_deg(u.vel_n_ms, u.vel_e_ms);
    g.course_true_deg = g.course_deg;
    // Magnetic course is not available from NAV-PVT. Left at zero rather
    // than filled with the true course: nothing reads it, and a wrong
    // value is worse than an obviously absent one.

    g.hour = u.hour;
    g.minute = u.minute;
    g.second = u.second;

    // Claim every sentence: one NAV-PVT carries what GGA, RMC, VTG and
    // GSA carry between them, and `updated` is how the consumer knows a
    // record is complete.
    g.updated =
        nmea::UPDATED_GGA | nmea::UPDATED_RMC | nmea::UPDATED_VTG | nmea::UPDATED_GSA;

    g
}

/// Course over ground in degrees, 0..360, from NED velocity.
fn heading_deg(vn: f32, ve: f32) -> f32 {
    // libm unconditionally -- see mag::MagSample::magnitude_ut.
    let a = libm::atan2f(ve, vn);
    let deg = a * (180.0 / core::f32::consts::PI);
    if deg < 0.0 {
        deg + 360.0
    } else {
        deg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix3d() -> ubx::GpsData {
        let mut u = ubx::GpsData::new();
        u.fix_type = ubx::FixType::Fix3D;
        u.fix_ok = true;
        u.satellites = 12;
        u.pdop = 1.4;
        u
    }

    #[test]
    fn heading_follows_the_ned_convention() {
        // North is 0, east 90, south 180, west 270. Getting atan2's
        // argument order backwards mirrors the compass about the
        // north-south axis and swaps east for west -- which is invisible
        // until the aircraft is asked to fly somewhere.
        assert!((heading_deg(1.0, 0.0) - 0.0).abs() < 1e-3);
        assert!((heading_deg(0.0, 1.0) - 90.0).abs() < 1e-3);
        assert!((heading_deg(-1.0, 0.0) - 180.0).abs() < 1e-3);
        assert!((heading_deg(0.0, -1.0) - 270.0).abs() < 1e-3);
    }

    #[test]
    fn speed_and_course_round_trip_back_to_the_original_velocity() {
        // This is the property that matters: pos_kf_task rebuilds
        // north/east from exactly these two fields, so whatever it gets
        // back must be the velocity the receiver reported.
        let mut u = fix3d();
        u.vel_n_ms = 3.0;
        u.vel_e_ms = -4.0;
        u.ground_speed_ms = 5.0;

        let g = ubx_to_nmea(&u);
        let crs = g.course_deg * (core::f32::consts::PI / 180.0);
        let vn = g.ground_speed_ms * crs.cos();
        let ve = g.ground_speed_ms * crs.sin();
        assert!((vn - 3.0).abs() < 1e-3, "vn={vn}");
        assert!((ve + 4.0).abs() < 1e-3, "ve={ve}");
    }

    #[test]
    fn fix_not_ok_is_reported_as_no_fix() {
        // u-blox keeps publishing the last fix_type with fix_ok clear.
        // Passing that through as a 3D fix would let the home position
        // latch onto an invalid solution.
        let mut u = fix3d();
        u.fix_ok = false;
        let g = ubx_to_nmea(&u);
        assert_eq!(g.fix_mode, nmea::FixMode::NoFix);
        assert_eq!(g.fix, nmea::FixQuality::NoFix);
        assert!(!g.rmc_valid);
    }

    #[test]
    fn dead_reckoning_is_not_a_position_fix() {
        let mut u = fix3d();
        u.fix_type = ubx::FixType::DeadReckoning;
        assert_eq!(ubx_to_nmea(&u).fix_mode, nmea::FixMode::NoFix);
        u.fix_type = ubx::FixType::TimeOnly;
        assert_eq!(ubx_to_nmea(&u).fix_mode, nmea::FixMode::NoFix);
    }

    #[test]
    fn fix_types_that_are_positions_map_across() {
        let mut u = fix3d();
        assert_eq!(ubx_to_nmea(&u).fix_mode, nmea::FixMode::Fix3D);
        u.fix_type = ubx::FixType::Fix2D;
        assert_eq!(ubx_to_nmea(&u).fix_mode, nmea::FixMode::Fix2D);
        u.fix_type = ubx::FixType::GnssDeadReckoning;
        assert_eq!(ubx_to_nmea(&u).fix_mode, nmea::FixMode::Fix3D);
    }

    #[test]
    fn position_and_dop_carry_through() {
        let mut u = fix3d();
        u.latitude = 51.5074;
        u.longitude = -0.1278;
        u.altitude_msl_m = 35.0;
        let g = ubx_to_nmea(&u);
        assert!((g.latitude - 51.5074).abs() < 1e-9);
        assert!((g.longitude + 0.1278).abs() < 1e-9);
        assert!((g.altitude_m - 35.0).abs() < 1e-6);
        assert_eq!(g.satellites, 12);
        // pDOP stands in for hDOP; see the note in ubx_to_nmea.
        assert!((g.hdop - 1.4).abs() < 1e-6);
    }

    #[test]
    fn a_complete_nav_pvt_marks_every_sentence_present() {
        let g = ubx_to_nmea(&fix3d());
        for bit in [
            nmea::UPDATED_GGA,
            nmea::UPDATED_RMC,
            nmea::UPDATED_VTG,
            nmea::UPDATED_GSA,
        ] {
            assert_ne!(g.updated & bit, 0);
        }
    }

    #[test]
    fn kmh_matches_ms() {
        let mut u = fix3d();
        u.ground_speed_ms = 10.0;
        assert!((ubx_to_nmea(&u).ground_speed_kmh - 36.0).abs() < 1e-4);
    }
}
