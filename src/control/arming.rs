// arming.rs — Arming state machine
//
// Safety-critical gatekeeper: motors must not spin until all
// pre-arm checks pass and the pilot explicitly commands arming.
//
// State transitions:
//
//   Disarmed ──(all checks pass + arm switch)──→ Armed
//   Armed ──(disarm switch OR IMU loss)──→ Disarmed
//   Armed ──(RC loss)──→ Armed + failsafe_active
//                        (control loop chooses GpsRescue,
//                         FailsafeLand, or FailsafeBlind based
//                         on what sensors are available)
//
// RC loss never disarms — that would crash the quad. The FSM stays
// Armed and sets `failsafe_active`; the control loop is responsible
// for picking the right descent strategy. The control loop calls
// `force_disarm()` once the descent has reached the ground (low-
// altitude detection or descent timeout).
//
// IMU loss is the only mid-flight auto-disarm: there's no safe
// recovery from losing attitude.
//
// Pre-arm checks:
//   1. Throttle low (< 5%)    — prevents arming at high throttle
//   2. Attitude level (< 25°) — prevents arming while tilted
//   3. IMU data fresh (< 50ms)— prevents arming with stale sensors
//   4. RC link active (< 500ms)— prevents arming without RC
//   5. Altitude reference     — at least one of baro / GPS-home must
//      be live. The pilot decides which (or both); we just refuse to
//      arm with no altitude sensor at all.
//
// The arm switch must transition from OFF→ON (edge-triggered) to
// prevent accidental re-arm after a failsafe disarm.

/// Pre-arm check results (bitflags for efficient status reporting).
#[derive(Debug, Clone, Copy, Default)]
pub struct PreArmChecks {
    pub throttle_low: bool,
    pub attitude_level: bool,
    pub imu_fresh: bool,
    pub rc_link_active: bool,
    /// `true` if at least one altitude sensor is live (baro publishing
    /// fresh samples, OR GPS home already latched). Ungated by
    /// `require_altitude_ref = false`.
    pub altitude_ref_ready: bool,
}

impl PreArmChecks {
    /// Returns true if all pre-arm checks pass.
    pub fn all_pass(&self) -> bool {
        self.throttle_low
            && self.attitude_level
            && self.imu_fresh
            && self.rc_link_active
            && self.altitude_ref_ready
    }
}

/// Arming state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmState {
    Disarmed,
    Armed,
}

/// Arming state machine.
pub struct ArmingStateMachine {
    pub state: ArmState,
    /// Previous arm switch state (for edge detection)
    prev_arm_switch: bool,
    /// True when the RC link is lost while armed. The control loop
    /// reads this to engage GPS rescue. Cleared automatically when
    /// RC recovers, or on disarm.
    pub failsafe_active: bool,
    /// Thresholds
    pub throttle_threshold: f32,
    pub max_arm_angle_deg: f32,
    pub max_imu_age_ms: u32,
    pub max_rc_age_ms: u32,
    /// When false, the altitude-reference gate is bypassed entirely
    /// (bench mode for boards with no baro AND no GPS hardware).
    /// Defaults to `true`; the soft gate is satisfied by either baro
    /// or GPS-home, so genuine bench arming on a normal FC needs no
    /// override.
    pub require_altitude_ref: bool,
}

impl ArmingStateMachine {
    pub fn new() -> Self {
        Self {
            state: ArmState::Disarmed,
            prev_arm_switch: false,
            failsafe_active: false,
            throttle_threshold: 0.05,
            max_arm_angle_deg: 25.0,
            max_imu_age_ms: 50,
            max_rc_age_ms: 500,
            require_altitude_ref: true,
        }
    }

    /// Run one update of the arming state machine.
    ///
    /// # Arguments
    /// * `arm_switch` — true if the arm switch is in the armed position
    /// * `throttle` — normalised throttle (0.0–1.0)
    /// * `roll_deg` — current roll angle in degrees
    /// * `pitch_deg` — current pitch angle in degrees
    /// * `imu_age_ms` — milliseconds since last IMU update
    /// * `rc_age_ms` — milliseconds since last RC packet
    /// * `baro_ready` — true if the baro is publishing fresh samples.
    ///   Combined with `gps_home_latched` to satisfy the altitude
    ///   reference gate.
    /// * `gps_home_latched` — true once the PosKF has captured a good
    ///   GPS fix as its home origin. Used both as one half of the
    ///   altitude-reference gate and (separately) by the RC-loss
    ///   branch to decide between failsafe-rescue and disarm.
    ///
    /// # Returns
    /// The current arming state after this update.
    pub fn update(
        &mut self,
        arm_switch: bool,
        throttle: f32,
        roll_deg: f32,
        pitch_deg: f32,
        imu_age_ms: u32,
        rc_age_ms: u32,
        baro_ready: bool,
        gps_home_latched: bool,
    ) -> ArmState {
        let checks = self.run_checks(
            throttle, roll_deg, pitch_deg, imu_age_ms, rc_age_ms, baro_ready, gps_home_latched,
        );

        // Edge detection: arm only on OFF→ON transition
        let arm_edge = arm_switch && !self.prev_arm_switch;
        self.prev_arm_switch = arm_switch;

        match self.state {
            ArmState::Disarmed => {
                if arm_edge && checks.all_pass() {
                    self.state = ArmState::Armed;
                    self.failsafe_active = false;
                }
            }
            ArmState::Armed => {
                if !arm_switch {
                    // Pilot disarm — always honoured.
                    self.state = ArmState::Disarmed;
                    self.failsafe_active = false;
                } else if !checks.imu_fresh {
                    // IMU loss — no safe recovery, must disarm.
                    self.state = ArmState::Disarmed;
                    self.failsafe_active = false;
                } else if !checks.rc_link_active {
                    // RC loss — always stay armed, signal failsafe.
                    // The control loop picks the descent strategy
                    // (GpsRescue / FailsafeLand / FailsafeBlind)
                    // based on available sensors, and calls
                    // `force_disarm()` once the quad has landed.
                    self.failsafe_active = true;
                    let _ = gps_home_latched; // surfaced via callsite, not used here
                } else {
                    // RC recovered — clear failsafe.
                    self.failsafe_active = false;
                }
                // Note: we do NOT disarm on throttle_high or attitude —
                // that would cause mid-flight disarms.
            }
        }

        self.state
    }

    /// Evaluate pre-arm checks without changing state.
    pub fn run_checks(
        &self,
        throttle: f32,
        roll_deg: f32,
        pitch_deg: f32,
        imu_age_ms: u32,
        rc_age_ms: u32,
        baro_ready: bool,
        gps_home_latched: bool,
    ) -> PreArmChecks {
        let attitude_mag = libm::sqrtf(roll_deg * roll_deg + pitch_deg * pitch_deg);

        PreArmChecks {
            throttle_low: throttle < self.throttle_threshold,
            attitude_level: attitude_mag < self.max_arm_angle_deg,
            imu_fresh: imu_age_ms < self.max_imu_age_ms,
            rc_link_active: rc_age_ms < self.max_rc_age_ms,
            altitude_ref_ready: baro_ready
                || gps_home_latched
                || !self.require_altitude_ref,
        }
    }

    /// Force disarm (for emergency stop / panic button / auto-land complete).
    pub fn force_disarm(&mut self) {
        self.state = ArmState::Disarmed;
        self.failsafe_active = false;
    }

    pub fn is_armed(&self) -> bool {
        self.state == ArmState::Armed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tuple order: arm_switch, throttle, roll, pitch, imu_age, rc_age,
    /// baro_ready, gps_home_latched.
    fn good_conditions() -> (bool, f32, f32, f32, u32, u32, bool, bool) {
        (true, 0.0, 0.0, 0.0, 5, 50, true, true)
    }

    #[test]
    fn test_starts_disarmed() {
        let sm = ArmingStateMachine::new();
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_arm_on_edge() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc, baro, gps) = good_conditions();

        // First update with switch ON = edge (prev was false)
        sm.update(true, thr, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_no_arm_without_edge() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc, baro, gps) = good_conditions();

        // Set prev_arm_switch to true (no edge)
        sm.prev_arm_switch = true;
        sm.update(true, thr, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_throttle_high() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.5, 0.0, 0.0, 5, 50, true, true);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_tilted() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 30.0, 0.0, 5, 50, true, true);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_stale_imu() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 100, 50, true, true);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_no_rc() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 5, 1000, true, true);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_without_any_altitude_ref() {
        // No baro AND no GPS home → soft gate fails.
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 5, 50, false, false);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_arm_with_baro_only() {
        // Pilot's call: baro alive, no GPS yet — soft gate satisfied.
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 5, 50, true, false);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_arm_with_gps_only() {
        // GPS home latched, baro absent — also fine.
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 5, 50, false, true);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_disarm_on_switch_off() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc, baro, gps) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        sm.update(false, thr, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_failsafe_on_rc_loss_with_gps() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, _, baro, gps) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, 50, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        // RC loss while armed with GPS home → stay armed, failsafe active
        sm.update(true, thr, roll, pitch, imu, 1000, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);
        assert!(sm.failsafe_active);
    }

    #[test]
    fn test_rc_loss_without_gps_keeps_armed() {
        // Updated 2026-05-08: previously this disarmed on RC loss
        // without GPS home. New policy is "never auto-disarm on RC
        // loss" — the control loop runs a controlled or blind descent
        // and disarms only when the quad has landed.
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, _, baro, _) = good_conditions();

        // Arm with baro only (no GPS home).
        sm.update(true, thr, roll, pitch, imu, 50, baro, false);
        assert_eq!(sm.state, ArmState::Armed);

        // RC loss without GPS home → stay armed, failsafe active.
        sm.update(true, thr, roll, pitch, imu, 1000, baro, false);
        assert_eq!(sm.state, ArmState::Armed);
        assert!(sm.failsafe_active);
    }

    #[test]
    fn test_rc_loss_with_no_altitude_or_gps_keeps_armed() {
        // Worst case: armed with bench bypass, RC lost, nothing left.
        // Still must not auto-disarm — the blind-descent failsafe
        // controller is the only thing that can stop the quad cleanly.
        let mut sm = ArmingStateMachine::new();
        sm.require_altitude_ref = false;
        let (_, thr, roll, pitch, imu, _, _, _) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, 50, false, false);
        assert_eq!(sm.state, ArmState::Armed);

        sm.update(true, thr, roll, pitch, imu, 1000, false, false);
        assert_eq!(sm.state, ArmState::Armed);
        assert!(sm.failsafe_active);
    }

    #[test]
    fn test_failsafe_clears_on_rc_recovery() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, _, baro, gps) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, 50, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        // RC loss → failsafe
        sm.update(true, thr, roll, pitch, imu, 1000, baro, gps);
        assert!(sm.failsafe_active);

        // RC recovers → failsafe clears, stays armed
        sm.update(true, thr, roll, pitch, imu, 50, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);
        assert!(!sm.failsafe_active);
    }

    #[test]
    fn test_failsafe_disarm_on_imu_loss() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, _, rc, baro, gps) = good_conditions();

        sm.update(true, thr, roll, pitch, 5, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        // IMU loss while armed → disarm
        sm.update(true, thr, roll, pitch, 200, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_disarm_on_throttle_high() {
        let mut sm = ArmingStateMachine::new();
        let (_, _, roll, pitch, imu, rc, baro, gps) = good_conditions();

        sm.update(true, 0.0, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        // High throttle while armed → stays armed (don't disarm mid-flight!)
        sm.update(true, 0.8, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_no_disarm_on_gps_home_loss_midflight() {
        // GPS-home is an arm-time gate only. Mid-flight GPS loss must
        // NOT trigger a disarm — the pilot needs to keep flying
        // (baro/IMU coast, rescue-to-home, etc.).
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc, baro, _) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, rc, baro, true);
        assert_eq!(sm.state, ArmState::Armed);

        // Simulate GPS loss while armed — should stay armed.
        sm.update(true, thr, roll, pitch, imu, rc, baro, false);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_rearm_after_failsafe_recovery() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, _, baro, gps) = good_conditions();

        // Arm
        sm.update(true, thr, roll, pitch, imu, 50, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        // RC loss → failsafe (stays armed with GPS)
        sm.update(true, thr, roll, pitch, imu, 1000, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);
        assert!(sm.failsafe_active);

        // RC recovers → back to normal armed
        sm.update(true, thr, roll, pitch, imu, 50, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);
        assert!(!sm.failsafe_active);

        // Pilot can now disarm normally
        sm.update(false, thr, roll, pitch, imu, 50, baro, gps);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_force_disarm() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc, baro, gps) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, rc, baro, gps);
        assert_eq!(sm.state, ArmState::Armed);

        sm.force_disarm();
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_checks_report() {
        let sm = ArmingStateMachine::new();
        let checks = sm.run_checks(0.0, 0.0, 0.0, 5, 50, true, true);
        assert!(checks.all_pass());

        let checks = sm.run_checks(0.1, 30.0, 0.0, 100, 1000, false, false);
        assert!(!checks.all_pass());
        assert!(!checks.throttle_low);
        assert!(!checks.attitude_level);
        assert!(!checks.imu_fresh);
        assert!(!checks.rc_link_active);
        assert!(!checks.altitude_ref_ready);
    }

    #[test]
    fn test_bench_bypass_allows_no_altitude_ref() {
        let mut sm = ArmingStateMachine::new();
        sm.require_altitude_ref = false;
        sm.update(true, 0.0, 0.0, 0.0, 5, 50, false, false);
        assert_eq!(sm.state, ArmState::Armed);
    }
}
