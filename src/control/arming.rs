// arming.rs — Arming state machine
//
// Safety-critical gatekeeper: motors must not spin until all
// pre-arm checks pass and the pilot explicitly commands arming.
//
// State transitions:
//
//   Disarmed ──(all checks pass + arm switch)──→ Armed
//   Armed ──(disarm switch OR failsafe OR check fail)──→ Disarmed
//
// Pre-arm checks:
//   1. Throttle low (< 5%)    — prevents arming at high throttle
//   2. Attitude level (< 25°) — prevents arming while tilted
//   3. IMU data fresh (< 50ms)— prevents arming with stale sensors
//   4. RC link active (< 500ms)— prevents arming without RC
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
}

impl PreArmChecks {
    /// Returns true if all pre-arm checks pass.
    pub fn all_pass(&self) -> bool {
        self.throttle_low && self.attitude_level && self.imu_fresh && self.rc_link_active
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
    /// Thresholds
    pub throttle_threshold: f32,
    pub max_arm_angle_deg: f32,
    pub max_imu_age_ms: u32,
    pub max_rc_age_ms: u32,
}

impl ArmingStateMachine {
    pub fn new() -> Self {
        Self {
            state: ArmState::Disarmed,
            prev_arm_switch: false,
            throttle_threshold: 0.05,
            max_arm_angle_deg: 25.0,
            max_imu_age_ms: 50,
            max_rc_age_ms: 500,
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
    ) -> ArmState {
        let checks = self.run_checks(throttle, roll_deg, pitch_deg, imu_age_ms, rc_age_ms);

        // Edge detection: arm only on OFF→ON transition
        let arm_edge = arm_switch && !self.prev_arm_switch;
        self.prev_arm_switch = arm_switch;

        match self.state {
            ArmState::Disarmed => {
                if arm_edge && checks.all_pass() {
                    self.state = ArmState::Armed;
                }
            }
            ArmState::Armed => {
                // Disarm on: switch off, failsafe, or critical check failure
                if !arm_switch {
                    self.state = ArmState::Disarmed;
                } else if !checks.rc_link_active || !checks.imu_fresh {
                    // Failsafe: disarm on sensor/RC loss
                    self.state = ArmState::Disarmed;
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
    ) -> PreArmChecks {
        let attitude_mag = libm::sqrtf(roll_deg * roll_deg + pitch_deg * pitch_deg);

        PreArmChecks {
            throttle_low: throttle < self.throttle_threshold,
            attitude_level: attitude_mag < self.max_arm_angle_deg,
            imu_fresh: imu_age_ms < self.max_imu_age_ms,
            rc_link_active: rc_age_ms < self.max_rc_age_ms,
        }
    }

    /// Force disarm (for emergency stop / panic button).
    pub fn force_disarm(&mut self) {
        self.state = ArmState::Disarmed;
    }

    pub fn is_armed(&self) -> bool {
        self.state == ArmState::Armed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn good_conditions() -> (bool, f32, f32, f32, u32, u32) {
        // arm_switch, throttle, roll, pitch, imu_age, rc_age
        (true, 0.0, 0.0, 0.0, 5, 50)
    }

    #[test]
    fn test_starts_disarmed() {
        let sm = ArmingStateMachine::new();
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_arm_on_edge() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc) = good_conditions();

        // First update with switch ON = edge (prev was false)
        sm.update(true, thr, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_no_arm_without_edge() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc) = good_conditions();

        // Set prev_arm_switch to true (no edge)
        sm.prev_arm_switch = true;
        sm.update(true, thr, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_throttle_high() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.5, 0.0, 0.0, 5, 50);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_tilted() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 30.0, 0.0, 5, 50);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_stale_imu() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 100, 50);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_arm_no_rc() {
        let mut sm = ArmingStateMachine::new();
        sm.update(true, 0.0, 0.0, 0.0, 5, 1000);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_disarm_on_switch_off() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Armed);

        sm.update(false, thr, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_failsafe_disarm_on_rc_loss() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, _) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, 50);
        assert_eq!(sm.state, ArmState::Armed);

        // RC loss while armed → disarm
        sm.update(true, thr, roll, pitch, imu, 1000);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_failsafe_disarm_on_imu_loss() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, _, rc) = good_conditions();

        sm.update(true, thr, roll, pitch, 5, rc);
        assert_eq!(sm.state, ArmState::Armed);

        // IMU loss while armed → disarm
        sm.update(true, thr, roll, pitch, 200, rc);
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_no_disarm_on_throttle_high() {
        let mut sm = ArmingStateMachine::new();
        let (_, _, roll, pitch, imu, rc) = good_conditions();

        sm.update(true, 0.0, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Armed);

        // High throttle while armed → stays armed (don't disarm mid-flight!)
        sm.update(true, 0.8, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_no_rearm_after_failsafe() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, _) = good_conditions();

        // Arm
        sm.update(true, thr, roll, pitch, imu, 50);
        assert_eq!(sm.state, ArmState::Armed);

        // RC loss → failsafe disarm
        sm.update(true, thr, roll, pitch, imu, 1000);
        assert_eq!(sm.state, ArmState::Disarmed);

        // Switch is still ON, RC comes back — should NOT re-arm
        // because there's no OFF→ON edge
        sm.update(true, thr, roll, pitch, imu, 50);
        assert_eq!(sm.state, ArmState::Disarmed);

        // Must toggle switch OFF then ON to re-arm
        sm.update(false, thr, roll, pitch, imu, 50);
        sm.update(true, thr, roll, pitch, imu, 50);
        assert_eq!(sm.state, ArmState::Armed);
    }

    #[test]
    fn test_force_disarm() {
        let mut sm = ArmingStateMachine::new();
        let (_, thr, roll, pitch, imu, rc) = good_conditions();

        sm.update(true, thr, roll, pitch, imu, rc);
        assert_eq!(sm.state, ArmState::Armed);

        sm.force_disarm();
        assert_eq!(sm.state, ArmState::Disarmed);
    }

    #[test]
    fn test_checks_report() {
        let sm = ArmingStateMachine::new();
        let checks = sm.run_checks(0.0, 0.0, 0.0, 5, 50);
        assert!(checks.all_pass());

        let checks = sm.run_checks(0.1, 30.0, 0.0, 100, 1000);
        assert!(!checks.all_pass());
        assert!(!checks.throttle_low);
        assert!(!checks.attitude_level);
        assert!(!checks.imu_fresh);
        assert!(!checks.rc_link_active);
    }
}
