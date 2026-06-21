//! Arm-time re-origin synchronisation.
//!
//! When the vehicle arms, the navigation task signals the PosKF to
//! re-anchor its origins and zero its state. That re-origin happens a few
//! milliseconds later, on the PosKF's own task tick. If the navigation
//! task captures altitude/position *targets* from the estimate inside that
//! window, it captures the stale pre-zero values — and once the KF zeroes,
//! target and estimate disagree, commanding a lurch the instant you arm
//! (the arm-into-altitude-mode launch bug).
//!
//! [`ArmOriginSync`] gates target capture on the PosKF having published an
//! estimate whose re-origin sequence counter has advanced past the value
//! observed at the arm edge — i.e. the zero is confirmed done.

/// Tracks PosKF arm-time re-origin completion for the navigation task.
///
/// The PosKF increments a monotonic `arm_origin_seq` each time it consumes
/// the arm latch (re-anchors + zeros) and publishes it in its estimate.
/// The navigation task feeds that counter here every tick.
#[derive(Debug, Default)]
pub struct ArmOriginSync {
    seq_at_arm: u32,
    armed_prev: bool,
}

impl ArmOriginSync {
    pub const fn new() -> Self {
        Self {
            seq_at_arm: 0,
            armed_prev: false,
        }
    }

    /// Call once per navigation tick.
    ///
    /// * `armed` — current arm state.
    /// * `current_seq` — `arm_origin_seq` from the latest published estimate.
    ///
    /// Returns `true` once the PosKF has re-origined for the current arm
    /// (its sequence counter has advanced past the value seen at the arm
    /// edge). Returns `false` while disarmed, and during the window after
    /// arming but before the re-origin lands.
    pub fn reoriginated(&mut self, armed: bool, current_seq: u32) -> bool {
        if armed && !self.armed_prev {
            // Rising edge of arm: latch the seq present now. The PosKF has
            // not yet consumed this arm's latch, so this is the pre-zero
            // value; re-origin is "done" once the published seq differs.
            self.seq_at_arm = current_seq;
        }
        self.armed_prev = armed;
        armed && current_seq != self.seq_at_arm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn withholds_targets_until_reorigin_lands() {
        let mut s = ArmOriginSync::new();
        // Disarmed: never ready.
        assert!(!s.reoriginated(false, 0));
        // Arm edge — records the current (pre-latch) seq; not ready yet.
        assert!(!s.reoriginated(true, 5));
        // Still waiting for the PosKF to consume the latch.
        assert!(!s.reoriginated(true, 5));
        // PosKF re-origined (seq advanced) → targets safe to capture.
        assert!(s.reoriginated(true, 6));
        // Stays ready for the rest of the arm, including later increments.
        assert!(s.reoriginated(true, 6));
        assert!(s.reoriginated(true, 7));
    }

    #[test]
    fn re_arm_requires_a_fresh_reorigin() {
        let mut s = ArmOriginSync::new();
        assert!(!s.reoriginated(true, 1)); // arm edge, seq_at_arm = 1
        assert!(s.reoriginated(true, 2)); // re-origin done
        // Disarm clears readiness.
        assert!(!s.reoriginated(false, 2));
        // Re-arm: must wait for the next re-origin even though seq is high.
        assert!(!s.reoriginated(true, 2)); // new arm edge, seq_at_arm = 2
        assert!(s.reoriginated(true, 3)); // next re-origin lands
    }
}
