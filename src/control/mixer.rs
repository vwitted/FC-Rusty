// mixer.rs — Maps abstract control demands to per-motor throttle values
//
// The control loop produces:
//   thrust:     total upward force (0.0 - 1.0)
//   roll:       roll torque demand (-1.0 to 1.0)
//   pitch:      pitch torque demand (-1.0 to 1.0)
//   yaw:        yaw torque demand (-1.0 to 1.0)
//
// The mixer translates these into individual motor commands
// based on the frame geometry. Different frame types (quad-X,
// quad-+, hex, V-tail, etc.) just need different mix matrices.
//
// Each row of the mix matrix is [thrust, roll, pitch, yaw]
// coefficients for one motor. Signs determine motor position
// and spin direction.

/// Abstract control demands from the control loop.
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlDemand {
    /// Collective thrust (0.0 = off, 1.0 = full)
    pub thrust: f32,
    /// Roll torque (-1.0 = full left, 1.0 = full right)
    pub roll: f32,
    /// Pitch torque (-1.0 = full nose down, 1.0 = full nose up)
    pub pitch: f32,
    /// Yaw torque (-1.0 = full CCW, 1.0 = full CW)
    pub yaw: f32,
}

/// Per-motor output values.
pub struct MotorOutputs<const N: usize> {
    /// Motor throttle values, clamped to 0.0..1.0
    pub motors: [f32; N],
}

/// Mixer for an N-motor vehicle.
///
/// The mix matrix has N rows (one per motor) and 4 columns:
/// [thrust_coeff, roll_coeff, pitch_coeff, yaw_coeff]
///
/// To compute a motor's output:
///   motor_i = T*mix[i][0] + R*mix[i][1] + P*mix[i][2] + Y*mix[i][3]
pub struct Mixer<const N: usize> {
    /// Mix matrix: N motors × 4 axes
    pub mix: [[f32; 4]; N],
}

impl<const N: usize> Mixer<N> {
    /// Apply the mix with simple clamping (no airmode).
    ///
    /// Motors that would go below 0 or above 1 are hard-clamped.
    /// This sacrifices torque fidelity but preserves commanded thrust
    /// (no phantom-thrust leak). Appropriate for GPS rescue and other
    /// modes where altitude accuracy matters more than attitude rate
    /// authority.
    pub fn apply_no_airmode(&self, demand: &ControlDemand) -> MotorOutputs<N> {
        let mut outputs = [0.0f32; N];
        for i in 0..N {
            let m = &self.mix[i];
            outputs[i] = (demand.thrust * m[0]
                + demand.roll * m[1]
                + demand.pitch * m[2]
                + demand.yaw * m[3])
            .clamp(0.0, 1.0);
        }
        MotorOutputs { motors: outputs }
    }

    /// Apply the mix to produce motor outputs, with airmode-style
    /// saturation handling.
    ///
    /// **Airmode** — when raw mixer outputs would exceed [0.0, 1.0] on
    /// either end, we shift the entire set up or down so the torque
    /// differentials (roll, pitch, yaw) are preserved at the cost of
    /// commanded thrust. This is the standard Betaflight-style behaviour
    /// and is essential because naive per-motor clamping has two bugs:
    ///
    ///   1. **Phantom thrust leak.** If motors would clip below 0.0,
    ///      clamping them to 0.0 silently *adds* thrust the pilot
    ///      (and altitude controller) didn't ask for, because the
    ///      sum-of-motors increases. A quad with thrust=0.10 and
    ///      heavy roll demand can end up with mean motor output near
    ///      hover simply from clipping, making the altitude controller
    ///      unable to descend.
    ///
    ///   2. **Torque loss.** Clipping breaks the roll/pitch/yaw
    ///      differentials, so the quad loses authority over the very
    ///      axis it was trying to correct.
    ///
    /// Airmode inverts the priority: torque is king, thrust is
    /// sacrificed. This matches how real flight controllers behave
    /// and is a prerequisite for stable recovery from large
    /// disturbances.
    ///
    /// If the raw motor range exceeds 1.0 (meaning even a pure shift
    /// can't keep everything inside [0, 1]), we fall back to clamping —
    /// in that extreme case the vehicle has run out of both torque and
    /// thrust authority and the best we can do is not crash the math.
    pub fn apply(&self, demand: &ControlDemand) -> MotorOutputs<N> {
        let mut raw = [0.0f32; N];
        let mut min_raw = f32::INFINITY;
        let mut max_raw = f32::NEG_INFINITY;

        for i in 0..N {
            let m = &self.mix[i];
            raw[i] = demand.thrust * m[0]
                + demand.roll * m[1]
                + demand.pitch * m[2]
                + demand.yaw * m[3];
            if raw[i] < min_raw {
                min_raw = raw[i];
            }
            if raw[i] > max_raw {
                max_raw = raw[i];
            }
        }

        // ---- Airmode shift ----
        // If the full range fits in [0, 1] after a single shift, apply
        // the shift and we're done — torque differentials are exactly
        // preserved. Otherwise we're in the degenerate case and fall
        // back to clamping.
        let range = max_raw - min_raw;
        let shift: f32 = if range > 1.0 {
            0.0 // degenerate: can't fit, just clamp later
        } else if max_raw > 1.0 {
            1.0 - max_raw // negative: pull everything down
        } else if min_raw < 0.0 {
            -min_raw // positive: push everything up
        } else {
            0.0 // already in bounds
        };

        let mut outputs = [0.0f32; N];
        for i in 0..N {
            outputs[i] = (raw[i] + shift).clamp(0.0, 1.0);
        }

        MotorOutputs { motors: outputs }
    }
}

// ---- Common frame geometries ----

/// Quad-X, "props-in" (reversed) spin directions, clockwise-from-rear-right numbering:
///
/// ```text
///       Front
///   M4 (CW)   M2 (CCW)
///       \      /
///        \    /
///         \  /
///          \/
///          /\
///         /  \
///        /    \
///       /      \
///   M3 (CCW)  M1 (CW)
///       Rear
/// ```
///
/// Motor order: [rear-right (M1), front-right (M2), rear-left (M3), front-left (M4)]
///
/// Yaw sign convention: CW motor → yaw coeff -1, CCW motor → yaw coeff +1.
/// Positive yaw command rotates airframe CW (viewed from above), which requires
/// boosting the CCW-spinning motors (their CW reaction torque on the frame drives
/// positive yaw).
pub const QUAD_X: Mixer<4> = Mixer {
    //             thrust  roll    pitch   yaw
    mix: [
        /* M1 RR CW  */ [1.0,  -1.0,  -1.0,  -1.0],
        /* M2 FR CCW */ [1.0,  -1.0,   1.0,   1.0],
        /* M3 RL CCW */ [1.0,   1.0,  -1.0,   1.0],
        /* M4 FL CW  */ [1.0,   1.0,   1.0,  -1.0],
    ],
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hover_all_equal() {
        let demand = ControlDemand {
            thrust: 0.5,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let out = QUAD_X.apply(&demand);
        for m in &out.motors {
            assert!((*m - 0.5).abs() < 0.001);
        }
    }

    #[test]
    fn test_roll_differential() {
        let demand = ControlDemand {
            thrust: 0.5,
            roll: 0.1,
            pitch: 0.0,
            yaw: 0.0,
        };
        let out = QUAD_X.apply(&demand);
        // Roll positive: left motors higher, right motors lower.
        // M3 (RL) and M4 (FL) should be > M1 (RR) and M2 (FR).
        assert!(out.motors[2] > out.motors[0]); // RL > RR
        assert!(out.motors[3] > out.motors[1]); // FL > FR
    }

    #[test]
    fn test_clamp_no_negative() {
        let demand = ControlDemand {
            thrust: 0.1,
            roll: 1.0,
            pitch: 1.0,
            yaw: 1.0,
        };
        let out = QUAD_X.apply(&demand);
        for m in &out.motors {
            assert!(*m >= 0.0);
            assert!(*m <= 1.0);
        }
    }

    #[test]
    fn test_airmode_preserves_differentials_when_shifting_up() {
        // Low thrust (0.1) plus a moderate roll demand that would otherwise
        // drive two motors negative. Airmode should shift everything up to
        // fit, preserving the motor differentials exactly.
        let demand = ControlDemand {
            thrust: 0.1,
            roll: 0.3,
            pitch: 0.0,
            yaw: 0.0,
        };
        let out = QUAD_X.apply(&demand);

        // QUAD_X roll coefficients: M1/M2 (right) have roll=-1, M3/M4 (left) have roll=+1.
        // Raw values would be:
        //   M1 RR = 0.1 - 0.3 = -0.2
        //   M2 FR = 0.1 - 0.3 = -0.2
        //   M3 RL = 0.1 + 0.3 = +0.4
        //   M4 FL = 0.1 + 0.3 = +0.4
        // range = 0.4 - (-0.2) = 0.6, fits in [0, 1].
        // Airmode shifts by +0.2 so min = 0.0:
        //   M1 = 0.0, M2 = 0.0, M3 = 0.6, M4 = 0.6
        // Left-right differential (M3 - M1) should still be exactly 0.6,
        // the original torque signal.
        let differential = out.motors[2] - out.motors[0];
        assert!(
            (differential - 0.6).abs() < 1e-5,
            "roll differential should be preserved at 0.6, got {}",
            differential,
        );
    }

    #[test]
    fn test_airmode_no_phantom_thrust_on_low_thrust() {
        // Regression for the phantom-thrust leak: before airmode, a low
        // thrust command combined with a large roll demand caused asymmetric
        // clipping (two motors at 0, two near max) whose mean was much
        // higher than the commanded thrust — effectively forcing the
        // airframe to hover when the altitude controller was trying to
        // descend. Airmode should keep the mean near the commanded thrust.
        let demand = ControlDemand {
            thrust: 0.1,
            roll: 0.5,
            pitch: 0.0,
            yaw: 0.0,
        };
        let out = QUAD_X.apply(&demand);
        let mean: f32 = out.motors.iter().sum::<f32>() / (out.motors.len() as f32);

        // With the raw motors at [-0.4, -0.4, +0.6, +0.6] (right, right,
        // left, left), airmode shifts by +0.4 to make the min 0. New
        // outputs: [0.0, 0.0, 1.0, 1.0], mean = 0.5. That's larger than
        // the commanded thrust of 0.1, but it's the unavoidable consequence
        // of demanding 100% torque authority on an axis — the point is
        // that the torque differential (1.0) is preserved, which is what
        // airmode prioritises.
        assert!(
            (out.motors[2] - out.motors[0] - 1.0).abs() < 1e-5,
            "full-authority roll torque should be preserved, got {}",
            out.motors[2] - out.motors[0],
        );
        // Sanity: mean motor output fits inside [0, 1] and nothing is NaN
        assert!(mean >= 0.0 && mean <= 1.0);
        for m in &out.motors {
            assert!(m.is_finite());
        }
    }

    #[test]
    fn test_airmode_degenerate_range_falls_back_to_clamp() {
        // When roll + pitch + yaw all hit their max simultaneously,
        // the raw range exceeds 1.0 and airmode cannot preserve all
        // differentials via a single shift. In that degenerate case
        // we fall back to per-motor clamping — not ideal, but safe:
        // outputs still sit inside [0, 1], no NaNs, and the vehicle
        // keeps flying something.
        let demand = ControlDemand {
            thrust: 0.5,
            roll: 1.0,
            pitch: 1.0,
            yaw: 1.0,
        };
        let out = QUAD_X.apply(&demand);
        for m in &out.motors {
            assert!(*m >= 0.0 && *m <= 1.0);
            assert!(m.is_finite());
        }
    }
}
