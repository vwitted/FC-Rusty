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
    /// Apply the mix to produce motor outputs.
    ///
    /// Outputs are clamped to 0.0..1.0. If any motor saturates,
    /// the others are not adjusted (no airmode-style compensation
    /// yet — that's a future improvement).
    pub fn apply(&self, demand: &ControlDemand) -> MotorOutputs<N> {
        let mut outputs = [0.0f32; N];

        for i in 0..N {
            let m = &self.mix[i];
            let raw = demand.thrust * m[0]
                + demand.roll * m[1]
                + demand.pitch * m[2]
                + demand.yaw * m[3];

            outputs[i] = raw.clamp(0.0, 1.0);
        }

        MotorOutputs { motors: outputs }
    }
}

// ---- Common frame geometries ----

/// Standard quad-X layout (Betaflight motor ordering):
///
/// ```text
///     Front
///   3 (CCW)  1 (CW)
///       \  /
///        \/
///        /\
///       /  \
///   2 (CW)  4 (CCW)
///     Rear
/// ```
///
/// Motor order: [rear-right, front-right, rear-left, front-left]
/// (Betaflight convention)
pub const QUAD_X: Mixer<4> = Mixer {
    //             thrust  roll    pitch   yaw
    mix: [
        /* M1 FR */ [1.0,  -1.0,   1.0,  -1.0],
        /* M2 RL */ [1.0,   1.0,  -1.0,  -1.0],
        /* M3 FL */ [1.0,   1.0,   1.0,   1.0],
        /* M4 RR */ [1.0,  -1.0,  -1.0,   1.0],
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
        // Roll positive: left motors higher, right motors lower
        // M3 (FL) and M2 (RL) should be > M1 (FR) and M4 (RR)
        assert!(out.motors[2] > out.motors[0]); // FL > FR
        assert!(out.motors[1] > out.motors[3]); // RL > RR
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
}
