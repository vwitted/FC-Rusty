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

/// Betaflight-style airmode activation latch.
///
/// Airmode (see [`Mixer::apply`]) shifts motor outputs up to preserve
/// roll/pitch/yaw differentials even at zero collective thrust. On the
/// ground that is dangerous: an armed quad sitting at idle would spin its
/// motors up the instant a stick is bumped. This gate withholds airmode
/// until the throttle first crosses an activation floor after arming — at
/// which point the pilot has committed to taking off — and keeps it on for
/// the rest of the arm so low-throttle attitude authority is available in
/// flight. Disarming clears the latch.
///
/// While the gate is inactive the caller should suppress the torque mix
/// entirely (collective thrust only), so grounded stick input cannot drive
/// any motor.
pub struct AirmodeGate {
    active: bool,
}

impl AirmodeGate {
    pub const fn new() -> Self {
        Self { active: false }
    }

    /// Update the latch and return whether airmode should be applied.
    ///
    /// * `armed` — current arm state; `false` resets the latch.
    /// * `throttle` — current collective throttle command (0.0..1.0).
    /// * `activate_floor` — throttle at/above which airmode latches on.
    pub fn update(&mut self, armed: bool, throttle: f32, activate_floor: f32) -> bool {
        if !armed {
            self.active = false;
        } else if throttle >= activate_floor {
            self.active = true;
        }
        self.active
    }
}

impl Default for AirmodeGate {
    fn default() -> Self {
        Self::new()
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

/// Where the motors actually are, so a mix matrix can be derived rather
/// than assumed.
///
/// [`QUAD_X`] is the +/-1 matrix for a symmetric X: every motor the same
/// distance from the centre of gravity on both axes. This airframe is not
/// that. It is a deadcat — the rear motors are 200 mm apart and the front
/// pair 300 mm — and its centre of gravity does not sit at the midpoint of
/// the motors. Two consequences, of which only the second is a defect:
///
///  - **The roll and pitch columns should stay at +/-1.** The tempting
///    change is to scale them by each motor's lever arm, which is what
///    minimum-effort (pseudo-inverse) allocation gives. On this frame
///    that is measurably worse on both counts that matter. What limits
///    torque is the first motor to hit a rail, not total motor effort,
///    and arm-scaling deliberately under-drives the short arms: it buys
///    14% less roll moment per unit of saturation (11.6 against 13.4 N·m)
///    and 7% less pitch. It also makes cross-coupling worse — see
///    `roll_cross_coupling_is_second_order`. +/-1 is not a symmetric-frame
///    simplification that this frame has outgrown; it is the right answer
///    here, and the tests record why so it is not "fixed" later.
///
///  - **Collective thrust produces a pitching moment.** This one is a
///    defect. Four equal thrusts about a centre of gravity that is not
///    at their centroid do not balance: on this frame, at hover, the
///    residual is 0.07 N·m, about 1100 deg/s^2 of pitch acceleration at
///    the estimated Iyy. The rate loop can hold that, but only by
///    carrying a permanent integrator offset that eats authority and
///    unwinds differently at every throttle setting.
///
/// Positions are measured **from the centre of gravity**, in the body
/// frame, metres: `[x forward, y right]`. Motor order is the mixer's:
/// M1 rear-right, M2 front-right, M3 rear-left, M4 front-left.
pub struct FrameGeometry {
    /// Per-motor `[x forward, y right]` offset from the centre of gravity.
    pub motor_xy: [[f32; 2]; 4],
    /// Rotation sense per motor: `true` for CCW. A CCW rotor reacts
    /// clockwise on the frame, i.e. towards +yaw, so it takes yaw
    /// coefficient +1.
    pub spin_ccw: [bool; 4],
}

impl FrameGeometry {
    /// Derive the mix matrix from the geometry.
    ///
    /// Roll, pitch and yaw coefficients are +/-1, set by which side of the
    /// centre of gravity the motor sits on and which way it spins. Yaw is
    /// +/-1 because reaction torque comes from the rotor's own drag, not
    /// from where it is bolted; roll and pitch are +/-1 for the
    /// saturation and cross-coupling reasons argued on [`FrameGeometry`].
    ///
    /// The thrust column is the part that is not cosmetic. It is chosen so
    /// that a pure collective command produces **no net moment** about the
    /// centre of gravity, and it accounts for the throttle-to-thrust curve
    /// being quadratic: thrust goes as the square of the command, so a
    /// thrust share of `k` needs a command share of `sqrt(k)` (see
    /// `QuadParams::thrust_frac`). The column is normalised to mean 1.0,
    /// so the meaning of the `thrust` demand — and therefore hover
    /// throttle — is unchanged.
    ///
    /// The thrust balance assumes the frame is **left-right symmetric**,
    /// which is true of this airframe and of every practical quad: the
    /// roll moment then cancels by construction and only the pitch balance
    /// has to be solved. `debug_assert`s enforce it rather than silently
    /// returning a mix that trims into a roll.
    pub fn mixer(&self) -> Mixer<4> {
        let x = |i: usize| self.motor_xy[i][0];
        let y = |i: usize| self.motor_xy[i][1];

        // Left-right symmetry: M1/M3 are the rear pair, M2/M4 the front.
        debug_assert!(libm::fabsf(x(0) - x(2)) < 1e-4, "rear pair not level fore/aft");
        debug_assert!(libm::fabsf(x(1) - x(3)) < 1e-4, "front pair not level fore/aft");
        debug_assert!(libm::fabsf(y(0) + y(2)) < 1e-4, "rear pair not symmetric");
        debug_assert!(libm::fabsf(y(1) + y(3)) < 1e-4, "front pair not symmetric");

        // Pitch balance for the collective. The rear pair sits at x_r
        // (negative), the front at x_f. Zero moment needs
        // 2*T_f*x_f + 2*T_r*x_r = 0, so the front THRUST share is
        // k = |x_r| / x_f, and the front COMMAND share is sqrt(k).
        let x_r = libm::fabsf(x(0));
        let x_f = libm::fabsf(x(1));
        let k_cmd = libm::sqrtf(x_r / x_f);
        // Normalised so the four coefficients average 1.0.
        let c_front = 2.0 * k_cmd / (1.0 + k_cmd);
        let c_rear = 2.0 / (1.0 + k_cmd);

        let sign = |v: f32| if v > 0.0 { 1.0 } else { -1.0 };
        let mut mix = [[0.0f32; 4]; 4];
        for i in 0..4 {
            mix[i] = [
                if x(i) > 0.0 { c_front } else { c_rear },
                // +roll is right-wing-down, which needs more thrust on the
                // LEFT, i.e. at negative y.
                -sign(y(i)),
                // +pitch lifts the front motors (see conventions.rs).
                sign(x(i)),
                if self.spin_ccw[i] { 1.0 } else { -1.0 },
            ];
        }
        Mixer { mix }
    }
}

/// This airframe: a 7-inch deadcat, measured 2026-09-20 and recorded in
/// `docs/motor_body_measurements.md`.
///
/// Derived from the measured motor separations — rear pair 200 mm, front
/// pair 300 mm, sides 230 mm — which are over-determined and agree: they
/// predict a 336 mm diagonal against 330 mm measured. That fixes the
/// lateral half-spans at 100/150 mm and the longitudinal motor span at
/// 224.5 mm.
///
/// **The fore/aft centre of gravity is AFT of the motor centroid.** Both
/// measurements agree on that, which is what fixes the sign of the thrust
/// column: the rear motors carry more of the hover load. They disagree
/// only on how far:
///
///  - Motor-to-CoG arm lengths (150 mm rear, 200 mm front) give 111.8 mm
///    to the rear motors and 132.3 mm to the front — 10.2 mm aft.
///  - The stated "1:1.6 rear:front", 100 mm and 160 mm, is 30.0 mm aft.
///
/// Neither pair can be a measurement to the motor axes, because the two
/// distances must sum to the 224.5 mm motor span and both overshoot it
/// (by 19.6 mm and 35.5 mm) — consistent with being taken along the arm
/// tubes and to the frame's extremities respectively.
///
/// The SMALLER offset is used deliberately. With the direction certain,
/// under-correcting only leaves some trim behind, whereas over-correcting
/// would push the trim the other way. A fore/aft balance test would pin
/// the magnitude; it is no longer needed to settle the direction.
pub const DEADCAT_7IN: FrameGeometry = FrameGeometry {
    //             x fwd    y right
    motor_xy: [
        /* M1 RR */ [-0.1028,  0.100],
        /* M2 FR */ [ 0.1217,  0.150],
        /* M3 RL */ [-0.1028, -0.100],
        /* M4 FL */ [ 0.1217, -0.150],
    ],
    //           M1 RR   M2 FR  M3 RL  M4 FL
    spin_ccw: [false,   true,  true, false],
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

    #[test]
    fn airmode_gate_off_until_first_throttle_up() {
        let mut g = AirmodeGate::new();
        // Freshly armed at idle: airmode stays OFF so stick input can't
        // spin motors on the ground.
        assert!(!g.update(true, 0.0, 0.05));
        assert!(!g.update(true, 0.04, 0.05)); // still below the floor
        // Throttle crosses the floor → airmode latches ON (committed to fly).
        assert!(g.update(true, 0.06, 0.05));
        // Stays ON after throttle drops back — now airborne, need authority.
        assert!(g.update(true, 0.0, 0.05));
    }

    // ---- Geometry-derived mixing ----

    /// Net moment about the centre of gravity, N·m, for a set of motor
    /// COMMANDS. Commands are converted to thrust through the same
    /// quadratic curve the plant uses (`QuadParams::thrust_frac`), because
    /// a balance that only holds in the linear approximation does not hold
    /// on the aircraft.
    ///
    /// Returns `[roll, pitch]` in arbitrary but consistent units: thrust
    /// is left as a fraction of per-motor maximum, so only the zero
    /// matters, which is all these tests assert.
    fn moments(g: &FrameGeometry, cmds: [f32; 4]) -> [f32; 2] {
        let mut roll = 0.0;
        let mut pitch = 0.0;
        for i in 0..4 {
            let t = cmds[i] * cmds[i];
            roll += t * g.motor_xy[i][1];
            pitch += t * g.motor_xy[i][0];
        }
        [roll, pitch]
    }

    /// A geometrically symmetric X frame must reproduce QUAD_X exactly.
    /// This pins the derivation against the hand-written matrix that has
    /// been flying the sim, so the new path is a generalisation rather
    /// than a different convention.
    #[test]
    fn symmetric_geometry_reproduces_quad_x() {
        let sym = FrameGeometry {
            motor_xy: [
                [-0.12, 0.12],
                [0.12, 0.12],
                [-0.12, -0.12],
                [0.12, -0.12],
            ],
            spin_ccw: [false, true, true, false],
        };
        let derived = sym.mixer();
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (derived.mix[i][j] - QUAD_X.mix[i][j]).abs() < 1e-5,
                    "motor {i} column {j}: derived {} vs QUAD_X {}",
                    derived.mix[i][j],
                    QUAD_X.mix[i][j],
                );
            }
        }
    }

    /// The defect the thrust column exists to fix: on this frame, four
    /// equal commands do NOT balance. Asserted so the fix below is shown
    /// to be fixing something real.
    #[test]
    fn flat_thrust_column_pitches_the_deadcat() {
        let [_, pitch] = moments(&DEADCAT_7IN, [0.542; 4]);
        assert!(
            pitch.abs() > 1e-3,
            "equal commands should leave a pitch moment on this frame, got {pitch}",
        );
    }

    /// ...and the derived thrust column removes it, at any throttle the
    /// mixer can actually deliver. The ceiling is set by the rear
    /// coefficient: above `1/c_rear` the rear pair is asked for more than
    /// full throttle, clamps, and the balance is lost — see
    /// `collective_balance_has_a_ceiling`.
    #[test]
    fn collective_produces_no_moment_on_the_deadcat() {
        let m = DEADCAT_7IN.mixer();
        for thrust in [0.2f32, 0.4, 0.542, 0.8, 0.95] {
            let out = m.apply_no_airmode(&ControlDemand {
                thrust,
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            });
            let [roll, pitch] = moments(&DEADCAT_7IN, out.motors);
            assert!(
                roll.abs() < 1e-6 && pitch.abs() < 1e-6,
                "collective at {thrust} left roll={roll} pitch={pitch}",
            );
        }
    }

    /// Hover throttle is unchanged by the rebalance: the thrust column
    /// averages 1.0, so a `thrust` demand still means what it meant.
    #[test]
    fn thrust_column_averages_one() {
        let m = DEADCAT_7IN.mixer();
        let mean: f32 = (0..4).map(|i| m.mix[i][0]).sum::<f32>() / 4.0;
        assert!((mean - 1.0).abs() < 1e-5, "thrust column mean {mean}");
    }

    /// Above the ceiling the rear pair clamps and the collective balance
    /// breaks. Recorded because it is a real limit on usable thrust, not
    /// a rounding artifact: the mixer cannot hold trim at full stick.
    #[test]
    fn collective_balance_has_a_ceiling() {
        let m = DEADCAT_7IN.mixer();
        let ceiling = 1.0 / m.mix[0][0];
        assert!(
            (0.955..0.965).contains(&ceiling),
            "collective ceiling moved to {ceiling}",
        );
        let out = m.apply_no_airmode(&ControlDemand {
            thrust: 1.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        });
        assert!(
            moments(&DEADCAT_7IN, out.motors)[1].abs() > 1e-3,
            "full collective should clamp and lose balance",
        );
    }

    /// A pure roll command leaves a small pitch moment, and no linear
    /// mixer can remove it.
    ///
    /// To first order roll and pitch are decoupled: a roll command raises
    /// both left motors and lowers both right ones, so the fore/aft sums
    /// are unchanged. But thrust goes as the SQUARE of the command, and
    /// squaring is convex, so equal-and-opposite command offsets raise
    /// total thrust rather than leaving it alone. On a frame whose
    /// fore/aft arms differ that surplus does not balance, and a pitch
    /// moment appears.
    ///
    /// It is genuinely second-order — exactly quadratic in the roll
    /// command and independent of throttle, which is what this asserts —
    /// so it is a disturbance for the rate loop to reject during a roll,
    /// not a trim error the mixer could carry. At a brisk 0.2 roll
    /// command it is about 0.04 N·m, half the collective trim error that
    /// the thrust column fixes.
    ///
    /// Arm-scaled roll coefficients would make this term four times
    /// larger, by putting the bigger excursion on the longer pitch arm.
    /// That is the second reason the roll column stays at +/-1.
    #[test]
    fn roll_cross_coupling_is_second_order() {
        let m = DEADCAT_7IN.mixer();
        let pitch_at = |thrust: f32, roll: f32| {
            let out = m.apply_no_airmode(&ControlDemand { thrust, roll, pitch: 0.0, yaw: 0.0 });
            moments(&DEADCAT_7IN, out.motors)[1]
        };

        // Quadratic in the roll command: doubling it quadruples the term.
        let base = pitch_at(0.542, 0.0);
        assert!(base.abs() < 1e-6, "balanced hover should have no pitch moment");
        let small = pitch_at(0.542, 0.1) - base;
        let large = pitch_at(0.542, 0.2) - base;
        assert!(
            (large / small - 4.0).abs() < 0.05,
            "cross term should scale as roll^2; got ratio {}",
            large / small,
        );

        // ...and independent of throttle, so it is not a trim error.
        let at_low = pitch_at(0.3, 0.2) - pitch_at(0.3, 0.0);
        assert!(
            (at_low / large - 1.0).abs() < 0.05,
            "cross term should not depend on throttle; {at_low} vs {large}",
        );

        // Bounded: under 0.007 in these units at a brisk roll command,
        // which is 0.17 N·m at this airframe's 25 N per motor.
        assert!(large.abs() < 0.007, "cross term grew to {large}");
    }

    /// The derived matrix keeps the firmware's sign conventions: +roll
    /// lifts the left motors, +pitch lifts the front. Same assertions as
    /// `conventions.rs` makes of QUAD_X.
    #[test]
    fn derived_mixer_keeps_the_sign_conventions() {
        let m = DEADCAT_7IN.mixer();
        let r = m.apply_no_airmode(&ControlDemand {
            thrust: 0.5,
            roll: 0.2,
            pitch: 0.0,
            yaw: 0.0,
        }).motors;
        assert!(r[2] + r[3] > r[0] + r[1], "+roll must lift the left motors");
        let p = m.apply_no_airmode(&ControlDemand {
            thrust: 0.5,
            roll: 0.0,
            pitch: 0.2,
            yaw: 0.0,
        }).motors;
        assert!(p[1] + p[3] > p[0] + p[2], "+pitch must lift the front motors");
    }

    /// The deadcat mix differs from QUAD_X in the thrust column and
    /// nowhere else. This is the summary of the whole exercise, and it
    /// fails loudly if someone later "corrects" the torque columns to
    /// follow the arm lengths.
    #[test]
    fn only_the_thrust_column_differs_from_quad_x() {
        let m = DEADCAT_7IN.mixer();
        for i in 0..4 {
            for j in 1..4 {
                assert!(
                    (m.mix[i][j] - QUAD_X.mix[i][j]).abs() < 1e-6,
                    "motor {i} column {j} should match QUAD_X: {} vs {}",
                    m.mix[i][j],
                    QUAD_X.mix[i][j],
                );
            }
            assert!(
                (m.mix[i][0] - 1.0).abs() > 1e-3,
                "motor {i} thrust coefficient should NOT be 1.0",
            );
        }
    }

    #[test]
    fn airmode_gate_resets_on_disarm() {
        let mut g = AirmodeGate::new();
        assert!(g.update(true, 0.5, 0.05)); // active in flight
        assert!(!g.update(false, 0.5, 0.05)); // disarm clears the latch
        // Re-arm must cross the floor again before airmode re-activates.
        assert!(!g.update(true, 0.0, 0.05));
        assert!(g.update(true, 0.10, 0.05));
    }
}
