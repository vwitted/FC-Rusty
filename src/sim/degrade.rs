// degrade.rs — sensor degradation and fault injection for the sim.
//
// QuadSim produces truth. This layer sits between truth and the control stack
// and makes the sensors lie, the way real ones do:
//
//   * Gaussian noise, per axis.
//   * Fixed bias — the thing that actually kills an estimator, because
//     unlike noise it does not average out.
//   * Vibration: a sinusoid coupled into gyro/accel. This is the "loose
//     motor" / prop-imbalance case and the interesting one, because a
//     resonance near the loop rate aliases and the filter cannot tell it
//     from real motion.
//   * Intermittent dropout, in runs rather than per-sample coin flips, so it
//     models a flaky bus or marginal joint rather than white noise on the
//     availability signal.
//   * Per-motor thrust scaling, for an asymmetric airframe.
//
// The point is a sweep from idealised to hopelessly degraded, so every knob
// is zero-by-default and `Degradation::none()` is bit-for-bit equivalent to
// running truth straight through. That property is tested — without it the
// zero point of every sweep measures the harness instead of the effect.

use super::sensors::Rng;

/// One sensor channel's degradation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelFault {
    /// Std-dev of additive white noise, in the channel's own units.
    pub sigma: f32,
    /// Constant offset, per axis, in the channel's own units.
    pub bias: [f32; 3],
    /// Vibration amplitude coupled into this channel.
    pub vib_amplitude: f32,
    /// Vibration frequency (Hz). Near the loop rate is the nasty case.
    pub vib_hz: f32,
    /// Probability the channel is online. 1.0 = always, 0.0 = dead.
    pub p_online: f32,
    /// Mean length of an online/offline run, in seconds.
    pub dropout_dwell_s: f32,
}

impl ChannelFault {
    pub const fn none() -> Self {
        Self {
            sigma: 0.0,
            bias: [0.0; 3],
            vib_amplitude: 0.0,
            vib_hz: 0.0,
            p_online: 1.0,
            dropout_dwell_s: 0.1,
        }
    }

    pub fn is_clean(&self) -> bool {
        self.sigma == 0.0
            && self.bias == [0.0; 3]
            && self.vib_amplitude == 0.0
            && self.p_online >= 1.0
    }
}

impl Default for ChannelFault {
    fn default() -> Self {
        Self::none()
    }
}

/// Whole-airframe degradation config.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Degradation {
    pub gyro: ChannelFault,
    pub accel: ChannelFault,
    /// Per-motor thrust scale. 1.0 = nominal, 0.8 = a motor down 20%.
    pub motor_scale: [f32; 4],
}

impl Degradation {
    pub const fn none() -> Self {
        Self {
            gyro: ChannelFault::none(),
            accel: ChannelFault::none(),
            motor_scale: [1.0; 4],
        }
    }

    pub fn is_clean(&self) -> bool {
        self.gyro.is_clean() && self.accel.is_clean() && self.motor_scale == [1.0; 4]
    }
}

impl Default for Degradation {
    fn default() -> Self {
        Self::none()
    }
}

/// Tracks online/offline runs for one channel.
#[derive(Debug, Clone, Copy)]
struct DropoutState {
    online: bool,
    started: bool,
    remaining_s: f32,
}

impl DropoutState {
    const fn new() -> Self {
        Self { online: true, started: false, remaining_s: 0.0 }
    }

    /// Advance; returns whether the channel is readable this step.
    ///
    /// A two-state chain that alternates deterministically, with mean run
    /// lengths of `dwell * p` online and `dwell * (1 - p)` offline. The duty
    /// cycle is then p by construction.
    ///
    /// The obvious alternative — flip a coin each run and scale the length —
    /// does NOT give duty p. It converges on p^2 / (p^2 + (1-p)^2), which is
    /// 0.9 at p = 0.75. Worth stating because the error is invisible at
    /// p = 0.5, where that expression happens to equal 0.5.
    ///
    /// Run lengths are exponentially distributed about their mean, so
    /// dropouts are bursty rather than periodic.
    fn tick(&mut self, f: &ChannelFault, dt: f32, rng: &mut Rng) -> bool {
        if f.p_online >= 1.0 {
            return true;
        }
        if f.p_online <= 0.0 {
            return false;
        }
        self.remaining_s -= dt;
        if self.remaining_s <= 0.0 {
            if self.started {
                self.online = !self.online;
            }
            self.started = true;
            let dwell = f.dropout_dwell_s.max(dt);
            let mean = dwell * if self.online { f.p_online } else { 1.0 - f.p_online };
            // Exponential sample; guard u away from 0 so ln stays finite.
            let u = rng.uniform().clamp(1e-6, 1.0);
            self.remaining_s = (mean * -libm::logf(u)).max(dt);
        }
        self.online
    }
}

/// Applies a `Degradation` to truth. Holds RNG and dropout state, so one
/// instance is one flight.
pub struct Degrader {
    cfg: Degradation,
    rng: Rng,
    t: f32,
    gyro_drop: DropoutState,
    accel_drop: DropoutState,
    last_gyro: [f32; 3],
    last_accel: [f32; 3],
}

impl Degrader {
    pub fn new(cfg: Degradation, seed: u64) -> Self {
        Self {
            cfg,
            rng: Rng::new(seed),
            t: 0.0,
            gyro_drop: DropoutState::new(),
            accel_drop: DropoutState::new(),
            last_gyro: [0.0; 3],
            last_accel: [0.0; 3],
        }
    }

    pub fn config(&self) -> &Degradation {
        &self.cfg
    }

    /// Scale commanded motor thrusts: a tired motor, damaged prop, dragging
    /// bearing — an airframe that is no longer symmetric.
    pub fn motors(&self, cmd: [f32; 4]) -> [f32; 4] {
        let s = self.cfg.motor_scale;
        [cmd[0] * s[0], cmd[1] * s[1], cmd[2] * s[2], cmd[3] * s[3]]
    }

    /// Corrupt one IMU sample. On dropout the last good value is HELD, which
    /// is what a driver that keeps its previous reading actually does — and
    /// is harsher on an estimator than a gap, because the staleness is silent.
    pub fn imu(&mut self, gyro: [f32; 3], accel: [f32; 3], dt: f32) -> ([f32; 3], [f32; 3]) {
        self.t += dt;

        let g_online = self.gyro_drop.tick(&self.cfg.gyro, dt, &mut self.rng);
        let a_online = self.accel_drop.tick(&self.cfg.accel, dt, &mut self.rng);

        let g = if g_online {
            let v = self.corrupt(gyro, self.cfg.gyro);
            self.last_gyro = v;
            v
        } else {
            self.last_gyro
        };
        let a = if a_online {
            let v = self.corrupt(accel, self.cfg.accel);
            self.last_accel = v;
            v
        } else {
            self.last_accel
        };
        (g, a)
    }

    fn corrupt(&mut self, v: [f32; 3], f: ChannelFault) -> [f32; 3] {
        // One physical imbalance is one rotation, so the same phase couples
        // into every axis rather than three independent sources.
        let vib = if f.vib_amplitude != 0.0 {
            f.vib_amplitude * libm::sinf(2.0 * core::f32::consts::PI * f.vib_hz * self.t)
        } else {
            0.0
        };
        let mut out = [0.0f32; 3];
        for i in 0..3 {
            let n = if f.sigma != 0.0 { self.rng.normal() * f.sigma } else { 0.0 };
            out[i] = v[i] + f.bias[i] + n + vib;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: [f32; 3] = [1.0, -2.0, 3.0];
    const A: [f32; 3] = [0.0, 0.0, -9.81];

    #[test]
    fn none_is_a_perfect_passthrough() {
        let mut d = Degrader::new(Degradation::none(), 1);
        for _ in 0..100 {
            let (g, a) = d.imu(G, A, 0.001);
            assert_eq!(g, G);
            assert_eq!(a, A);
        }
        assert_eq!(d.motors([0.1, 0.2, 0.3, 0.4]), [0.1, 0.2, 0.3, 0.4]);
        assert!(Degradation::none().is_clean());
    }

    #[test]
    fn bias_is_constant_and_does_not_average_out() {
        let cfg = Degradation {
            gyro: ChannelFault { bias: [0.5, 0.0, 0.0], ..ChannelFault::none() },
            ..Degradation::none()
        };
        let mut d = Degrader::new(cfg, 1);
        let mut sum = 0.0;
        for _ in 0..1000 {
            sum += d.imu(G, A, 0.001).0[0];
        }
        assert!((sum / 1000.0 - 1.5).abs() < 1e-3);
    }

    #[test]
    fn noise_has_the_requested_spread_and_zero_mean() {
        let cfg = Degradation {
            gyro: ChannelFault { sigma: 2.0, ..ChannelFault::none() },
            ..Degradation::none()
        };
        let mut d = Degrader::new(cfg, 7);
        let n = 20000;
        let (mut sum, mut sq) = (0.0f32, 0.0f32);
        for _ in 0..n {
            let e = d.imu(G, A, 0.001).0[0] - G[0];
            sum += e;
            sq += e * e;
        }
        let mean = sum / n as f32;
        let sd = (sq / n as f32 - mean * mean).sqrt();
        assert!(mean.abs() < 0.1, "mean {} should be ~0", mean);
        assert!((sd - 2.0).abs() < 0.2, "sd {} should be ~2", sd);
    }

    #[test]
    fn vibration_is_periodic_and_bounded_by_amplitude() {
        let cfg = Degradation {
            gyro: ChannelFault { vib_amplitude: 10.0, vib_hz: 100.0, ..ChannelFault::none() },
            ..Degradation::none()
        };
        let mut d = Degrader::new(cfg, 1);
        let (mut lo, mut hi) = (f32::MAX, f32::MIN);
        for _ in 0..2000 {
            let v = d.imu(G, A, 0.0001).0[0] - G[0];
            lo = lo.min(v);
            hi = hi.max(v);
        }
        assert!(hi <= 10.01 && lo >= -10.01, "bounded: {} {}", lo, hi);
        assert!(hi > 9.0 && lo < -9.0, "should reach both extremes");
    }

    #[test]
    fn a_dead_channel_holds_its_last_value() {
        let cfg = Degradation {
            gyro: ChannelFault { p_online: 0.0, ..ChannelFault::none() },
            ..Degradation::none()
        };
        let mut d = Degrader::new(cfg, 1);
        assert_eq!(d.imu(G, A, 0.001).0, [0.0; 3]);
    }

    #[test]
    fn partial_dropout_lands_near_the_requested_duty_cycle() {
        for p in [0.25f32, 0.5, 0.75] {
            let cfg = Degradation {
                gyro: ChannelFault {
                    p_online: p,
                    dropout_dwell_s: 0.01,
                    ..ChannelFault::none()
                },
                ..Degradation::none()
            };
            let mut d = Degrader::new(cfg, 42);
            let n = 20000;
            let mut online = 0;
            for i in 0..n {
                // Truth must MOVE, or a held sample is indistinguishable from
                // a fresh one and the measurement is vacuous.
                let truth = [i as f32, 0.0, 0.0];
                if d.imu(truth, A, 0.001).0[0] == truth[0] {
                    online += 1;
                }
            }
            let duty = online as f32 / n as f32;
            assert!((duty - p).abs() < 0.05, "p={} gave duty {}", p, duty);
        }
    }

    #[test]
    fn motor_scale_models_an_asymmetric_airframe() {
        let cfg = Degradation { motor_scale: [1.0, 1.0, 0.7, 1.0], ..Degradation::none() };
        let d = Degrader::new(cfg, 1);
        assert_eq!(d.motors([1.0; 4]), [1.0, 1.0, 0.7, 1.0]);
        assert!(!cfg.is_clean());
    }
}
