// dual_imu.rs — the board's TWO gyros, fused the way the firmware fuses them.
//
// The DAKEFPV H743 carries dual ICM-42688P: IMU1 on SPI1, IMU2 on SPI4
// (ROTATION_PITCH_180). `dual_icm_read_task` in main.rs polls both at 8 kHz,
// averages them with `RawImu::averaged`, and runs ONE filter chain on the
// fused result. If one read fails it filters the survivor alone, deliberately,
// so the consumer sees a consistent spectral response across dropouts.
//
// Modelling one gyro instead of two is not a neutral simplification, because
// averaging helps for some faults and not at all for others:
//
//   * White noise is INDEPENDENT between two parts, so averaging buys √2.
//   * Vibration is COMMON-MODE. Both parts are bolted to the same PCB and
//     see the same structural resonance in the same phase; averaging buys
//     exactly nothing. This is the case the harness most wants to get right,
//     because prop imbalance is the realistic failure.
//   * Bias is in between — partly per-part, partly common thermal.
//
// A single-channel model cannot express that split, so it is wrong in
// opposite directions on different axes of the same sweep. Hence this.
//
// What is deliberately NOT modelled yet: the ICM-42688P's own on-chip filter
// and ODR, and quantisation. Those add loop phase lag on top of the
// firmware's 150 Hz software filter, and phase lag is exactly what the
// current sweep finding turns on — see the note at the bottom of this file.

use super::degrade::ChannelFault;
use super::sensors::Rng;

/// Which sensors produced this sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fusion {
    /// Both read OK; output is their mean.
    Both,
    /// One read failed; output is the survivor alone. Same phase, √2 more
    /// noise. This is a DEGRADED state, not a gap -- the rate loop still has
    /// a gyro.
    Single(u8),
    /// Neither read; the last good sample is held. Silent staleness, which
    /// is harsher on an estimator than an explicit gap.
    Held,
}

/// One sensor's faults. Gyro and accel are separate because they are
/// separate signals off the same die: a gyro noise sweep must not quietly
/// inject the same noise into the accel channel, which downstream stands in
/// for attitude-estimate error and reaches the controller by another path.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuFault {
    pub gyro: ChannelFault,
    pub accel: ChannelFault,
}

impl ImuFault {
    pub const fn none() -> Self {
        Self { gyro: ChannelFault::none(), accel: ChannelFault::none() }
    }
    /// Gyro-only fault; accel left clean.
    pub const fn gyro(f: ChannelFault) -> Self {
        Self { gyro: f, accel: ChannelFault::none() }
    }
    pub fn is_clean(&self) -> bool {
        self.gyro.is_clean() && self.accel.is_clean()
    }
}

/// Faults for a two-sensor IMU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DualImuConfig {
    /// Applied to each sensor independently — its own noise draw, its own
    /// bias, its own dropout run. This is the part averaging can reduce.
    pub per_sensor: [ImuFault; 2],
    /// Applied identically to both, same vibration phase. Airframe motion
    /// the two parts genuinely share. Averaging cannot touch this.
    ///
    /// `common.gyro.p_online` is a CORRELATED outage: both sensors drop
    /// together. That is the realistic way to lose both, because main.rs
    /// awaits IMU1 then IMU2 inside one task -- starve that task, or dip the
    /// shared 3V3 rail, and neither read lands. Per-sensor dropout stays
    /// independent (separate buses, separate CS), so the two mechanisms
    /// compose: both-down happens either by coincidence, (1-p)^2, or by a
    /// common outage.
    pub common: ImuFault,
    /// Seconds between the two reads. The firmware awaits IMU1 then IMU2
    /// inside one 125 us tick, so IMU2's sample is roughly one SPI burst
    /// later. Averaging samples taken at different instants is a small
    /// phase error; 0.0 disables it.
    pub skew_s: f32,
}

impl DualImuConfig {
    pub const fn none() -> Self {
        Self {
            per_sensor: [ImuFault::none(), ImuFault::none()],
            common: ImuFault::none(),
            skew_s: 0.0,
        }
    }

    /// Both sensors given the same independent-noise fault. The common
    /// channel stays clean, so this is the case averaging SHOULD help.
    pub const fn independent(f: ChannelFault) -> Self {
        let g = ImuFault::gyro(f);
        Self { per_sensor: [g, g], common: ImuFault::none(), skew_s: 0.0 }
    }

    /// A fault both sensors share — prop imbalance, frame resonance. The
    /// case averaging should NOT help.
    pub const fn common_mode(f: ChannelFault) -> Self {
        Self {
            per_sensor: [ImuFault::none(), ImuFault::none()],
            common: ImuFault::gyro(f),
            skew_s: 0.0,
        }
    }

    pub fn is_clean(&self) -> bool {
        self.per_sensor[0].is_clean() && self.per_sensor[1].is_clean()
            && self.common.is_clean() && self.skew_s == 0.0
    }
}

impl Default for DualImuConfig {
    fn default() -> Self {
        Self::none()
    }
}

/// Two sensors, corrupted independently and fused the way the firmware does.
/// One instance is one flight.
pub struct DualImu {
    cfg: DualImuConfig,
    /// One RNG per sensor, so sensor 1's noise sequence does not shift when
    /// sensor 2's config changes. Without this, comparing "one sensor" to
    /// "two sensors" would also be comparing two different noise draws.
    rng: [Rng; 2],
    t: f32,
    drop: [super::degrade::DropoutState; 2],
    /// Correlated outage taking both sensors down together.
    common_drop: super::degrade::DropoutState,
    common_rng: Rng,
    prev_truth: Option<([f32; 3], [f32; 3])>,
    last_fused: ([f32; 3], [f32; 3]),
}

impl DualImu {
    pub fn new(cfg: DualImuConfig, seed: u64) -> Self {
        Self {
            cfg,
            rng: [Rng::new(seed), Rng::new(seed ^ 0x9E37_79B9_7F4A_7C15)],
            t: 0.0,
            drop: [super::degrade::DropoutState::new(); 2],
            common_drop: super::degrade::DropoutState::new(),
            common_rng: Rng::new(seed ^ 0xD1B5_4A32_D192_ED03),
            prev_truth: None,
            last_fused: ([0.0; 3], [0.0; 3]),
        }
    }

    pub fn config(&self) -> &DualImuConfig {
        &self.cfg
    }

    /// Read both sensors and fuse. Returns the fused sample and which
    /// sensors contributed.
    pub fn read(
        &mut self,
        gyro: [f32; 3],
        accel: [f32; 3],
        dt: f32,
    ) -> (([f32; 3], [f32; 3]), Fusion) {
        self.t += dt;
        let prev = self.prev_truth.unwrap_or((gyro, accel));
        self.prev_truth = Some((gyro, accel));

        // IMU2 samples now; IMU1 sampled `skew_s` earlier. Both are causal:
        // interpolate backwards from the previous truth rather than forwards
        // into a future the sim has not computed yet.
        let truth1 = if self.cfg.skew_s > 0.0 && dt > 0.0 {
            let f = (self.cfg.skew_s / dt).clamp(0.0, 1.0);
            (lerp3(gyro, prev.0, f), lerp3(accel, prev.1, f))
        } else {
            (gyro, accel)
        };
        let truth2 = (gyro, accel);

        // One physical imbalance is one rotation, so common vibration enters
        // both sensors at the SAME phase. Computing it once is what makes it
        // common-mode; drawing it per sensor would quietly turn the airframe's
        // single resonance into two, which averaging would then wrongly halve.
        let common_vib_g = vib_at(&self.cfg.common.gyro, self.t);
        let common_vib_a = vib_at(&self.cfg.common.accel, self.t);

        // A failed SPI read loses that chip's gyro AND accel, so sensor
        // availability is one decision per sensor, keyed off its gyro fault.
        // Tick every chain unconditionally, so a common outage does not
        // desynchronise the per-sensor RNG streams and silently change the
        // noise a run sees.
        let bus_up = self.common_drop.tick(&self.cfg.common.gyro, dt, &mut self.common_rng);
        let on0 = self.drop[0].tick(&self.cfg.per_sensor[0].gyro, dt, &mut self.rng[0]) && bus_up;
        let on1 = self.drop[1].tick(&self.cfg.per_sensor[1].gyro, dt, &mut self.rng[1]) && bus_up;

        let s0 = self.sample(0, truth1, common_vib_g, common_vib_a);
        let s1 = self.sample(1, truth2, common_vib_g, common_vib_a);

        let (fused, how) = match (on0, on1) {
            (true, true) => ((mean3(s0.0, s1.0), mean3(s0.1, s1.1)), Fusion::Both),
            (true, false) => (s0, Fusion::Single(0)),
            (false, true) => (s1, Fusion::Single(1)),
            (false, false) => (self.last_fused, Fusion::Held),
        };
        if how != Fusion::Held {
            self.last_fused = fused;
        }
        (fused, how)
    }

    /// One sensor's view: common fault (shared phase, shared bias) plus its
    /// own independent noise and bias.
    fn sample(
        &mut self,
        i: usize,
        truth: ([f32; 3], [f32; 3]),
        common_vib_g: f32,
        common_vib_a: f32,
    ) -> ([f32; 3], [f32; 3]) {
        let per = self.cfg.per_sensor[i];
        let com = self.cfg.common;
        let per_vib_g = vib_at(&per.gyro, self.t);
        let per_vib_a = vib_at(&per.accel, self.t);
        let mut g = [0.0f32; 3];
        let mut a = [0.0f32; 3];
        for k in 0..3 {
            g[k] = truth.0[k] + com.gyro.bias[k] + per.gyro.bias[k]
                + common_vib_g + per_vib_g
                + draw(&mut self.rng[i], com.gyro.sigma)
                + draw(&mut self.rng[i], per.gyro.sigma);
            a[k] = truth.1[k] + com.accel.bias[k] + per.accel.bias[k]
                + common_vib_a + per_vib_a
                + draw(&mut self.rng[i], com.accel.sigma)
                + draw(&mut self.rng[i], per.accel.sigma);
        }
        (g, a)
    }
}

fn draw(rng: &mut Rng, sigma: f32) -> f32 {
    if sigma != 0.0 { rng.normal() * sigma } else { 0.0 }
}

fn vib_at(f: &ChannelFault, t: f32) -> f32 {
    if f.vib_amplitude != 0.0 {
        f.vib_amplitude * libm::sinf(2.0 * core::f32::consts::PI * f.vib_hz * t)
    } else {
        0.0
    }
}

fn mean3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5]
}

/// `f` = 0 gives `a`, `f` = 1 gives `b`.
fn lerp3(a: [f32; 3], b: [f32; 3], f: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * f,
        a[1] + (b[1] - a[1]) * f,
        a[2] + (b[2] - a[2]) * f,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: [f32; 3] = [1.0, -2.0, 3.0];
    const A: [f32; 3] = [0.0, 0.0, -9.81];
    const DT: f32 = 1.0 / 8000.0;

    fn std_of(v: &[f32]) -> f32 {
        let n = v.len() as f32;
        let m = v.iter().sum::<f32>() / n;
        (v.iter().map(|x| (x - m) * (x - m)).sum::<f32>() / n).sqrt()
    }

    #[test]
    fn none_is_a_perfect_passthrough() {
        let mut d = DualImu::new(DualImuConfig::none(), 1);
        for _ in 0..100 {
            let ((g, a), how) = d.read(G, A, DT);
            assert_eq!(g, G);
            assert_eq!(a, A);
            assert_eq!(how, Fusion::Both);
        }
    }

    /// The load-bearing claim for having two sensors at all.
    #[test]
    fn independent_noise_averages_down_by_root_two() {
        const S: f32 = 4.0;
        let f = ChannelFault { sigma: S, ..ChannelFault::none() };
        let mut d = DualImu::new(DualImuConfig::independent(f), 7);
        let mut fused = Vec::new();
        for _ in 0..40_000 {
            fused.push(d.read(G, A, DT).0 .0[0]);
        }
        let got = std_of(&fused);
        let want = S / 2.0_f32.sqrt();
        assert!(
            (got - want).abs() < want * 0.05,
            "fused sigma {got} should be ~{want} (= {S}/sqrt2)"
        );
    }

    /// And the claim that stops anyone reading the first test as "two
    /// sensors always help".
    #[test]
    fn common_mode_vibration_does_not_average_down() {
        const AMP: f32 = 5.0;
        let f = ChannelFault { vib_amplitude: AMP, vib_hz: 50.0, ..ChannelFault::none() };
        let mut d = DualImu::new(DualImuConfig::common_mode(f), 7);
        let mut peak = 0.0f32;
        for _ in 0..40_000 {
            let v = d.read(G, A, DT).0 .0[0] - G[0];
            peak = peak.max(v.abs());
        }
        assert!(
            (peak - AMP).abs() < AMP * 0.02,
            "common-mode vibration should survive averaging intact: peak {peak}, amp {AMP}"
        );
    }

    /// Same amplitude injected per-sensor instead would halve, which is
    /// precisely the error a single-channel model makes invisible.
    #[test]
    fn per_sensor_vibration_would_average_down_unlike_common_mode() {
        const AMP: f32 = 5.0;
        let f = ChannelFault { vib_amplitude: AMP, vib_hz: 50.0, ..ChannelFault::none() };
        let mut common = DualImu::new(DualImuConfig::common_mode(f), 7);
        let mut split = DualImu::new(
            DualImuConfig {
                per_sensor: [ImuFault::gyro(f), ImuFault::none()],
                ..DualImuConfig::none()
            },
            7,
        );
        let (mut pc, mut ps) = (0.0f32, 0.0f32);
        for _ in 0..40_000 {
            pc = pc.max((common.read(G, A, DT).0 .0[0] - G[0]).abs());
            ps = ps.max((split.read(G, A, DT).0 .0[0] - G[0]).abs());
        }
        assert!(pc > ps * 1.9, "common {pc} should be ~2x one-sensor {ps}");
    }

    /// Losing one IMU is a DEGRADED read, not a gap. The firmware filters the
    /// survivor alone; the rate loop never goes blind. The single-channel
    /// model's dropout knob implies a state the hardware cannot reach.
    #[test]
    fn one_sensor_dead_still_tracks_truth() {
        let dead = ChannelFault { p_online: 0.0, ..ChannelFault::none() };
        let cfg = DualImuConfig {
            per_sensor: [ImuFault::none(), ImuFault::gyro(dead)],
            ..DualImuConfig::none()
        };
        let mut d = DualImu::new(cfg, 3);
        for _ in 0..500 {
            let ((g, _), how) = d.read(G, A, DT);
            assert_eq!(how, Fusion::Single(0));
            assert_eq!(g, G, "survivor should pass truth through untouched");
        }
    }

    #[test]
    fn both_dead_holds_the_last_good_sample() {
        let dead = ChannelFault { p_online: 0.0, ..ChannelFault::none() };
        let mut d = DualImu::new(
            DualImuConfig {
                per_sensor: [ImuFault::gyro(dead), ImuFault::gyro(dead)],
                ..DualImuConfig::none()
            },
            3,
        );
        let ((g, _), how) = d.read(G, A, DT);
        assert_eq!(how, Fusion::Held);
        assert_eq!(g, [0.0; 3], "nothing good yet, so the held value is the initial one");
    }

    /// Read skew makes the fused sample lag truth by skew/2, because one
    /// sensor is current and the other is `skew` old.
    #[test]
    fn read_skew_lags_the_fused_sample_by_half_the_skew() {
        let cfg = DualImuConfig { skew_s: DT * 0.5, ..DualImuConfig::none() };
        let mut d = DualImu::new(cfg, 1);
        // Truth ramps at a known rate so lag shows up as a fixed offset.
        let rate = 1000.0; // units per second
        let mut last = 0.0;
        for k in 0..200 {
            let truth = rate * (k as f32 * DT);
            last = d.read([truth, 0.0, 0.0], A, DT).0 .0[0];
            if k == 199 {
                let expect_lag = rate * (DT * 0.5) * 0.5;
                let got_lag = truth - last;
                assert!(
                    (got_lag - expect_lag).abs() < expect_lag * 0.05,
                    "lag {got_lag} should be ~{expect_lag} (rate * skew / 2)"
                );
            }
        }
        assert!(last > 0.0);
    }

    /// The bug this split fixes: a gyro-only fault must leave the accel
    /// channel untouched. Collapsing them made a gyro noise sweep inject the
    /// same noise into accel, which the harness reads as attitude-estimate
    /// error -- so "dual gyros" came out WORSE than one, for a reason that
    /// had nothing to do with gyros.
    #[test]
    fn a_gyro_fault_does_not_leak_into_the_accel_channel() {
        let f = ChannelFault { sigma: 5.0, bias: [1.0; 3], ..ChannelFault::none() };
        let mut d = DualImu::new(DualImuConfig::independent(f), 11);
        for _ in 0..2_000 {
            let ((g, a), _) = d.read(G, A, DT);
            assert_eq!(a, A, "accel must be untouched by a gyro-only fault");
            assert!(g != G, "gyro should be corrupted");
        }
    }

    /// A common outage takes BOTH sensors down, however healthy each is
    /// individually. This is the shared-task / shared-rail failure, and it
    /// is the realistic way to reach Fusion::Held.
    #[test]
    fn a_common_outage_downs_both_healthy_sensors() {
        let cfg = DualImuConfig {
            common: ImuFault::gyro(ChannelFault { p_online: 0.0, ..ChannelFault::none() }),
            ..DualImuConfig::none()
        };
        let mut d = DualImu::new(cfg, 5);
        for _ in 0..200 {
            assert_eq!(d.read(G, A, DT).1, Fusion::Held);
        }
    }

    /// Independent per-sensor dropout reaches both-down only by coincidence,
    /// at about (1-p)^2 -- rare where a common outage is not.
    #[test]
    fn independent_dropout_reaches_both_down_at_roughly_p_squared() {
        const P: f32 = 0.7;
        let flaky = ChannelFault {
            p_online: P,
            dropout_dwell_s: 0.01,
            ..ChannelFault::none()
        };
        let cfg = DualImuConfig {
            per_sensor: [ImuFault::gyro(flaky), ImuFault::gyro(flaky)],
            ..DualImuConfig::none()
        };
        let mut d = DualImu::new(cfg, 9);
        let (mut held, mut n) = (0usize, 0usize);
        for _ in 0..400_000 {
            if d.read(G, A, DT).1 == Fusion::Held {
                held += 1;
            }
            n += 1;
        }
        let got = held as f32 / n as f32;
        let want = (1.0 - P) * (1.0 - P);
        assert!(
            (got - want).abs() < 0.03,
            "both-down fraction {got} should be ~{want} = (1-p)^2"
        );
    }

    #[test]
    fn clean_config_reports_clean() {
        assert!(DualImuConfig::none().is_clean());
        let noisy = ChannelFault { sigma: 1.0, ..ChannelFault::none() };
        assert!(!DualImuConfig::independent(noisy).is_clean());
        assert!(!DualImuConfig::common_mode(noisy).is_clean());
    }
}
