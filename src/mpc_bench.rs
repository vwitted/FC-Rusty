//! MPC solve-time bench.
//!
//! Times `AttitudeMpc` at a menu of horizons on the target, so the choice
//! of horizon and MPC rate can be cut to what the board can compute (see
//! docs/2026-09-12-sim-direction.md). The firmware `run()` is compiled only
//! under the `mpc-bench` feature; the scenario set and statistics below are
//! pure and host-tested.
//!
//! Output, one line per horizon:
//!
//!   MPC_BENCH,ph,ch,construct_us,solve_min_us,solve_mean_us,solve_max_us,max_iters,unconverged,solves
//!
//! Solve times are measured with the flight entry's core configuration
//! (480 MHz, D-cache off) and nothing else running. Flight adds interrupt
//! load from the 8 kHz loop, so treat them as lower bounds and keep
//! headroom.

/// Attitude and rate states each horizon is timed from, as ([roll, pitch,
/// yaw] rad, [p, q, r] rad/s). Spans level flight to a 120 degree upset, so
/// the table covers the slow cases as well as the typical one.
pub fn scenarios() -> [([f32; 3], [f32; 3]); 7] {
    let d = core::f32::consts::PI / 180.0;
    [
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ([1.0 * d, -1.0 * d, 0.0], [0.0, 0.0, 0.0]),
        ([5.0 * d, 5.0 * d, 10.0 * d], [0.0, 0.0, 0.0]),
        ([20.0 * d, -20.0 * d, 0.0], [30.0 * d, 0.0, 0.0]),
        ([45.0 * d, 0.0, 90.0 * d], [0.0, 60.0 * d, 0.0]),
        ([0.0, 0.0, 0.0], [200.0 * d, -200.0 * d, 100.0 * d]),
        ([120.0 * d, 30.0 * d, 180.0 * d], [0.0, 0.0, 0.0]),
    ]
}

/// Consecutive solves timed from each scenario, warm-starting as in flight.
/// The first solve after a reset is the cold case and is usually the slowest.
pub const SOLVES_PER_SCENARIO: u32 = 50;

/// Running cycle-count statistics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CycleStats {
    pub n: u32,
    pub min: u32,
    pub max: u32,
    pub sum: u64,
}

impl CycleStats {
    pub const fn new() -> Self {
        Self { n: 0, min: u32::MAX, max: 0, sum: 0 }
    }

    pub fn add(&mut self, cycles: u32) {
        self.n += 1;
        self.min = self.min.min(cycles);
        self.max = self.max.max(cycles);
        self.sum += cycles as u64;
    }

    pub fn mean(&self) -> u32 {
        if self.n == 0 {
            0
        } else {
            (self.sum / self.n as u64) as u32
        }
    }
}

/// Cycles to microseconds at `core_hz`.
pub fn cycles_to_us(cycles: u32, core_hz: f32) -> f32 {
    cycles as f32 * 1.0e6 / core_hz
}

#[cfg(feature = "mpc-bench")]
pub use hw::run;

#[cfg(feature = "mpc-bench")]
mod hw {
    use super::*;
    use crate::control::mpc::{AttitudeMpc, MpcModel, MPC_PERIOD_US};
    use cortex_m::peripheral::DWT;
    use embassy_time::{Duration, Timer};

    /// One horizon's results.
    #[derive(Clone, Copy)]
    struct Report {
        ph: usize,
        ch: usize,
        construct: u32,
        solve: CycleStats,
        max_iters: usize,
        unconverged: u32,
    }

    /// Build and time one horizon. Synchronous on purpose: the MPC lives on
    /// the stack for the duration of this call rather than in the async
    /// task's future, and no await can interleave with a measurement.
    fn measure<const PH: usize, const CH: usize>() -> Report {
        let t0 = DWT::cycle_count();
        let mut mpc = AttitudeMpc::<PH, CH>::with_model(MpcModel::FIRMWARE);
        let construct = DWT::cycle_count().wrapping_sub(t0);

        let mut solve = CycleStats::new();
        let mut max_iters = 0usize;
        let mut unconverged = 0u32;
        for (angles, rates) in scenarios() {
            mpc.reset();
            mpc.set_reference([0.0; 3], [0.0; 3]);
            for _ in 0..SOLVES_PER_SCENARIO {
                let t = DWT::cycle_count();
                let out = mpc.solve(angles, rates);
                solve.add(DWT::cycle_count().wrapping_sub(t));
                max_iters = max_iters.max(out.iterations);
                if !out.converged {
                    unconverged += 1;
                }
            }
        }
        Report { ph: PH, ch: CH, construct, solve, max_iters, unconverged }
    }

    fn report(r: Report, core_hz: f32) {
        let us = |c: u32| cycles_to_us(c, core_hz);
        defmt::info!(
            "MPC_BENCH,{=usize},{=usize},{=f32},{=f32},{=f32},{=f32},{=usize},{=u32},{=u32}",
            r.ph,
            r.ch,
            us(r.construct),
            us(r.solve.min),
            us(r.solve.mean()),
            us(r.solve.max),
            r.max_iters,
            r.unconverged,
            r.solve.n,
        );
    }

    /// Time each listed prediction horizon, with the control horizon one
    /// shorter. Each value is a separate instantiation of the solver.
    macro_rules! bench {
        ($core_hz:expr; $($ph:literal),+ $(,)?) => {
            $( report(measure::<$ph, { $ph - 1 }>(), $core_hz); )+
        };
    }

    /// Bench entry. Never returns.
    pub async fn run(core_hz: f32) -> ! {
        defmt::info!("mpc-bench build: [{=str}]", env!("FC_BUILD_STAMP"));
        defmt::warn!("MPC BENCH: no motors are driven. Timing the solver at each horizon.");
        defmt::info!(
            "MPC_BENCH_HEADER,ph,ch,construct_us,solve_min_us,solve_mean_us,solve_max_us,max_iters,unconverged,solves"
        );
        bench!(core_hz; 4, 6, 8, 10, 12, 15, 20, 25, 30);
        defmt::info!(
            "MPC_BENCH_END: MPC period {=u64} us. A horizon is usable only if solve_max_us fits inside the period with headroom for the rest of the navigation task.",
            MPC_PERIOD_US,
        );
        loop {
            Timer::after(Duration::from_secs(3600)).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stats_track_min_max_and_mean() {
        let mut s = CycleStats::new();
        for c in [300u32, 100, 200] {
            s.add(c);
        }
        assert_eq!((s.n, s.min, s.max, s.mean()), (3, 100, 300, 200));
    }

    #[test]
    fn empty_stats_report_a_zero_mean() {
        assert_eq!(CycleStats::new().mean(), 0);
    }

    #[test]
    fn cycles_convert_at_the_core_clock() {
        // 480 cycles at 480 MHz is one microsecond.
        assert!((cycles_to_us(480, 480_000_000.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn scenarios_span_level_flight_to_a_large_upset() {
        let s = scenarios();
        assert!(s.iter().all(|(a, r)| a.iter().chain(r.iter()).all(|v| v.is_finite())));
        assert_eq!(s[0], ([0.0; 3], [0.0; 3]));
        let max_angle = s.iter().flat_map(|(a, _)| a.iter()).fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(max_angle > 100.0_f32.to_radians());
    }
}
