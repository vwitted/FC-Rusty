// lib.rs — Library root for fc-rusty
//
// Exposes control algorithms and simulation modules for use by
// both the firmware binary and desktop simulation examples.
//
// The firmware binary (main.rs) has its own module declarations
// for drivers and embassy-specific code.

// no_std for embedded builds; fall back to std during `cargo test`
// so the test harness and panic handler are available on host.
#![cfg_attr(not(test), no_std)]

// Bench motor-test mode. The firmware `run()` lives in the binary's module
// tree (it needs the embassy DShot driver); here we expose only the pure
// config layer for host tests. `main.rs` declares the module under the
// `motor-test` feature for the firmware build.
#[cfg(test)]
pub mod motor_test;

// Versioned flash config store. `record` is pure (host-tested); the
// firmware flash wrapper lives in main.rs's module tree (needs embassy).
pub mod persist {
    pub mod record;
}

pub mod control {
    pub mod altitude;
    pub mod arm_origin;
    pub mod arming;
    pub mod cal_led;
    pub mod mag_cal;
    pub mod mixer;
    pub mod modes;
    pub mod pid;
    pub mod mpc;
    pub mod position;
}

pub mod drivers {
    pub mod dshot_bb_decode;
    pub mod dshot_bb_frame;
    pub mod dshot_frame;
    pub mod orientation;
    pub mod nmea;
}

pub mod estimation;
pub mod gps_accel;

pub mod attitude_mekf;

pub mod imu_filter;

pub mod conventions;

pub mod sim {
    #[path = "sim.rs"]
    mod sim;
    #[path = "sensors.rs"]
    pub mod sensors;
    #[path = "degrade.rs"]
    pub mod degrade;
    #[path = "dual_imu.rs"]
    pub mod dual_imu;
    #[path = "harness.rs"]
    pub mod harness;
    pub use sim::*;
}
