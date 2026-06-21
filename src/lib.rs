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

pub mod control {
    pub mod altitude;
    pub mod arm_origin;
    pub mod arming;
    pub mod mixer;
    pub mod pid;
    pub mod mpc;
    pub mod position;
}

pub mod drivers {
    pub mod dshot_frame;
    pub mod dshot_telemetry;
    pub mod nmea;
}

pub mod estimation;

pub mod attitude_mekf;

pub mod imu_filter;

pub mod sim {
    #[path = "sim.rs"]
    mod sim;
    #[path = "sensors.rs"]
    pub mod sensors;
    pub use sim::*;
}
