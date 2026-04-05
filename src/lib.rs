// lib.rs — Library root for fc-rusty
//
// Exposes control algorithms and simulation modules for use by
// both the firmware binary and desktop simulation examples.
//
// The firmware binary (main.rs) has its own module declarations
// for drivers and embassy-specific code.

#![no_std]

pub mod control {
    pub mod altitude;
    pub mod arming;
    pub mod mixer;
    pub mod pid;
    pub mod mpc;
}

pub mod drivers {
    pub mod nmea;
}

pub mod sim {
    #[path = "sim.rs"]
    mod sim;
    pub use sim::*;
}
