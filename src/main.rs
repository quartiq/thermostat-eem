//! # Thermostat_EEM Firmware
//!
//! Firmware repository for "Thermostat EEM", a multichannel temperature controller.
//!
//! This software is currently just the groundwork for the future developments.

#![no_std]
#![no_main]

pub mod hardware;

extern crate panic_halt;
pub extern crate stm32h7xx_hal;

use hardware::hal;
use rtt_logger::RTTLogger;

use log::info;

#[rtic::app(device = hal::stm32, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_c: init::Context) -> (Shared, Local, init::Monotonics) {
        static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Trace);
        rtt_target::rtt_init_print!();
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();
        info!("---Hello World");
        (Shared {}, Local {}, init::Monotonics())
    }
}
