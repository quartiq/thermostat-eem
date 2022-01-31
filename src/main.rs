//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;

extern crate panic_halt;
pub extern crate stm32h7xx_hal;

use hardware::hal;

use log::info;

#[rtic::app(device = hal::stm32, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut Thermostat = hardware::setup::setup(c.core, c.device);

        (Shared {}, Local {}, init::Monotonics())
    }
}
