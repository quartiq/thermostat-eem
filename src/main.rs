//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;

extern crate panic_halt;
pub extern crate stm32h7xx_hal;
use hardware::hal; //, system_timer::SystemTimer};

use log::info;

#[rtic::app(device = hal::stm32, peripherals = true)]
mod app {
    use super::*;

    // #[monotonic(binds = TIM15)]
    // type Monotonic = SystemTimer;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // setup Thermostat hardware
        let mut Thermostat = hardware::setup::setup(c.core, c.device);

        (Shared {}, Local {}, init::Monotonics())
    }
}
