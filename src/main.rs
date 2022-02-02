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
        let mut thermostat = hardware::setup::setup(c.core, c.device);

        let mut network = NetworkUsers::new(
            thermostat.net.stack,
            thermostat.net.phy,
            env!("CARGO_BIN_NAME"),
            thermostat.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("10.34.16.10")
                .parse()
                .unwrap(),
        );

        (Shared {}, Local {}, init::Monotonics())
    }
}
