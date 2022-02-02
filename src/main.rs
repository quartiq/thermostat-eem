//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;
pub mod net;

extern crate panic_halt;
pub extern crate stm32h7xx_hal;
use hardware::hal;
use log::info;
use net::{
    data_stream::{FrameGenerator, StreamFormat, StreamTarget},
    miniconf::Miniconf,
    telemetry::{Telemetry, TelemetryBuffer},
    NetworkState, NetworkUsers,
};
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use systick_monotonic::*;

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Settings {
    led: [bool; 8],
}

impl Default for Settings {
    fn default() -> Self {
        Self { led: [false; 8] }
    }
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<10_000>; // ToDo: Is this enough?
    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // setup Thermostat hardware
        let (mut thermostat, mono) = hardware::setup::setup(c.core, c.device);

        let mut network = NetworkUsers::new(
            thermostat.net.stack,
            thermostat.net.phy,
            env!("CARGO_BIN_NAME"),
            thermostat.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("10.42.0.1")
                .parse()
                .unwrap(),
        );

        ethernet_link::spawn().unwrap();

        info!("init done");

        (Shared { network }, Local {}, init::Monotonics(mono))
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    // settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        // info!("polling ethernet");
        c.shared
            .network
            .lock(|network| network.processor.handle_link());
        ethernet_link::spawn_after(1.secs()).unwrap();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }
}
