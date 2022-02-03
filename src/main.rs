//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;
pub mod net;

extern crate panic_halt;
pub extern crate stm32h7xx_hal;
use hardware::{adc::Adc, hal};
use log::info;
use net::{
    data_stream::{FrameGenerator, StreamFormat, StreamTarget},
    miniconf::Miniconf,
    telemetry::{Telemetry, TelemetryBuffer},
    NetworkState, NetworkUsers,
};
use serde::Deserialize;
use systick_monotonic::*;

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
pub struct AdcFilterSettings {
    pub odr: u32,
    pub order: u32,
    pub enhfilt: u32,
    pub enhfilten: u32,
}

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Settings {
    led: [bool; 8],
    adcfiltersettings: AdcFilterSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            led: [false; 8],
            adcfiltersettings: AdcFilterSettings {
                odr: 0b10101,   // 10Hz output data rate
                order: 0,       // Sinc5+Sinc1 filter
                enhfilt: 0b110, // 16.67 SPS, 92 dB rejection, 60 ms settling
                enhfilten: 1,   // enable postfilter
            },
        }
    }
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {

    use hardware::setup::ThermostatDevices;

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<10_000>; // ToDo: Is this enough?

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
    }

    #[local]
    struct Local {
        adc: Adc,
    }

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

        let settings = Settings::default();
        ethernet_link::spawn().unwrap();

        thermostat.adc.set_filters(settings.adcfiltersettings);

        let local = Local {
            adc: thermostat.adc,
        };

        let shared = Shared {
            network: network,
            settings: settings,
        };

        info!("init done");

        (shared, local, init::Monotonics(mono))
    }

    #[idle(shared=[network], local=[adc])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    // settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    info!("adc.read_data(): {}", c.local.adc.read_data().0);
                }
            }
        }
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        info!("polling ethernet");
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
