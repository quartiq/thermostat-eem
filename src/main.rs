//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;
pub mod net;

use defmt::{info, Format};
use defmt_rtt as _; // global logger
use panic_probe as _; // gloibal panic handler

use hardware::{
    adc::Adc,
    adc_internal::AdcInternal,
    dac::Dac,
    gpio::{Gpio, Led, PoePower},
    hal,
    pwm::{Limit, Pwm},
    system_timer::SystemTimer,
    OutputChannel,
};
use net::{miniconf::Miniconf, serde::Serialize, NetworkState, NetworkUsers};
use systick_monotonic::*;

#[derive(Copy, Clone, Debug, Miniconf, Format)]
pub struct OutputSettings {
    /// En-/Disables the TEC driver.
    ///
    /// # Value
    /// true to shut the driver down, false to enable the driver.
    pub shutdown: bool,

    /// TEC positive current limit in ampere.
    ///
    /// # Value
    /// 0.0 to 3.0
    pub current_limit_positive: f32,

    /// TEC negative current limit in ampere.
    ///
    /// # Value
    /// -3.0 to 0.0
    pub current_limit_negative: f32,

    /// Maximum absolute (positive and negative) TEC voltage in volt.
    ///
    /// # Value
    /// 0.0 to 5.0
    pub voltage_limit: f32,

    /// TEC current in ampere.
    ///
    /// # Value
    /// -3.0 to a bit less than 3.0
    pub current: f32,
}

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Settings {
    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any positive non-zero value. Will be rounded to milliseconds.
    telemetry_period: f32,

    /// Array of settings for the Thermostat output channels.
    ///
    /// # Path
    /// `output_settings/<n>`
    /// * <n> specifies which channel to configure. <n> := [0, 1, 2, 3]
    ///
    /// # Value
    /// Any positive non-zero value. Will be rounded to milliseconds.
    output_settings: [OutputSettings; 4],

    /// LED0 state.
    ///
    /// # Path
    /// `led`
    ///
    /// # Value
    /// "true" or "false".
    led: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            telemetry_period: 1.0,
            output_settings: [OutputSettings {
                shutdown: true,
                current_limit_negative: -0.5,
                current_limit_positive: 0.5,
                voltage_limit: 0.5,
                current: 0.0,
            }; 4],
            led: false,
        }
    }
}

#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Telemetry {
    p3v3_voltage: f32,
    p5v_voltage: f32,
    p12v_voltage: f32,
    p12v_current: f32,
    output_vref: [f32; 4],
    output_current: [f32; 4],
    output_voltage: [f32; 4],
    poe: PoePower,
    overtemp: bool,
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000>; // 1ms resolution
    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
        telemetry: Telemetry,
        gpio: Gpio,
    }

    #[local]
    struct Local {
        adc: Adc,
        dac: Dac,
        pwm: Pwm,
        adc_internal: AdcInternal,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Initialize monotonic
        let systick = c.core.SYST;
        let mono = Systick::new(systick, 400_000_000);
        let clock = SystemTimer::new(|| monotonics::now().ticks());

        // setup Thermostat hardware
        let thermostat = hardware::setup::setup(c.device, clock);

        let network = NetworkUsers::new(
            thermostat.net.stack,
            thermostat.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            thermostat.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("10.42.0.1")
                .parse()
                .unwrap(),
        );

        ethernet_link::spawn().unwrap();
        settings_update::spawn().unwrap();
        telemetry_task::spawn().unwrap();

        let local = Local {
            adc: thermostat.adc,
            dac: thermostat.dac,
            pwm: thermostat.pwm,
            adc_internal: thermostat.adc_internal,
        };

        let shared = Shared {
            network,
            settings: Settings::default(),
            telemetry: Telemetry::default(),
            gpio: thermostat.gpio,
        };

        (shared, local, init::Monotonics(mono))
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => settings_update::spawn().unwrap(),
                NetworkState::Updated => {}
                NetworkState::NoChange => {}
            }
        }
    }

    #[task(priority = 1, local=[dac, pwm], shared=[settings, network, gpio])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c
            .shared
            .network
            .lock(|network| *network.miniconf.settings());
        c.shared.settings.lock(|current| *current = settings);

        // led is proxy for real settings and telemetry later
        c.shared
            .gpio
            .lock(|gpio| gpio.set_led(Led::Led0, settings.led.into()));

        // update DAC state
        let dac = c.local.dac;
        let pwm = c.local.pwm;
        for (i, s) in settings.output_settings.iter().enumerate() {
            let ch = OutputChannel::try_from(i).unwrap();
            // TODO: implement what happens if user chooses invalid value
            pwm.set_limit(Limit::Voltage(ch), s.voltage_limit).unwrap();
            pwm.set_limit(Limit::PositiveCurrent(ch), s.current_limit_positive)
                .unwrap();
            pwm.set_limit(Limit::NegativeCurrent(ch), s.current_limit_negative)
                .unwrap();
            dac.set_current(ch, s.current).unwrap();
            c.shared
                .gpio
                .lock(|gpio| gpio.set_shutdown(ch, s.shutdown.into()));
            info!("DAC channel no {:?}: {:?}", i, s);
        }
    }

    #[task(priority = 1, local=[adc_internal], shared=[network, settings, telemetry, gpio])]
    fn telemetry_task(mut c: telemetry_task::Context) {
        let mut telemetry: Telemetry = c.shared.telemetry.lock(|telemetry| *telemetry);

        let adc_int = c.local.adc_internal;
        telemetry.p3v3_voltage = adc_int.read_p3v3_voltage();
        telemetry.p5v_voltage = adc_int.read_p5v_voltage();
        telemetry.p12v_voltage = adc_int.read_p12v_voltage();
        telemetry.p12v_current = adc_int.read_p12v_current();
        for i in 0..4 {
            let ch = OutputChannel::try_from(i).unwrap();
            telemetry.output_vref[i] = adc_int.read_output_vref(ch);
            telemetry.output_voltage[i] = adc_int.read_output_voltage(ch);
            telemetry.output_current[i] = adc_int.read_output_current(ch);
        }
        c.shared.gpio.lock(|gpio| {
            telemetry.overtemp = gpio.overtemp();
            telemetry.poe = gpio.poe();
        });

        c.shared
            .network
            .lock(|network| network.telemetry.publish(&telemetry));

        // let stuff = c.local.adc.adcs.0.read_data();
        // info!("stuff: {:?}", stuff);

        // TODO: validate telemetry period.
        let telemetry_period = c.shared.settings.lock(|settings| settings.telemetry_period);
        telemetry_task::spawn_after(((telemetry_period * 1000.0) as u64).millis()).unwrap();
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared
            .network
            .lock(|network| network.processor.handle_link());
        ethernet_link::spawn_after(1.secs()).unwrap();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = EXTI15_10, priority = 2, local=[adc])]
    fn adc(c: adc::Context) {
        let adc = c.local.adc;
        let _ = adc.handle_interrupt();
        // spawn iir (isr_out)
    }

    // #[task(binds = EXTI15_10, priority = 1, local=[adc])]
    // fn adc(c: adc::Context) {
    //     let adc = c.local.adc;
    //     let data = adc.adcs.0.read_data();
    //     info!("data phy 0: {:?}", data);
    //     let data = adc.adcs.1.read_data();
    //     info!("data phy 1: {:?}", data);
    //     let data = adc.adcs.2.read_data();
    //     info!("data phy 2: {:?}", data);
    //     let data = adc.adcs.3.read_data();
    //     info!("data phy 3: {:?}", data);
    //     adc.adcs.0.set_cs(false);
    //     adc.rdyn.clear_interrupt_pending_bit();
    // }
}
