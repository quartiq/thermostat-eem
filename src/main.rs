//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;
pub mod net;
pub mod output_channel;

use defmt_rtt as _; // global logger
use panic_probe as _; // global panic handler

use defmt::info;
use enum_iterator::IntoEnumIterator;
use hardware::{
    adc::{Adc, AdcCode, InputChannel, StateMachine},
    adc_internal::AdcInternal,
    dac::{Dac, DacCode},
    gpio::{Gpio, Led, PoePower},
    hal,
    pwm::{Limit, Pwm},
    system_timer::SystemTimer,
    OutputChannelIdx,
};
use idsp::iir;
use net::{miniconf::Miniconf, serde::Serialize, NetworkState, NetworkUsers};
use systick_monotonic::*;

#[derive(Copy, Clone, Debug, Miniconf)]
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

    /// Output channel settings. Each output channel has one associated datapath
    /// consisting of input weights to route and weigh all 8 input temperatures
    /// into an IIR.
    ///
    /// # Value
    /// See [output_channel::OutputChannel]
    pub output_channel: output_channel::OutputChannel,
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
                // TODO sensible defaults.
                output_channel: output_channel::OutputChannel::new(
                    0.1,
                    -100.,
                    100.,
                    [0., 1., 0., 0., 0., 0., 0., 0.],
                ),
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
    channel_temperatures: [f32; 8],
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
        channel_temperatures: [f64; 8], // input channel temperature in °C
        dac: Dac,
    }

    #[local]
    struct Local {
        adc_sm: StateMachine<Adc>,
        pwm: Pwm,
        adc_internal: AdcInternal,
        iir_state: [iir::Vec5<f64>; 4],
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Initialize monotonic
        let systick = c.core.SYST;
        let clock = SystemTimer::new(|| monotonics::now().ticks());

        // setup Thermostat hardware
        let thermostat = hardware::setup::setup(c.device, clock);

        let mono = Systick::new(systick, thermostat.clocks.sysclk().to_Hz());

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

        let settings = Settings::default();

        ethernet_link::spawn().unwrap();
        settings_update::spawn(settings).unwrap();
        telemetry_task::spawn().unwrap();

        let local = Local {
            adc_sm: thermostat.adc_sm,
            pwm: thermostat.pwm,
            adc_internal: thermostat.adc_internal,
            iir_state: [[0.; 5]; 4],
        };

        let shared = Shared {
            dac: thermostat.dac,
            network,
            settings,
            telemetry: Telemetry::default(),
            gpio: thermostat.gpio,
            channel_temperatures: [0.0; 8],
        };

        (shared, local, init::Monotonics(mono))
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            c.shared.network.lock(|net| match net.update() {
                NetworkState::SettingsChanged => {
                    settings_update::spawn(*net.miniconf.settings()).unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {}
            })
        }
    }

    #[task(priority = 1, local=[pwm], shared=[dac, settings, gpio], capacity=1)]
    fn settings_update(mut c: settings_update::Context, settings: Settings) {
        // Verify settings and make them available
        c.shared
            .settings
            .lock(|current_settings| *current_settings = settings);

        // led is proxy for real settings and telemetry later
        c.shared
            .gpio
            .lock(|gpio| gpio.set_led(Led::Led0, settings.led.into()));

        let pwm = c.local.pwm;
        for ch in OutputChannelIdx::into_enum_iter() {
            let s = settings.output_settings[ch as usize];
            // TODO: implement what happens if user chooses invalid value. currently just panick.
            pwm.set_limit(Limit::Voltage(ch), s.voltage_limit).unwrap();
            pwm.set_limit(Limit::PositiveCurrent(ch), s.current_limit_positive)
                .unwrap();
            pwm.set_limit(Limit::NegativeCurrent(ch), s.current_limit_negative)
                .unwrap();
            c.shared
                .dac
                .lock(|dac| dac.set(ch, s.current.try_into().unwrap()));
            c.shared
                .gpio
                .lock(|gpio| gpio.set_shutdown(ch, s.shutdown.into()));
        }
    }

    #[task(priority = 1, local=[adc_internal], shared=[network, settings, telemetry, gpio, channel_temperatures])]
    fn telemetry_task(mut c: telemetry_task::Context) {
        let mut telemetry: Telemetry = c.shared.telemetry.lock(|telemetry| *telemetry);

        let adc_int = c.local.adc_internal;
        telemetry.p3v3_voltage = adc_int.read_p3v3_voltage();
        telemetry.p5v_voltage = adc_int.read_p5v_voltage();
        telemetry.p12v_voltage = adc_int.read_p12v_voltage();
        telemetry.p12v_current = adc_int.read_p12v_current();
        for ch in OutputChannelIdx::into_enum_iter() {
            let idx = ch as usize;
            telemetry.output_vref[idx] = adc_int.read_output_vref(ch);
            telemetry.output_voltage[idx] = adc_int.read_output_voltage(ch);
            telemetry.output_current[idx] = adc_int.read_output_current(ch);
        }
        c.shared.gpio.lock(|gpio| {
            telemetry.overtemp = gpio.overtemp();
            telemetry.poe = gpio.poe();
        });

        c.shared
            .network
            .lock(|network| network.telemetry.publish(&telemetry));

        // TODO: validate telemetry period.
        let telemetry_period = c.shared.settings.lock(|settings| settings.telemetry_period);
        telemetry_task::spawn_after(((telemetry_period * 1000.0) as u64).millis()).unwrap();
    }

    #[task(priority = 2, shared=[dac], capacity = 4)]
    fn convert_current_and_set_dac(
        mut c: convert_current_and_set_dac::Context,
        output_ch: OutputChannelIdx,
        current: f32,
    ) {
        let dac_code = DacCode::try_from(current).unwrap();
        c.shared.dac.lock(|dac| dac.set(output_ch, dac_code));
    }

    #[task(priority = 2, shared=[channel_temperatures, settings], local=[iir_state], capacity = 4)]
    fn process_output_channel(c: process_output_channel::Context, output_ch: OutputChannelIdx) {
        let idx = output_ch as usize;
        let output_current = (c.shared.settings, c.shared.channel_temperatures).lock(
            |settings, channel_temperatures| {
                settings.output_settings[idx].output_channel.update(
                    channel_temperatures,
                    &mut c.local.iir_state[idx],
                    false,
                )
            },
        );
        info!("output_current: {:?}", output_current);
        convert_current_and_set_dac::spawn(output_ch, output_current).unwrap();
    }

    // Higher priority than telemetry but lower than adc data readout.
    // 8 capacity to allow for max. 8 conversions to be queued.
    #[task(priority = 2, shared=[channel_temperatures, telemetry], capacity = 8)]
    fn convert_adc_code(c: convert_adc_code::Context, input_ch: InputChannel, adc_code: AdcCode) {
        let idx = input_ch as usize;
        // convert ADC code to °C and store in channel_temperatures array and telemetry
        (c.shared.channel_temperatures, c.shared.telemetry).lock(|temp, tele| {
            temp[idx] = adc_code.into();
            tele.channel_temperatures[idx] = temp[idx] as f32;
        });
        // start processing when the last adc channel has been read out
        if input_ch == InputChannel::Seven {
            process_output_channel::spawn(OutputChannelIdx::Zero).unwrap();
            process_output_channel::spawn(OutputChannelIdx::One).unwrap();
            process_output_channel::spawn(OutputChannelIdx::Two).unwrap();
            process_output_channel::spawn(OutputChannelIdx::Three).unwrap();
        }
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

    #[task(priority = 3, binds = EXTI15_10, local=[adc_sm])]
    fn adc_readout(c: adc_readout::Context) {
        let (input_ch, adc_code) = c.local.adc_sm.handle_interrupt();
        convert_adc_code::spawn(input_ch, adc_code).unwrap();
    }
}
