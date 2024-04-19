//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

use core::fmt::Write;

pub mod hardware;
pub mod net;
pub mod output_channel;
pub mod statistics;

use panic_probe as _; // global panic handler
use strum::IntoEnumIterator;

use hardware::{
    adc::AdcPhy,
    adc::{sm::StateMachine, Adc, AdcCode},
    adc_internal::AdcInternal,
    dac::{Dac, DacCode},
    gpio::{Gpio, PoePower},
    hal,
    pwm::{Limit, Pwm},
    system_timer::SystemTimer,
    OutputChannelIdx,
};
use miniconf::Tree;
use net::{
    data_stream::{FrameGenerator, StreamFormat, StreamTarget},
    Alarm, NetworkState, NetworkUsers,
};
use output_channel::{OutputChannel, State};
use serde::Serialize;
use statistics::{Buffer, Statistics};
use systick_monotonic::{ExtU64, Systick};

#[derive(Clone, Debug, Tree)]
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
    /// `output_channel/<n>`
    /// * `<n> := [0, 1, 2, 3]` specifies which channel to configure.
    ///
    /// # Value
    /// See [OutputChannel]
    #[tree(depth = 3)]
    output_channel: [OutputChannel; 4],

    /// Alarm settings.
    ///
    /// # Path
    /// `Alarm`
    ///
    /// # Value
    /// See [Alarm]
    #[tree(depth = 3)]
    alarm: Alarm,

    stream_target: StreamTarget,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            telemetry_period: 1.0,
            output_channel: Default::default(),
            alarm: Default::default(),
            stream_target: Default::default(),
        }
    }
}

/// Telemetry for various quantities that are continuously monitored by eg. the MCU ADC.
#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Monitor {
    p3v3_voltage: f32,
    p5v_voltage: f32,
    p12v_voltage: f32,
    p12v_current: f32,
    /// Measurement of the output reference voltages.
    output_vref: [f32; 4],
    /// Measurement of the output currents.
    output_current: [f32; 4],
    /// Measurement of the output voltages.
    output_voltage: [f32; 4],
    /// See [PoePower]
    poe: PoePower,
    /// Overtemperature status.
    overtemp: bool,
}

/// Thermostat-EEM Telemetry.
#[derive(Serialize, Copy, Clone, Debug, Default)]
pub struct Telemetry {
    /// see [Monitor]
    monitor: Monitor,
    /// `[<adc>][<channel>]` array of [Statistics]. `None` for disabled channels.
    statistics: [[Option<Statistics>; 4]; 4],
    /// Alarm status for each enabled input channel. `None` for disabled channels.
    alarm: [[Option<bool>; 4]; 4],
    /// Output current in Amperes for each Thermostat output channel.
    output_current: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Stream {
    temperature: [[f32; 4]; 4],
    current: [f32; 4],
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000>; // 1ms resolution

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, 4>,
        settings: Settings,
        telemetry: Telemetry,
        gpio: Gpio,
        temperature: [[f64; 4]; 4], // input temperature array in °C. Organized as [Adc_idx,  Channel_idx].
        statistics: [[Buffer; 4]; 4], // input statistics buffer for processing telemetry. Organized as [Adc_idx,  Channel_idx].
    }

    #[local]
    struct Local {
        adc_sm: StateMachine<Adc>,
        dac: Dac,
        pwm: Pwm,
        adc_internal: AdcInternal,
        iir_state: [[f64; 4]; 4],
        generator: FrameGenerator,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Initialize monotonic
        let clock = SystemTimer::new(|| monotonics::now().ticks());

        // setup Thermostat hardware
        let thermostat = hardware::setup::setup(c.device, clock);

        let settings: Settings = Settings::default();

        let mut id = heapless::String::<32>::new();
        write!(&mut id, "{}", thermostat.net.mac_address).unwrap();

        let mut network = NetworkUsers::new(
            thermostat.net.stack,
            thermostat.net.phy,
            clock,
            &id,
            option_env!("BROKER").unwrap_or("mqtt"),
            thermostat.metadata,
        );

        let generator = network.configure_streaming(StreamFormat::ThermostatEem as _);

        let local = Local {
            adc_sm: thermostat.adc_sm,
            pwm: thermostat.pwm,
            adc_internal: thermostat.adc_internal,
            iir_state: Default::default(),
            dac: thermostat.dac,
            generator,
        };

        let shared = Shared {
            network,
            settings,
            telemetry: Default::default(),
            gpio: thermostat.gpio,
            temperature: Default::default(),
            statistics: Default::default(),
        };

        // Apply initial settings
        settings::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        telemetry::spawn().unwrap();
        alarm::spawn().unwrap();

        (
            shared,
            local,
            init::Monotonics(Systick::new(
                c.core.SYST,
                thermostat.clocks.sysclk().to_Hz(),
            )),
        )
    }

    #[idle(shared=[network, settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            (&mut c.shared.network, &mut c.shared.settings).lock(|net, settings| {
                match net.update(settings) {
                    NetworkState::SettingsChanged => settings::spawn().unwrap(),
                    NetworkState::Updated => {}
                    NetworkState::NoChange => {}
                }
            })
        }
    }

    #[task(priority = 1, local=[pwm], shared=[network, settings, gpio])]
    fn settings(c: settings::Context) {
        let pwm = c.local.pwm;
        (c.shared.network, c.shared.gpio, c.shared.settings).lock(|network, gpio, settings| {
            for (ch, s) in OutputChannelIdx::iter().zip(settings.output_channel.iter_mut()) {
                s.finalize_settings(); // clamp limits and normalize weights
                pwm.set_limit(Limit::Voltage(ch), s.voltage_limit).unwrap();
                let [pos, neg] = s.current_limits();
                pwm.set_limit(Limit::PositiveCurrent(ch), pos).unwrap();
                pwm.set_limit(Limit::NegativeCurrent(ch), neg).unwrap();
                gpio.set_shutdown(ch, (s.state == State::Off).into());
                gpio.set_led(ch.into(), (s.state != State::Off).into()); // fix leds to channel state
            }

            network.direct_stream(settings.stream_target.into());
        });
    }

    #[task(priority = 1, local=[adc_internal], shared=[network, settings, telemetry, gpio, statistics])]
    fn telemetry(mut c: telemetry::Context) {
        let mut telemetry: Telemetry = c.shared.telemetry.lock(|telemetry| *telemetry);
        let adc_int = c.local.adc_internal;
        telemetry.monitor.p3v3_voltage = adc_int.read_p3v3_voltage();
        telemetry.monitor.p5v_voltage = adc_int.read_p5v_voltage();
        telemetry.monitor.p12v_voltage = adc_int.read_p12v_voltage();
        telemetry.monitor.p12v_current = adc_int.read_p12v_current();
        for ch in OutputChannelIdx::iter() {
            let idx = ch as usize;
            telemetry.monitor.output_vref[idx] = adc_int.read_output_vref(ch);
            telemetry.monitor.output_voltage[idx] = adc_int.read_output_voltage(ch);
            telemetry.monitor.output_current[idx] = adc_int.read_output_current(ch);
        }
        c.shared.gpio.lock(|gpio| {
            telemetry.monitor.overtemp = gpio.overtemp();
            telemetry.monitor.poe = gpio.poe();
        });

        // Finalize temperature telemetry and reset buffer
        for phy_i in 0..4 {
            for cfg_i in 0..4 {
                c.shared.statistics.lock(|buff| {
                    telemetry.statistics[phy_i][cfg_i] = buff[phy_i][cfg_i].into();
                    buff[phy_i][cfg_i] = Default::default();
                });
            }
        }

        c.shared
            .network
            .lock(|network| network.telemetry.publish(&telemetry));

        // TODO: validate telemetry period.
        let telemetry_period = c.shared.settings.lock(|settings| settings.telemetry_period);
        telemetry::spawn_after(((telemetry_period * 1000.0) as u64).millis()).unwrap();
    }

    #[task(priority = 1, shared=[network, settings, temperature, telemetry])]
    fn alarm(mut c: alarm::Context) {
        let alarm = c.shared.settings.lock(|settings| settings.alarm.clone());
        if alarm.armed {
            let temperatures = c.shared.temperature.lock(|temp| *temp);
            let mut alarms = [[None; 4]; 4];
            let mut alarm_state = false;
            for phy_i in 0..4 {
                for cfg_i in 0..4 {
                    if let Some(l) = &alarm.temperature_limits[phy_i][cfg_i] {
                        let a = !(l[0]..l[1]).contains(&(temperatures[phy_i][cfg_i] as _));
                        alarms[phy_i][cfg_i] = Some(a);
                        alarm_state |= a;
                    }
                }
            }
            c.shared
                .telemetry
                .lock(|telemetry| telemetry.alarm = alarms);
            c.shared
                .network
                .lock(|net| net.telemetry.publish_alarm(&alarm.target, &alarm_state));
        }
        // Note that you have to wait for a full period of the previous setting first for a change of period to take affect.
        alarm::spawn_after(((alarm.period * 1000.0) as u64).millis()).unwrap();
    }

    // Higher priority than telemetry but lower than adc data readout.
    #[task(priority = 2, shared=[temperature, statistics, telemetry, settings], local=[iir_state, generator, dac], capacity=4)]
    fn process(c: process::Context, phy: AdcPhy, ch: usize, adc_code: AdcCode) {
        let temp = adc_code.into();
        (
            c.shared.temperature,
            c.shared.statistics,
            c.shared.telemetry,
            c.shared.settings,
        )
            .lock(|temperature, statistics, telemetry, settings| {
                temperature[phy as usize][ch] = temp;
                statistics[phy as usize][ch].update(temp as _);

                // Start processing when the last ADC has been read out.
                // This implies a zero-order hold (aka the input sample will not be updated at every signal processing step) if more than one channel is enabled on an ADC.
                if phy != AdcPhy::Three {
                    return;
                }

                for ch in OutputChannelIdx::iter() {
                    let idx = ch as usize;
                    let current = settings.output_channel[idx]
                        .update(temperature, &mut c.local.iir_state[idx])
                        as f32;
                    telemetry.output_current[idx] = current;
                    c.local.dac.set(ch, DacCode::try_from(current).unwrap());
                }
                let mut s = Stream {
                    temperature: [[0.0; 4]; 4],
                    current: telemetry.output_current,
                };
                for (t, u) in s
                    .temperature
                    .iter_mut()
                    .flatten()
                    .zip(temperature.iter().flatten())
                {
                    *t = *u as _;
                }
                let b = bytemuck::bytes_of(&s);
                c.local.generator.add(|buf| {
                    for (b, s) in buf.iter_mut().zip(b.iter()) {
                        b.write(*s);
                    }
                    b.len()
                });
            });
    }

    #[task(priority = 3, binds = EXTI15_10, local=[adc_sm])]
    fn adc_readout(c: adc_readout::Context) {
        let (phy, ch, adc_code) = c.local.adc_sm.handle_interrupt();
        process::spawn(phy, ch, adc_code).unwrap();
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
}
