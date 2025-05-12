//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;
pub mod net;
pub mod output_channel;
pub mod settings;
pub mod statistics;

use panic_probe as _; // global panic handler
use strum::IntoEnumIterator;

use hardware::{
    adc::AdcPhy,
    adc::{sm::StateMachine, Adc, AdcCode, Ntc, Sensor},
    adc_internal::AdcInternal,
    dac::{Dac, DacCode},
    gpio::{Gpio, PoePower},
    hal,
    pwm::{Limit, Pwm},
    OutputChannelIdx, SerialTerminal, SystemTimer, Systick, UsbDevice,
};

use rtic_monotonics::Monotonic;
use rtic_sync::{channel::*, make_channel};

use fugit::ExtU32;
use miniconf::{Leaf, StrLeaf, TreeDeserialize, TreeKey, TreeSerialize};
use net::{
    data_stream::{FrameGenerator, StreamFormat, StreamTarget},
    Alarm, NetworkState, NetworkUsers,
};
use output_channel::{OutputChannel, State};
use serde::Serialize;
use settings::NetSettings;
use statistics::{Buffer, Statistics};

#[derive(Clone, Debug, TreeSerialize, TreeDeserialize, TreeKey, Default)]
pub struct InputChannel {
    #[tree(rename = "typ")]
    sensor: StrLeaf<Sensor>,
    #[tree(rename="sensor", typ = "Sensor", defer=(*self.sensor))]
    _sensor: (),
}

#[derive(Clone, Debug, TreeSerialize, TreeDeserialize, TreeKey)]
pub struct ThermostatEem {
    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any positive non-zero value. Will be rounded to milliseconds.
    telemetry_period: Leaf<f32>,

    /// Input sensor configuration
    input: [[Option<InputChannel>; 4]; 4],

    /// Array of settings for the Thermostat output channels.
    ///
    /// # Path
    /// `output_channel/<n>`
    /// * `<n> := [0, 1, 2, 3]` specifies which channel to configure.
    ///
    /// # Value
    /// See [OutputChannel]
    output: [OutputChannel; 4],

    /// Alarm settings.
    ///
    /// # Path
    /// `Alarm`
    ///
    /// # Value
    /// See [Alarm]
    alarm: Alarm,

    stream: Leaf<StreamTarget>,
}

impl Default for ThermostatEem {
    fn default() -> Self {
        Self {
            telemetry_period: 1.0.into(),
            input: Default::default(),
            output: Default::default(),
            alarm: Default::default(),
            stream: Default::default(),
        }
    }
}

#[derive(Clone, Debug, TreeSerialize, TreeDeserialize, TreeKey)]
pub struct Settings {
    pub thermostat_eem: ThermostatEem,

    pub net: NetSettings,
}

impl settings::AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            thermostat_eem: ThermostatEem::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            thermostat_eem: ThermostatEem::default(),
            net: NetSettings::new(self.net.mac),
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

#[derive(Clone, Debug)]
struct Data {
    phy: AdcPhy,
    ch: usize,
    adc_code: AdcCode,
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<ThermostatEem, 7>,
        settings: Settings,
        telemetry: Telemetry,
        gpio: Gpio,
        temperature: [[f64; 4]; 4], // input temperature array in °C. Organized as [Adc_idx,  Channel_idx].
        statistics: [[Buffer; 4]; 4], // input statistics buffer for processing telemetry. Organized as [Adc_idx,  Channel_idx].
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings, 8>,
        adc_sm: StateMachine<Adc>,
        dac: Dac,
        pwm: Pwm,
        adc_internal: AdcInternal,
        generator: FrameGenerator,
        process: Sender<'static, Data, 4>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        // Initialize monotonic
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // setup Thermostat hardware
        let mut thermostat = hardware::setup::setup::<Settings, 8>(c.core, c.device, clock);

        for (channel, mux) in thermostat
            .settings
            .thermostat_eem
            .input
            .as_flattened_mut()
            .iter_mut()
            .zip(thermostat.adc_input_config.as_flattened().iter())
        {
            if let Some(mux) = mux {
                let r_ref = if mux.is_single_ended() { 5.0e3 } else { 10.0e3 };
                *channel = Some(InputChannel {
                    // This isn't ideal as it confilcts with the default/clear notion of serial-settings.
                    sensor: Sensor::Ntc(Ntc::new(25.0, 10.0e3, r_ref, 3988.0)).into(),
                    ..Default::default()
                });
            }
        }

        let mut network = NetworkUsers::new(
            thermostat.net.stack,
            thermostat.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &thermostat.settings.net,
            thermostat.metadata,
        );

        let generator = network.configure_streaming(StreamFormat::ThermostatEem as _);

        let (process, r) = make_channel!(Data, 4);

        let local = Local {
            usb_terminal: thermostat.usb_serial,
            adc_sm: thermostat.adc_sm,
            pwm: thermostat.pwm,
            adc_internal: thermostat.adc_internal,
            dac: thermostat.dac,
            generator,
            process,
        };

        let shared = Shared {
            usb: thermostat.usb,
            network,
            settings: thermostat.settings,
            telemetry: Default::default(),
            gpio: thermostat.gpio,
            temperature: Default::default(),
            statistics: Default::default(),
        };

        process::spawn(r).unwrap();

        // Apply initial settings
        settings::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        telemetry::spawn().unwrap();
        alarm::spawn().unwrap();
        usb::spawn().unwrap();

        (shared, local)
    }

    #[idle(shared=[network, settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            (&mut c.shared.network, &mut c.shared.settings).lock(|net, settings| {
                match net.update(&mut settings.thermostat_eem) {
                    NetworkState::SettingsChanged => settings::spawn().unwrap(),
                    NetworkState::Updated => {}
                    NetworkState::NoChange => {}
                }
            })
        }
    }

    #[task(priority = 1, local=[pwm], shared=[network, settings, gpio])]
    async fn settings(c: settings::Context) {
        let pwm = c.local.pwm;
        (c.shared.network, c.shared.gpio, c.shared.settings).lock(|network, gpio, settings| {
            for (ch, s) in OutputChannelIdx::iter().zip(settings.thermostat_eem.output.iter_mut()) {
                pwm.set_limit(Limit::Voltage(ch), *s.voltage_limit).unwrap();
                let [pos, neg] = s.current_limits();
                pwm.set_limit(Limit::PositiveCurrent(ch), pos).unwrap();
                pwm.set_limit(Limit::NegativeCurrent(ch), neg).unwrap();
                gpio.set_shutdown(ch, (*s.state == State::Off).into());
                gpio.set_led(ch.into(), (*s.state != State::Off).into()); // fix leds to channel state
            }

            network.direct_stream(*settings.thermostat_eem.stream);
        });
    }

    #[task(priority = 1, local=[adc_internal], shared=[network, settings, telemetry, gpio, statistics])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let mut telemetry: Telemetry = c.shared.telemetry.lock(|telemetry| *telemetry);
            let adc_int = &mut c.local.adc_internal;
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
            let telemetry_period = c
                .shared
                .settings
                .lock(|settings| settings.thermostat_eem.telemetry_period);
            Systick::delay((*telemetry_period as u32).secs()).await;
        }
    }

    #[task(priority = 1, shared=[network, settings, temperature, telemetry])]
    async fn alarm(mut c: alarm::Context) {
        loop {
            let alarm = c
                .shared
                .settings
                .lock(|settings| settings.thermostat_eem.alarm.clone());
            if *alarm.armed {
                let temperatures = c.shared.temperature.lock(|temp| *temp);
                let mut alarms = [[None; 4]; 4];
                let alarm_state = alarm
                    .temperature_limits
                    .as_flattened()
                    .iter()
                    .zip(temperatures.as_flattened().iter())
                    .zip(alarms.as_flattened_mut().iter_mut())
                    .any(|((l, t), a)| {
                        *a = l.map(|l| !(*l[0]..*l[1]).contains(&(*t as _)));
                        a.unwrap_or_default()
                    });
                c.shared
                    .telemetry
                    .lock(|telemetry| telemetry.alarm = alarms);
                c.shared
                    .network
                    .lock(|net| net.telemetry.publish_alarm(&alarm.target, &alarm_state));
            }
            // Note that you have to wait for a full period of the previous setting first for a change of period to take affect.
            Systick::delay(((*alarm.period * 1000.0) as u32).millis()).await;
        }
    }

    // Higher priority than telemetry but lower than adc data readout.
    #[task(priority = 2, shared=[temperature, statistics, telemetry, settings], local=[generator, dac])]
    async fn process(mut c: process::Context, mut data: Receiver<'static, Data, 4>) {
        while let Ok(Data { phy, ch, adc_code }) = data.recv().await {
            let temp = c.shared.settings.lock(|settings| {
                let input = settings.thermostat_eem.input[phy as usize][ch]
                    .as_ref()
                    .unwrap();
                input.sensor.convert(adc_code)
            });
            (
                &mut c.shared.temperature,
                &mut c.shared.statistics,
                &mut c.shared.telemetry,
                &mut c.shared.settings,
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
                        let current = settings.thermostat_eem.output[idx].update(temperature);
                        telemetry.output_current[idx] = current;
                        c.local.dac.set(ch, DacCode::try_from(current).unwrap());
                    }
                    let mut s = Stream {
                        temperature: [[0.0; 4]; 4],
                        current: telemetry.output_current,
                    };
                    for (t, u) in s
                        .temperature
                        .as_flattened_mut()
                        .iter_mut()
                        .zip(temperature.as_flattened().iter())
                    {
                        *t = *u as _;
                    }
                    c.local.generator.add(|buf| {
                        let b = bytemuck::cast_slice(bytemuck::bytes_of(&s));
                        buf[..b.len()].copy_from_slice(b);
                        b.len()
                    });
                });
        }
    }

    #[task(priority = 3, binds = EXTI15_10, local=[adc_sm, process])]
    fn adc_readout(c: adc_readout::Context) {
        let (phy, ch, adc_code) = c.local.adc_sm.handle_interrupt();
        if let Err(e) = c.local.process.try_send(Data { phy, ch, adc_code }) {
            log::warn!("Processing queue overflow: {e:?}");
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) {
        loop {
            // Handle the USB serial terminal.
            c.shared.usb.lock(|usb| {
                usb.poll(&mut [c.local.usb_terminal.interface_mut().inner_mut()]);
            });

            c.shared.settings.lock(|settings| {
                if c.local.usb_terminal.poll(settings).unwrap() {
                    settings::spawn().unwrap()
                }
            });

            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, shared=[network])]
    async fn ethernet_link(mut c: ethernet_link::Context) {
        loop {
            c.shared
                .network
                .lock(|network| network.processor.handle_link());
            Systick::delay(1.secs()).await;
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }
}
