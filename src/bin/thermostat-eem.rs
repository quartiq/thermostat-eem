//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use thermostat_eem::{output_channel, statistics};

use panic_probe as _; // global panic handler

use fugit::ExtU32;
use miniconf::Tree;
use num_traits::Float as _;
use output_channel::{OutputChannel, State};
use platform::{AppSettings, NetSettings};
use rtic_sync::{channel::*, make_channel};
use serde::Serialize;
use statistics::{Buffer, Statistics};
use stream::Target;
use thermostat_eem::convert::{AdcPhy, DacCode, Ntc, PoePower, Sensor};

#[derive(Clone, Debug, Tree, Default)]
pub struct InputChannel {
    sensor: Sensor,
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.sensor)]
    _typ: (),
}

#[derive(Clone, Debug, Tree)]
pub struct ThermostatEem {
    /// Specifies the telemetry output period in seconds.
    telemetry_period: f32,

    /// Input sensor configuration
    input: [[Option<InputChannel>; 4]; 4],

    /// Array of settings for the Thermostat output channels.
    output: [OutputChannel; 4],

    /// Alarm settings.
    alarm: output_channel::Alarm,

    #[tree(with=miniconf::leaf)]
    stream: Target,
}

impl Default for ThermostatEem {
    fn default() -> Self {
        Self {
            telemetry_period: 5.0.into(),
            input: Default::default(),
            output: Default::default(),
            alarm: Default::default(),
            stream: Default::default(),
        }
    }
}

#[derive(Clone, Debug, Tree, Default)]
pub struct Settings {
    pub thermostat_eem: ThermostatEem,

    pub net: NetSettings,
}

impl AppSettings for Settings {
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
            net: NetSettings::new(self.net.mac),
            ..Default::default()
        }
    }
}

/// Telemetry for various quantities that are continuously monitored by eg. the MCU ADC.
#[derive(Serialize, Clone, Default, Debug)]
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
    /// CPU temperature
    cpu_temp: f32,
}

/// Thermostat-EEM Telemetry.
#[derive(Serialize, Clone, Debug, Default)]
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
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Stream {
    temperature: [[f32; 4]; 4],
    current: [f32; 4],
}

#[cfg(not(target_os = "none"))]
fn main() {
    use miniconf::{json::to_json_value, json_schema::TreeJsonSchema};
    let s = Settings::default();
    println!(
        "{}",
        serde_json::to_string_pretty(&to_json_value(&s).unwrap()).unwrap()
    );
    let mut schema = TreeJsonSchema::new(Some(&s)).unwrap();
    schema
        .root
        .insert("title".to_string(), "Thermostat-EEM".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC]))]
mod app {
    use super::*;
    use rtic_monotonics::Monotonic;
    use stream::{Format, FrameGenerator};
    use thermostat_eem::OutputChannelIdx;
    use thermostat_eem::hardware::{
        SerialTerminal, SystemTimer, Systick, UsbDevice,
        adc::{Adc, Data, sm::StateMachine},
        adc_internal::AdcInternal,
        dac::Dac,
        gpio::Gpio,
        hal,
        net::{NetworkState, NetworkUsers},
        pwm::{Limit, Pwm},
        setup::setup,
    };

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<ThermostatEem>,
        settings: Settings,
        telemetry: Telemetry,
        gpio: Gpio,
        temperature: [[f64; 4]; 4], // input temperature array in °C. Organized as [Adc_idx,  Channel_idx].
        statistics: [[Buffer; 4]; 4], // input statistics buffer for processing telemetry. Organized as [Adc_idx,  Channel_idx].
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
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
        let mut thermostat = setup::<Settings, 8>(c.core, c.device, clock);

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
                    sensor: Sensor::Ntc(Ntc::new(25.0, 10.0e3, r_ref, 3988.0))
                        .into(),
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

        let generator = network.configure_streaming(Format::ThermostatEem);

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
        blink::spawn().unwrap();

        (shared, local)
    }

    #[idle(shared=[network, settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            (&mut c.shared.network, &mut c.shared.settings).lock(
                |net, settings| match net.update(&mut settings.thermostat_eem) {
                    NetworkState::SettingsChanged => settings::spawn().unwrap(),
                    NetworkState::Updated => {}
                    NetworkState::NoChange => {}
                },
            )
        }
    }

    #[task(priority = 1, local=[pwm], shared=[network, settings, gpio])]
    async fn settings(c: settings::Context) {
        let pwm = c.local.pwm;
        (c.shared.network, c.shared.gpio, c.shared.settings).lock(
            |network, gpio, settings| {
                for (s, ch) in settings
                    .thermostat_eem
                    .output
                    .iter_mut()
                    .zip(OutputChannelIdx::ALL)
                {
                    pwm.set_limit(Limit::Voltage(ch), s.voltage_limit).unwrap();
                    let [pos, neg] = s.current_limits();
                    pwm.set_limit(Limit::PositiveCurrent(ch), pos).unwrap();
                    pwm.set_limit(Limit::NegativeCurrent(ch), neg).unwrap();
                    gpio.set_shutdown(ch, (s.state == State::Off).into());
                }

                network.direct_stream(settings.thermostat_eem.stream);
            },
        );
    }

    #[task(priority = 1, shared=[gpio, settings, telemetry])]
    async fn blink(mut c: blink::Context) {
        const DUTY_PERIOD: u32 = 10; // ms
        const DUTY_STEPS: i32 = 20;
        const DUTY_DECADES: f32 = -4.; // lower error -> higher duty cycle
        const MAX_ERROR: f32 = 10.; // MAX_ERROR K for MIN_DUTY steps duty
        const MIN_DUTY: i32 = 1;
        loop {
            (&mut c.shared.telemetry, &mut c.shared.gpio).lock(
                |telemetry, gpio| {
                    for (alarm, ch) in
                        telemetry.alarm.iter().zip(OutputChannelIdx::ALL)
                    {
                        gpio.set_led_red(
                            ch.into(),
                            alarm.iter().any(|a| a.unwrap_or_default()).into(),
                        );
                    }
                },
            );

            let duty = (&mut c.shared.settings)
                .lock(|settings| {
                    OutputChannelIdx::ALL.each_ref().map(|ch| {
                        let s = &settings.thermostat_eem.output[*ch as usize];
                        (s.state != State::Off).then_some(s.error())
                    })
                })
                .map(|e| {
                    e.map(|e| {
                        (((e.abs() * MAX_ERROR.recip()).log10()
                            * (DUTY_STEPS as f32 / DUTY_DECADES))
                            as i32)
                            .max(MIN_DUTY)
                    })
                    .unwrap_or_default()
                });
            for i in 0..DUTY_STEPS {
                (&mut c.shared.gpio).lock(|gpio| {
                    for (duty, ch) in duty.iter().zip(OutputChannelIdx::ALL) {
                        gpio.set_led_green(ch.into(), (*duty > i).into());
                    }
                });
                Systick::delay(DUTY_PERIOD.millis()).await;
            }
        }
    }

    #[task(priority = 1, local=[adc_internal], shared=[network, settings, telemetry, gpio, statistics])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let mut telemetry: Telemetry =
                c.shared.telemetry.lock(|telemetry| telemetry.clone());
            let adc_int = &mut c.local.adc_internal;
            telemetry.monitor.cpu_temp = adc_int.read_temperature();
            telemetry.monitor.p3v3_voltage = adc_int.read_p3v3_voltage();
            telemetry.monitor.p5v_voltage = adc_int.read_p5v_voltage();
            telemetry.monitor.p12v_voltage = adc_int.read_p12v_voltage();
            telemetry.monitor.p12v_current = adc_int.read_p12v_current();
            for ch in OutputChannelIdx::ALL {
                let idx = ch as usize;
                telemetry.monitor.output_vref[idx] =
                    adc_int.read_output_vref(ch);
                telemetry.monitor.output_voltage[idx] =
                    adc_int.read_output_voltage(ch);
                telemetry.monitor.output_current[idx] =
                    adc_int.read_output_current(ch);
            }
            c.shared.gpio.lock(|gpio| {
                telemetry.monitor.overtemp = gpio.overtemp();
                telemetry.monitor.poe = gpio.poe();
            });

            // Finalize temperature telemetry and reset buffer
            for phy_i in 0..4 {
                for cfg_i in 0..4 {
                    c.shared.statistics.lock(|buff| {
                        telemetry.statistics[phy_i][cfg_i] =
                            core::mem::take(&mut buff[phy_i][cfg_i]).into();
                    });
                }
            }

            c.shared.network.lock(|network| {
                network.telemetry.publish_telemetry(&telemetry)
            });

            let telemetry_period = c
                .shared
                .settings
                .lock(|settings| settings.thermostat_eem.telemetry_period);
            Systick::delay(
                ((telemetry_period * 1000.0) as u32).max(500).millis(),
            )
            .await;
        }
    }

    #[task(priority = 1, shared=[network, settings, temperature, telemetry])]
    async fn alarm(mut c: alarm::Context) {
        loop {
            let config = c
                .shared
                .settings
                .lock(|settings| settings.thermostat_eem.alarm.clone());
            let temperatures = c.shared.temperature.lock(|temp| *temp);
            let mut alarms = [[None; 4]; 4];
            for ((l, t), a) in config
                .temperature_limits
                .as_flattened()
                .iter()
                .zip(temperatures.as_flattened().iter())
                .zip(alarms.as_flattened_mut().iter_mut())
            {
                *a = l.as_ref().map(|l| !l.contains(&(*t as _)));
            }
            let all = alarms
                .iter()
                .any(|a| a.iter().any(|a| a.unwrap_or_default()));

            c.shared
                .telemetry
                .lock(|telemetry| telemetry.alarm = alarms);
            if let Some(target) = config.target.as_ref() {
                c.shared
                    .network
                    .lock(|net| net.telemetry.publish(&target, &all))
                    .map_err(|e| {
                        log::error!("Telemetry publishing error: {:?}", e)
                    })
                    .ok();
            }
            // Note that you have to wait for a full period of the previous setting first for a change of period to take affect.
            Systick::delay(((config.period * 1000.0) as u32).max(100).millis())
                .await;
        }
    }

    // Higher priority than telemetry but lower than adc data readout.
    #[task(priority = 2, shared=[temperature, statistics, telemetry, settings], local=[generator, dac])]
    async fn process(
        mut c: process::Context,
        mut data: Receiver<'static, Data, 4>,
    ) {
        while let Ok(Data { phy, ch, code }) = data.recv().await {
            (
                &mut c.shared.temperature,
                &mut c.shared.statistics,
                &mut c.shared.settings,
            )
                .lock(|temperature, statistics, settings| {
                    let temp = settings.thermostat_eem.input[phy as usize]
                        [ch.value() as usize]
                        .as_ref()
                        .unwrap()
                        .sensor
                        .convert(code);
                    temperature[phy as usize][ch.value() as usize] = temp;
                    statistics[phy as usize][ch.value() as usize]
                        .update(temp as _);
                });
            // Start processing when the last ADC has been read out.
            // This implies a zero-order hold: an input will not be updated at
            // every process()/for every update() if more than one channel is
            // enabled on an ADC.
            if phy == AdcPhy::Three {
                (
                    &mut c.shared.temperature,
                    &mut c.shared.telemetry,
                    &mut c.shared.settings,
                )
                    .lock(
                        |temperature, telemetry, settings| {
                            for ch in OutputChannelIdx::ALL.iter() {
                                let current = settings.thermostat_eem.output
                                    [*ch as usize]
                                    .update(temperature);
                                telemetry.output_current[*ch as usize] =
                                    current;
                                c.local.dac.set(
                                    *ch,
                                    DacCode::try_from(current).unwrap(),
                                );
                            }
                            let s = Stream {
                                temperature: temperature
                                    .each_ref()
                                    .map(|t| t.each_ref().map(|t| *t as _)),
                                current: telemetry.output_current,
                            };
                            c.local.generator.add(|buf| {
                                let b = bytemuck::cast_slice(
                                    bytemuck::bytes_of(&s),
                                );
                                buf[..b.len()].copy_from_slice(b);
                                b.len()
                            });
                        },
                    );
            }
        }
    }

    #[task(priority = 3, binds = EXTI15_10, local=[adc_sm, process])]
    fn adc_readout(c: adc_readout::Context) {
        let data = c.local.adc_sm.handle_interrupt();
        if let Err(e) = c.local.process.try_send(data) {
            log::warn!("Processing queue overflow: {e:?}");
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) {
        loop {
            // Handle the USB serial terminal.
            c.shared.usb.lock(|usb| {
                usb.poll(&mut [c
                    .local
                    .usb_terminal
                    .interface_mut()
                    .inner_mut()]);
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
