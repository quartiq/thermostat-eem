//! # Thermostat_EEM
//!
//! Firmware for "Thermostat EEM", a multichannel temperature controller.

#![no_std]
#![no_main]

pub mod hardware;
pub mod net;
pub mod output_channel;
pub mod statistics;

use panic_probe as _; // global panic handler

use enum_iterator::all;
use hardware::{
    ad7172::AdcChannel,
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
use idsp::iir;
use net::{miniconf::Miniconf, serde::Serialize, Alarm, NetworkState, NetworkUsers};
use statistics::{Buffer, Statistics};
use systick_monotonic::*;

#[derive(Clone, Debug, Miniconf)]
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
    /// * <n> specifies which channel to configure. <n> := [0, 1, 2, 3]
    ///
    /// # Value
    /// See [output_channel::OutputChannel]
    #[miniconf(defer)]
    output_channel: miniconf::Array<output_channel::OutputChannel, 4>,

    /// Alarm settings.
    ///
    /// # Path
    /// `Alarm`
    ///
    /// # Value
    /// See [Alarm]
    #[miniconf(defer)]
    alarm: Alarm,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            telemetry_period: 1.0,
            output_channel: [output_channel::OutputChannel::new(0., -0., 0.); 4].into(),
            alarm: Alarm {
                armed: false,
                target: heapless::String::<128>::default(),
                period_ms: 1000,
                temperature_limits: Default::default(),
            },
        }
    }
}

#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Monitor {
    p3v3_voltage: f32,
    p5v_voltage: f32,
    p12v_voltage: f32,
    p12v_current: f32,
    output_vref: [f32; 4],
    output_current: [f32; 4],
    output_voltage: [f32; 4],
    poe: PoePower,
    overtemp: bool,
    alarm: [[Option<bool>; 4]; 4], // Alarm status for each input channel
}

#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Telemetry {
    monitor: Monitor,
    statistics: [[Option<Statistics>; 4]; 4],
    output_current: [f32; 4],
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

        /// These two can be the same generic datatype for the inputs with one being f64 and one buffer
        temperature: [[Option<f64>; 4]; 4], // input temperature array in °C. Organized as [Adc_idx,  Channel_idx].
        statistics_buff: [[Option<Buffer>; 4]; 4], // input statistics buffer for processing telemetry. Organized as [Adc_idx,  Channel_idx].Buffer; 4]; 4], // temperature buffer for processing telemetry. Organized as [Adc_idx,  Channel_idx].
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

        let local = Local {
            adc_sm: thermostat.adc_sm,
            pwm: thermostat.pwm,
            adc_internal: thermostat.adc_internal,
            iir_state: [[0.; 5]; 4],
        };

        let mut settings = Settings::default();

        // Initialize enabled temperatures, statistics buffers and alarm.
        let mut telemetry = Telemetry::default();
        let mut temperature: [[Option<f64>; 4]; 4] = [[None; 4]; 4];
        let mut statistics_buff: [[Option<Buffer>; 4]; 4] = [[None; 4]; 4];
        thermostat
            .adc_channels
            .iter()
            .flatten()
            .zip(temperature.iter_mut().flatten())
            .zip(statistics_buff.iter_mut().flatten())
            .zip(telemetry.monitor.alarm.iter_mut().flatten())
            .zip(settings.alarm.temperature_limits.iter_mut().flatten())
            .for_each(|((((ch, temp), buff), alarm), limits)| {
                if *ch {
                    (*temp, *buff, *alarm, *limits) = (
                        Some(0.),
                        Some(Buffer::default()),
                        Some(false),
                        Some([f32::MIN, f32::MAX]),
                    )
                }
            });

        // Initialize the output weights.
        settings.output_channel.iter_mut().for_each(|ch| {
            ch.weights
                .iter_mut()
                .flatten()
                .zip(thermostat.adc_channels.iter().flatten())
                .for_each(|(w, en)| {
                    if *en {
                        *w = Some(0.0)
                    }
                });
        });

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
            settings.clone(),
        );

        settings_update::spawn(settings.clone()).unwrap();
        ethernet_link::spawn().unwrap();
        telemetry_task::spawn().unwrap();
        mqtt_alarm::spawn().unwrap();

        let shared = Shared {
            dac: thermostat.dac,
            network,
            settings,
            telemetry,
            gpio: thermostat.gpio,
            temperature,
            statistics_buff,
        };

        (shared, local, init::Monotonics(mono))
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            c.shared.network.lock(|net| match net.update() {
                NetworkState::SettingsChanged => {
                    settings_update::spawn(net.miniconf.settings().clone()).unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {}
            })
        }
    }

    #[task(priority = 1, local=[pwm], shared=[dac, settings, gpio], capacity=1)]
    fn settings_update(mut c: settings_update::Context, mut settings: Settings) {
        // Limit y_min and y_max values here. Will be incorporated into miniconf response later.
        settings.output_channel.iter_mut().for_each(|ch| {
            ch.iir.y_max = ch
                .iir
                .y_max
                .clamp(-DacCode::MAX_CURRENT as _, DacCode::MAX_CURRENT as _);
            ch.iir.y_min = ch
                .iir
                .y_min
                .clamp(-DacCode::MAX_CURRENT as _, DacCode::MAX_CURRENT as _);
        });

        let pwm = c.local.pwm;
        for ch in all::<OutputChannelIdx>() {
            let mut s = settings.output_channel[ch as usize];
            let current_limits = s.finalize_settings(); // clamp limits and normalize weights
            pwm.set_limit(Limit::Voltage(ch), s.voltage_limit).unwrap();
            // give 5% extra headroom for PWM current limits
            pwm.set_limit(Limit::PositiveCurrent(ch), current_limits[0])
                .unwrap();
            pwm.set_limit(Limit::NegativeCurrent(ch), current_limits[1])
                .unwrap();
            c.shared.gpio.lock(|gpio| {
                gpio.set_shutdown(ch, s.shutdown.into());
                gpio.set_led(ch.into(), (!s.shutdown).into()) // fix leds to channel state
            });
        }

        // Verify settings and make them available
        c.shared.settings.lock(|current_settings| {
            *current_settings = settings;
        });
    }

    #[task(priority = 1, local=[adc_internal], shared=[network, settings, telemetry, gpio, statistics_buff])]
    fn telemetry_task(mut c: telemetry_task::Context) {
        let mut telemetry: Telemetry = c.shared.telemetry.lock(|telemetry| *telemetry);
        let adc_int = c.local.adc_internal;
        telemetry.monitor.p3v3_voltage = adc_int.read_p3v3_voltage();
        telemetry.monitor.p5v_voltage = adc_int.read_p5v_voltage();
        telemetry.monitor.p12v_voltage = adc_int.read_p12v_voltage();
        telemetry.monitor.p12v_current = adc_int.read_p12v_current();
        for ch in all::<OutputChannelIdx>() {
            let idx = ch as usize;
            telemetry.monitor.output_vref[idx] = adc_int.read_output_vref(ch);
            telemetry.monitor.output_voltage[idx] = adc_int.read_output_voltage(ch);
            telemetry.monitor.output_current[idx] = adc_int.read_output_current(ch);
        }
        c.shared.gpio.lock(|gpio| {
            telemetry.monitor.overtemp = gpio.overtemp();
            telemetry.monitor.poe = gpio.poe();
        });

        // finalize temperature telemetry and reset buffer
        c.shared.statistics_buff.lock(|buff| {
            buff.iter_mut()
                .flatten()
                .zip(telemetry.statistics.iter_mut().flatten())
                .for_each(|(buff, stat)| {
                    *stat = buff.map(|b| b.into());
                    *buff = buff.map(|_| Buffer::default());
                })
        });

        c.shared
            .network
            .lock(|network| network.telemetry.publish(&telemetry));

        // TODO: validate telemetry period.
        let telemetry_period = c.shared.settings.lock(|settings| settings.telemetry_period);
        telemetry_task::spawn_after(((telemetry_period * 1000.0) as u64).millis()).unwrap();
    }

    #[task(priority = 1, shared=[network, settings, temperature, telemetry])]
    fn mqtt_alarm(mut c: mqtt_alarm::Context) {
        let alarm = c.shared.settings.lock(|settings| settings.alarm.clone());
        if alarm.armed {
            let temperatures = c.shared.temperature.lock(|temp| *temp);
            let mut alarm_tele = c.shared.telemetry.lock(|telemetry| telemetry.monitor.alarm);
            let mut alarm_state = false;
            for ((&temp, limits), alarm_tele) in temperatures
                .iter()
                .flatten()
                .zip(alarm.temperature_limits.iter().flatten())
                .zip(alarm_tele.iter_mut().flatten())
            {
                // It is OK here to use [f32::MIN, f32::MAX] as limits where there is an enabled channel but no limits.
                // This should never happen anyways since we set the limits to [f32::MIN, f32::MAX] per default for enabled channels.
                *alarm_tele = temp.map(|temp| {
                    let ls = limits.unwrap_or([f32::MIN, f32::MAX]);
                    !(ls[0]..ls[1]).contains(&(temp as f32))
                });
                if *alarm_tele == Some(true) {
                    alarm_state = true;
                }
            }
            c.shared
                .network
                .lock(|net| net.telemetry.publish_alarm(&alarm.target, &alarm_state));
        }
        // Note that you have to wait for a full period of the previous setting first for a change of period to take affect.
        mqtt_alarm::spawn_after(alarm.period_ms.millis()).unwrap();
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

    #[task(priority = 2, shared=[temperature, settings, telemetry], local=[iir_state], capacity = 4)]
    fn process_output_channel(mut c: process_output_channel::Context, output_ch: OutputChannelIdx) {
        let idx = output_ch as usize;
        let output_current =
            (c.shared.settings, c.shared.temperature).lock(|settings, temperature| {
                settings.output_channel[idx].update(temperature, &mut c.local.iir_state[idx], false)
            });
        c.shared
            .telemetry
            .lock(|tele| tele.output_current[idx] = output_current);
        convert_current_and_set_dac::spawn(output_ch, output_current).unwrap();
    }

    // Higher priority than telemetry but lower than adc data readout.
    #[task(priority = 2, shared=[temperature, statistics_buff], capacity = 4)]
    fn convert_adc_code(
        mut c: convert_adc_code::Context,
        phy: AdcPhy,
        ch: AdcChannel,
        adc_code: AdcCode,
    ) {
        let (phy_i, ch_i) = (phy as usize, ch as usize);
        let temperature = adc_code.into();
        c.shared.temperature.lock(|temp| {
            temp[phy_i][ch_i] = Some(temperature);
        });
        c.shared.statistics_buff.lock(|stat_buff| {
            if let Some(buff) = &mut stat_buff[phy_i][ch_i] {
                buff.update(temperature);
            }
        });
        // Start processing when the last ADC has been read out.
        // This implies a zero-order hold (aka the input sample will not be updated at every signal processing step) if more than one channel is enabled on an ADC.
        if phy == AdcPhy::Three {
            process_output_channel::spawn(OutputChannelIdx::Zero).unwrap();
            process_output_channel::spawn(OutputChannelIdx::One).unwrap();
            process_output_channel::spawn(OutputChannelIdx::Two).unwrap();
            process_output_channel::spawn(OutputChannelIdx::Three).unwrap();
        }
    }

    #[task(priority = 3, binds = EXTI15_10, local=[adc_sm])]
    fn adc_readout(c: adc_readout::Context) {
        let (phy, ch, adc_code) = c.local.adc_sm.handle_interrupt();
        convert_adc_code::spawn(phy, ch, adc_code).unwrap();
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
