use super::hal::{
    gpio::{gpioe::*, gpiog::*, Output, PushPull},
    hal::digital::v2::{OutputPin, PinState},
};

use super::Channel;

#[allow(clippy::type_complexity)]
pub struct GpioPins {
    // Front panel LEDs
    pub led: (
        PG9<Output<PushPull>>,
        PG10<Output<PushPull>>,
        PE8<Output<PushPull>>,
        PE10<Output<PushPull>>,
        PE12<Output<PushPull>>,
        PG15<Output<PushPull>>,
        PE15<Output<PushPull>>,
        PG8<Output<PushPull>>,
    ),
    pub shdn: (
        PG4<Output<PushPull>>,
        PG5<Output<PushPull>>,
        PG6<Output<PushPull>>,
        PG7<Output<PushPull>>,
    ),
}

pub enum State {
    Assert,
    Deassert,
}

impl From<bool> for State {
    fn from(other: bool) -> State {
        match other {
            true => State::Assert,
            false => State::Deassert,
        }
    }
}

impl From<State> for PinState {
    fn from(other: State) -> PinState {
        match other {
            State::Assert => PinState::High,
            State::Deassert => PinState::Low,
        }
    }
}

/// GPIO pins.
///
/// shutdown - TEC driver shutdown signals
pub struct Gpio {
    pins: GpioPins,
}

impl Gpio {
    /// Construct a new GPIO driver for all Thermostat output channels.
    ///
    /// # Args
    /// * `shdn_pins` - SHDN pin tuple.
    pub fn new(pins: GpioPins) -> Self {
        let mut gpio = Gpio { pins };
        for i in 0..4 {
            let ch = Channel::try_from(i).unwrap();
            gpio.set_shutdown(ch, State::Assert);
        }
        for i in 0..8 {
            gpio.set_led(i, State::Deassert);
        }
        gpio
    }

    /// Set or reset the shutdown pin of an output channel.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `shutdown` - TEC driver shutdown. True to set shutdown mode, false to enable the driver.
    pub fn set_shutdown(&mut self, ch: Channel, shutdown: State) {
        let s = !PinState::from(shutdown);
        match ch {
            Channel::Ch0 => self.pins.shdn.0.set_state(s),
            Channel::Ch1 => self.pins.shdn.1.set_state(s),
            Channel::Ch2 => self.pins.shdn.2.set_state(s),
            Channel::Ch3 => self.pins.shdn.3.set_state(s),
        }
        .unwrap()
    }

    pub fn set_led(&mut self, index: i32, state: State) {
        let s = PinState::from(state);
        match index {
            0 => self.pins.led.0.set_state(s),
            1 => self.pins.led.1.set_state(s),
            2 => self.pins.led.2.set_state(s),
            3 => self.pins.led.3.set_state(s),
            4 => self.pins.led.4.set_state(s),
            5 => self.pins.led.5.set_state(s),
            6 => self.pins.led.6.set_state(s),
            7 => self.pins.led.7.set_state(s),
            _ => panic!(),
        }
        .unwrap()
    }
}
