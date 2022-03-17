use super::hal::{
    gpio::{gpiog::*, Output, PushPull},
    hal::digital::v2::OutputPin,
};

use super::Channel;

#[allow(clippy::type_complexity)]
pub struct GpioPins {
    pub shdn: (
        PG4<Output<PushPull>>,
        PG5<Output<PushPull>>,
        PG6<Output<PushPull>>,
        PG7<Output<PushPull>>,
    ),
}

/// GPIO pins.
///
/// shutdown - TEC driver shutdown signals
#[allow(clippy::type_complexity)]
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
            gpio.set_shutdown(ch, true);
        }
        gpio
    }

    /// Set or reset the shutdown pin of an output channel.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `shutdown` - TEC driver shutdown. True to set shutdown mode, false to enable the driver.
    pub fn set_shutdown(&mut self, ch: Channel, shutdown: bool) {
        match (ch, shutdown) {
            (Channel::Ch0, false) => self.pins.shdn.0.set_high().unwrap(),
            (Channel::Ch1, false) => self.pins.shdn.1.set_high().unwrap(),
            (Channel::Ch2, false) => self.pins.shdn.2.set_high().unwrap(),
            (Channel::Ch3, false) => self.pins.shdn.3.set_high().unwrap(),
            (Channel::Ch0, true) => self.pins.shdn.0.set_low().unwrap(),
            (Channel::Ch1, true) => self.pins.shdn.1.set_low().unwrap(),
            (Channel::Ch2, true) => self.pins.shdn.2.set_low().unwrap(),
            (Channel::Ch3, true) => self.pins.shdn.3.set_low().unwrap(),
        }
    }
}
