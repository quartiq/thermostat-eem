use arbitrary_int::u3;
use strum::IntoEnumIterator;

use super::hal::{
    gpio::{ErasedPin, Input, Output, PushPull, gpiod::*, gpioe::*, gpiof::*, gpiog::*},
    hal_02::digital::v2::PinState,
};

use crate::{
    OutputChannelIdx,
    convert::{PoePower, TecFrequency},
};

pub struct Gpio {
    pub hwrev: [ErasedPin<Input>; 4],
    // Front panel LEDs
    pub led: [ErasedPin<Output>; 8],
    pub shdn: [ErasedPin<Output>; 4],
    pub poe_pwr: PF2<Input>,
    pub at_event: PE7<Input>,
    pub eem_pwr: PD0<Output<PushPull>>,
    pub tec_freq: PD2<Output<PushPull>>,
    pub overtemp: PG12<Input>,
}

#[derive(Copy, Clone, Debug)]
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

impl From<TecFrequency> for PinState {
    fn from(other: TecFrequency) -> PinState {
        match other {
            TecFrequency::Low => PinState::Low,
            TecFrequency::High => PinState::High,
        }
    }
}

pub type Led = u3;

// Channel enabled indicator LEDs. First set of four from top to bottom.
impl From<OutputChannelIdx> for Led {
    fn from(other: OutputChannelIdx) -> Led {
        Led::new(other as _)
    }
}

impl Gpio {
    /// Construct a new GPIO driver for all Thermostat QPIOs.
    /// - Sets all output channels into shutdown mode.
    /// - Sets all LEDs to off.
    /// - Sets EEM power to off.
    /// - Sets TEC frequency to TecFrequency::Low.
    ///
    /// # Args
    /// * `pins` - Thermostat GPIO pins.
    pub fn init(&mut self) {
        for i in OutputChannelIdx::iter() {
            self.set_shutdown(i, State::Assert);
        }
        for i in 0u8..8 {
            self.set_led(Led::new(i), State::Deassert);
        }
        self.set_eem_pwr(false);
        self.set_tec_frequency(TecFrequency::Low);
    }

    /// Set or reset the shutdown pin of an output channel.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `shutdown` - TEC driver shutdown. True to set shutdown mode, false to enable the driver.
    pub fn set_shutdown(&mut self, ch: OutputChannelIdx, shutdown: State) {
        self.shdn[ch as usize].set_state(!PinState::from(shutdown));
    }

    pub fn set_led(&mut self, led: Led, state: State) {
        self.led[led.value() as usize].set_state(PinState::from(state));
    }

    pub fn hwrev(&self) -> u8 {
        self.hwrev
            .iter()
            .enumerate()
            .map(|(i, p)| (p.is_high() as u8) << i)
            .sum::<u8>()
            .try_into()
            .unwrap()
    }

    pub fn hwrev_str(&self) -> &'static str {
        match self.hwrev() {
            0 => "v1.0",
            1 => "v1.1",
            _ => "unknown",
        }
    }

    pub fn poe(&self) -> PoePower {
        match (self.poe_pwr.is_high(), self.at_event.is_high()) {
            (false, _) => PoePower::Absent,
            (true, false) => PoePower::Low,
            (true, true) => PoePower::High,
        }
    }

    pub fn set_eem_pwr(&mut self, enabled: bool) {
        self.eem_pwr.set_state(enabled.into());
    }

    pub fn set_tec_frequency(&mut self, frequency: TecFrequency) {
        self.tec_freq.set_state(frequency.into());
    }

    pub fn overtemp(&self) -> bool {
        self.overtemp.is_low()
    }
}
