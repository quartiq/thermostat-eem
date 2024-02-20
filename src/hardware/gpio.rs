use strum::IntoEnumIterator;

use super::hal::{
    gpio::{gpiod::*, gpioe::*, gpiof::*, gpiog::*, ErasedPin, Input, Output, PushPull},
    hal::digital::v2::PinState,
};
use crate::net::serde::Serialize;

use super::OutputChannelIdx;

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

#[derive(Copy, Clone, Debug, Serialize)]
pub enum PoePower {
    /// No Power over Ethernet detected
    Absent,
    /// 802.3af (12.95 W) Power over Ethernet present
    Low,
    /// 802.3at (25.5 W) Power over Ethernet present
    High,
}

impl Default for PoePower {
    fn default() -> Self {
        Self::Absent
    }
}

/// TEC driver PWM frequency setting
#[derive(Copy, Clone, Debug)]
pub enum TecFrequency {
    /// Low frequency (~500 kHz)
    Low,
    /// High frequency (~1 MHz)
    High,
}

impl From<TecFrequency> for PinState {
    fn from(other: TecFrequency) -> PinState {
        match other {
            TecFrequency::Low => PinState::Low,
            TecFrequency::High => PinState::High,
        }
    }
}

#[derive(Copy, Clone, Debug, strum::EnumIter)]
#[repr(usize)]
pub enum Led {
    Led0 = 0,
    Led1 = 1,
    Led2 = 2,
    Led3 = 3,
    Led4 = 4,
    Led5 = 5,
    Led6 = 6,
    Led7 = 7,
}

// Channel enabled indicator LEDs. First set of four from top to bottom.
impl From<OutputChannelIdx> for Led {
    fn from(other: OutputChannelIdx) -> Led {
        match other {
            OutputChannelIdx::Zero => Led::Led0,
            OutputChannelIdx::One => Led::Led1,
            OutputChannelIdx::Two => Led::Led2,
            OutputChannelIdx::Three => Led::Led3,
        }
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
        for i in Led::iter() {
            self.set_led(i, State::Deassert);
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
        self.led[led as usize].set_state(PinState::from(state));
    }

    pub fn hwrev(&self) -> u8 {
        self.hwrev
            .iter()
            .enumerate()
            .map(|(i, p)| (p.is_high() as u8) << i)
            .sum()
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
