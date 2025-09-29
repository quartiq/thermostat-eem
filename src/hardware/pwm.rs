//! Thermostat TEC driver IC voltage/current limits PWM driver.
//!
//! The Thermostat TEC driver ICs feature current limits controlled by an analog voltage input.
//! This voltage is controlled by the MCU using low-pass filtered PWM outputs.
//!
use super::hal::{
    gpio::{Alternate, gpiob::*, gpioc::*, gpiod::*, gpioe::*},
    hal_02::PwmPin,
    prelude::*,
    pwm::{C1, C2, C3, C4, ComplementaryDisabled, ComplementaryImpossible},
    rcc::{CoreClocks, rec},
    stm32::{TIM1, TIM3, TIM4},
    time::KiloHertz,
};
use crate::{
    OutputChannelIdx,
    convert::{R_SENSE, VREF_TEC},
};

/// TEC limit types
///
/// Voltage - Upper and lower voltage limit
/// PositiveCurrent - Upper current limit
/// NegativeCurrent - Lower current limit
pub enum Limit {
    Voltage(OutputChannelIdx),
    NegativeCurrent(OutputChannelIdx),
    PositiveCurrent(OutputChannelIdx),
}

/// PWM value out of bounds error.
#[derive(Debug)]
pub enum Error {
    Bounds,
}

/// Pwm pins.
///
/// `voltage.<n>` - voltage limit pin
/// `positive_current.<n>` - positive current limit pin
/// `negative_current.<n>` - negative current limit pin
/// * `<n>` specifies Thermostat output channel
#[allow(clippy::type_complexity)]
pub struct PwmPins {
    pub voltage: (
        PE9<Alternate<1>>,
        PE11<Alternate<1>>,
        PE13<Alternate<1>>,
        PE14<Alternate<1>>,
    ),
    pub negative_current: (
        PC6<Alternate<2>>,
        PB5<Alternate<2>>,
        PC8<Alternate<2>>,
        PC9<Alternate<2>>,
    ),
    pub positive_current: (
        PD12<Alternate<2>>,
        PD13<Alternate<2>>,
        PD14<Alternate<2>>,
        PD15<Alternate<2>>,
    ),
}

type Pt0<T, const S: u8> = super::hal::pwm::Pwm<T, S, ComplementaryDisabled>;
type Pt1<T, const S: u8> = super::hal::pwm::Pwm<T, S, ComplementaryImpossible>;

/// PWM driver struct containing the PWM output pins.
#[allow(clippy::type_complexity)]
pub struct Pwm {
    voltage: (Pt0<TIM1, C1>, Pt0<TIM1, C2>, Pt0<TIM1, C3>, Pt1<TIM1, C4>),
    positive_current:
        (Pt1<TIM4, C1>, Pt1<TIM4, C2>, Pt1<TIM4, C3>, Pt1<TIM4, C4>),
    negative_current:
        (Pt1<TIM3, C1>, Pt1<TIM3, C2>, Pt1<TIM3, C3>, Pt1<TIM3, C4>),
}

impl Pwm {
    // PWM constants
    const MAX_DUTY: u32 = 10000; // Maximum duty cycle valid for all channels.
    const V_PWM: f32 = 3.3; // MCU PWM pin output high voltage

    /// Construct a new PWM driver for all Thermostat output channel limits.
    ///
    /// # Args
    /// * `clocks` - Reference to CoreClocks
    /// * `tim_rec` - Peripheral Reset and Enable Control for the timer 1/2/4
    /// * `tim` - Timer 1/2/4 peripherals
    /// * `pins` - PWM output pins
    pub fn new(
        clocks: &CoreClocks,
        tim_rec: (rec::Tim1, rec::Tim3, rec::Tim4),
        tim: (TIM1, TIM3, TIM4),
        pins: PwmPins,
    ) -> Pwm {
        // PWM freqency. 20kHz is ~80dB down with the installed second order 160Hz lowpass.
        const F_PWM: KiloHertz = KiloHertz::kHz(20);

        let mut voltage =
            tim.0.pwm(pins.voltage, F_PWM.convert(), tim_rec.0, clocks);
        let mut negative_current = tim.1.pwm(
            pins.negative_current,
            F_PWM.convert(),
            tim_rec.1,
            clocks,
        );
        let mut positive_current = tim.2.pwm(
            pins.positive_current,
            F_PWM.convert(),
            tim_rec.2,
            clocks,
        );
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }
        init_pwm_pin(&mut voltage.0);
        init_pwm_pin(&mut voltage.1);
        init_pwm_pin(&mut voltage.2);
        init_pwm_pin(&mut voltage.3);
        init_pwm_pin(&mut negative_current.0);
        init_pwm_pin(&mut negative_current.1);
        init_pwm_pin(&mut negative_current.2);
        init_pwm_pin(&mut negative_current.3);
        init_pwm_pin(&mut positive_current.0);
        init_pwm_pin(&mut positive_current.1);
        init_pwm_pin(&mut positive_current.2);
        init_pwm_pin(&mut positive_current.3);

        Pwm {
            voltage,
            negative_current,
            positive_current,
        }
    }

    /// Set PWM TEC limits.
    ///
    /// # Args
    /// * `lim` - TEC limit type
    /// * `val` - value to be set (in ampere/volt)
    pub fn set_limit(&mut self, lim: Limit, val: f32) -> Result<(), Error> {
        /// Convert maximum current to relative pulsewidth for the (analog voltage)
        /// max output current inputs of the TEC driver.
        pub fn current_limit_to_duty_cycle(i: f32) -> f32 {
            i * ((VREF_TEC * R_SENSE) / (Pwm::V_PWM * 0.15))
        }

        /// Convert maximum voltage to relative pulsewidth for the (analog voltage)
        /// max output voltage of the TEC driver.
        pub fn voltage_limit_to_duty_cycle(v: f32) -> f32 {
            v * (1.0 / 4.0 / Pwm::V_PWM)
        }

        fn set_pwm_channel<P: PwmPin<Duty = u16>>(
            channel: &mut P,
            duty: f32,
        ) -> Result<(), Error> {
            let value = (duty * Pwm::MAX_DUTY as f32) as i32;
            if !(0..=Pwm::MAX_DUTY as _).contains(&value) {
                return Err(Error::Bounds);
            }
            channel.set_duty(value as u16);
            Ok(())
        }
        match lim {
            Limit::Voltage(OutputChannelIdx::Zero) => set_pwm_channel(
                &mut self.voltage.0,
                voltage_limit_to_duty_cycle(val),
            ),
            Limit::Voltage(OutputChannelIdx::One) => set_pwm_channel(
                &mut self.voltage.1,
                voltage_limit_to_duty_cycle(val),
            ),
            Limit::Voltage(OutputChannelIdx::Two) => set_pwm_channel(
                &mut self.voltage.2,
                voltage_limit_to_duty_cycle(val),
            ),
            Limit::Voltage(OutputChannelIdx::Three) => set_pwm_channel(
                &mut self.voltage.3,
                voltage_limit_to_duty_cycle(val),
            ),
            Limit::NegativeCurrent(OutputChannelIdx::Zero) => set_pwm_channel(
                &mut self.negative_current.0,
                current_limit_to_duty_cycle(-val),
            ),
            Limit::NegativeCurrent(OutputChannelIdx::One) => set_pwm_channel(
                &mut self.negative_current.1,
                current_limit_to_duty_cycle(-val),
            ),
            Limit::NegativeCurrent(OutputChannelIdx::Two) => set_pwm_channel(
                &mut self.negative_current.2,
                current_limit_to_duty_cycle(-val),
            ),
            Limit::NegativeCurrent(OutputChannelIdx::Three) => set_pwm_channel(
                &mut self.negative_current.3,
                current_limit_to_duty_cycle(-val),
            ),
            Limit::PositiveCurrent(OutputChannelIdx::Zero) => set_pwm_channel(
                &mut self.positive_current.0,
                current_limit_to_duty_cycle(val),
            ),
            Limit::PositiveCurrent(OutputChannelIdx::One) => set_pwm_channel(
                &mut self.positive_current.1,
                current_limit_to_duty_cycle(val),
            ),
            Limit::PositiveCurrent(OutputChannelIdx::Two) => set_pwm_channel(
                &mut self.positive_current.2,
                current_limit_to_duty_cycle(val),
            ),
            Limit::PositiveCurrent(OutputChannelIdx::Three) => set_pwm_channel(
                &mut self.positive_current.3,
                current_limit_to_duty_cycle(val),
            ),
        }
    }
}
