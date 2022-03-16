///! Thermostat TEC driver IC voltage/current limits PWM driver.
///!
///! The Thermostat TEC driver ICs feature current limits controlled by an analog voltage input.
///! This voltage is controlled by the MCU using low-pass filtered PWM outputs.
///!
use super::{
    hal::{
        gpio::{gpiob::*, gpioc::*, gpiod::*, gpioe::*, Alternate, AF1, AF2},
        hal::PwmPin,
        prelude::*,
        pwm::{ActiveHigh, ComplementaryDisabled, ComplementaryImpossible, C1, C2, C3, C4},
        rcc::{rec, CoreClocks},
        stm32::{TIM1, TIM3, TIM4},
        time::KiloHertz,
    },
    Channel,
};

/// TEC limit types
///
/// Voltage - Upper and lower voltage limit
/// PositiveCurrent - Upper current limit
/// NegativeCurrent - Lower current limit
pub enum Limit {
    Voltage,
    PositiveCurrent,
    NegativeCurrent,
}

/// PWM value out of bounds error.
#[derive(Debug)]
pub enum Error {
    Bounds,
}

/// Pwm pins.
///
/// voltage<n> - voltage limit pin
/// positive_current<n> - positive current limit pin
/// negative_current<n> - negative current limit pin
/// * <n> specifies Thermostat output channel
pub struct PwmPins {
    pub voltage0: PE9<Alternate<AF1>>,
    pub voltage1: PE11<Alternate<AF1>>,
    pub voltage2: PE13<Alternate<AF1>>,
    pub voltage3: PE14<Alternate<AF1>>,
    pub positive_current0: PD12<Alternate<AF2>>,
    pub positive_current1: PD13<Alternate<AF2>>,
    pub positive_current2: PD14<Alternate<AF2>>,
    pub positive_current3: PD15<Alternate<AF2>>,
    pub negative_current0: PC6<Alternate<AF2>>,
    pub negative_current1: PB5<Alternate<AF2>>,
    pub negative_current2: PC8<Alternate<AF2>>,
    pub negative_current3: PC9<Alternate<AF2>>,
}

type Pt0<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

/// PWM driver struct containing the PWM output pins.
pub struct Pwm {
    voltage0: Pt0<TIM1, C1>,
    voltage1: Pt0<TIM1, C2>,
    voltage2: Pt0<TIM1, C3>,
    voltage3: Pt1<TIM1, C4>,
    positive_current0: Pt1<TIM4, C1>,
    positive_current1: Pt1<TIM4, C2>,
    positive_current2: Pt1<TIM4, C3>,
    positive_current3: Pt1<TIM4, C4>,
    negative_current0: Pt1<TIM3, C1>,
    negative_current1: Pt1<TIM3, C2>,
    negative_current2: Pt1<TIM3, C3>,
    negative_current3: Pt1<TIM3, C4>,
}

impl Pwm {
    /// Construct a new PWM driver for all Thermostat output channels.
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
        const F_PWM: KiloHertz = KiloHertz(20);

        let (mut voltage0, mut voltage1, mut voltage2, mut voltage3) = tim.0.pwm(
            (pins.voltage0, pins.voltage1, pins.voltage2, pins.voltage3),
            F_PWM,
            tim_rec.0,
            clocks,
        );
        let (
            mut negative_current0,
            mut negative_current1,
            mut negative_current2,
            mut negative_current3,
        ) = tim.1.pwm(
            (
                pins.negative_current0,
                pins.negative_current1,
                pins.negative_current2,
                pins.negative_current3,
            ),
            F_PWM,
            tim_rec.1,
            clocks,
        );
        let (
            mut positive_current0,
            mut positive_current1,
            mut positive_current2,
            mut positive_current3,
        ) = tim.2.pwm(
            (
                pins.positive_current0,
                pins.positive_current1,
                pins.positive_current2,
                pins.positive_current3,
            ),
            F_PWM,
            tim_rec.2,
            clocks,
        );
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }
        init_pwm_pin(&mut voltage0);
        init_pwm_pin(&mut voltage1);
        init_pwm_pin(&mut voltage2);
        init_pwm_pin(&mut voltage3);
        init_pwm_pin(&mut negative_current0);
        init_pwm_pin(&mut negative_current1);
        init_pwm_pin(&mut negative_current2);
        init_pwm_pin(&mut negative_current3);
        init_pwm_pin(&mut positive_current0);
        init_pwm_pin(&mut positive_current1);
        init_pwm_pin(&mut positive_current2);
        init_pwm_pin(&mut positive_current3);

        Pwm {
            voltage0,
            voltage1,
            voltage2,
            voltage3,
            positive_current0,
            positive_current1,
            positive_current2,
            positive_current3,
            negative_current0,
            negative_current1,
            negative_current2,
            negative_current3,
        }
    }

    /// Set PWM TEC limits.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `limit` - TEC limit type
    pub fn set(&mut self, ch: Channel, lim: Limit, val: f32) -> Result<(), Error> {
        // PWM constants
        const R_SENSE: f32 = 0.05; // TEC current sense resistor
        const V_PWM: f32 = 3.3; // MCU PWM pin output high voltage

        /// Convert maximum current to relative pulsewidth for the (analog voltage)
        /// max output current inputs of the TEC driver.
        pub fn i_to_pwm(i: f32) -> f32 {
            i * ((10.0 * R_SENSE) / V_PWM)
        }

        /// Convert maximum voltage to relative pulsewidth for the (analog voltage)
        /// max output voltage of the TEC driver.
        pub fn v_to_pwm(v: f32) -> f32 {
            v * (1.0 / 4.0 / V_PWM)
        }

        fn set_pwm<P: PwmPin<Duty = u16>>(pin: &mut P, duty: f32) -> Result<(), Error> {
            let max = pin.get_max_duty() as f32;
            let value = duty * max;
            if !(0.0..max).contains(&value) {
                return Err(Error::Bounds);
            }
            pin.set_duty(value as u16);
            Ok(())
        }
        match (ch, lim) {
            (Channel::Ch0, Limit::Voltage) => set_pwm(&mut self.voltage0, v_to_pwm(val)),
            (Channel::Ch1, Limit::Voltage) => set_pwm(&mut self.voltage1, v_to_pwm(val)),
            (Channel::Ch2, Limit::Voltage) => set_pwm(&mut self.voltage2, v_to_pwm(val)),
            (Channel::Ch3, Limit::Voltage) => set_pwm(&mut self.voltage3, v_to_pwm(val)),
            (Channel::Ch0, Limit::PositiveCurrent) => {
                set_pwm(&mut self.positive_current0, i_to_pwm(val))
            }
            (Channel::Ch1, Limit::PositiveCurrent) => {
                set_pwm(&mut self.positive_current1, i_to_pwm(val))
            }
            (Channel::Ch2, Limit::PositiveCurrent) => {
                set_pwm(&mut self.positive_current2, i_to_pwm(val))
            }
            (Channel::Ch3, Limit::PositiveCurrent) => {
                set_pwm(&mut self.positive_current3, i_to_pwm(val))
            }
            (Channel::Ch0, Limit::NegativeCurrent) => {
                set_pwm(&mut self.negative_current0, i_to_pwm(-val))
            }
            (Channel::Ch1, Limit::NegativeCurrent) => {
                set_pwm(&mut self.negative_current1, i_to_pwm(-val))
            }
            (Channel::Ch2, Limit::NegativeCurrent) => {
                set_pwm(&mut self.negative_current2, i_to_pwm(-val))
            }
            (Channel::Ch3, Limit::NegativeCurrent) => {
                set_pwm(&mut self.negative_current3, i_to_pwm(-val))
            }
        }
    }
}
