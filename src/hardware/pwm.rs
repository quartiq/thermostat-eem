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
    Channel, R_SENSE, VREF_TEC,
};

/// TEC limit types
///
/// Voltage - Upper and lower voltage limit
/// PositiveCurrent - Upper current limit
/// NegativeCurrent - Lower current limit
pub enum Limit {
    Voltage(Channel),
    PositiveCurrent(Channel),
    NegativeCurrent(Channel),
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
#[allow(clippy::type_complexity)]
pub struct PwmPins {
    pub voltage: (
        PE9<Alternate<AF1>>,
        PE11<Alternate<AF1>>,
        PE13<Alternate<AF1>>,
        PE14<Alternate<AF1>>,
    ),
    pub positive_current: (
        PD12<Alternate<AF2>>,
        PD13<Alternate<AF2>>,
        PD14<Alternate<AF2>>,
        PD15<Alternate<AF2>>,
    ),
    pub negative_current: (
        PC6<Alternate<AF2>>,
        PB5<Alternate<AF2>>,
        PC8<Alternate<AF2>>,
        PC9<Alternate<AF2>>,
    ),
}

type Pt0<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

/// PWM driver struct containing the PWM output pins.
#[allow(clippy::type_complexity)]
pub struct Pwm {
    voltage: (Pt0<TIM1, C1>, Pt0<TIM1, C2>, Pt0<TIM1, C3>, Pt1<TIM1, C4>),
    positive_current: (Pt1<TIM4, C1>, Pt1<TIM4, C2>, Pt1<TIM4, C3>, Pt1<TIM4, C4>),
    negative_current: (Pt1<TIM3, C1>, Pt1<TIM3, C2>, Pt1<TIM3, C3>, Pt1<TIM3, C4>),
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

        let mut voltage = tim.0.pwm(pins.voltage, F_PWM, tim_rec.0, clocks);
        let mut negative_current = tim.1.pwm(pins.negative_current, F_PWM, tim_rec.1, clocks);
        let mut positive_current = tim.2.pwm(pins.positive_current, F_PWM, tim_rec.2, clocks);
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
    /// * `ch` - Thermostat output channel
    /// * `limit` - TEC limit type
    pub fn set(&mut self, lim: Limit, val: f32) -> Result<(), Error> {
        // PWM constants
        const V_PWM: f32 = 3.3; // MCU PWM pin output high voltage

        /// Convert maximum current to relative pulsewidth for the (analog voltage)
        /// max output current inputs of the TEC driver.
        pub fn i_to_pwm(i: f32) -> f32 {
            i * ((VREF_TEC * R_SENSE) / (V_PWM * 0.15))
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
        match lim {
            Limit::Voltage(Channel::Ch0) => set_pwm(&mut self.voltage.0, v_to_pwm(val)),
            Limit::Voltage(Channel::Ch1) => set_pwm(&mut self.voltage.1, v_to_pwm(val)),
            Limit::Voltage(Channel::Ch2) => set_pwm(&mut self.voltage.2, v_to_pwm(val)),
            Limit::Voltage(Channel::Ch3) => set_pwm(&mut self.voltage.3, v_to_pwm(val)),
            Limit::PositiveCurrent(Channel::Ch0) => {
                set_pwm(&mut self.positive_current.0, i_to_pwm(val))
            }
            Limit::PositiveCurrent(Channel::Ch1) => {
                set_pwm(&mut self.positive_current.1, i_to_pwm(val))
            }
            Limit::PositiveCurrent(Channel::Ch2) => {
                set_pwm(&mut self.positive_current.2, i_to_pwm(val))
            }
            Limit::PositiveCurrent(Channel::Ch3) => {
                set_pwm(&mut self.positive_current.3, i_to_pwm(val))
            }
            Limit::NegativeCurrent(Channel::Ch0) => {
                set_pwm(&mut self.negative_current.0, i_to_pwm(-val))
            }
            Limit::NegativeCurrent(Channel::Ch1) => {
                set_pwm(&mut self.negative_current.1, i_to_pwm(-val))
            }
            Limit::NegativeCurrent(Channel::Ch2) => {
                set_pwm(&mut self.negative_current.2, i_to_pwm(-val))
            }
            Limit::NegativeCurrent(Channel::Ch3) => {
                set_pwm(&mut self.negative_current.3, i_to_pwm(-val))
            }
        }
    }
}
