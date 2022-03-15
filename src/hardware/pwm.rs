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
    Channel, R_SENSE, V_PWM,
};

// PWM freqency. 20kHz is ~80dB down with the installed second order 160Hz lowpass.
const F_PWM: KiloHertz = KiloHertz(20);

/// Convert maximum current to relative pulsewidth for the (analog voltage)
/// max output current inputs of the TEC driver.
pub fn i_to_pwm(i: f32) -> f32 {
    i * ((10.0 * R_SENSE) / V_PWM)
}

/// Convert maximum voltage to relative pulsewidth for the (analog voltage)
/// max output voltage of the TEC driver.
pub fn v_to_pwm(v: f32) -> f32 {
    v / (4.0 * V_PWM)
}

/// TEC limit types
///
/// LimV - Upper and lower voltage limit
/// LimIUp - Upper current limit
/// LimILow - Lower current limit
pub enum Limit {
    LimV,
    LimIUp,
    LimILow,
}

/// Pwm pins.
///
/// lim_v<n>_pin - voltage limit pin
/// lim_i_up<n>_pin - upper current limit pin
/// lim_i_low<n>_pin - lower current limit pin
/// * <n> specifies Thermostat output channel
pub struct PwmPins {
    pub lim_v0_pin: PE9<Alternate<AF1>>,
    pub lim_v1_pin: PE11<Alternate<AF1>>,
    pub lim_v2_pin: PE13<Alternate<AF1>>,
    pub lim_v3_pin: PE14<Alternate<AF1>>,
    pub lim_i_up0_pin: PD12<Alternate<AF2>>,
    pub lim_i_up1_pin: PD13<Alternate<AF2>>,
    pub lim_i_up2_pin: PD14<Alternate<AF2>>,
    pub lim_i_up3_pin: PD15<Alternate<AF2>>,
    pub lim_i_low0_pin: PC6<Alternate<AF2>>,
    pub lim_i_low1_pin: PB5<Alternate<AF2>>,
    pub lim_i_low2_pin: PC8<Alternate<AF2>>,
    pub lim_i_low3_pin: PC9<Alternate<AF2>>,
}

type Pt0<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

/// PWM driver struct containing the PWM output pins.
pub struct Pwm {
    lim_v0: Pt0<TIM1, C1>,
    lim_v1: Pt0<TIM1, C2>,
    lim_v2: Pt0<TIM1, C3>,
    lim_v3: Pt1<TIM1, C4>,
    lim_i_up0: Pt1<TIM4, C1>,
    lim_i_up1: Pt1<TIM4, C2>,
    lim_i_up2: Pt1<TIM4, C3>,
    lim_i_up3: Pt1<TIM4, C4>,
    lim_i_low0: Pt1<TIM3, C1>,
    lim_i_low1: Pt1<TIM3, C2>,
    lim_i_low2: Pt1<TIM3, C3>,
    lim_i_low3: Pt1<TIM3, C4>,
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
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }

        let (mut lim_v0, mut lim_v1, mut lim_v2, mut lim_v3) = tim.0.pwm(
            (
                pins.lim_v0_pin,
                pins.lim_v1_pin,
                pins.lim_v2_pin,
                pins.lim_v3_pin,
            ),
            F_PWM,
            tim_rec.0,
            clocks,
        );
        init_pwm_pin(&mut lim_v0);
        init_pwm_pin(&mut lim_v1);
        init_pwm_pin(&mut lim_v2);
        init_pwm_pin(&mut lim_v3);

        let (mut lim_i_low0, mut lim_i_low1, mut lim_i_low2, mut lim_i_low3) = tim.1.pwm(
            (
                pins.lim_i_low0_pin,
                pins.lim_i_low1_pin,
                pins.lim_i_low2_pin,
                pins.lim_i_low3_pin,
            ),
            F_PWM,
            tim_rec.1,
            clocks,
        );
        init_pwm_pin(&mut lim_i_low0);
        init_pwm_pin(&mut lim_i_low1);
        init_pwm_pin(&mut lim_i_low2);
        init_pwm_pin(&mut lim_i_low3);

        let (mut lim_i_up0, mut lim_i_up1, mut lim_i_up2, mut lim_i_up3) = tim.2.pwm(
            (
                pins.lim_i_up0_pin,
                pins.lim_i_up1_pin,
                pins.lim_i_up2_pin,
                pins.lim_i_up3_pin,
            ),
            F_PWM,
            tim_rec.2,
            clocks,
        );
        init_pwm_pin(&mut lim_i_up0);
        init_pwm_pin(&mut lim_i_up1);
        init_pwm_pin(&mut lim_i_up2);
        init_pwm_pin(&mut lim_i_up3);

        Pwm {
            lim_v0,
            lim_v1,
            lim_v2,
            lim_v3,
            lim_i_up0,
            lim_i_up1,
            lim_i_up2,
            lim_i_up3,
            lim_i_low0,
            lim_i_low1,
            lim_i_low2,
            lim_i_low3,
        }
    }

    /// Set PWM TEC limits.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `limit` - TEC limit type
    pub fn set(&mut self, ch: Channel, lim: Limit, val: f32) {
        fn set_pwm<P: PwmPin<Duty = u16>>(pin: &mut P, duty: f32) {
            let max = pin.get_max_duty();
            let value = ((duty * (max as f32)) as u16).min(max);
            pin.set_duty(value);
        }
        match (ch, lim) {
            (Channel::Ch0, Limit::LimV) => set_pwm(&mut self.lim_v0, v_to_pwm(val)),
            (Channel::Ch1, Limit::LimV) => set_pwm(&mut self.lim_v1, v_to_pwm(val)),
            (Channel::Ch2, Limit::LimV) => set_pwm(&mut self.lim_v2, v_to_pwm(val)),
            (Channel::Ch3, Limit::LimV) => set_pwm(&mut self.lim_v3, v_to_pwm(val)),
            (Channel::Ch0, Limit::LimIUp) => set_pwm(&mut self.lim_i_up0, i_to_pwm(val)),
            (Channel::Ch1, Limit::LimIUp) => set_pwm(&mut self.lim_i_up1, i_to_pwm(val)),
            (Channel::Ch2, Limit::LimIUp) => set_pwm(&mut self.lim_i_up2, i_to_pwm(val)),
            (Channel::Ch3, Limit::LimIUp) => set_pwm(&mut self.lim_i_up3, i_to_pwm(val)),
            (Channel::Ch0, Limit::LimILow) => set_pwm(&mut self.lim_i_low0, i_to_pwm(val)),
            (Channel::Ch1, Limit::LimILow) => set_pwm(&mut self.lim_i_low1, i_to_pwm(val)),
            (Channel::Ch2, Limit::LimILow) => set_pwm(&mut self.lim_i_low2, i_to_pwm(val)),
            (Channel::Ch3, Limit::LimILow) => set_pwm(&mut self.lim_i_low3, i_to_pwm(val)),
        }
    }
}
