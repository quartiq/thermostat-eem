///! Thermostat PWM driver
///!
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

const F_PWM: KiloHertz = KiloHertz(20); // PWM freqency. 20kHz is ~80dB down with the installed second order 160Hz lowpass

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

pub enum Limit {
    MaxV,
    MaxIPos,
    MaxINeg,
}

pub struct PwmPins {
    pub max_v0_pin: PE9<Alternate<AF1>>,
    pub max_v1_pin: PE11<Alternate<AF1>>,
    pub max_v2_pin: PE13<Alternate<AF1>>,
    pub max_v3_pin: PE14<Alternate<AF1>>,
    pub max_i_pos0_pin: PD12<Alternate<AF2>>,
    pub max_i_pos1_pin: PD13<Alternate<AF2>>,
    pub max_i_pos2_pin: PD14<Alternate<AF2>>,
    pub max_i_pos3_pin: PD15<Alternate<AF2>>,
    pub max_i_neg0_pin: PC6<Alternate<AF2>>,
    pub max_i_neg1_pin: PB5<Alternate<AF2>>,
    pub max_i_neg2_pin: PC8<Alternate<AF2>>,
    pub max_i_neg3_pin: PC9<Alternate<AF2>>,
}

type Pt0<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = super::hal::pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

pub struct Pwm {
    max_v0: Pt0<TIM1, C1>,
    max_v1: Pt0<TIM1, C2>,
    max_v2: Pt0<TIM1, C3>,
    max_v3: Pt1<TIM1, C4>,
    max_i_pos0: Pt1<TIM4, C1>,
    max_i_pos1: Pt1<TIM4, C2>,
    max_i_pos2: Pt1<TIM4, C3>,
    max_i_pos3: Pt1<TIM4, C4>,
    max_i_neg0: Pt1<TIM3, C1>,
    max_i_neg1: Pt1<TIM3, C2>,
    max_i_neg2: Pt1<TIM3, C3>,
    max_i_neg3: Pt1<TIM3, C4>,
}

impl Pwm {
    pub fn new(
        clocks: &CoreClocks,
        tim_rcc: (rec::Tim1, rec::Tim3, rec::Tim4),
        tim: (TIM1, TIM3, TIM4),
        pins: PwmPins,
    ) -> Pwm {
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }

        let (mut max_v0, mut max_v1, mut max_v2, mut max_v3) = tim.0.pwm(
            (
                pins.max_v0_pin,
                pins.max_v1_pin,
                pins.max_v2_pin,
                pins.max_v3_pin,
            ),
            F_PWM,
            tim_rcc.0,
            clocks,
        );
        init_pwm_pin(&mut max_v0);
        init_pwm_pin(&mut max_v1);
        init_pwm_pin(&mut max_v2);
        init_pwm_pin(&mut max_v3);

        let (mut max_i_pos0, mut max_i_pos1, mut max_i_pos2, mut max_i_pos3) = tim.2.pwm(
            (
                pins.max_i_pos0_pin,
                pins.max_i_pos1_pin,
                pins.max_i_pos2_pin,
                pins.max_i_pos3_pin,
            ),
            F_PWM,
            tim_rcc.2,
            clocks,
        );
        init_pwm_pin(&mut max_i_pos0);
        init_pwm_pin(&mut max_i_pos1);
        init_pwm_pin(&mut max_i_pos2);
        init_pwm_pin(&mut max_i_pos3);

        let (mut max_i_neg0, mut max_i_neg1, mut max_i_neg2, mut max_i_neg3) = tim.1.pwm(
            (
                pins.max_i_neg0_pin,
                pins.max_i_neg1_pin,
                pins.max_i_neg2_pin,
                pins.max_i_neg3_pin,
            ),
            F_PWM,
            tim_rcc.1,
            clocks,
        );
        init_pwm_pin(&mut max_i_neg0);
        init_pwm_pin(&mut max_i_neg1);
        init_pwm_pin(&mut max_i_neg2);
        init_pwm_pin(&mut max_i_neg3);

        Pwm {
            max_v0,
            max_v1,
            max_v2,
            max_v3,
            max_i_pos0,
            max_i_pos1,
            max_i_pos2,
            max_i_pos3,
            max_i_neg0,
            max_i_neg1,
            max_i_neg2,
            max_i_neg3,
        }
    }

    /// Set PWM to limit current.
    pub fn set(&mut self, ch: Channel, lim: Limit, val: f32) {
        fn set_pwm<P: PwmPin<Duty = u16>>(pin: &mut P, duty: f32) {
            let max = pin.get_max_duty();
            let value = ((duty * (max as f32)) as u16).min(max);
            pin.set_duty(value);
        }
        match (ch, lim) {
            (Channel::Ch0, Limit::MaxV) => set_pwm(&mut self.max_v0, v_to_pwm(val)),
            (Channel::Ch1, Limit::MaxV) => set_pwm(&mut self.max_v1, v_to_pwm(val)),
            (Channel::Ch2, Limit::MaxV) => set_pwm(&mut self.max_v2, v_to_pwm(val)),
            (Channel::Ch3, Limit::MaxV) => set_pwm(&mut self.max_v3, v_to_pwm(val)),
            (Channel::Ch0, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos0, i_to_pwm(val)),
            (Channel::Ch1, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos1, i_to_pwm(val)),
            (Channel::Ch2, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos2, i_to_pwm(val)),
            (Channel::Ch3, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos3, i_to_pwm(val)),
            (Channel::Ch0, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg0, i_to_pwm(val)),
            (Channel::Ch1, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg1, i_to_pwm(val)),
            (Channel::Ch2, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg2, i_to_pwm(val)),
            (Channel::Ch3, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg3, i_to_pwm(val)),
        }
    }
}
