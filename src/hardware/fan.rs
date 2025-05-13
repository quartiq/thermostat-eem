use super::hal::{
    gpio::{gpiob::*, gpioc::*, Alternate},
    hal_02::PwmPin,
    prelude::*,
    pwm::Pwm,
    pwm::{ComplementaryDisabled, C2},
    // rcc::ResetEnable,
    rcc::{rec, CoreClocks},
    stm32::{TIM2, TIM8},
    time::KiloHertz,
};

pub struct FanPins {
    pub tacho: PB10<Alternate<1>>,
    pub pwm: PC7<Alternate<3>>,
}

pub enum Error {
    Bounds,
}

pub struct Fan {
    pwm: Pwm<TIM8, C2, ComplementaryDisabled>,
    // tacho:
}

impl Fan {
    pub fn new(
        clocks: &CoreClocks,
        tim_rec: (rec::Tim2, rec::Tim8),
        tim: (TIM2, TIM8),
        pins: FanPins,
    ) -> Fan {
        const F_PWM: KiloHertz = KiloHertz::kHz(20);
        let mut pwm = tim.1.pwm(pins.pwm, F_PWM.convert(), tim_rec.1, clocks);
        pwm.set_duty(0);
        pwm.enable();

        // const F_TACHO: Hertz = Hertz(2);
        // let mut _tacho = tim.0.pwm(pins.tacho, F_TACHO, tim_rec.0, clocks);
        // let tim2 = unsafe { &*TIM2::PTR };
        // tim2.ccer.modify(|_, w| w.cc3e())

        Fan {
            pwm,
            // tacho,
        }
    }

    pub fn set_duty(&mut self, duty: f32) -> Result<i32, Error> {
        let max = self.pwm.get_max_duty() as i32;
        let code = (duty * max as f32) as i32;
        if !(0..max).contains(&code) {
            return Err(Error::Bounds);
        }
        self.pwm.set_duty(code as u16);
        Ok(code)
    }
}
