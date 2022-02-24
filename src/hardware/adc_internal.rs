use super::hal::{
    adc,
    delay::Delay,
    gpio::{gpioc::*, gpiof::*, Analog, Input},
    pac,
    prelude::*,
    rcc::{rec, CoreClocks},
    stm32::{ADC1, ADC2, ADC3},
};
use cortex_m::peripheral::SYST;
use embedded_hal::blocking::delay::DelayUs;

pub enum IAdc {
    P5v,
    P12v,
    P3v,
    I12v,
}

pub struct AdcInternal {
    adc1: adc::Adc<ADC1, adc::Enabled>,
    adc2: adc::Adc<ADC2, adc::Enabled>,
    adc3: adc::Adc<ADC3, adc::Enabled>,
    p5v: PC0<Analog>,
    p12v: PC2<Analog>,
    p3v: PF7<Analog>,
    i12v: PF8<Analog>,
}

impl AdcInternal {
    pub fn new(
        delay: &mut impl DelayUs<u8>,
        clocks: &CoreClocks,
        adc12_rcc: rec::Adc12,
        adc3_rcc: rec::Adc3,
        adc1: ADC1,
        adc2: ADC2,
        adc3: ADC3,
        p5v: PC0<Analog>,
        p12v: PC2<Analog>,
        p3v: PF7<Analog>,
        i12v: PF8<Analog>,
    ) -> Self {
        // Setup ADC1 and ADC2
        let (adc1, adc2) = adc::adc12(adc1, adc2, delay, adc12_rcc, &clocks);
        let adc3 = adc::Adc::adc3(adc3, delay, adc3_rcc, &clocks);

        let mut adc1 = adc1.enable();
        adc1.set_resolution(adc::Resolution::SIXTEENBIT);

        let mut adc2 = adc2.enable();
        adc2.set_resolution(adc::Resolution::SIXTEENBIT);

        let mut adc3 = adc3.enable();
        adc3.set_resolution(adc::Resolution::SIXTEENBIT);

        AdcInternal {
            adc1,
            adc2,
            adc3,
            p5v,
            p12v,
            p3v,
            i12v,
        }
    }

    pub fn read(&mut self, iadc: IAdc) -> u32 {
        match iadc {
            IAdc::P5v => self.adc1.read(&mut self.p5v).unwrap(),
            IAdc::P12v => self.adc1.read(&mut self.p12v).unwrap(),
            IAdc::P3v => self.adc3.read(&mut self.p3v).unwrap(),
            IAdc::I12v => self.adc3.read(&mut self.i12v).unwrap(),
        }
    }
}
