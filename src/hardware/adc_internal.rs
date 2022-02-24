use super::hal::{
    adc,
    delay::Delay,
    gpio::{gpioc::*, Analog, Input},
    pac,
    prelude::*,
    rcc::{rec, CoreClocks},
    stm32::{ADC1, ADC2, ADC3},
};
use cortex_m::peripheral::SYST;
use embedded_hal::blocking::delay::DelayUs;

pub struct AdcInternal {
    adc1: adc::Adc<ADC1, adc::Enabled>,
    adc2: adc::Adc<ADC2, adc::Enabled>,
    u5v: PC0<Analog>,
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
        u5v: PC0<Analog>,
    ) -> Self {
        // Setup ADC1 and ADC2
        let (adc1, adc2) = adc::adc12(adc1, adc2, delay, adc12_rcc, &clocks);

        let mut adc1 = adc1.enable();
        adc1.set_resolution(adc::Resolution::SIXTEENBIT);

        let mut adc2 = adc2.enable();
        adc2.set_resolution(adc::Resolution::SIXTEENBIT);

        AdcInternal { adc1, adc2, u5v }
    }

    pub fn read(mut self) -> u32 {
        self.adc1.read(&mut self.u5v).unwrap()
    }
}
