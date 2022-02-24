use super::hal::{
    adc,
    gpio::{gpioa::*, gpiob::*, gpioc::*, gpiof::*, Analog},
    hal::blocking::delay::DelayUs,
    prelude::*,
    rcc::{rec, CoreClocks},
    stm32::{ADC1, ADC2, ADC3},
};
// use embedded_hal::blocking::delay::DelayUs;

pub enum Channel {
    Zero,
    One,
    Two,
    Three,
}

pub enum Supply {
    P5v,
    P12v,
    P3v,
    I12v,
}

pub enum IAdc {
    Supply(Supply),
    TecU(Channel),
    TecI(Channel),
}

pub struct AdcInternal {
    adc1: adc::Adc<ADC1, adc::Enabled>,
    adc2: adc::Adc<ADC2, adc::Enabled>,
    adc3: adc::Adc<ADC3, adc::Enabled>,
    p5v: PC0<Analog>,
    p12v: PC2<Analog>,
    p3v: PF7<Analog>,
    i12v: PF8<Analog>,
    tecu0: PC3<Analog>,
    tecu1: PA0<Analog>,
    tecu2: PA3<Analog>,
    tecu3: PA4<Analog>,
    teci0: PA5<Analog>,
    teci1: PA6<Analog>,
    teci2: PB0<Analog>,
    teci3: PB1<Analog>,
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
        tecu0: PC3<Analog>,
        tecu1: PA0<Analog>,
        tecu2: PA3<Analog>,
        tecu3: PA4<Analog>,
        teci0: PA5<Analog>,
        teci1: PA6<Analog>,
        teci2: PB0<Analog>,
        teci3: PB1<Analog>,
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
            tecu0,
            tecu1,
            tecu2,
            tecu3,
            teci0,
            teci1,
            teci2,
            teci3,
        }
    }

    pub fn read(&mut self, iadc: IAdc) -> u32 {
        match iadc {
            IAdc::Supply(Supply::P5v) => self.adc1.read(&mut self.p5v).unwrap(),
            IAdc::Supply(Supply::P12v) => self.adc1.read(&mut self.p12v).unwrap(),
            IAdc::Supply(Supply::P3v) => self.adc3.read(&mut self.p3v).unwrap(),
            IAdc::Supply(Supply::I12v) => self.adc3.read(&mut self.i12v).unwrap(),
            IAdc::TecU(Channel::Zero) => self.adc1.read(&mut self.tecu0).unwrap(),
            IAdc::TecU(Channel::One) => self.adc1.read(&mut self.tecu1).unwrap(),
            IAdc::TecU(Channel::Two) => self.adc1.read(&mut self.tecu2).unwrap(),
            IAdc::TecU(Channel::Three) => self.adc1.read(&mut self.tecu3).unwrap(),
            IAdc::TecI(Channel::Zero) => self.adc1.read(&mut self.teci0).unwrap(),
            IAdc::TecI(Channel::One) => self.adc1.read(&mut self.teci1).unwrap(),
            IAdc::TecI(Channel::Two) => self.adc1.read(&mut self.teci2).unwrap(),
            IAdc::TecI(Channel::Three) => self.adc1.read(&mut self.teci3).unwrap(),
        }
    }
}
