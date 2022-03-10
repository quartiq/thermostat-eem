use super::hal::{
    adc,
    gpio::{gpioa::*, gpiob::*, gpioc::*, gpiof::*, Analog},
    hal::blocking::delay::DelayUs,
    prelude::*,
    rcc::{rec, CoreClocks},
    stm32::{ADC1, ADC2, ADC3},
};

pub enum TecChannel {
    Zero,
    One,
    Two,
    Three,
}

pub enum Supply {
    P5v,
    P12v,
    P3v3,
    I12v,
}

const V_REF: f32 = 3.0; // ADC reference voltage
const DIV_P12V: f32 = 1.6 / (1.6 + 6.8); // Resistor divider 12V rail
const DIV_P5V: f32 = 6.8 / (10.0 + 6.8); // Resistor divider 5V rail
const DIV_P3V3: f32 = 6.8 / (1.6 + 6.8); // Resistor divider 3V3 rail
const CONF_I12V: f32 = 0.005 * (10000.0 / 100.0); // 12V current measurement resistor configuration for LT6106

pub type TecUPins = (PC3<Analog>, PA0<Analog>, PA3<Analog>, PA4<Analog>);
pub type TecIPins = (PA5<Analog>, PA6<Analog>, PB0<Analog>, PB1<Analog>);
pub type SupplyPins = (PC0<Analog>, PC2<Analog>, PF7<Analog>, PF8<Analog>);

pub struct AdcInternal {
    adc1: adc::Adc<ADC1, adc::Enabled>,
    adc3: adc::Adc<ADC3, adc::Enabled>,
    supply: SupplyPins,
    tecu: TecUPins,
    teci: TecIPins,
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
        supply: SupplyPins,
        tecu: TecUPins,
        teci: TecIPins,
    ) -> Self {
        // Setup ADC1 and ADC2
        let (adc1, _) = adc::adc12(adc1, adc2, delay, adc12_rcc, clocks);
        let adc3 = adc::Adc::adc3(adc3, delay, adc3_rcc, clocks);

        let mut adc1 = adc1.enable();
        adc1.set_resolution(adc::Resolution::SIXTEENBIT);

        let mut adc3 = adc3.enable();
        adc3.set_resolution(adc::Resolution::SIXTEENBIT);

        AdcInternal {
            adc1,
            adc3,
            supply,
            tecu,
            teci,
        }
    }

    pub fn read_tecu(&mut self, tecu: TecChannel) -> u32 {
        match tecu {
            TecChannel::Zero => self.adc1.read(&mut self.tecu.0).unwrap(),
            TecChannel::One => self.adc1.read(&mut self.tecu.1).unwrap(),
            TecChannel::Two => self.adc1.read(&mut self.tecu.2).unwrap(),
            TecChannel::Three => self.adc1.read(&mut self.tecu.3).unwrap(),
        }
    }

    pub fn read_teci(&mut self, teci: TecChannel) -> u32 {
        match teci {
            TecChannel::Zero => self.adc1.read(&mut self.teci.0).unwrap(),
            TecChannel::One => self.adc1.read(&mut self.teci.1).unwrap(),
            TecChannel::Two => self.adc1.read(&mut self.teci.2).unwrap(),
            TecChannel::Three => self.adc1.read(&mut self.teci.3).unwrap(),
        }
    }

    pub fn read_supply(&mut self, supply: Supply) -> u32 {
        match supply {
            Supply::P5v => self.adc1.read(&mut self.supply.0).unwrap(),
            Supply::P12v => self.adc1.read(&mut self.supply.1).unwrap(),
            Supply::P3v3 => self.adc3.read(&mut self.supply.2).unwrap(),
            Supply::I12v => self.adc3.read(&mut self.supply.3).unwrap(),
        }
    }

    pub fn read_p12v(&mut self) -> f32 {
        // reads the 12V rail voltage and returns the result in volts
        let factor = (V_REF / self.adc1.max_sample() as f32) / DIV_P12V;
        self.read_supply(Supply::P12v) as f32 * factor
    }

    pub fn read_p5v(&mut self) -> f32 {
        // reads the 5V rail voltage and returns the result in volts
        let factor = (V_REF / self.adc1.max_sample() as f32) / DIV_P5V;
        self.read_supply(Supply::P5v) as f32 * factor
    }

    pub fn read_p3v3(&mut self) -> f32 {
        // reads the 3.3V rail voltage and returns the result in volts
        let factor = (V_REF / self.adc1.max_sample() as f32) / DIV_P3V3;
        self.read_supply(Supply::P3v3) as f32 * factor
    }

    pub fn read_i12v(&mut self) -> f32 {
        // reads the 12V rail current and returns the result in amperes
        let factor = (V_REF / self.adc1.max_sample() as f32) / CONF_I12V;
        self.read_supply(Supply::I12v) as f32 * factor
    }
}
