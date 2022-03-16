use super::hal::{
    adc,
    gpio::{gpioa::*, gpiob::*, gpioc::*, gpiof::*, Analog},
    hal::blocking::delay::DelayUs,
    prelude::*,
    rcc::{rec, CoreClocks},
    stm32::{ADC1, ADC2, ADC3},
};
use super::Channel;

const V_REF: f32 = 3.0; // ADC reference voltage

pub enum Supply {
    P3v3,
    P5v,
    P12v,
    I12v,
}

pub enum AdcChannel {
    OutputVoltage(Channel),
    OutputCurrent(Channel),
    Supply(Supply),
}

pub struct AdcPins {
    pub output_voltage: (PC3<Analog>, PA0<Analog>, PA3<Analog>, PA4<Analog>),
    pub output_current: (PA5<Analog>, PA6<Analog>, PB0<Analog>, PB1<Analog>),
    pub supply: (PC0<Analog>, PC2<Analog>, PF7<Analog>, PF8<Analog>),
}

pub struct AdcInternal {
    adc1: adc::Adc<ADC1, adc::Enabled>,
    adc3: adc::Adc<ADC3, adc::Enabled>,
    pins: AdcPins,
}

impl AdcInternal {
    pub fn new(
        delay: &mut impl DelayUs<u8>,
        clocks: &CoreClocks,
        adc_rcc: (rec::Adc12, rec::Adc3),
        adc: (ADC1, ADC2, ADC3),
        pins: AdcPins,
    ) -> Self {
        // Setup ADC1 and ADC2
        let (adc1, _) = adc::adc12(adc.0, adc.1, delay, adc_rcc.0, clocks);
        let adc3 = adc::Adc::adc3(adc.2, delay, adc_rcc.1, clocks);

        let mut adc1 = adc1.enable();
        adc1.set_resolution(adc::Resolution::SIXTEENBIT);

        let mut adc3 = adc3.enable();
        adc3.set_resolution(adc::Resolution::SIXTEENBIT);

        AdcInternal { adc1, adc3, pins }
    }

    pub fn read(&mut self, ch: AdcChannel) -> u32 {
        match ch {
            AdcChannel::OutputVoltage(ch) => self.read_output_voltage(ch),
            AdcChannel::OutputCurrent(ch) => self.read_output_current(ch),
            AdcChannel::Supply(ch) => self.read_supply(ch),
        }
    }

    pub fn read_output_voltage(&mut self, ch: Channel) -> u32 {
        let p = &mut self.pins.output_voltage;
        match ch {
            Channel::Ch0 => self.adc1.read(&mut p.0).unwrap(),
            Channel::Ch1 => self.adc1.read(&mut p.1).unwrap(),
            Channel::Ch2 => self.adc1.read(&mut p.2).unwrap(),
            Channel::Ch3 => self.adc1.read(&mut p.3).unwrap(),
        }
    }

    pub fn read_output_current(&mut self, ch: Channel) -> u32 {
        let p = &mut self.pins.output_current;
        match ch {
            Channel::Ch0 => self.adc1.read(&mut p.0).unwrap(),
            Channel::Ch1 => self.adc1.read(&mut p.1).unwrap(),
            Channel::Ch2 => self.adc1.read(&mut p.2).unwrap(),
            Channel::Ch3 => self.adc1.read(&mut p.3).unwrap(),
        }
    }

    pub fn read_supply(&mut self, ch: Supply) -> u32 {
        let p = &mut self.pins.supply;
        match ch {
            Supply::P3v3 => self.adc3.read(&mut p.2).unwrap(),
            Supply::P5v => self.adc1.read(&mut p.0).unwrap(),
            Supply::P12v => self.adc1.read(&mut p.1).unwrap(),
            Supply::I12v => self.adc3.read(&mut p.3).unwrap(),
        }
    }

    /// reads the 3.3V rail voltage in volt
    pub fn read_p3v3(&mut self) -> f32 {
        const DIV: f32 = 6.8 / (1.6 + 6.8); // Resistor divider 3V3 rail
        let factor = (V_REF / DIV) / self.adc1.max_sample() as f32;
        self.read_supply(Supply::P3v3) as f32 * factor
    }

    /// reads the 5V rail voltage in volt
    pub fn read_p5v(&mut self) -> f32 {
        const DIV: f32 = 6.8 / (10.0 + 6.8); // Resistor divider 5V rail
        let factor = (V_REF / DIV) / self.adc1.max_sample() as f32;
        self.read_supply(Supply::P5v) as f32 * factor
    }

    /// reads the 12V rail voltage in volt
    pub fn read_p12v(&mut self) -> f32 {
        const DIV: f32 = 1.6 / (1.6 + 6.8); // Resistor divider 12V rail
        let factor = (V_REF / DIV) / self.adc1.max_sample() as f32;
        self.read_supply(Supply::P12v) as f32 * factor
    }

    /// reads the 12V rail current in ampere
    pub fn read_i12v(&mut self) -> f32 {
        const GAIN: f32 = 0.005 * (10000.0 / 100.0); // 12V current measurement resistor configuration for LT6106
        let factor = (V_REF / GAIN) / self.adc1.max_sample() as f32;
        self.read_supply(Supply::I12v) as f32 * factor
    }
}
