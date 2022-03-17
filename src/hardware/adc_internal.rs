use super::hal::{
    adc,
    gpio::{gpioa::*, gpiob::*, gpioc::*, gpiof::*, Analog},
    hal::blocking::delay::DelayUs,
    prelude::*,
    rcc::{rec, CoreClocks},
    stm32::{ADC1, ADC2, ADC3},
};
use super::{
    dac::{R_SENSE, VREF_TEC},
    Channel,
};

const V_REF: f32 = 3.0; // ADC reference voltage

pub enum Supply {
    P3v3Voltage,
    P5vVoltage,
    P12vVoltage,
    P12vCurrent,
}

pub enum AdcChannel {
    OutputVoltage(Channel),
    OutputCurrent(Channel),
    Supply(Supply),
}

pub struct AdcPins {
    pub output_voltage: (PC3<Analog>, PA0<Analog>, PA3<Analog>, PA4<Analog>),
    pub output_current: (PA5<Analog>, PA6<Analog>, PB0<Analog>, PB1<Analog>),
    pub p3v3_voltage: PF7<Analog>,
    pub p5v_voltage: PC0<Analog>,
    pub p12v_voltage: PC2<Analog>,
    pub p12v_current: PF8<Analog>,
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
        let (adc1, _adc2) = adc::adc12(adc.0, adc.1, delay, adc_rcc.0, clocks);
        let adc3 = adc::Adc::adc3(adc.2, delay, adc_rcc.1, clocks);

        let mut adc1 = adc1.enable();
        adc1.set_resolution(adc::Resolution::SIXTEENBIT);

        let mut adc3 = adc3.enable();
        adc3.set_resolution(adc::Resolution::SIXTEENBIT);

        AdcInternal { adc1, adc3, pins }
    }

    pub fn read(&mut self, ch: AdcChannel) -> f32 {
        match ch {
            AdcChannel::OutputVoltage(ch) => self.read_output_voltage(ch),
            AdcChannel::OutputCurrent(ch) => self.read_output_current(ch),
            AdcChannel::Supply(ch) => self.read_supply(ch),
        }
    }

    pub fn read_output_voltage(&mut self, ch: Channel) -> f32 {
        let p = &mut self.pins.output_voltage;
        let code: u32 = match ch {
            Channel::Ch0 => self.adc1.read(&mut p.0),
            Channel::Ch1 => self.adc1.read(&mut p.1),
            Channel::Ch2 => self.adc1.read(&mut p.2),
            Channel::Ch3 => self.adc1.read(&mut p.3),
        }
        .unwrap();
        const SCALE: f32 = V_REF * 20.0 / 5.0; // Differential voltage sense gain
        const OFFSET: f32 = -VREF_TEC * SCALE / V_REF; // Differential voltage sense offset
        code as f32 / self.adc1.max_sample() as f32 * SCALE + OFFSET
    }

    pub fn read_output_current(&mut self, ch: Channel) -> f32 {
        let p = &mut self.pins.output_current;
        let code: u32 = match ch {
            Channel::Ch0 => self.adc1.read(&mut p.0),
            Channel::Ch1 => self.adc1.read(&mut p.1),
            Channel::Ch2 => self.adc1.read(&mut p.2),
            Channel::Ch3 => self.adc1.read(&mut p.3),
        }
        .unwrap();
        const SCALE: f32 = V_REF / R_SENSE * 8.0; // MAX1968 ITEC scale
        const OFFSET: f32 = -VREF_TEC * SCALE / (V_REF / R_SENSE); // MAX1968 ITEC offset
        code as f32 / self.adc1.max_sample() as f32 * SCALE + OFFSET
    }

    pub fn read_supply(&mut self, ch: Supply) -> f32 {
        match ch {
            Supply::P3v3Voltage => self.read_p3v3_voltage(),
            Supply::P5vVoltage => self.read_p5v_voltage(),
            Supply::P12vVoltage => self.read_p12v_voltage(),
            Supply::P12vCurrent => self.read_p12v_current(),
        }
    }

    /// reads the 3.3V rail voltage in volt
    pub fn read_p3v3_voltage(&mut self) -> f32 {
        const DIV: f32 = 6.8 / (1.6 + 6.8); // Resistor divider 3V3 rail
        let code: u32 = self.adc3.read(&mut self.pins.p3v3_voltage).unwrap();
        code as f32 / self.adc3.max_sample() as f32 * (V_REF / DIV)
    }

    /// reads the 5V rail voltage in volt
    pub fn read_p5v_voltage(&mut self) -> f32 {
        const DIV: f32 = 6.8 / (10.0 + 6.8); // Resistor divider 5V rail
        let code: u32 = self.adc1.read(&mut self.pins.p5v_voltage).unwrap();
        code as f32 / self.adc1.max_sample() as f32 * (V_REF / DIV)
    }

    /// reads the 12V rail voltage in volt
    pub fn read_p12v_voltage(&mut self) -> f32 {
        const DIV: f32 = 1.6 / (1.6 + 6.8); // Resistor divider 12V rail
        let code: u32 = self.adc1.read(&mut self.pins.p12v_voltage).unwrap();
        code as f32 / self.adc1.max_sample() as f32 * (V_REF / DIV)
    }

    /// reads the 12V rail current in ampere
    pub fn read_p12v_current(&mut self) -> f32 {
        const GAIN: f32 = 0.005 * (10000.0 / 100.0); // 12V current measurement resistor configuration for LT6106
        let code: u32 = self.adc3.read(&mut self.pins.p12v_current).unwrap();
        code as f32 / self.adc3.max_sample() as f32 * (V_REF / GAIN)
    }
}
