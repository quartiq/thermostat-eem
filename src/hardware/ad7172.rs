// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

use core::fmt::Debug;

use defmt::info;

use super::hal::hal::{
    blocking::{
        delay::DelayUs,
        spi::{Transfer, Write},
    },
    digital::v2::OutputPin,
};

// ADC Register Adresses
#[allow(unused)]
pub enum AdcReg {
    STATUS = 0x00,
    ADCMODE = 0x1,
    IFMODE = 0x2,
    DATA = 0x04,
    ID = 0x7,
    FILTCON0 = 0x28,
    FILTCON1 = 0x29,
    FILTCON2 = 0x2a,
    FILTCON3 = 0x2b,
    CH0 = 0x10,
    CH1 = 0x11,
    CH2 = 0x12,
    CH3 = 0x13,
    SETUPCON0 = 0x20,
    SETUPCON1 = 0x21,
    SETUPCON2 = 0x22,
    SETUPCON3 = 0x23,
    OFFSET0 = 0x30,
    OFFSET1 = 0x31,
    OFFSET2 = 0x32,
    OFFSET3 = 0x33,
    GAIN0 = 0x38,
    GAIN1 = 0x39,
    GAIN2 = 0x3a,
    GAIN3 = 0x3b,
}

// *Note*: Register bitfields are not exhaustive.

/// ADCMODE register settings.
pub struct Adcmode;
#[allow(unused)]
impl Adcmode {
    pub const REF_EN: u32 = 1 << 15; // Internal reference enable
    pub const MODE_CONTINOUS_CONVERSION: u32 = 0b000 << 4; // Continuous conversion mode
    pub const CLOCKSEL_INTERNAL_OSC: u32 = 0b00 << 2; // Internal oscillator clock source
    pub const CLOCKSEL_INTERNAL_OSC_OUT: u32 = 0b01 << 2; // Internal oscillator clock source and output
    pub const CLOCKSEL_EXTERNAL_CLOCK: u32 = 0b10 << 2; // External clock input
    pub const CLOCKSEL_EXTERNAL_OSC: u32 = 0b11 << 2; // External oscillator clock source
}

/// ADC IFMODE register settings.
pub struct Ifmode;
#[allow(unused)]
impl Ifmode {
    pub const DATA_STAT: u32 = 1 << 6; // enable status reg to be appended after data output
}

/// ADC CH register settings. Valid for registers CH0-CH3.
pub struct Channel;
#[allow(unused)]
impl Channel {
    pub const CH_EN: u32 = 1 << 15; // enable channel
    pub const SETUP_SEL_0: u32 = 0b00 << 12; // Use setup register set 0
    pub const SETUP_SEL_1: u32 = 0b01 << 12; // Use setup register set 1
    pub const SETUP_SEL_2: u32 = 0b10 << 12; // Use setup register set 2
    pub const SETUP_SEL_3: u32 = 0b11 << 12; // Use setup register set 3
    pub const AINPOS_AIN0: u32 = 0b00000 << 5; // select AIN0 for positive channel input
    pub const AINPOS_AIN1: u32 = 0b00001 << 5; // select AIN1 for positive channel input
    pub const AINPOS_AIN2: u32 = 0b00010 << 5; // select AIN2 for positive channel input
    pub const AINPOS_AIN3: u32 = 0b00011 << 5; // select AIN3 for positive channel input
    pub const AINPOS_AIN4: u32 = 0b00100 << 5; // select AIN4 for positive channel input
    pub const AINNEG_AIN0: u32 = 0b00000; // select AIN0 for negative channel input
    pub const AINNEG_AIN1: u32 = 0b00001; // select AIN1 for negative channel input
    pub const AINNEG_AIN2: u32 = 0b00010; // select AIN2 for negative channel input
    pub const AINNEG_AIN3: u32 = 0b00011; // select AIN3 for negative channel input
    pub const AINNEG_AIN4: u32 = 0b00100; // select AIN4 for negative channel input
}

/// ADC SETUPCON register settings. Valid for registers SETUPCON0-SETUPCON3.
pub struct Setupcon;
#[allow(unused)]
impl Setupcon {
    pub const BIPOLAR: u32 = 1 << 12; // Bipolar input
    pub const UNIPOLAR: u32 = 0 << 12; // Unipolar input
    pub const REFBUFP: u32 = 1 << 11; // REFBUF+
    pub const REFBUFN: u32 = 1 << 10; // REFBUF-
    pub const AINBUFP: u32 = 1 << 9; // AINBUF+
    pub const AINBUFN: u32 = 1 << 8; // AINBUF-
    pub const REF_SEL_EXTERNAL: u32 = 0b00 << 4; // External reference
    pub const REF_SEL_INTERNAL: u32 = 0b10 << 4; // Internal 2,5V reference
    pub const REF_SEL_DIAGNOSTIC: u32 = 0b11 << 4; // diagnostic reference
}

/// ADC FILTCON register settings. Valid for registers FILTCON0-FILTCON3.
pub struct Filtcon;
#[allow(unused)]
impl Filtcon {
    pub const ENHFILTEN: u32 = 1 << 11; // enable postfilter
    pub const ENHFILT_27: u32 = 0b010 << 8; // 27 SPS, 47 dB rejection, 36.7 ms settling postfilter
    pub const ENHFILT_21: u32 = 0b011 << 8; // 21.25 SPS, 62 dB rejection, 40 ms settling postfilter
    pub const ENHFILT_20: u32 = 0b101 << 8; // 20 SPS, 86 dB rejection, 50 ms settling postfilter
    pub const ENHFILT_16: u32 = 0b110 << 8; // 16.67 SPS, 92 dB rejection, 60 ms settling postfilter
    pub const ORDER_SINC5SINC1: u32 = 0b00 << 5; // Sinc5 + Sinc1 sigma delta filter
    pub const ORDER_SINC3: u32 = 0b11 << 5; // Sinc3 sigma delta filter
    pub const ODR_1_25: u32 = 0b10110; // Output data rate 1.25 Hz
    pub const ODR_10: u32 = 0b10011; // Output data rate 10 Hz
    pub const ODR_20: u32 = 0b10001; // Output data rate 20 Hz
    pub const ODR_1007: u32 = 0b01010; // Output data rate 1007 Hz
}

/// DAC value out of bounds error.
#[derive(Debug)]
pub enum Error {
    AdcId,
}

pub struct Ad7172<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> Ad7172<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    <SPI as Write<u8>>::Error: core::fmt::Debug,
    <SPI as Transfer<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    <CS>::Error: core::fmt::Debug,
{
    pub fn new(delay: &mut impl DelayUs<u16>, spi: SPI, mut cs: CS) -> Result<Self, Error> {
        // set CS high first
        cs.set_high().unwrap();
        let mut adc = Ad7172 { spi, cs };
        adc.reset();

        // 500 us delay after reset.
        delay.delay_us(5000u16);

        let id = adc.read(AdcReg::ID);
        // check that ID is 0x00DX, as per datasheet
        info!("id: {:x}", id);
        if id & 0xf0 != 0x00d0 {
            return Err(Error::AdcId);
        }

        Ok(adc)
    }

    pub fn reset(&mut self) {
        // 64 cycles high for ADC reset
        let mut buf = [0xFFu8; 8];
        self.cs.set_low().unwrap();
        let _result = self.spi.transfer(&mut buf).unwrap();
        self.cs.set_high().unwrap();
    }

    /// Read a ADC register of size in bytes. Max. size 4 bytes.
    pub fn read(&mut self, addr: AdcReg) -> u32 {
        let size = Ad7172::<SPI, CS>::get_reg_width(&addr);
        let mut buf = [0u8; 8];
        buf[7 - size] = addr as u8 | 0x40; // addr with read flag
        self.cs.set_low().unwrap();
        self.spi.transfer(&mut buf[7 - size..]).unwrap();
        self.cs.set_high().unwrap();
        (u64::from_be_bytes(buf) & ((1 << (size * 8)) - 1)) as u32
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    pub fn write(&mut self, addr: AdcReg, data: u32) {
        let size = Ad7172::<SPI, CS>::get_reg_width(&addr);
        let mut buf = data.to_be_bytes();
        buf[3 - size] = addr as _;
        self.cs.set_low().unwrap();
        self.spi.write(&buf[3 - size..]).unwrap();
        self.cs.set_high().unwrap();
    }

    /// Reads the data register and returns data and status information.
    /// The DATA_STAT bit has to be set in the IFMODE register.
    /// If DATA_STAT bit is not set, the content of status is undefined but data is still valid.
    pub fn read_data(&mut self) -> (u32, u8) {
        let data_ch = self.read(AdcReg::DATA);
        let ch = (data_ch & 0xff) as u8;
        let data = data_ch >> 8;
        (data, ch)
    }

    fn get_reg_width(reg: &AdcReg) -> usize {
        match reg {
            AdcReg::STATUS => 1,
            AdcReg::ADCMODE => 2,
            AdcReg::IFMODE => 2,
            AdcReg::DATA => 4, // If DATA_STAT bit is not set this is 3 bytes but a 4 byte read will also yield 3 valid bytes.
            AdcReg::ID => 2,
            AdcReg::FILTCON0 => 2,
            AdcReg::FILTCON1 => 2,
            AdcReg::FILTCON2 => 2,
            AdcReg::FILTCON3 => 2,
            AdcReg::CH0 => 2,
            AdcReg::CH1 => 2,
            AdcReg::CH2 => 2,
            AdcReg::CH3 => 2,
            AdcReg::SETUPCON0 => 2,
            AdcReg::SETUPCON1 => 2,
            AdcReg::SETUPCON2 => 2,
            AdcReg::SETUPCON3 => 2,
            AdcReg::OFFSET0 => 3,
            AdcReg::OFFSET1 => 3,
            AdcReg::OFFSET2 => 3,
            AdcReg::OFFSET3 => 3,
            AdcReg::GAIN0 => 3,
            AdcReg::GAIN1 => 3,
            AdcReg::GAIN2 => 3,
            AdcReg::GAIN3 => 3,
        }
    }
}
