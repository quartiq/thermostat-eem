// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

// use core::fmt::Error;

use core::fmt::Debug;

use defmt::info;

use super::hal::hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

// ADC Register Adresses
#[allow(unused)]
pub enum AdcReg {
    ID = 0x7,
    ADCMODE = 0x1,
    IFMODE = 0x2,
    DATA = 0x04,
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

// ADC SETUPCON register settings.
#[allow(unused)]
enum Setupcon {
    REFBUFP = 1 << 11,    // REFBUF+
    REFBUFN = 1 << 10,    // REFBUF-
    AINBUFP = 1 << 9,     // AINBUF+
    AINBUFN = 1 << 8,     // AINBUF-
    BIUNIPOLAR = 1 << 12, // BI_UNIPOLAR
    INTREF = 10 << 4,     // Internal 2,5V reference
    DIAREF = 11 << 4,     // diagnostic reference
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
    pub fn new(spi: SPI, mut cs: CS) -> Result<Self, Error> {
        // set CS high first
        cs.set_high().unwrap();
        let mut adc = Ad7172 { spi, cs };
        adc.reset();

        let id = adc.read_reg(AdcReg::ID, 2);
        // check that ID is 0x00DX, as per datasheet
        // currently this seems to sometimes read 0x40CE sometimes. To be investigated.
        // if id & 0xf0 == 0xd0 {
        //     return Err(Error::AdcId);
        // }
        info!("adc id: {:x}", id);

        // Setup ADCMODE register. Internal reference, internal clock, no delay, continuous conversion.
        adc.write_reg(AdcReg::ADCMODE, 2, 0x8000);

        // Setup IFMODE register. Only enable data stat to get channel info on conversions.
        adc.write_reg(AdcReg::IFMODE, 2, 0b100_0000);

        adc.setup_channels();

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
    pub fn read_reg(&mut self, addr: AdcReg, size: usize) -> u32 {
        self.cs.set_low().unwrap();
        let mut buf = [0u8; 8];
        buf[7 - size] = addr as u8 | 0x40; // addr with read flag
        self.spi.transfer(&mut buf[7 - size..]).unwrap();
        let data = u64::from_be_bytes(buf) & ((1 << size * 8) - 1);
        self.cs.set_high().unwrap();
        return data as u32;
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    pub fn write_reg(&mut self, addr: AdcReg, size: usize, data: u32) {
        self.cs.set_low().unwrap();
        let mut buf = data.to_be_bytes();
        buf[3 - size] = addr as _;
        self.spi.write(&mut buf[3 - size..]).unwrap();
        self.cs.set_high().unwrap();
    }

    /// Reads the status register and returns the value. This is different from reading a normal register.
    pub fn get_status_reg(&mut self) -> u8 {
        let mut addr_buf = [0];
        self.cs.set_low().unwrap();
        self.spi.transfer(&mut addr_buf).unwrap();
        self.cs.set_high().unwrap();
        addr_buf[0]
    }

    /// Reads the data register and returns data and channel information.
    /// The DATA_STAT bit has to be set in the IFMODE register.
    pub fn read_data(&mut self) -> (u32, u8) {
        let data_ch = self.read_reg(AdcReg::DATA, 4);
        let ch = (data_ch & 0x3) as u8;
        let data = data_ch >> 8;
        (data, ch)
    }

    /// Setup ADC channels.
    fn setup_channels(&mut self) {
        // enable first channel and configure Ain0, Ain1,
        // set config 0 for second channel,
        self.write_reg(AdcReg::CH0, 2, 0x8001);

        // enable second channel and configure Ain2, Ain3,
        // set config 1 for second channel,
        self.write_reg(AdcReg::CH1, 2, 0x9043);

        // Setup configuration register ch0
        self.write_reg(
            AdcReg::SETUPCON0,
            2,
            Setupcon::REFBUFP as u32
                | Setupcon::REFBUFN as u32
                | Setupcon::AINBUFP as u32
                | Setupcon::AINBUFN as u32,
            // Unipolar
            // External Reference
        );

        // Setup configuration register ch1
        self.write_reg(
            AdcReg::SETUPCON1,
            2,
            Setupcon::REFBUFP as u32
                | Setupcon::REFBUFN as u32
                | Setupcon::AINBUFP as u32
                | Setupcon::AINBUFN as u32,
            // Unipolar
            // External Reference
        );

        // Setup filter register ch0. 10Hz data rate. Sinc5Sinc1 Filter. F16SPS 50/60Hz Filter.
        self.write_reg(AdcReg::FILTCON0, 2, 0b110 << 8 | 1 << 11 | 0b10011);

        // Setup filter register ch1. 10Hz data rate. Sinc5Sinc1 Filter. F16SPS 50/60Hz Filter.
        self.write_reg(AdcReg::FILTCON1, 2, 0b110 << 8 | 1 << 11 | 0b10011);
    }
}
