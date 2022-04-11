// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

use core::fmt::Debug;

use super::hal::hal::blocking::spi::{Transfer, Write};

// ADC Register Adresses
#[allow(unused)]
pub enum AdcReg {
    STATUS = 0x00,
    ADCMODE = 0x1,
    IFMODE = 0x2,
    DATA = 0x04,
    GPIOCON = 0x06,
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
#[allow(non_snake_case)]
pub mod Adcmode {
    pub mod RefEn {
        pub const ENABLED: u32 = 1 << 15;
    }
    pub mod Mode {
        pub const CONTINOUS_CONVERSION: u32 = 0b000 << 4;
    }
    pub mod Clocksel {
        pub const INTERNAL_OSC: u32 = 0b00 << 2;
        pub const INTERNAL_OSC_OUT: u32 = 0b01 << 2;
        pub const EXTERNAL_CLOCK: u32 = 0b10 << 2;
        pub const EXTERNAL_OSC: u32 = 0b11 << 2;
    }
}

/// ADC IFMODE register settings.
#[allow(non_snake_case)]
pub mod Ifmode {
    pub mod DataStat {
        pub const ENABLED: u32 = 1 << 6;
    }
}

/// ADC GPIOCON register settings.
#[allow(non_snake_case)]
pub mod Gpiocon {
    pub mod SyncEn {
        pub const ENABLED: u32 = 1 << 11;
        pub const DISABLED: u32 = 0 << 11;
    }
}

/// ADC CH register settings. Valid for registers CH0-CH3.
#[allow(non_snake_case)]
pub mod Channel {
    pub mod ChEn {
        pub const ENABLED: u32 = 1 << 15;
    }
    pub mod SetupSel {
        pub const SETUP_0: u32 = 0b00 << 12;
        pub const SETUP_1: u32 = 0b01 << 12;
        pub const SETUP_2: u32 = 0b10 << 12;
        pub const SETUP_3: u32 = 0b11 << 12;
    }
    pub mod Ainpos {
        pub const AIN0: u32 = 0b00000 << 5;
        pub const AIN1: u32 = 0b00001 << 5;
        pub const AIN2: u32 = 0b00010 << 5;
        pub const AIN3: u32 = 0b00011 << 5;
        pub const AIN4: u32 = 0b00100 << 5;
    }
    pub mod Ainneg {
        pub const AIN0: u32 = 0b00000;
        pub const AIN1: u32 = 0b00001;
        pub const AIN2: u32 = 0b00010;
        pub const AIN3: u32 = 0b00011;
        pub const AIN4: u32 = 0b00100;
    }
}

/// ADC SETUPCON register settings. Valid for registers SETUPCON0-SETUPCON3.
#[allow(non_snake_case)]
pub mod Setupcon {
    pub mod BiUnipolar {
        pub const BIPOLAR: u32 = 1 << 12;
        pub const UNIPOLAR: u32 = 0 << 12;
    }
    pub mod Refbufp {
        pub const ENABLED: u32 = 1 << 11;
    }
    pub mod Refbufn {
        pub const ENABLED: u32 = 1 << 10;
    }
    pub mod Ainbufp {
        pub const ENABLED: u32 = 1 << 9;
    }
    pub mod Ainbufn {
        pub const ENABLED: u32 = 1 << 8;
    }
    pub mod Refsel {
        pub const EXTERNAL: u32 = 0b00 << 4;
        pub const INTERNAL: u32 = 0b10 << 4;
        pub const DIAGNOSTIC: u32 = 0b11 << 4;
    }
}

/// ADC FILTCON register settings. Valid for registers FILTCON0-FILTCON3.
#[allow(non_snake_case)]
pub mod Filtcon {
    pub mod Enhfilten {
        pub const ENABLED: u32 = 1 << 11;
    }
    pub mod Enhfilt {
        pub const SPS_27: u32 = 0b010 << 8;
        pub const SPS_21: u32 = 0b011 << 8;
        pub const SPS_20: u32 = 0b101 << 8;
        pub const SPS_16: u32 = 0b110 << 8;
    }
    pub mod Order {
        pub const SINC5SINC1: u32 = 0b00 << 5;
        pub const SINC3: u32 = 0b11 << 5;
    }
    pub mod Odr {
        pub const ODR_1_25: u32 = 0b10110;
        pub const ODR_10: u32 = 0b10011;
        pub const ODR_20: u32 = 0b10001;
        pub const ODR_1007: u32 = 0b01010;
    }
}

/// DAC value out of bounds error.
#[derive(Debug)]
pub enum Error {
    AdcId,
}

pub struct Ad7172<SPI> {
    spi: SPI,
}

impl<SPI> Ad7172<SPI>
where
    SPI: Transfer<u8> + Write<u8>,
    <SPI as Write<u8>>::Error: core::fmt::Debug,
    <SPI as Transfer<u8>>::Error: core::fmt::Debug,
{
    pub fn new(spi: SPI) -> Self {
        Ad7172 { spi }
    }

    pub fn reset(&mut self) {
        // 64 cycles high for ADC reset
        let mut buf = [0xFFu8; 8];
        let _result = self.spi.transfer(&mut buf).unwrap();
    }

    /// Read a ADC register of size in bytes. Max. size 4 bytes.
    pub fn read(&mut self, addr: AdcReg) -> u32 {
        let size = Self::get_reg_width(&addr);
        let mut buf = [0u8; 8];
        buf[7 - size] = addr as u8 | 0x40; // addr with read flag
        self.spi.transfer(&mut buf[7 - size..]).unwrap();
        (u64::from_be_bytes(buf) & ((1 << (size * 8)) - 1)) as u32
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    pub fn write(&mut self, addr: AdcReg, data: u32) {
        let size = Self::get_reg_width(&addr);
        let mut buf = data.to_be_bytes();
        buf[3 - size] = addr as _;
        self.spi.write(&buf[3 - size..]).unwrap();
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
            AdcReg::GPIOCON => 2,
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
