// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

use core::fmt::Debug;
use num_enum::TryFromPrimitive;

use super::hal::hal::blocking::spi::{Transfer, Write};

// ADC Register Adresses
#[allow(unused)]
pub enum AdcReg {
    STATUS = 0x00,
    ADCMODE = 0x1,
    IFMODE = 0x2,
    REGCHECK = 0x3,
    DATA = 0x04,
    GPIOCON = 0x06,
    ID = 0x7,
    CH0 = 0x10,
    CH1 = 0x11,
    CH2 = 0x12,
    CH3 = 0x13,
    SETUPCON0 = 0x20,
    SETUPCON1 = 0x21,
    SETUPCON2 = 0x22,
    SETUPCON3 = 0x23,
    FILTCON0 = 0x28,
    FILTCON1 = 0x29,
    FILTCON2 = 0x2a,
    FILTCON3 = 0x2b,
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

#[allow(non_snake_case)]
pub mod Comms {
    pub mod Wen {
        pub const ENABLED: u32 = 0 << 7;
        pub const DISABLED: u32 = 1 << 7;
    }
    pub mod RW {
        pub const WRITE: u32 = 0 << 6;
        pub const READ: u32 = 1 << 6;
    }
}

#[derive(Clone, Copy, TryFromPrimitive, Debug)]
#[repr(usize)]
pub enum AdcChannel {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

pub struct Status(u8);
impl Status {
    pub fn channel(&self) -> AdcChannel {
        AdcChannel::try_from(self.0 as usize & 0x3).unwrap()
    }
}

impl From<u8> for Status {
    fn from(x: u8) -> Self {
        Self(x)
    }
}

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
        pub const DISABLED: u32 = 0 << 15;
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
        pub const TEMPERATURESENSOR_P: u32 = 0b00101 << 5;
        pub const TEMPERATURESENSOR_N: u32 = 0b00110 << 5;
        pub const AVDD_MINUS_AVSS_OVER_5_P: u32 = 0b00111 << 5;
        pub const AVDD_MINUS_AVSS_OVER_5_N: u32 = 0b01000 << 5;
        pub const REF_P: u32 = 0b01001 << 5;
        pub const REF_N: u32 = 0b01010 << 5;
    }
    pub mod Ainneg {
        pub const AIN0: u32 = 0b00000;
        pub const AIN1: u32 = 0b00001;
        pub const AIN2: u32 = 0b00010;
        pub const AIN3: u32 = 0b00011;
        pub const AIN4: u32 = 0b00100;
        pub const TEMPERATURESENSOR_P: u32 = 0b00101 << 5;
        pub const TEMPERATURESENSOR_N: u32 = 0b00110 << 5;
        pub const AVDD_MINUS_AVSS_OVER_5_P: u32 = 0b00111 << 5;
        pub const AVDD_MINUS_AVSS_OVER_5_N: u32 = 0b01000 << 5;
        pub const REF_P: u32 = 0b01001 << 5;
        pub const REF_N: u32 = 0b01010 << 5;
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
        self.spi.transfer(&mut buf).unwrap();
    }

    /// Read a ADC register of size in bytes. Max. size 4 bytes.
    pub fn read(&mut self, addr: AdcReg) -> u32 {
        let size = Self::reg_width(&addr);
        let mut buf = [0u8; 8];
        buf[7 - size] = addr as u8 | Comms::RW::READ as u8; // addr with read flag
        self.spi.transfer(&mut buf[7 - size..]).unwrap();
        (u64::from_be_bytes(buf) & ((1 << (size * 8)) - 1)) as u32
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    pub fn write(&mut self, addr: AdcReg, data: u32) {
        let size = Self::reg_width(&addr);
        let mut buf = data.to_be_bytes();
        buf[3 - size] = addr as u8 | Comms::RW::WRITE as u8;
        self.spi.write(&buf[3 - size..]).unwrap();
    }

    /// Reads the data register and returns data and status information.
    /// The DATA_STAT bit has to be set in the IFMODE register.
    /// If DATA_STAT bit is not set, the content of status is undefined but data is still valid.
    pub fn read_data(&mut self) -> (u32, u8) {
        let res = self.read(AdcReg::DATA);
        let status = res as u8;
        let data = res >> 8;
        (data, status)
    }

    fn reg_width(reg: &AdcReg) -> usize {
        match reg {
            AdcReg::STATUS => 1,
            AdcReg::REGCHECK => 3,
            AdcReg::DATA => 4, // If DATA_STAT bit is not set this is 3 bytes but a 4 byte read will also yield 3 valid bytes.
            AdcReg::OFFSET0 => 3,
            AdcReg::OFFSET1 => 3,
            AdcReg::OFFSET2 => 3,
            AdcReg::OFFSET3 => 3,
            AdcReg::GAIN0 => 3,
            AdcReg::GAIN1 => 3,
            AdcReg::GAIN2 => 3,
            AdcReg::GAIN3 => 3,
            _ => 2,
        }
    }
}
