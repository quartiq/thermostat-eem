// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

use core::fmt::Debug;
use bilge::prelude::*;

use super::hal::hal::blocking::spi::{Transfer, Write};

// ADC Register Addresses
#[bitsize(6)]
#[derive(FromBits, PartialEq, Debug)]
pub enum Register {
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
    #[fallback]
    Reserved,
}

#[bitsize(8)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct Comms {
    register: Register,
    #[doc(alias="RW")]
    read: bool,
    #[doc(alias="WEN_N")]
    ignore: bool,
}

#[bitsize(8)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct Status {
    pub channel: u2,
    reserved: u2,
    reg_error: bool,
    crc_error: bool,
    adc_error: bool,
    #[doc(alias("RDY_N"))]
    busy: bool,
}

#[bitsize(2)]
#[derive(FromBits, Debug, PartialEq)]
pub enum ClockSel {
    InternalOsc = 0,
    InternalOscOut = 1,
    ExternalClock = 2,
    ExternalOsc = 3,
}

#[bitsize(3)]
#[derive(FromBits, Debug, PartialEq)]
pub enum Mode {
    Continuous = 0,
    Single = 1,
    Standby = 2,
    PowerDown = 3,
    InternalOffset = 4,
    Reserved = 5,
    SystemOffset = 6,
    SystemGain = 7,
}

#[bitsize(16)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct AdcMode {
    reserved: u2,
    clocksel: ClockSel,
    mode: Mode,
    reserved: u1,
    delay: u3,
    reserved: u2,
    single_cycle: bool,
    #[doc(alias("HIDE_DELAY_N"))]
    show_delay: bool,
    ref_en: bool,
}

#[bitsize(16)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct IfMode {
    reserved: u1,
    crc_en: u2,
    reserved: u1,
    reg_check: bool,
    data_stat: bool,
    contread: bool,
    dout_reset: bool,
    reserved: u2,
    iostrength: bool,
    alt_sync: bool,
    reserved: u3,
}


#[bitsize(16)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct GpioCon {
    gp_data: u2,
    op_en: u2,
    ip_en: u2,
    reserved: u2,
    err_dat: bool,
    err_en: u2,
    sync_en: bool,
    mux_io: bool,
    reserved: u3,
}

#[bitsize(5)]
#[derive(FromBits, Debug, PartialEq)]
pub enum Mux {
    Ain0 = 0,
    Ain1 = 1,
    Ain2 = 2,
    Ain3 = 3,
    Ain4 = 4,
    TempP = 17,
    TempN = 18,
    AvddAvss5P = 19,
    AvddAvss5N = 20,
    RefP = 21,
    RefN = 22,
    #[fallback]
    Reserved
}

#[bitsize(16)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct Channel {
    ainneg: Mux,
    ainpos: Mux,
    reserved: u2,
    setup_sel: u2,
    reserved: u1,
    en: bool,
}

#[bitsize(2)]
#[derive(FromBits, Debug, PartialEq)]
pub enum RefSel {
    External = 0,
    Internal = 2,
    AvddAvss = 3,
    Reserved = 1,
}

#[bitsize(1)]
#[derive(FromBits, Debug, PartialEq)]
pub enum Coding {
    Unipolar = 0,
    Bipolar = 1,
}

#[bitsize(16)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct SetupCon {
    reserved: u4,
    ref_sel: RefSel,
    reserved: u1,
    burnout_en: bool,
    ainbufn: bool,
    ainbufp: bool,
    refbufn: bool,
    refbufp: bool,
    coding: Coding,
    reserved: u3,
}

#[bitsize(5)]
#[derive(FromBits, Debug, PartialEq)]
pub enum Odr {
    _1007 = 0b01010,
    _20 = 0b10001,
    _10 = 0b10011,
    _1_25 = 0b10110,
    // ...
    #[fallback]
    Reserved,
}

#[bitsize(2)]
#[derive(FromBits, Debug, PartialEq)]
pub enum Order {
    Sinc5Sinc1 = 0,
    Sinc3 = 3,
    #[fallback]
    Reserved,
}

#[bitsize(3)]
#[derive(FromBits, Debug, PartialEq)]
pub enum Enhfilt {
    _27 = 2,
    _21 = 3,
    _20 = 5,
    _17 = 6,
    #[fallback]
    Reserved,
}

#[bitsize(16)]
#[derive(FromBits, DebugBits, PartialEq)]
pub struct FiltCon {
    odr: Odr,
    order: Order,
    reserved: u1,
    enhfilt: Enhfilt,
    enhfilt_en: bool,
    reserved: u3,
    sinc3_map: bool,
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
    pub fn read(&mut self, addr: Register) -> u32 {
        let size = Self::reg_width(&addr);
        let mut buf = [0u8; 8];
        buf[7 - size] = Comms::new(addr, true, false).into(); // addr with read flag
        self.spi.transfer(&mut buf[7 - size..]).unwrap();
        (u64::from_be_bytes(buf) & ((1 << (size * 8)) - 1)) as u32
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    pub fn write(&mut self, addr: Register, data: u32) {
        let size = Self::reg_width(&addr);
        let mut buf = data.to_be_bytes();
        buf[3 - size] = Comms::new(addr, false, false).into();
        self.spi.write(&buf[3 - size..]).unwrap();
    }

    /// Reads the data register and returns data and status information.
    /// The DATA_STAT bit has to be set in the IFMODE register.
    /// If DATA_STAT bit is not set, the content of status is undefined but data is still valid.
    pub fn read_data(&mut self) -> (u32, u8) {
        let res = self.read(Register::DATA);
        let status = res as u8;
        let data = res >> 8;
        (data, status)
    }

    fn reg_width(reg: &Register) -> usize {
        match reg {
            Register::STATUS => 1,
            Register::REGCHECK => 3,
            Register::DATA => 4, // If DATA_STAT bit is not set this is 3 bytes but a 4 byte read will also yield 3 valid bytes.
            Register::OFFSET0 => 3,
            Register::OFFSET1 => 3,
            Register::OFFSET2 => 3,
            Register::OFFSET3 => 3,
            Register::GAIN0 => 3,
            Register::GAIN1 => 3,
            Register::GAIN2 => 3,
            Register::GAIN3 => 3,
            _ => 2,
        }
    }
}
