// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

use arbitrary_int::{u2, u3};
use bitbybit::{bitenum, bitfield};
use core::fmt::Debug;

use super::hal::hal_02::blocking::spi::{Transfer, Write};

// ADC Register Addresses
#[bitenum(u6)]
#[derive(PartialEq, Debug)]
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
}

#[bitfield(u8, default = 0x00)]
#[derive(Debug, PartialEq)]
pub struct Comms {
    #[bits(0..=5, w)]
    pub register: Option<Register>,
    #[bit(6, w)]
    #[doc(alias = "RW")]
    read: bool,
    #[bit(7, w)]
    #[doc(alias = "WEN_N")]
    ignore: bool,
}

#[bitfield(u8, default = 0x80)]
#[derive(Debug, PartialEq)]
pub struct Status {
    #[bits(0..=1, r)]
    channel: u2,
    #[bit(4, r)]
    reg_error: bool,
    #[bit(5, r)]
    crc_error: bool,
    #[bit(6, r)]
    adc_error: bool,
    #[bit(7, r)]
    #[doc(alias("RDY_N"))]
    busy: bool,
}

#[bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq)]
pub enum ClockSel {
    InternalOsc = 0,
    InternalOscOut = 1,
    ExternalClock = 2,
    ExternalOsc = 3,
}

#[bitenum(u3)]
#[derive(Debug, PartialEq)]
pub enum Mode {
    Continuous = 0,
    Single = 1,
    Standby = 2,
    PowerDown = 3,
    InternalOffset = 4,
    SystemOffset = 6,
    SystemGain = 7,
}

#[bitfield(u16, default = 0x0000)]
#[derive(Debug, PartialEq)]
pub struct AdcMode {
    #[bits(2..=3, rw)]
    clocksel: ClockSel,
    #[bits(4..=6, rw)]
    mode: Option<Mode>,
    #[bits(8..=10, rw)]
    delay: u3,
    #[bit(13, rw)]
    single_cycle: bool,
    #[bit(14, rw)]
    #[doc(alias("HIDE_DELAY_N"))]
    show_delay: bool,
    #[bit(15, rw)]
    ref_en: bool,
}

#[bitfield(u16, default = 0x0000)]
#[derive(Debug, PartialEq)]
pub struct IfMode {
    #[bit(0, rw)]
    wl16: bool,
    #[bits(2..=3, rw)]
    crc_en: u2,
    #[bit(5, rw)]
    reg_check: bool,
    #[bit(6, rw)]
    data_stat: bool,
    #[bit(7, rw)]
    contread: bool,
    #[bit(8, rw)]
    dout_reset: bool,
    #[bit(11, rw)]
    iostrength: bool,
    #[bit(12, rw)]
    alt_sync: bool,
}

#[bitfield(u16, default = 0x0800)]
#[derive(Debug, PartialEq)]
pub struct GpioCon {
    #[bits(0..=1, rw)]
    gp_data: u2,
    #[bits(2..=3, rw)]
    op_en: u2,
    #[bits(4..=5, rw)]
    ip_en: u2,
    #[bit(8, rw)]
    err_dat: bool,
    #[bits(9..=10, rw)]
    err_en: u2,
    #[bit(11, rw)]
    sync_en: bool,
    #[bit(12, rw)]
    mux_io: bool,
}

#[bitenum(u5)]
#[derive(Debug, PartialEq)]
pub enum Mux {
    Ain0 = 0b00000,
    Ain1 = 0b00001,
    Ain2 = 0b00010,
    Ain3 = 0b00011,
    Ain4 = 0b00100,
    TempP = 0b10001,
    TempN = 0b10010,
    AvddAvss5P = 0b10011,
    AvddAvss5N = 0b10100,
    RefP = 0b10101,
    RefN = 0b10110,
}

#[bitfield(u16, default = 0x0001)] // deviate default for simplicity
#[derive(Debug, PartialEq)]
pub struct Channel {
    #[bits(0..=4, rw)]
    ainneg: Option<Mux>,
    #[bits(5..=9, rw)]
    ainpos: Option<Mux>,
    #[bits(12..=13, rw)]
    setup_sel: u2,
    #[bit(15, rw)]
    en: bool,
}

#[bitenum(u2)]
#[derive(Debug, PartialEq)]
pub enum RefSel {
    External = 0,
    Internal = 2,
    AvddAvss = 3,
}

#[bitfield(u16, default = 0x1000)]
#[derive(Debug, PartialEq)]
pub struct SetupCon {
    #[bits(4..=5, rw)]
    ref_sel: Option<RefSel>,
    #[bit(7, rw)]
    burnout_en: bool,
    #[bit(8, rw)]
    ainbufn: bool,
    #[bit(9, rw)]
    ainbufp: bool,
    #[bit(10, rw)]
    refbufn: bool,
    #[bit(11, rw)]
    refbufp: bool,
    #[bit(12, rw)]
    bipolar: bool,
}

#[bitenum(u5)]
#[derive(Debug, PartialEq)]
pub enum Odr {
    _31250a = 0b00000,
    _31250f = 0b00101,
    _10417 = 0b00111,
    _5208 = 0b01000,
    _2597 = 0b01001,
    _1007 = 0b01010,
    _200 = 0b01101,
    _100 = 0b01110,
    _20 = 0b10001,
    _10 = 0b10011,
    _1_25 = 0b10110,
    // ...
}

#[bitenum(u2)]
#[derive(Debug, PartialEq)]
pub enum Order {
    Sinc5Sinc1 = 0,
    Sinc3 = 3,
}

#[bitenum(u3)]
#[derive(Debug, PartialEq)]
pub enum Enhfilt {
    _27 = 2,
    _21_25 = 3,
    _20 = 5,
    _16_67 = 6,
}

#[bitfield(u16, default = 0x0500)]
#[derive(Debug, PartialEq)]
pub struct FiltCon {
    #[bits(0..=4, rw)]
    odr: Option<Odr>,
    #[bits(5..=6, rw)]
    order: Option<Order>,
    #[bits(8..=10, rw)]
    enhfilt: Option<Enhfilt>,
    #[bit(11, rw)]
    enhfilt_en: bool,
    #[bit(15, rw)]
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
        buf[7 - size] = Comms::builder()
            .with_register(addr)
            .with_read(true)
            .with_ignore(false)
            .build()
            .raw_value();
        self.spi.transfer(&mut buf[7 - size..]).unwrap();
        (u64::from_be_bytes(buf) & ((1 << (size * 8)) - 1)) as u32
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    pub fn write(&mut self, addr: Register, data: u32) {
        let size = Self::reg_width(&addr);
        let mut buf = data.to_be_bytes();
        buf[3 - size] = Comms::builder()
            .with_register(addr)
            .with_read(false)
            .with_ignore(false)
            .build()
            .raw_value();
        self.spi.write(&buf[3 - size..]).unwrap();
    }

    /// Reads the data register and returns data and status information.
    /// The DATA_STAT bit has to be set in the IFMODE register.
    /// If DATA_STAT bit is not set, the content of status is undefined but data is still valid.
    pub fn read_data(&mut self) -> (u32, Status) {
        let res = self.read(Register::DATA);
        (res >> 8, Status::new_with_raw_value(res as _))
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
