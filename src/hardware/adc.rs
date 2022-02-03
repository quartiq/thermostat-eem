// Thermostat ADC driver
// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)
// SingularitySurfer 2022

use byteorder::{BigEndian, ByteOrder};
use log::{info, warn};

use super::hal::{
    gpio::{gpioe::*, Alternate, Output, PushPull, AF5},
    hal::{blocking::spi::Transfer, blocking::spi::Write, digital::v2::OutputPin},
    prelude::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::Enabled,
    spi::Spi,
    stm32::SPI4,
    time::MegaHertz,
};

use crate::AdcFilterSettings;

/// SPI Mode 3
pub const SPI_MODE: spi::Mode = spi::Mode {
    polarity: spi::Polarity::IdleHigh,
    phase: spi::Phase::CaptureOnSecondTransition,
};

pub const SPI_CLOCK: MegaHertz = MegaHertz(2);

// ADC Register Adresses
#[allow(unused)]
enum AdcReg {
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

pub type AdcSpi = Spi<
    SPI4,
    (
        PE2<Alternate<AF5>>,
        PE5<Alternate<AF5>>,
        PE6<Alternate<AF5>>,
    ),
>;

pub struct AdcPins {
    pub sck: PE2<Alternate<AF5>>,
    pub miso: PE5<Alternate<AF5>>,
    pub mosi: PE6<Alternate<AF5>>,
    pub cs: PE0<Output<PushPull>>,
}

pub struct Adc {
    spi: Spi<SPI4, Enabled, u8>,
    cs: PE0<Output<PushPull>>,
}

impl Adc {
    pub fn new(prec: rec::Spi4, spi4: SPI4, mut pins: AdcPins, clocks: &CoreClocks) -> Self {
        pins.cs.set_high().unwrap();

        let spi: Spi<_, _, u8> = spi4.spi(
            (pins.sck, pins.miso, pins.mosi),
            spi::MODE_0,
            1.mhz(),
            prec,
            clocks,
        );
        let mut adc = Adc { spi, cs: pins.cs };
        adc.reset();

        info!("ADC ID: {:#X}", adc.read_reg(AdcReg::ID, 2));

        // Setup ADCMODE register. Internal reference, internal clock, no delay, continuous conversion.
        adc.write_reg(AdcReg::ADCMODE, 2, 0x8000);

        // Setup IFMODE register. Only enable data stat to get channel info on conversions.
        adc.write_reg(AdcReg::IFMODE, 2, 0b100_0000);

        adc.setup_channels();

        adc
    }

    /// Reset ADC.
    pub fn reset(&mut self) {
        let mut buf = [0xFFu8; 8];
        self.cs.set_low().unwrap();
        let result = self.spi.transfer(&mut buf);
        self.cs.set_high().unwrap();
        match result {
            Err(e) => {
                warn!("ADC reset failed! {:?}", e)
            }
            Ok(_) => {
                info!("ADC reset succeeded")
            }
        };
    }

    /// Read a ADC register of size in bytes.
    fn read_reg(&mut self, addr: AdcReg, size: u8) -> u32 {
        let mut buf = [addr as u8 | 0x40, 0, 0, 0, 0];
        self.cs.set_low().unwrap();

        self.spi.transfer(&mut buf[..(size + 1) as usize]).unwrap();
        let data = match size {
            1 => buf[1].clone() as u32,
            2 => BigEndian::read_u16(&buf[1..3]) as u32,
            3 => BigEndian::read_u24(&buf[1..4]) as u32,
            4 => BigEndian::read_u32(&buf[1..5]) as u32,
            _ => 0,
        };
        self.cs.set_high().unwrap();
        return data;
    }

    /// Write a ADC register of size in bytes.
    fn write_reg(&mut self, addr: AdcReg, size: u8, data: u32) {
        let mut addr_buf = [addr as u8];
        self.cs.set_low().unwrap();
        self.spi.write(&mut addr_buf).unwrap();
        let mut buf = [0, 0, 0, 0];
        BigEndian::write_u32(&mut buf, data);
        match size {
            1 => self.spi.transfer(&mut buf[3..4]).unwrap(),
            2 => self.spi.transfer(&mut buf[2..4]).unwrap(),
            3 => self.spi.transfer(&mut buf[1..4]).unwrap(),
            4 => self.spi.transfer(&mut buf[0..4]).unwrap(),
            _ => &[0],
        };
        self.cs.set_high().unwrap();
    }

    /// Reads the status register and returns the value.
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
        let datach = self.read_reg(AdcReg::DATA, 4);
        let ch = (datach & 0x3) as u8;
        let data = datach >> 8;
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

    /// Set both ADC channel filter config to the same settings.
    pub fn set_filters(&mut self, set: AdcFilterSettings) {
        let reg: u32 = (set.odr | set.order << 5 | set.enhfilt << 8 | set.enhfilten << 11) as u32;
        self.write_reg(AdcReg::FILTCON0, 2, reg);
        self.write_reg(AdcReg::FILTCON1, 2, reg);
    }
}
