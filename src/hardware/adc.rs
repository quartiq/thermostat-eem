// Thermostat ADC driver
// (AD7172 https://www.analog.com/media/en/technical-documentation/data-sheets/AD7172-2.pdf)

use byteorder::{BigEndian, ByteOrder};
use log::{info, warn};
use num_enum::TryFromPrimitive;

use super::hal::{
    gpio::{gpioe::*, Alternate, Output, PushPull, AF5},
    hal::{blocking::spi::Transfer, blocking::spi::Write, digital::v2::OutputPin},
    prelude::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::Enabled,
    spi::Spi,
    stm32::SPI4,
};

#[derive(Clone, Copy, TryFromPrimitive)]
#[repr(usize)]
pub enum InputChannel {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
    Six = 6,
    Seven = 7,
}

// Physical ADC devices on Thermostat
pub enum AdcPhy {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

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

pub struct AdcPins {
    pub cs: (
        PE0<Output<PushPull>>,
        PE1<Output<PushPull>>,
        PE3<Output<PushPull>>,
        PE4<Output<PushPull>>,
    ),
}

pub struct Adc {
    spi: Spi<SPI4, Enabled, u8>,
    pins: AdcPins,
}

impl Adc {
    /// Construct a new ADC driver for all Thermostat input channels.
    ///
    /// # Args
    /// * `spi4_rec` - Peripheral Reset and Enable Control for SPI4
    /// * `spi4` - SPI4 peripheral
    /// * `clocks` - Reference to CoreClocks
    /// * `pins` - DAC chip select pins.
    pub fn new(
        clocks: &CoreClocks,
        spi4_rec: rec::Spi4,
        spi4: SPI4,
        sck: PE2<Alternate<AF5>>,
        miso: PE5<Alternate<AF5>>,
        mosi: PE6<Alternate<AF5>>,
        mut pins: AdcPins,
    ) -> Self {
        // set all CS high first
        pins.cs.0.set_high().unwrap();
        pins.cs.1.set_high().unwrap();
        pins.cs.2.set_high().unwrap();
        pins.cs.3.set_high().unwrap();

        // SPI at 1 MHz. SPI MODE_0: idle low, capture on first transition
        let spi: Spi<_, _, u8> =
            spi4.spi((sck, miso, mosi), spi::MODE_0, 1.mhz(), spi4_rec, clocks);
        let mut adc = Adc { spi, pins };
        adc.reset();

        info!("ADC ID: {:#X}", adc.read_reg(AdcReg::ID, 2));

        // Setup ADCMODE register. Internal reference, external clock, no delay, continuous conversion.
        adc.write_reg(AdcReg::ADCMODE, 2, 0x8008);

        // Setup IFMODE register. Only enable data stat to get channel info on conversions.
        adc.write_reg(AdcReg::IFMODE, 2, 0b100_0000);

        adc.setup_channels();

        adc
    }

    /// Reset ADC.  TODO: forward error up.
    pub fn reset(&mut self) {
        let mut buf = [0xFFu8; 8];
        self.pins.cs.0.set_low().unwrap();
        let result = self.spi.transfer(&mut buf);
        self.pins.cs.0.set_high().unwrap();
        match result {
            Err(e) => {
                warn!("ADC reset failed! {:?}", e)
            }
            Ok(_) => {
                info!("ADC reset succeeded")
            }
        };
    }

    /// Read a ADC register of size in bytes. Max. size 3 bytes.
    fn read_reg(&mut self, addr: AdcReg, size: usize) -> u32 {
        self.pins.cs.0.set_low().unwrap();
        let mut buf = [0u8; 4];
        buf[3 - size] = addr as u8 | 0x40; // addr with read flag
        self.spi.transfer(&mut buf[3 - size..]).unwrap();
        let data = u32::from_be_bytes(buf) & ((1 << size * 8) - 1);
        self.pins.cs.0.set_high().unwrap();
        return data;
    }

    /// Write a ADC register of size in bytes. Max. size 3 bytes.
    fn write_reg(&mut self, addr: AdcReg, size: usize, data: u32) {
        self.pins.cs.0.set_low().unwrap();
        let mut buf = data.to_be_bytes();
        buf[4 - size] = addr as _;
        self.spi.write(&buf[4 - size..]).unwrap();
        self.pins.cs.0.set_high().unwrap();
    }

    /// Reads the status register and returns the value.
    pub fn get_status_reg(&mut self) -> u8 {
        let mut addr_buf = [0];
        self.pins.cs.0.set_low().unwrap();
        self.spi.transfer(&mut addr_buf).unwrap();
        self.pins.cs.0.set_high().unwrap();
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
}
