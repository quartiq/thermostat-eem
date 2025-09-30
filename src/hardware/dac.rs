//! Thermostat DAC driver
//!
//! This file contains the driver for the 4 Thermostat DAC output channels.
//! To convert a 18 bit word into an analog current Thermostat uses a DAC to
//! convert the word into a voltage and a subsequent TEC driver IC that produces
//! a current proportional to the DAC voltage.
//!
//! The 4 channel DAC ICs share an SPI bus and are addressed using individual "sync"
//! signals, similar to a chip select signal.
//! DAC datasheet: `<https://www.analog.com/media/en/technical-documentation/data-sheets/AD5680.pdf>`
//! TEC driver datasheet: `<https://datasheets.maximintegrated.com/en/ds/MAX1968-MAX1969.pdf>`
//!

use super::hal::{
    gpio::{self, gpioc},
    hal_02::blocking::spi::Write,
    prelude::*,
    rcc, spi, stm32,
    time::MegaHertz,
};

use crate::OutputChannelIdx;
use crate::convert::DacCode;

// Note: Up to 30MHz clock valid according to DAC datasheet. This lead to spurious RxFIFO overruns on the STM side when probing the spi clock with a scope probe.
const SPI_CLOCK: MegaHertz = MegaHertz::MHz(8);

/// DAC gpio pins.
///
/// `sync[n]` - DAC IC adressing signals
///   where `n` specifies Thermostat output channel
pub struct DacPins {
    pub sync: [gpio::ErasedPin<gpio::Output>; 4],
}

/// DAC driver struct containing the SPI bus and the gpio pins.
pub struct Dac {
    spi: spi::Spi<stm32::SPI3, spi::Enabled, u8>,
    pins: DacPins,
}

impl Dac {
    /// Construct a new DAC driver for all Thermostat output channels.
    ///
    /// # Args
    /// * `clocks` - Reference to CoreClocks
    /// * `spi3_rec` - Peripheral Reset and Enable Control for SPI3
    /// * `spi3` - SPI3 peripheral
    /// * `sck` - SPI3 sck pin
    /// * `mosi` - SPI3 mosi pin
    /// * `pins` - DAC sync pins.
    pub fn new(
        clocks: &rcc::CoreClocks,
        spi3_rec: rcc::rec::Spi3,
        spi3: stm32::SPI3,
        sck: gpioc::PC10<gpio::Alternate<6>>,
        mosi: gpioc::PC12<gpio::Alternate<6>>,
        pins: DacPins,
    ) -> Self {
        let spi = spi3.spi(
            (sck, spi::NoMiso, mosi),
            spi::MODE_1,
            SPI_CLOCK.convert(),
            spi3_rec,
            clocks,
        );

        let mut dac = Dac { spi, pins };
        for pin in dac.pins.sync.iter_mut() {
            pin.set_high();
        }

        // default to zero current
        for i in OutputChannelIdx::ALL {
            dac.set(i, (0.0).try_into().unwrap());
        }
        dac
    }

    /// Set the DAC output on a channel.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `dac_code` - dac output code to transfer
    pub fn set(&mut self, ch: OutputChannelIdx, dac_code: DacCode) {
        self.pins.sync[ch as usize].set_low();
        // 24 bit write. 4 MSB are zero and 2 LSB are ignored for a 18 bit DAC output.
        let buf = u32::from(dac_code).to_be_bytes();
        self.spi.write(&buf[1..]).unwrap();
        self.pins.sync[ch as usize].set_high();
    }
}
