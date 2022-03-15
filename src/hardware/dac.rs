///! Thermostat DAC driver
///!
///! This file contains the driver for the 4 Thermostat DAC output channels.
///! To convert a 20 bit word into an analog current Thermostat uses a DAC to
///! convert the word into a voltage and a subsequent TEC driver IC that produces
///! a current proportional to the DAC voltage.
///!
///! The 4 channel DAC ICs share an SPI bus and are addressed using individual "sync"
///! signals, similar to a chip select signal.
///! The TEC driver ICs feature a shutdown mode controlled by a shutdown signal and
///! current limits controlled by another input voltage. The shutdown signal is aggregated
///! into the DAC driver using a gpio, while the current limits make use of the Thermostat
///! PWM driver.
///! DAC datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5680.pdf
///! TEC driver datasheet: https://datasheets.maximintegrated.com/en/ds/MAX1968-MAX1969.pdf
///!
use super::{
    hal::{
        gpio::{gpioc::*, gpiog::*, Alternate, Output, PushPull, AF6},
        hal::{blocking::spi::Write, digital::v2::OutputPin},
        prelude::*,
        rcc::{rec, CoreClocks},
        spi::{Enabled, NoMiso, Spi, MODE_1},
        stm32::SPI3,
        time::MegaHertz,
    },
    MAXCODE, R_SENSE, VREF_DAC, VREF_TEC,
};

use super::Channel;

// Note: 30MHz clock valid according to DAC datasheet. This lead to spurious RxFIFO overruns on the STM side when probing the spi clock with a scope probe.
const SPI_CLOCK: MegaHertz = MegaHertz(8);

/// Convert TEC drive current to dac code.
fn i_to_dac(i: f32) -> u32 {
    let v = (i * 10.0 * R_SENSE) + VREF_TEC;
    ((v * MAXCODE) / VREF_DAC) as u32
}

/// DAC value out of bounds error.
#[derive(Debug)]
pub struct Bounds;

/// DAC gpio pins.
///
/// sync<n> - DAC IC adressing signals
/// shdn<n> - TEC driver shutdown signals
/// * <n> specifies Thermostat output channel
pub struct DacGpio {
    pub sync0: PG3<Output<PushPull>>,
    pub sync1: PG2<Output<PushPull>>,
    pub sync2: PG1<Output<PushPull>>,
    pub sync3: PG0<Output<PushPull>>,
    pub shdn0: PG4<Output<PushPull>>,
    pub shdn1: PG5<Output<PushPull>>,
    pub shdn2: PG6<Output<PushPull>>,
    pub shdn3: PG7<Output<PushPull>>,
}

/// DAC driver struct containing the SPI bus and the gpio pins.
pub struct Dac {
    spi: Spi<SPI3, Enabled, u8>,
    gpio: DacGpio,
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
    /// * `gpio` - DAC gpios including DAC sync pins and TEC shutdown pins.
    pub fn new(
        clocks: &CoreClocks,
        spi3_rec: rec::Spi3,
        spi3: SPI3,
        sck: PC10<Alternate<AF6>>,
        mosi: PC12<Alternate<AF6>>,
        gpio: DacGpio,
    ) -> Self {
        let spi = spi3.spi((sck, NoMiso, mosi), MODE_1, SPI_CLOCK, spi3_rec, clocks);

        let mut dac = Dac { spi, gpio };

        dac.gpio.sync0.set_high().unwrap();
        dac.gpio.sync1.set_high().unwrap();
        dac.gpio.sync2.set_high().unwrap();
        dac.gpio.sync3.set_high().unwrap();

        // default to zero amps
        dac.set(0.0, Channel::Ch0).unwrap();
        dac.set(0.0, Channel::Ch1).unwrap();
        dac.set(0.0, Channel::Ch2).unwrap();
        dac.set(0.0, Channel::Ch3).unwrap();
        dac
    }

    /// Set the DAC output to current on a channel.
    ///
    /// # Args
    /// * `curr` - Set current in ampere
    /// * `ch` - Thermostat output channel
    pub fn set(&mut self, curr: f32, ch: Channel) -> Result<(), Bounds> {
        let value = i_to_dac(curr);
        if !(0..1 << 20).contains(&value) {
            return Err(Bounds);
        }

        match ch {
            Channel::Ch0 => {
                self.gpio.sync0.set_low().unwrap();
                // 24 bit write. 4 MSB and 2 LSB are ignored for a 20 bit DAC output.
                self.spi.write(&(value << 2).to_be_bytes()[1..]).unwrap();
                self.gpio.sync0.set_high().unwrap();
            }
            Channel::Ch1 => {
                self.gpio.sync1.set_low().unwrap();
                self.spi.write(&(value << 2).to_be_bytes()[1..]).unwrap();
                self.gpio.sync1.set_high().unwrap();
            }
            Channel::Ch2 => {
                self.gpio.sync2.set_low().unwrap();
                self.spi.write(&(value << 2).to_be_bytes()[1..]).unwrap();
                self.gpio.sync2.set_high().unwrap();
            }
            Channel::Ch3 => {
                self.gpio.sync3.set_low().unwrap();
                self.spi.write(&(value << 2).to_be_bytes()[1..]).unwrap();
                self.gpio.sync3.set_high().unwrap();
            }
        }
        Ok(())
    }

    /// Set or reset the shutdown pin of an output channel.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `shutdown` - TEC driver shutdown. True to set shutdown mode, false to reset shutdown mode.
    pub fn set_shutdown(&mut self, ch: Channel, shutdown: bool) {
        match (ch, shutdown) {
            (Channel::Ch0, true) => self.gpio.shdn0.set_high().unwrap(),
            (Channel::Ch1, true) => self.gpio.shdn1.set_high().unwrap(),
            (Channel::Ch2, true) => self.gpio.shdn2.set_high().unwrap(),
            (Channel::Ch3, true) => self.gpio.shdn3.set_high().unwrap(),
            (Channel::Ch0, false) => self.gpio.shdn0.set_low().unwrap(),
            (Channel::Ch1, false) => self.gpio.shdn1.set_low().unwrap(),
            (Channel::Ch2, false) => self.gpio.shdn2.set_low().unwrap(),
            (Channel::Ch3, false) => self.gpio.shdn3.set_low().unwrap(),
        }
    }
}
