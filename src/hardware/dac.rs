///! Thermostat DAC driver
///!
///! This file contains the driver for the 4 Thermostat DAC output channels.
///! To convert a 18 bit word into an analog current Thermostat uses a DAC to
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
use super::hal::{
    gpio::{gpioc::*, gpiog::*, Alternate, Output, PushPull, AF6},
    hal::{blocking::spi::Write, digital::v2::OutputPin},
    prelude::*,
    rcc::{rec, CoreClocks},
    spi::{Enabled, NoMiso, Spi, MODE_1},
    stm32::SPI3,
    time::MegaHertz,
};

use super::OutputChannel;

// Note: 30MHz clock valid according to DAC datasheet. This lead to spurious RxFIFO overruns on the STM side when probing the spi clock with a scope probe.
const SPI_CLOCK: MegaHertz = MegaHertz(8);

// DAC and PWM shared constants
pub const R_SENSE: f32 = 0.05; // TEC current sense resistor
pub const VREF_TEC: f32 = 1.5; // TEC driver reference voltage

/// DAC value out of bounds error.
#[derive(Debug)]
pub enum Error {
    Bounds,
}

/// DAC gpio pins.
///
/// sync<n> - DAC IC adressing signals
/// shdn<n> - TEC driver shutdown signals
/// * <n> specifies Thermostat output channel
#[allow(clippy::type_complexity)]
pub struct DacPins {
    pub sync: (
        PG3<Output<PushPull>>,
        PG2<Output<PushPull>>,
        PG1<Output<PushPull>>,
        PG0<Output<PushPull>>,
    ),
}

/// DAC driver struct containing the SPI bus and the gpio pins.
pub struct Dac {
    spi: Spi<SPI3, Enabled, u8>,
    gpio: DacPins,
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
        gpio: DacPins,
    ) -> Self {
        let spi = spi3.spi((sck, NoMiso, mosi), MODE_1, SPI_CLOCK, spi3_rec, clocks);

        let mut dac = Dac { spi, gpio };

        dac.gpio.sync.0.set_high().unwrap();
        dac.gpio.sync.1.set_high().unwrap();
        dac.gpio.sync.2.set_high().unwrap();
        dac.gpio.sync.3.set_high().unwrap();

        // default to zero current
        for i in 0..4 {
            let ch = OutputChannel::try_from(i).unwrap();
            dac.set_current(ch, 0.0).unwrap();
        }
        dac
    }

    /// Set the DAC output to current on a channel.
    ///
    /// # Args
    /// * `ch` - Thermostat output channel
    /// * `current` - Set current in Ampere
    pub fn set_current(&mut self, ch: OutputChannel, current: f32) -> Result<(), Error> {
        // DAC constants
        const MAX_DAC_WORD: i32 = 1 << 20; // maximum DAC dataword (exclusive) plus 2 bit due to interface alignment
        const VREF_DAC: f32 = 3.0; // DAC reference voltage

        // Current to DAC word conversion
        let ctli_voltage = (current * 10.0 * R_SENSE) + VREF_TEC;
        let dac_code = (ctli_voltage * (MAX_DAC_WORD as f32 / VREF_DAC)) as i32;

        if !(0..MAX_DAC_WORD).contains(&dac_code) {
            return Err(Error::Bounds);
        };

        let buf = &(dac_code as u32).to_be_bytes()[1..];

        match ch {
            OutputChannel::Zero => {
                self.gpio.sync.0.set_low().unwrap();
                // 24 bit write. 4 MSB are zero and 2 LSB are ignored for a 18 bit DAC output.
                self.spi.write(buf).unwrap();
                self.gpio.sync.0.set_high().unwrap();
            }
            OutputChannel::One => {
                self.gpio.sync.1.set_low().unwrap();
                self.spi.write(buf).unwrap();
                self.gpio.sync.1.set_high().unwrap();
            }
            OutputChannel::Two => {
                self.gpio.sync.2.set_low().unwrap();
                self.spi.write(buf).unwrap();
                self.gpio.sync.2.set_high().unwrap();
            }
            OutputChannel::Three => {
                self.gpio.sync.3.set_low().unwrap();
                self.spi.write(buf).unwrap();
                self.gpio.sync.3.set_high().unwrap();
            }
        }
        Ok(())
    }
}
