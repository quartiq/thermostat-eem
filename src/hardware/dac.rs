use defmt::info;

///! Thermostat DAC/TEC driver
///!
///! This file contains all of the drivers to convert an 18 bit word to an analog current.
///! On Thermostat this used the ad5680 DAC and the MAX1968 TEC driver. The (analog voltage)
///! max output voltages/current settings are driven by PWMs of the STM32.
use super::{
    hal::{
        gpio::{gpioc::*, gpiog::*, Alternate, Output, PushPull, AF6},
        hal::{blocking::spi::Write, digital::v2::OutputPin},
        prelude::*,
        rcc::{rec, CoreClocks},
        spi::{Enabled, NoMiso, Spi, MODE_1},
        stm32::SPI3,
        time::{KiloHertz, MegaHertz},
    },
    MAXCODE, R_SENSE, VREF_DAC, VREF_TEC,
};

use super::Channel;

const SPI_CLOCK: MegaHertz = MegaHertz(30); // DAC SPI clock speed
const MAX_VALUE: u32 = 0x3FFFF; // Maximum DAC output value
const F_PWM: KiloHertz = KiloHertz(20); // PWM freqency. 20kHz is ~80dB down with the installed second order 160Hz lowpass

/// Convert TEC drive current to dac code.
fn i_to_dac(i: f32) -> u32 {
    let v = (i * 10.0 * R_SENSE) + VREF_TEC;
    ((v * MAXCODE) / VREF_DAC) as u32
}

/// Convert dac code to TEC drive current.
fn dac_to_i(val: u32) -> f32 {
    let v = VREF_DAC * (val as f32 / MAXCODE);
    (v - VREF_TEC) / (10.0 * R_SENSE)
}

/// DAC value out of bounds error.
#[derive(Debug)]
pub struct Bounds;

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

/// DAC: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5680.pdf
/// Peltier Driver: https://datasheets.maximintegrated.com/en/ds/MAX1968-MAX1969.pdf
pub struct Dac {
    spi: Spi<SPI3, Enabled, u8>,
    gpio: DacGpio,
}

impl Dac {
    pub fn new(
        clocks: &CoreClocks,
        prec: rec::Spi3,
        spi3: SPI3,
        sck: PC10<Alternate<AF6>>,
        mosi: PC12<Alternate<AF6>>,
        gpio: DacGpio,
    ) -> Self {
        let spi = spi3.spi(
            (sck, NoMiso, mosi),
            MODE_1,
            SPI_CLOCK,
            prec,
            clocks, // default pll1_q clock source
        );

        let mut dac = Dac { spi, gpio };

        dac.gpio.sync0.set_high().unwrap();
        dac.gpio.sync1.set_high().unwrap();
        dac.gpio.sync2.set_high().unwrap();
        dac.gpio.sync3.set_high().unwrap();

        // default to zero amps
        dac.set(0.0, Channel::Ch0);
        dac.set(0.0, Channel::Ch1);
        dac.set(0.0, Channel::Ch2);
        dac.set(0.0, Channel::Ch3);
        dac
    }

    /// Set the DAC output to current on a channel.
    pub fn set(&mut self, curr: f32, ch: Channel) -> Result<(), Bounds> {
        let value = i_to_dac(curr);
        if !(0..1 << 20).contains(&value) {
            return Err(Bounds);
        }

        match ch {
            Channel::Ch0 => {
                self.gpio.sync0.set_low().unwrap();
                self.spi.write(&mut (value << 2).to_be_bytes()[1..]); // 24 bit write. 4 MSB and 2 LSB are ignored for a 20 bit DAC output.
                self.gpio.sync0.set_high().unwrap();
            }
            Channel::Ch1 => {
                self.gpio.sync1.set_low().unwrap();
                self.spi.write(&mut (value << 2).to_be_bytes()[1..]); // 24 bit write. 4 MSB and 2 LSB are ignored for a 20 bit DAC output.
                self.gpio.sync1.set_high().unwrap();
            }
            Channel::Ch2 => {
                self.gpio.sync2.set_low().unwrap();
                self.spi.write(&mut (value << 2).to_be_bytes()[1..]); // 24 bit write. 4 MSB and 2 LSB are ignored for a 20 bit DAC output.
                self.gpio.sync2.set_high().unwrap();
            }
            Channel::Ch3 => {
                self.gpio.sync3.set_low().unwrap();
                self.spi.write(&mut (value << 2).to_be_bytes()[1..]); // 24 bit write. 4 MSB and 2 LSB are ignored for a 20 bit DAC output.
                self.gpio.sync3.set_high().unwrap();
            }
        }
        Ok(())
    }

    /// Sets or resets the shutdown pin of an output channel.
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
