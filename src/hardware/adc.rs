// Thermostat ADC struct.

use num_enum::TryFromPrimitive;
use shared_bus_rtic::SharedBus;

use super::ad7172::Ad7172;

use super::hal::{
    gpio::{gpioe::*, Alternate, Output, PushPull, AF5},
    hal::blocking::delay::DelayUs,
    hal::digital::v2::OutputPin,
    prelude::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::{Enabled, Spi},
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

pub struct AdcPins {
    pub cs: (
        PE0<Output<PushPull>>,
        PE1<Output<PushPull>>,
        PE3<Output<PushPull>>,
        PE4<Output<PushPull>>,
    ),
}

pub struct Adc {
    pub adcs: (
        Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE0<Output<PushPull>>>,
        Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE1<Output<PushPull>>>,
        Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE3<Output<PushPull>>>,
        Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE4<Output<PushPull>>>,
    ),
}

impl Adc {
    /// Construct a new ADC driver for all Thermostat input channels.
    ///
    /// # Args
    /// * `clocks` - Reference to CoreClocks
    /// * `spi4_rec` - Peripheral Reset and Enable Control for SPI4
    /// * `spi4` - SPI4 peripheral
    /// * `sck` - Spi sck pin
    /// * `miso` - Spi miso pin
    /// * `mosi` - Spi mosi pin
    /// * `pins` - ADC chip select pins.
    pub fn new(
        delay: &mut impl DelayUs<u16>,
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

        let bus_manager = shared_bus_rtic::new!(spi, Spi<SPI4, Enabled>);

        Adc {
            adcs: (
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.0).unwrap(),
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.1).unwrap(),
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.2).unwrap(),
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.3).unwrap(),
            ),
        }
    }
}
