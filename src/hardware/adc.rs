// Thermostat ADC struct.

use num_enum::TryFromPrimitive;
use shared_bus_rtic::SharedBus;

use super::ad7172::Ad7172;

use super::hal::{
    gpio::{gpioe::*, Alternate, Output, PushPull, AF5},
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
    // spi: Spi<SPI4, Enabled, u8>,
    pub adcs: Ad7172<SharedBus<Spi<SPI4, Enabled>>>,
    // bus_manager: &'static _
}

impl Adc {
    /// Construct a new ADC driver for all Thermostat input channels.
    ///
    /// # Args
    /// * `spi4_rec` - Peripheral Reset and Enable Control for SPI4
    /// * `spi4` - SPI4 peripheral
    /// * `clocks` - Reference to CoreClocks
    /// * `pins` - ADC chip select pins.
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

        let bus_manager = shared_bus_rtic::new!(spi, Spi<SPI4, Enabled>);
        // shared_bus::BusManagerSimple::new(spi);

        let ad7172 = Ad7172::new(bus_manager.acquire(), pins.cs.0);

        let adc = Adc {
            adcs: ad7172,
            // bus_manager: bus_manager,
        };
        adc
    }
}
