// Thermostat ADC struct.

use num_enum::TryFromPrimitive;
use shared_bus_rtic::SharedBus;

use super::ad7172::{Ad7172, AdcReg, Adcmode, Channel, Filtcon, Ifmode, Setupcon};

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

        let mut adc = Adc {
            adcs: (
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.0).unwrap(),
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.1).unwrap(),
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.2).unwrap(),
                Ad7172::new(delay, bus_manager.acquire(), pins.cs.3).unwrap(),
            ),
        };

        Adc::setup_adc(&mut adc.adcs.0);
        Adc::setup_adc(&mut adc.adcs.1);
        Adc::setup_adc(&mut adc.adcs.2);
        Adc::setup_adc(&mut adc.adcs.3);

        adc
    }

    /// Setup an adc on Thermostat-EEM.
    fn setup_adc<CS>(adc: &mut Ad7172<SharedBus<Spi<SPI4, Enabled>>, CS>)
    where
        CS: OutputPin,
        <CS>::Error: core::fmt::Debug,
    {
        // Setup ADCMODE register. Internal reference, internal clock, no delay, continuous conversion.
        adc.write(
            AdcReg::ADCMODE,
            Adcmode::REF_EN | Adcmode::MODE_CONTINOUS_CONVERSION | Adcmode::CLOCKSEL_EXTERNAL_CLOCK,
        );

        // Setup IFMODE register. Only enable data stat to get channel info on conversions.
        adc.write(AdcReg::IFMODE, Ifmode::DATA_STAT);

        // enable first channel and configure Ain0, Ain1,
        // set config 0 for first channel.
        adc.write(
            AdcReg::CH0,
            Channel::SETUP_SEL_0 | Channel::AINPOS_AIN0 | Channel::AINNEG_AIN1,
        );

        // enable second channel and configure Ain2, Ain3,
        // set config 0 for second channel too.
        adc.write(
            AdcReg::CH1,
            Channel::SETUP_SEL_0 | Channel::AINPOS_AIN2 | Channel::AINNEG_AIN3,
        );

        // Setup firstconfiguration register
        adc.write(
            AdcReg::SETUPCON0,
            Setupcon::UNIPOLAR
                | Setupcon::REFBUFP
                | Setupcon::REFBUFN
                | Setupcon::AINBUFP
                | Setupcon::AINBUFN
                | Setupcon::REF_SEL_EXTERNAL,
        );

        // Setup first filter configuration register. 10Hz data rate. Sinc5Sinc1 Filter. No postfilter.
        adc.write(
            AdcReg::FILTCON0,
            Filtcon::ORDER_SINC5SINC1 | Filtcon::ODR_10,
        );
    }
}
