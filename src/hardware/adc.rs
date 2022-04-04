// Thermostat ADC struct.

use defmt::{info, Format};
use num_enum::TryFromPrimitive;
use shared_bus_rtic::SharedBus;

use super::ad7172;

use super::hal::{
    gpio::{
        gpiob::*, gpioc::*, gpioe::*, Alternate, ExtiPin, Input, Output, PullUp, PushPull, AF5,
    },
    hal::blocking::delay::DelayUs,
    hal::digital::v2::OutputPin,
    prelude::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::{Enabled, Spi},
    stm32::SPI4,
};

macro_rules! read_adc {
    ($self:ident, $adcs:ident, $adc:tt, $nextadc:tt) => {{
        let data = $self.$adcs.$adc.read_data();
        $self.rdyn.clear_interrupt_pending_bit();
        $self.current += 1;
        if $self.current >= Self::SCHEDULE.len() {
            $self.current = 0;
        }
        $self.$adcs.$nextadc.set_cs(false);
        data
    }};
}

#[derive(Clone, Copy, TryFromPrimitive, Debug, Format)]
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
#[derive(Clone, Copy)]

pub enum AdcPhy {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

type O = Output<PushPull>;
type Adcs = (
    ad7172::Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE0<O>>,
    ad7172::Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE1<O>>,
    ad7172::Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE3<O>>,
    ad7172::Ad7172<SharedBus<Spi<SPI4, Enabled>>, PE4<O>>,
);
type SpiPins = (
    PE2<Alternate<AF5>>,
    PE5<Alternate<AF5>>,
    PE6<Alternate<AF5>>,
);
/// All pins for all ADCs.
/// * `spi` - Spi clk, miso, mosi (in this order).
/// * `cs` - The four chip select pins.
/// * `rdyn` - ADC rdyn input (this is the same ad spi dout).
/// * `sync` - ADC sync pin (shared for all adc phys).
pub struct AdcPins {
    pub spi: SpiPins,
    pub cs: (PE0<O>, PE1<O>, PE3<O>, PE4<O>),
    pub rdyn: PC11<Input<PullUp>>,
    pub sync: PB11<O>,
}

pub struct Adc {
    pub adcs: Adcs,
    pub rdyn: PC11<Input<PullUp>>,
    pub sync: PB11<O>,
    pub current: usize, // Schedule position
}

impl Adc {
    const SCHEDULE: [(AdcPhy, InputChannel); 8] = [
        (AdcPhy::Zero, InputChannel::Zero),
        (AdcPhy::One, InputChannel::Two),
        (AdcPhy::Two, InputChannel::Four),
        (AdcPhy::Three, InputChannel::Six),
        (AdcPhy::Zero, InputChannel::One),
        (AdcPhy::One, InputChannel::Three),
        (AdcPhy::Two, InputChannel::Five),
        (AdcPhy::Three, InputChannel::Seven),
    ];
    /// Construct a new ADC driver for all Thermostat input channels.
    ///
    /// # Args
    /// * `clocks` - Reference to CoreClocks
    /// * `spi4_rec` - Peripheral Reset and Enable Control for SPI4
    /// * `spi4` - SPI4 peripheral
    /// * `pins` - All ADC pins
    pub fn new(
        delay: &mut impl DelayUs<u16>,
        clocks: &CoreClocks,
        spi4_rec: rec::Spi4,
        spi4: SPI4,
        mut pins: AdcPins,
    ) -> Self {
        // set all CS high first
        pins.cs.0.set_high().unwrap();
        pins.cs.1.set_high().unwrap();
        pins.cs.2.set_high().unwrap();
        pins.cs.3.set_high().unwrap();

        // set sync low first for synchronization at rising edge
        pins.sync.set_low().unwrap();

        // SPI at 1 MHz. SPI MODE_0: idle low, capture on first transition
        let spi: Spi<_, _, u8> = spi4.spi(pins.spi, spi::MODE_0, 12500.khz(), spi4_rec, clocks);

        let bus_manager = shared_bus_rtic::new!(spi, Spi<SPI4, Enabled>);

        let mut adc = Adc {
            adcs: (
                ad7172::Ad7172::new(delay, bus_manager.acquire(), pins.cs.0).unwrap(),
                ad7172::Ad7172::new(delay, bus_manager.acquire(), pins.cs.1).unwrap(),
                ad7172::Ad7172::new(delay, bus_manager.acquire(), pins.cs.2).unwrap(),
                ad7172::Ad7172::new(delay, bus_manager.acquire(), pins.cs.3).unwrap(),
            ),
            rdyn: pins.rdyn,
            sync: pins.sync,
            current: 0,
        };

        Adc::setup_adc(&mut adc.adcs.0);
        Adc::setup_adc(&mut adc.adcs.1);
        Adc::setup_adc(&mut adc.adcs.2);
        Adc::setup_adc(&mut adc.adcs.3);

        // set sync high after initialization of all phys
        // TODO: double check timing after last setup and generally more datasheet studying for this
        adc.sync.set_high().unwrap();

        adc
    }

    /// Setup an adc on Thermostat-EEM.
    fn setup_adc<CS>(adc: &mut ad7172::Ad7172<SharedBus<Spi<SPI4, Enabled>>, CS>)
    where
        CS: OutputPin,
        <CS>::Error: core::fmt::Debug,
    {
        // Setup ADCMODE register. Internal reference, internal clock, no delay, continuous conversion.
        adc.write(
            ad7172::AdcReg::ADCMODE,
            ad7172::Adcmode::RefEn::ENABLED
                | ad7172::Adcmode::Mode::CONTINOUS_CONVERSION
                | ad7172::Adcmode::Clocksel::EXTERNAL_CLOCK,
        );

        // Setup IFMODE register. Only enable data stat to get channel info on conversions.
        adc.write(ad7172::AdcReg::IFMODE, ad7172::Ifmode::DataStat::ENABLED);

        // enable first channel and configure Ain0, Ain1,
        // set config 0 for first channel.
        adc.write(
            ad7172::AdcReg::CH0,
            ad7172::Channel::ChEn::ENABLED
                | ad7172::Channel::SetupSel::SETUP_0
                | ad7172::Channel::Ainpos::AIN0
                | ad7172::Channel::Ainneg::AIN1,
        );

        // enable second channel and configure Ain2, Ain3,
        // set config 0 for second channel too.
        adc.write(
            ad7172::AdcReg::CH1,
            ad7172::Channel::ChEn::ENABLED
                | ad7172::Channel::SetupSel::SETUP_0
                | ad7172::Channel::Ainpos::AIN2
                | ad7172::Channel::Ainneg::AIN3,
        );

        // Setup firstconfiguration register
        adc.write(
            ad7172::AdcReg::SETUPCON0,
            ad7172::Setupcon::BiUnipolar::UNIPOLAR
                | ad7172::Setupcon::Refbufn::ENABLED
                | ad7172::Setupcon::Refbufp::ENABLED
                | ad7172::Setupcon::Ainbufn::ENABLED
                | ad7172::Setupcon::Ainbufp::ENABLED
                | ad7172::Setupcon::Refsel::EXTERNAL,
        );

        // Setup first filter configuration register. 10Hz data rate. Sinc5Sinc1 Filter. No postfilter.
        adc.write(
            ad7172::AdcReg::FILTCON0,
            ad7172::Filtcon::Order::SINC5SINC1 | ad7172::Filtcon::Odr::ODR_1_25,
        );

        // Re-apply (also set after ADC reset) SYNC_EN flag in gpio register for standard synchronization
        adc.write(ad7172::AdcReg::GPIOCON, ad7172::Gpiocon::SyncEn::ENABLED);
    }

    /// Handle adc interrupt.
    pub fn handle_interrupt(&mut self) -> (InputChannel, u32) {
        let (phy, ch) = Self::SCHEDULE[self.current];
        let (data, status) = match phy {
            AdcPhy::Zero => read_adc!(self, adcs, 0, 1),
            AdcPhy::One => read_adc!(self, adcs, 1, 2),
            AdcPhy::Two => read_adc!(self, adcs, 2, 3),
            AdcPhy::Three => read_adc!(self, adcs, 3, 0),
        };
        info!("ch: {:?}", ch as u8);
        info!("status: {:?}", status);
        assert_eq!(status & 0x3, ch as u8 & 1); // check if correct input channels

        (ch, data) // data as °C
    }
}
