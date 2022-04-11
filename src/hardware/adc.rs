// Thermostat ADC struct.

use defmt::Format;
use num_enum::TryFromPrimitive;

use super::ad7172;

use super::hal::{
    gpio::{
        gpiob::*, gpioc::*, gpioe::*, Alternate, ExtiPin, Input, Output, PullUp, PushPull, AF5,
    },
    hal::blocking::delay::DelayUs,
    hal::digital::v2::OutputPin,
    hal::digital::v2::PinState::{High, Low},
    prelude::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::{Enabled, Spi},
    stm32::SPI4,
};

macro_rules! set_cs {
    ($self:ident, $phy:ident, $state:ident) => {
        match $phy {
            AdcPhy::Zero => $self.cs.0.set_state($state).unwrap(),
            AdcPhy::One => $self.cs.1.set_state($state).unwrap(),
            AdcPhy::Two => $self.cs.2.set_state($state).unwrap(),
            AdcPhy::Three => $self.cs.3.set_state($state).unwrap(),
        };
    };
}

macro_rules! setup_adc {
    ($adc:ident, $delay:ident, $phy:tt) => {
        $adc.cs.$phy.set_low().unwrap();
        Adc::setup_adc(&mut $adc.adcs, $delay);
        $adc.cs.$phy.set_high().unwrap();
    };
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
type Adcs = ad7172::Ad7172<Spi<SPI4, Enabled>>;
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
    pub cs: (PE0<O>, PE1<O>, PE3<O>, PE4<O>),
    pub current_position: usize, // Schedule position
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

        // SPI MODE_3: idle high, capture on second transition
        let spi: Spi<_, _, u8> = spi4.spi(pins.spi, spi::MODE_3, 12500.khz(), spi4_rec, clocks);

        let mut adc = Adc {
            adcs: ad7172::Ad7172::new(spi),
            rdyn: pins.rdyn,
            sync: pins.sync,
            cs: pins.cs,
            current_position: 0,
        };

        setup_adc!(adc, delay, 0);
        setup_adc!(adc, delay, 1);
        setup_adc!(adc, delay, 2);
        setup_adc!(adc, delay, 3);

        // set sync high after initialization of all phys
        // TODO: double check timing after last setup and generally more datasheet studying for this
        adc.sync.set_high().unwrap();

        adc
    }

    /// Setup an adc on Thermostat-EEM.
    fn setup_adc(adc: &mut Adcs, delay: &mut impl DelayUs<u16>) {
        adc.reset();

        delay.delay_us(500u16);

        let id = adc.read(ad7172::AdcReg::ID);
        // check that ID is 0x00DX, as per datasheet
        if id & 0xfff0 != 0x00d0 {
            // return Err(Error::AdcId);
            // TODO return error insted of panicing here
            panic!();
        }

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
        let (current_phy, ch) = Self::SCHEDULE[self.current_position];

        let (data, status) = self.adcs.read_data();

        self.rdyn.clear_interrupt_pending_bit();

        set_cs!(self, current_phy, High);

        self.current_position = (self.current_position + 1) % Self::SCHEDULE.len();

        let (current_phy, _) = Self::SCHEDULE[self.current_position];

        set_cs!(self, current_phy, Low);

        assert_eq!(status & 0x3, ch as u8 & 1); // check if correct input channel

        (ch, data) // data as °C
    }

    /// Initiate the sampling sequence.
    pub fn initiate_sampling(&mut self) {
        // select first adc to initiate sampling sequence
        let (first_phy, _) = Self::SCHEDULE[self.current_position];
        set_cs!(self, first_phy, Low);
    }
}
