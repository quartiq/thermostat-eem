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

pub enum AdcPhy {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

type Adcs = ad7172::Ad7172<Spi<SPI4, Enabled>>;

#[allow(clippy::complexity)]
/// All pins for all ADCs.
/// * `spi` - Spi clk, miso, mosi (in this order).
/// * `cs` - The four chip select pins.
/// * `rdyn` - ADC rdyn input (this is the same ad spi dout).
/// * `sync` - ADC sync pin (shared for all adc phys).
pub struct AdcPins {
    pub spi: (
        PE2<Alternate<AF5>>,
        PE5<Alternate<AF5>>,
        PE6<Alternate<AF5>>,
    ),
    pub cs: (
        PE0<Output<PushPull>>,
        PE1<Output<PushPull>>,
        PE3<Output<PushPull>>,
        PE4<Output<PushPull>>,
    ),
    pub rdyn: PC11<Input<PullUp>>,
    pub sync: PB11<Output<PushPull>>,
}

#[allow(clippy::complexity)]
pub struct Adc {
    pub adcs: Adcs,
    pub rdyn: PC11<Input<PullUp>>,
    pub sync: PB11<Output<PushPull>>,
    pub cs: (
        PE0<Output<PushPull>>,
        PE1<Output<PushPull>>,
        PE3<Output<PushPull>>,
        PE4<Output<PushPull>>,
    ),
    pub schedule_index: usize, // Currently active index into SCHEDULE
}

impl Adc {
    /// ADC data readout schedule.
    /// There are 4 physical ADCs present on Thermostat-EEM. Each of them has up to
    /// four individual input channels. To allow flexibility in the configuration of the
    /// channels, this readout schedule defines the ordering of readout of the channels.
    ///
    /// *Note*: The schedule has to  correspond to the configuration of the individual ADCs.
    /// For this specific schedule all the ADCs are configured the same and are synced so they
    /// all start sampling at the same time. The schedule now first reads out the first channel
    /// of each ADC (corresponding to Thermostat channels 0,2,4,6), then the second channel of
    /// each ADC (Thermostat channels 1,3,5,7) and then starts over.
    pub const SCHEDULE: [(AdcPhy, InputChannel); 8] = [
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
        // deassert all CS first
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
            schedule_index: 0,
        };

        adc.cs.0.set_low().unwrap();
        adc.setup_adc(delay);
        adc.cs.0.set_high().unwrap();
        adc.cs.1.set_low().unwrap();
        adc.setup_adc(delay);
        adc.cs.1.set_high().unwrap();
        adc.cs.2.set_low().unwrap();
        adc.setup_adc(delay);
        adc.cs.2.set_high().unwrap();
        adc.cs.3.set_low().unwrap();
        adc.setup_adc(delay);
        adc.cs.3.set_high().unwrap();

        // set sync high after initialization of all phys
        // TODO: double check timing after last setup and generally more datasheet studying for this
        adc.sync.set_high().unwrap();

        adc
    }

    /// Setup an adc on Thermostat-EEM.
    fn setup_adc(&mut self, delay: &mut impl DelayUs<u16>) {
        self.adcs.reset();

        delay.delay_us(500u16);

        let id = self.adcs.read(ad7172::AdcReg::ID);
        // check that ID is 0x00DX, as per datasheet
        if id & 0xfff0 != 0x00d0 {
            // return Err(Error::AdcId);
            // TODO return error insted of panicing here
            panic!();
        }

        // Setup ADCMODE register. Internal reference, internal clock, no delay, continuous conversion.
        self.adcs.write(
            ad7172::AdcReg::ADCMODE,
            ad7172::Adcmode::RefEn::ENABLED
                | ad7172::Adcmode::Mode::CONTINOUS_CONVERSION
                | ad7172::Adcmode::Clocksel::EXTERNAL_CLOCK,
        );

        // Setup IFMODE register. Only enable data stat to get channel info on conversions.
        self.adcs
            .write(ad7172::AdcReg::IFMODE, ad7172::Ifmode::DataStat::ENABLED);

        // enable first channel and configure Ain0, Ain1,
        // set config 0 for first channel.
        self.adcs.write(
            ad7172::AdcReg::CH0,
            ad7172::Channel::ChEn::ENABLED
                | ad7172::Channel::SetupSel::SETUP_0
                | ad7172::Channel::Ainpos::AIN0
                | ad7172::Channel::Ainneg::AIN1,
        );

        // enable second channel and configure Ain2, Ain3,
        // set config 0 for second channel too.
        self.adcs.write(
            ad7172::AdcReg::CH1,
            ad7172::Channel::ChEn::ENABLED
                | ad7172::Channel::SetupSel::SETUP_0
                | ad7172::Channel::Ainpos::AIN2
                | ad7172::Channel::Ainneg::AIN3,
        );

        // Setup firstconfiguration register
        self.adcs.write(
            ad7172::AdcReg::SETUPCON0,
            ad7172::Setupcon::BiUnipolar::UNIPOLAR
                | ad7172::Setupcon::Refbufn::ENABLED
                | ad7172::Setupcon::Refbufp::ENABLED
                | ad7172::Setupcon::Ainbufn::ENABLED
                | ad7172::Setupcon::Ainbufp::ENABLED
                | ad7172::Setupcon::Refsel::EXTERNAL,
        );

        // Setup first filter configuration register. 10Hz data rate. Sinc5Sinc1 Filter. No postfilter.
        self.adcs.write(
            ad7172::AdcReg::FILTCON0,
            ad7172::Filtcon::Order::SINC5SINC1 | ad7172::Filtcon::Odr::ODR_1_25,
        );

        // Re-apply (also set after ADC reset) SYNC_EN flag in gpio register for standard synchronization
        self.adcs
            .write(ad7172::AdcReg::GPIOCON, ad7172::Gpiocon::SyncEn::ENABLED);
    }

    /// Handle adc interrupt.
    /// 
    /// This routine is called every time the currently selected ADC on Thermostat reports that it has data ready
    /// to be read out by pulling the dout line low. It then reads out the ADC data via SPI and
    /// uses the SCHEDULE to decide which ADC will have data ready next. It then deselects the
    /// current ADC and selects the next in line. Finally it checks weather the data is from the
    /// expected ADC channel. The next ADC will then trigger the interrupt again once it has finished
    /// sampling (or when it is selected if it is done at this point) and the routine will start again. 
    /// Obviously at the beginning of the program the data readout has to be initiated by selecting one
    /// ADC manually, outside this routine.
    pub fn handle_interrupt(&mut self) -> (InputChannel, u32) {
        let (current_phy, ch) = &Self::SCHEDULE[self.schedule_index];
        let (data, status) = self.adcs.read_data();
        self.rdyn.clear_interrupt_pending_bit();
        set_cs!(self, current_phy, High);
        self.schedule_index = (self.schedule_index + 1) % Self::SCHEDULE.len();
        let (current_phy, _) = &Self::SCHEDULE[self.schedule_index];
        set_cs!(self, current_phy, Low);
        assert_eq!(status & 0x3, *ch as u8 & 1); // check if correct ADC input channel
        (*ch, data)
    }

    /// Initiate the sampling sequence.
    pub fn initiate_sampling(&mut self) {
        // select first adc to initiate sampling sequence
        let (first_phy, _) = &Self::SCHEDULE[self.schedule_index];
        set_cs!(self, first_phy, Low);
    }
}
