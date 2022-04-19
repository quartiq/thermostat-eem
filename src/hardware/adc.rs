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

use num_traits::float::Float;

/// A type representing an ADC sample.
#[derive(Copy, Clone, Debug, Format)]
pub struct AdcCode(pub u32);
impl AdcCode {
    const GAIN: f32 = 0x555555 as _; // Default ADC gain from datasheet.
    const R_REF: f32 = 2.0 * 5000.0; // Ratiometric resistor setup. 5.0K high and low side.
    const ZERO_C: f32 = 273.15; // 0°C in °K
    const B: f32 = 3988.0; // NTC beta value. TODO: This should probaply be changeable.
    const T_N: f32 = 25.0; // Reference Temperature for B-parameter equation.
    const R_N: f32 = 10000.0; // TEC resistance at T_N.
}

impl From<u32> for AdcCode {
    /// Construct an ADC code from a provided binary (ADC-formatted) code.
    fn from(value: u32) -> Self {
        Self(value)
    }
}

impl From<AdcCode> for f32 {
    /// Convert raw ADC codes to temperature value in °C using the AD7172 input voltage to code
    /// relation, the ratiometric resistor setup and the "B-parameter" equation (a simple form of the
    /// Steinhart-Hart equation). This is a treadeoff between computation and absolute temperature
    /// accuracy. The f32 output dataformat leads to an output quantization of about 31 uK.
    /// Additionally there is some error (in addition to the re-quantization) introduced during the
    /// various computation steps. If the input data has less than about 5 bit RMS noise, f32 should be
    /// avoided.
    /// Valid under the following conditions:
    /// * Unipolar ADC input
    /// * Unchanged ADC GAIN and OFFSET registers (default reset values)
    /// * Resistor setup as on Thermostat-EEM
    /// * Imput values not close to minimum/maximum (~1000 codes difference)
    fn from(code: AdcCode) -> f32 {
        // Inverted equation from datasheet p. 40 with V_Ref normalized to 1 as this cancels out in resistance.
        let relative_voltage =
            (code.0 as f32) * ((0x400000 as f32) / (2.0 * (1 << 23) as f32 * AdcCode::GAIN * 0.75));
        // Voltage divider normalized to V_Ref = 1, inverted to get to NTC resistance.
        let relative_resistance =
            (relative_voltage) / (1.0 - relative_voltage) * (AdcCode::R_REF / AdcCode::R_N);
        // https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
        let temperature_kelvin_inv = 1.0 / (AdcCode::T_N + AdcCode::ZERO_C)
            + (1.0 / AdcCode::B) * (relative_resistance).ln();
        (1.0 / temperature_kelvin_inv) - AdcCode::ZERO_C
    }
}

impl From<AdcCode> for f64 {
    /// Convert raw ADC codes to temperature value in °C using the AD7172 input voltage to code
    /// relation, the ratiometric resistor setup and the "B-parameter" equation (a simple form of the
    /// Steinhart-Hart equation). This is a treadeoff between computation and absolute temperature
    /// accuracy. The f64 dataformat should not limit the dynamic range or produce significant arithmetic
    /// errors.
    /// Valid under the following conditions:
    /// * Unipolar ADC input
    /// * Unchanged ADC GAIN and OFFSET registers (default reset values)
    /// * Resistor setup as on Thermostat-EEM
    /// * Imput values not close to minimum/maximum (~1000 codes difference)
    fn from(code: AdcCode) -> f64 {
        // Inverted equation from datasheet p. 40 with V_Ref normalized to 1 as this cancels out in resistance.
        let relative_voltage = (code.0 as f64)
            * ((0x400000 as f64) / (2.0 * (1 << 23) as f64 * AdcCode::GAIN as f64 * 0.75));
        // Voltage divider normalized to V_Ref = 1, inverted to get to NTC resistance.
        let relative_resistance = (relative_voltage) / (1.0 - relative_voltage)
            * (AdcCode::R_REF as f64 / AdcCode::R_N as f64);
        // https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
        let temperature_kelvin_inv = 1.0 / (AdcCode::T_N as f64 + AdcCode::ZERO_C as f64)
            + (1.0 / AdcCode::B as f64) * (relative_resistance).ln();
        (1.0 / temperature_kelvin_inv) - AdcCode::ZERO_C as f64
    }
}

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

#[derive(Clone, Copy, TryFromPrimitive, Debug, Format, PartialEq)]
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
    adcs: Adcs,
    pub rdyn: PC11<Input<PullUp>>,
    sync: PB11<Output<PushPull>>,
    cs: (
        PE0<Output<PushPull>>,
        PE1<Output<PushPull>>,
        PE3<Output<PushPull>>,
        PE4<Output<PushPull>>,
    ),
    schedule_index: usize, // Currently active index into SCHEDULE
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
    /// * `delay` - delay struct with DelayUs implementation
    /// * `clocks` - Reference to CoreClocks
    /// * `spi4_rec` - Peripheral Reset and Enable Control for SPI4
    /// * `spi4` - SPI4 peripheral
    /// * `pins` - All ADC pins
    pub fn new(
        delay: &mut impl DelayUs<u16>,
        clocks: &CoreClocks,
        spi4_rec: rec::Spi4,
        spi4: SPI4,
        pins: AdcPins,
    ) -> Self {
        // SPI MODE_3: idle high, capture on second transition
        let spi: Spi<_, _, u8> = spi4.spi(pins.spi, spi::MODE_3, 12500.khz(), spi4_rec, clocks);

        let mut adc = Adc {
            adcs: ad7172::Ad7172::new(spi),
            rdyn: pins.rdyn,
            sync: pins.sync,
            cs: pins.cs,
            schedule_index: 0,
        };

        adc.setup(delay);
        adc
    }

    fn setup(&mut self, delay: &mut impl DelayUs<u16>) {
        // deassert all CS first
        self.cs.0.set_high().unwrap();
        self.cs.1.set_high().unwrap();
        self.cs.2.set_high().unwrap();
        self.cs.3.set_high().unwrap();

        // set sync low first for synchronization at rising edge
        self.sync.set_low().unwrap();

        self.cs.0.set_low().unwrap();
        self.setup_adc(delay);
        self.cs.0.set_high().unwrap();
        self.cs.1.set_low().unwrap();
        self.setup_adc(delay);
        self.cs.1.set_high().unwrap();
        self.cs.2.set_low().unwrap();
        self.setup_adc(delay);
        self.cs.2.set_high().unwrap();
        self.cs.3.set_low().unwrap();
        self.setup_adc(delay);
        self.cs.3.set_high().unwrap();

        // set sync high after initialization of all ADCs
        self.sync.set_high().unwrap();

        // set up sampling sequence by selection first ADC according to schedule
        self.rdyn.clear_interrupt_pending_bit();
        let (current_phy, _) = &Self::SCHEDULE[self.schedule_index];
        set_cs!(self, current_phy, Low);
    }

    /// Setup an ADC on Thermostat-EEM.
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

        self.adcs.write(
            ad7172::AdcReg::ADCMODE,
            ad7172::Adcmode::RefEn::ENABLED
                | ad7172::Adcmode::Mode::CONTINOUS_CONVERSION
                | ad7172::Adcmode::Clocksel::EXTERNAL_CLOCK,
        );

        self.adcs
            .write(ad7172::AdcReg::IFMODE, ad7172::Ifmode::DataStat::ENABLED);

        self.adcs.write(
            ad7172::AdcReg::CH0,
            ad7172::Channel::ChEn::ENABLED
                | ad7172::Channel::SetupSel::SETUP_0
                | ad7172::Channel::Ainpos::AIN0
                | ad7172::Channel::Ainneg::AIN1,
        );

        self.adcs.write(
            ad7172::AdcReg::CH1,
            ad7172::Channel::ChEn::ENABLED
                | ad7172::Channel::SetupSel::SETUP_0
                | ad7172::Channel::Ainpos::AIN2
                | ad7172::Channel::Ainneg::AIN3,
        );

        self.adcs.write(
            ad7172::AdcReg::SETUPCON0,
            ad7172::Setupcon::BiUnipolar::UNIPOLAR
                | ad7172::Setupcon::Refbufn::ENABLED
                | ad7172::Setupcon::Refbufp::ENABLED
                | ad7172::Setupcon::Ainbufn::ENABLED
                | ad7172::Setupcon::Ainbufp::ENABLED
                | ad7172::Setupcon::Refsel::EXTERNAL,
        );

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
    /// uses the SCHEDULE to decide which ADC will have data ready next. It then clears the intrrupt pending flag
    /// (which does not trigger an interrupt right away since the currently selected ADC does not have new data),
    /// deselects the current ADC and selects the next in line. Finally it checks weather the data is from the
    /// expected ADC channel. The next ADC will then trigger the interrupt again once it has finished
    /// sampling (or when it is selected if it is done at this point) and the routine will start again.
    /// Obviously at the beginning of the program the data readout has to be initiated by selecting one
    /// ADC manually, outside this routine.
    pub fn handle_interrupt(&mut self) -> (InputChannel, AdcCode) {
        let (current_phy, ch) = &Self::SCHEDULE[self.schedule_index];
        let (data, status) = self.adcs.read_data();
        self.rdyn.clear_interrupt_pending_bit();
        set_cs!(self, current_phy, High);
        self.schedule_index = (self.schedule_index + 1) % Self::SCHEDULE.len();
        let (current_phy, _) = &Self::SCHEDULE[self.schedule_index];
        set_cs!(self, current_phy, Low);
        assert_eq!(status & 0x3, *ch as u8 & 1); // check if correct ADC input channel
        (*ch, data.into())
    }
}
