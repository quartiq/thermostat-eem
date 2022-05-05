// Thermostat ADC struct.

use defmt::Format;
use enum_iterator::IntoEnumIterator;
use num_enum::{TryFromPrimitive, TryFromPrimitiveError};
use smlang::statemachine;

use super::ad7172::{self, AdcChannel};

use super::hal::{
    self, device,
    gpio::{self, gpiob, gpioc, gpioe, ExtiPin},
    hal::blocking::delay::DelayUs,
    hal::digital::v2::PinState,
    prelude::*,
    rcc, spi, stm32,
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
    const T_N: f32 = 25.0 + AdcCode::ZERO_C; // Reference Temperature for B-parameter equation.
    const R_N: f32 = 10000.0; // TEC resistance at T_N.

    // ADC relative full scale per LSB
    // Inverted equation from datasheet p. 40 with V_Ref normalized to 1 as this cancels out in resistance.
    const FS_PER_LSB: f32 = 0x400000 as f32 / (2.0 * (1 << 23) as f32 * AdcCode::GAIN * 0.75);
    // Relative resistance
    const R_REF_N: f32 = AdcCode::R_REF / AdcCode::R_N;
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
        let relative_voltage = code.0 as f32 * AdcCode::FS_PER_LSB;
        // Voltage divider normalized to V_Ref = 1, inverted to get to NTC resistance.
        let relative_resistance = relative_voltage / (1.0 - relative_voltage) * AdcCode::R_REF_N;
        // https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
        let temperature_kelvin_inv =
            1.0 / AdcCode::T_N + 1.0 / AdcCode::B * relative_resistance.ln();
        1.0 / temperature_kelvin_inv - AdcCode::ZERO_C
    }
}

impl From<AdcCode> for f64 {
    /// Like `From<AdcCode> for f32` but for `f64` and correspondingly higher dynamic rande.
    fn from(code: AdcCode) -> f64 {
        let relative_voltage = (code.0 as f32 * AdcCode::FS_PER_LSB) as f64;
        let relative_resistance =
            relative_voltage / (1.0 - relative_voltage) * AdcCode::R_REF_N as f64;
        let temperature_kelvin_inv =
            1.0 / AdcCode::T_N as f64 + 1.0 / AdcCode::B as f64 * relative_resistance.ln();
        1.0 / temperature_kelvin_inv - AdcCode::ZERO_C as f64
    }
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

/// Decode a tuple of `(AdcPhy, AdcChannel)` to a homogeneous Thermostat input channel index.
/// This is not the sampling schedule but merely the mapping between thermostat
/// channel and ADC phy/channel. It would change if fewer or single-ended channels would be used.
impl TryFrom<(AdcPhy, AdcChannel)> for InputChannel {
    type Error = TryFromPrimitiveError<Self>;
    fn try_from((phy, ch): (AdcPhy, AdcChannel)) -> Result<Self, Self::Error> {
        Self::try_from(((phy as usize) << 1) + ch as usize)
    }
}

#[derive(Clone, Copy, TryFromPrimitive, Debug, Format, IntoEnumIterator)]
#[repr(usize)]
pub enum AdcPhy {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

impl AdcPhy {
    /// ADC phy readout schedule.
    /// There are 4 physical ADCs present on Thermostat-EEM. Each of them has up to
    /// four individual input channels. To allow flexibility in the configuration of the
    /// channels, this readout schedule defines the ordering of readout of the channels.
    ///
    /// *Note*: The schedule has to  correspond to the configuration of the individual ADCs.
    /// For this specific schedule all the ADCs are configured the same and are synced so they
    /// all start sampling at the same time. The schedule now first reads out each phy
    /// round-robin.
    /// This corresponds to the sequence of Thermostat channels 0,2,4,6,1,3,5,7.
    ///
    /// The schedule would change if the incoming sample sequence changes (heterogeneous
    /// channel/adc configuration).
    pub fn next(&self, _ch: &AdcChannel) -> Self {
        // Round-robin
        Self::try_from((*self as usize + 1) & 0x3).unwrap()
    }
}

#[allow(clippy::complexity)]
/// All pins for all ADCs.
/// * `spi` - Spi clk, miso, mosi (in this order).
/// * `cs` - The four chip select pins.
/// * `rdyn` - ADC rdyn input (this is the same ad spi dout).
/// * `sync` - ADC sync pin (shared for all adc phys).
pub struct AdcPins {
    pub spi: (
        gpioe::PE2<gpio::Alternate<5>>,
        gpioe::PE5<gpio::Alternate<5>>,
        gpioe::PE6<gpio::Alternate<5>>,
    ),
    pub cs: (
        gpioe::PE0<gpio::Output<gpio::PushPull>>,
        gpioe::PE1<gpio::Output<gpio::PushPull>>,
        gpioe::PE3<gpio::Output<gpio::PushPull>>,
        gpioe::PE4<gpio::Output<gpio::PushPull>>,
    ),
    pub rdyn: gpioc::PC11<gpio::Input>,
    pub sync: gpiob::PB11<gpio::Output<gpio::PushPull>>,
}

#[allow(clippy::complexity)]
pub struct Adc {
    adcs: ad7172::Ad7172<hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled>>,
    cs: (
        gpioe::PE0<gpio::Output<gpio::PushPull>>,
        gpioe::PE1<gpio::Output<gpio::PushPull>>,
        gpioe::PE3<gpio::Output<gpio::PushPull>>,
        gpioe::PE4<gpio::Output<gpio::PushPull>>,
    ),
    rdyn: gpioc::PC11<gpio::Input>,
    sync: gpiob::PB11<gpio::Output<gpio::PushPull>>,
}

impl Adc {
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
        clocks: &rcc::CoreClocks,
        spi4_rec: rcc::rec::Spi4,
        spi4: stm32::SPI4,
        pins: AdcPins,
    ) -> Self {
        let rdyn_pullup = pins.rdyn.internal_pull_up(true);
        // SPI MODE_3: idle high, capture on second transition
        let spi: spi::Spi<_, _, u8> =
            spi4.spi(pins.spi, spi::MODE_3, 12500.kHz(), spi4_rec, clocks);

        let mut adc = Adc {
            adcs: ad7172::Ad7172::new(spi),
            cs: pins.cs,
            rdyn: rdyn_pullup,
            sync: pins.sync,
        };

        adc.setup(delay);
        adc
    }

    fn setup(&mut self, delay: &mut impl DelayUs<u16>) {
        // deassert all CS first
        self.set_cs(AdcPhy::Zero, PinState::High);
        self.set_cs(AdcPhy::One, PinState::High);
        self.set_cs(AdcPhy::Two, PinState::High);
        self.set_cs(AdcPhy::Three, PinState::High);

        // set sync low first for synchronization at rising edge
        self.sync.set_low();

        for phy in AdcPhy::into_enum_iter() {
            self.selected(phy, |adc| adc.setup_adc(delay));
        }

        // set sync high after initialization of all ADCs
        self.sync.set_high();
    }

    /// Set the chip-select line of an `AdcPhy` to a `PinState`.
    fn set_cs(&mut self, phy: AdcPhy, state: PinState) {
        match phy {
            AdcPhy::Zero => self.cs.0.set_state(state),
            AdcPhy::One => self.cs.1.set_state(state),
            AdcPhy::Two => self.cs.2.set_state(state),
            AdcPhy::Three => self.cs.3.set_state(state),
        };
    }

    /// Call a closure while the given `AdcPhy` is selected (while its chip
    /// select is asserted).
    fn selected<F, R>(&mut self, phy: AdcPhy, func: F) -> R
    where
        F: FnOnce(&mut Self) -> R,
    {
        self.set_cs(phy, PinState::Low);
        let res = func(self);
        self.set_cs(phy, PinState::High);
        res
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

    pub fn read_data(&mut self) -> (AdcCode, Option<ad7172::Status>) {
        let (data, status) = self.adcs.read_data();
        (data.into(), Some(status.into()))
    }
}

statemachine! {
    transitions: {
        *Stopped + Start / start = Selected(AdcPhy),
        Selected(AdcPhy) + Read(AdcChannel) / next = Selected(AdcPhy),
        Selected(AdcPhy) + Stop / stop = Stopped,
    }
}

impl StateMachineContext for Adc {
    /// The data readout has to be initiated by selecting the first ADC.
    fn start(&mut self) -> AdcPhy {
        // set up sampling sequence by selecting the first ADC according to schedule
        self.rdyn.clear_interrupt_pending_bit();
        self.set_cs(AdcPhy::Zero, PinState::Low);
        AdcPhy::Zero
    }

    /// Uses the schedule implemented in `AdcPhy::next()` to decide which ADC will have data ready next.
    /// It clears the interupt pending flag
    /// (which does not trigger an interrupt right away since the currently selected ADC does not have new data),
    /// deselects the current ADC and selects the next in line. The next ADC will then trigger the interrupt
    /// again once it has finished
    /// sampling (or when it is selected if it is done at this point) and the routine will start again.
    fn next(&mut self, phy: &AdcPhy, ch: &AdcChannel) -> AdcPhy {
        self.set_cs(*phy, PinState::High);
        self.rdyn.clear_interrupt_pending_bit();
        let next = phy.next(ch);
        self.set_cs(next, PinState::Low);
        next
    }

    fn stop(&mut self, phy: &AdcPhy) {
        self.set_cs(*phy, PinState::High);
        self.rdyn.clear_interrupt_pending_bit();
    }
}

impl StateMachine<Adc> {
    /// Set up the RDY pin, start generating interrupts, and start the state machine.
    pub fn start(&mut self, exti: &mut device::EXTI, syscfg: &mut device::SYSCFG) {
        let adc = self.context_mut();
        adc.rdyn.make_interrupt_source(syscfg);
        adc.rdyn.trigger_on_edge(exti, gpio::Edge::Falling);
        adc.rdyn.enable_interrupt(exti);
        self.process_event(Events::Start).unwrap();
    }

    /// Handle ADC RDY interrupt.
    ///
    /// This routine is called every time the currently selected ADC on Thermostat reports that it has data ready
    /// to be read out by pulling the dout line low. It then reads out the ADC data via SPI.
    pub fn handle_interrupt(&mut self) -> (InputChannel, AdcCode) {
        if let States::Selected(phy) = *self.state() {
            let (code, status) = self.context_mut().read_data();
            let adc_ch = status.unwrap().channel();
            self.process_event(Events::Read(adc_ch)).unwrap();
            let input_ch = InputChannel::try_from((phy, adc_ch)).unwrap();
            (input_ch, code)
        } else {
            panic!("Unexpected State")
        }
    }
}
