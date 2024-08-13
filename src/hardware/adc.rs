// Thermostat ADC struct.

use arbitrary_int::u2;
use num_traits::float::Float;
use smlang::statemachine;
use strum::IntoEnumIterator;

use super::ad7172;

use super::hal::{
    self, device,
    gpio::{self, gpiob, gpioc, gpioe, ExtiPin},
    hal::blocking::delay::DelayUs,
    hal::digital::v2::PinState,
    prelude::*,
    rcc, spi, stm32,
};

/// A type representing an ADC sample.
/// Might be extended to support different input types (other NTCs, ref resistors etc.) in the future.
#[derive(Copy, Clone, Debug)]
pub struct AdcCode(u32);
impl AdcCode {
    const GAIN: f32 = 0x555555 as _; // Default ADC gain from datasheet.
    const R_REF: f32 = 2.0 * 5000.0; // Ratiometric 5.0K high and low side or single ended 10K.
    const ZERO_C: f32 = 273.15; // 0°C in °K
    const B: f32 = 3988.0; // NTC beta value.
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

impl From<AdcCode> for u32 {
    fn from(code: AdcCode) -> u32 {
        code.0
    }
}

impl From<AdcCode> for f32 {
    /// Convert raw ADC codes to temperature value in °C using the AD7172 input voltage to code
    /// relation, the ratiometric resistor setup and the "B-parameter" equation (a simple form of the
    /// Steinhart-Hart equation). This is a tradeoff between computation and absolute temperature
    /// accuracy. The f32 output dataformat leads to an output quantization of about 31 uK.
    /// Additionally there is some error (in addition to the re-quantization) introduced during the
    /// various computation steps. If the input data has less than about 5 bit RMS noise, f32 should be
    /// avoided.
    /// Valid under the following conditions:
    /// * Unipolar ADC input
    /// * Unchanged ADC GAIN and OFFSET registers (default reset values)
    /// * Resistor setup as on Thermostat-EEM breakout board/AI-ARTIQ headboard
    ///   (either ratiometric 5.0K high and low side or single ended 10K)
    /// * Input values not close to minimum/maximum (~1000 codes difference)
    ///
    /// Maybe this will be extended in the future to support more complex temperature sensing configurations.
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
    /// Like `From<AdcCode> for f32` but for `f64` and correspondingly higher dynamic range.
    fn from(code: AdcCode) -> f64 {
        let relative_voltage = (code.0 as f32 * AdcCode::FS_PER_LSB) as f64;
        let relative_resistance =
            relative_voltage / (1.0 - relative_voltage) * AdcCode::R_REF_N as f64;
        let temperature_kelvin_inv =
            1.0 / AdcCode::T_N as f64 + 1.0 / AdcCode::B as f64 * relative_resistance.ln();
        1.0 / temperature_kelvin_inv - AdcCode::ZERO_C as f64
    }
}

#[derive(Clone, Copy, Debug, strum::EnumIter, PartialEq, Eq)]
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
    /// four individual input channels. The ADCs all share the same clock and sample rate
    /// and read out one after the other in a round-robin fashion. Therefore the sample rate
    /// of a channel depends on how many channels are enabled on an ADC.
    pub fn next(&self) -> Self {
        // Round-robin
        match self {
            Self::Zero => Self::One,
            Self::One => Self::Two,
            Self::Two => Self::Three,
            Self::Three => Self::Zero,
        }
    }
}

#[derive(Debug)]
pub enum Error {
    Ident,
}

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
    pub cs: [gpio::ErasedPin<gpio::Output>; 4],
    pub rdyn: gpioc::PC11<gpio::Input>,
    pub sync: gpiob::PB11<gpio::Output<gpio::PushPull>>,
}

/// ADC configuration structure.
/// Could be extended with further configuration options for the ADCs in the future.
#[derive(Clone, Copy, Debug)]
pub struct AdcConfig {
    /// Configuration for all ADC inputs. Four ADCs with four inputs each.
    /// `Some(([AdcInput], [AdcInput]))` positive and negative channel inputs or None to disable the channel.
    pub input_config: [[Option<(ad7172::Mux, ad7172::Mux)>; 4]; 4],
}

/// Full Adc structure which holds all the ADC peripherals and auxillary pins on Thermostat-EEM and the configuration.
pub struct Adc {
    adcs: ad7172::Ad7172<hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled>>,
    cs: [gpio::ErasedPin<gpio::Output>; 4],
    rdyn: gpioc::PC11<gpio::Input>,
    sync: gpiob::PB11<gpio::Output<gpio::PushPull>>,
    config: AdcConfig,
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
        config: AdcConfig,
    ) -> Result<Self, Error> {
        let rdyn_pullup = pins.rdyn.internal_pull_up(true);
        // SPI MODE_3: idle high, capture on second transition
        let spi: spi::Spi<_, _, u8> =
            spi4.spi(pins.spi, spi::MODE_3, 12500.kHz(), spi4_rec, clocks);

        let mut adc = Self {
            adcs: ad7172::Ad7172::new(spi),
            cs: pins.cs,
            rdyn: rdyn_pullup,
            sync: pins.sync,
            config,
        };

        adc.setup(delay, config)?;
        Ok(adc)
    }

    /// Setup all ADCs to the specifies [AdcConfig].
    fn setup(&mut self, delay: &mut impl DelayUs<u16>, config: AdcConfig) -> Result<(), Error> {
        // deassert all CS first
        for pin in self.cs.iter_mut() {
            pin.set_state(PinState::High);
        }

        // set sync low first for synchronization at rising edge
        self.sync.set_low();

        for phy in AdcPhy::iter() {
            log::info!("AD7172 {:?}", phy);
            self.selected(phy, |adc| {
                adc.setup_adc(delay, config.input_config[phy as usize])
            })?;
        }

        // set sync high after initialization of all ADCs
        self.sync.set_high();
        Ok(())
    }

    /// Returns the configuration of which ADC channels are enabled.
    pub fn channels(&self) -> [[bool; 4]; 4] {
        let mut result = [[false; 4]; 4];
        for (cfg, ch) in self
            .config
            .input_config
            .iter()
            .flatten()
            .zip(result.iter_mut().flatten())
        {
            *ch = cfg.is_some();
        }
        result
    }

    /// Call a closure while the given `AdcPhy` is selected (while its chip
    /// select is asserted).
    fn selected<F, R>(&mut self, phy: AdcPhy, func: F) -> R
    where
        F: FnOnce(&mut Self) -> R,
    {
        self.cs[phy as usize].set_state(PinState::Low);
        let res = func(self);
        self.cs[phy as usize].set_state(PinState::High);
        res
    }

    /// Debug measurements on selected ADC
    fn report(&mut self, delay: &mut impl DelayUs<u16>) {
        self.adcs.write(
            ad7172::Register::GPIOCON,
            ad7172::GpioCon::DEFAULT.with_sync_en(false).raw_value() as _,
        );

        self.adcs.write(
            ad7172::Register::FILTCON0,
            ad7172::FiltCon::DEFAULT
                .with_odr(ad7172::Odr::_20)
                .raw_value() as _,
        );

        self.adcs.write(
            ad7172::Register::IFMODE,
            ad7172::IfMode::DEFAULT.with_data_stat(true).raw_value() as _,
        );

        let adcmode = ad7172::AdcMode::DEFAULT
            .with_mode(ad7172::Mode::Single)
            .with_single_cycle(true)
            .with_ref_en(true)
            .raw_value() as _;

        for (name, refsel, ainposneg, scale) in [
            (
                "Avdd-Avss (int ref=2.5V)",
                ad7172::RefSel::Internal,
                (ad7172::Mux::AvddAvss5P, ad7172::Mux::AvddAvss5N),
                5.0 * 2.5,
            ),
            (
                "Avdd-Avss (ext ref=5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::AvddAvss5P, ad7172::Mux::AvddAvss5N),
                5.0 * 5.0,
            ),
            (
                "Ain0-Ain4 (ext ref=5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::Ain0, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain1-Ain4 (ext ref=5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::Ain1, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain2-Ain4 (ext ref=5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::Ain2, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain3-Ain4 (ext ref=5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::Ain3, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ref--Ain4 (ext ref=5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::RefN, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Temperature (K) (int ref=2.5V)",
                ad7172::RefSel::Internal,
                (ad7172::Mux::TempP, ad7172::Mux::TempN),
                2.5 / 477e-6,
            ),
        ] {
            self.adcs.write(
                ad7172::Register::SETUPCON0,
                ad7172::SetupCon::builder()
                    .with_ref_sel(refsel)
                    .with_burnout_en(false)
                    .with_ainbufn(true)
                    .with_ainbufp(true)
                    .with_refbufn(true)
                    .with_refbufp(true)
                    .with_bipolar(true)
                    .build()
                    .raw_value() as _,
            );

            self.adcs.write(
                ad7172::Register::CH0,
                ad7172::Channel::builder()
                    .with_ainneg(ainposneg.1)
                    .with_ainpos(ainposneg.0)
                    .with_setup_sel(u2::new(0))
                    .with_en(true)
                    .build()
                    .raw_value() as _,
            );

            self.adcs.write(ad7172::Register::ADCMODE, adcmode);

            while self.rdyn.is_high() {}
            let (data, status) = self.adcs.read_data();
            assert!(!status.busy());
            assert!(!status.reg_error());
            assert!(!status.crc_error());
            assert_eq!(status.channel(), u2::new(0));
            log::info!(
                "{name}: {}{}",
                (data as i32 - 0x800000) as f32 * scale / (1 << 23) as f32,
                if status.adc_error() {
                    " (ADC Error)"
                } else {
                    ""
                },
            );
        }

        self.adcs.reset();
        delay.delay_us(500);
    }

    /// Setup an ADC on Thermostat-EEM.
    fn setup_adc(
        &mut self,
        delay: &mut impl DelayUs<u16>,
        input_config: [Option<(ad7172::Mux, ad7172::Mux)>; 4],
    ) -> Result<(), Error> {
        self.adcs.reset();
        delay.delay_us(500);

        let id = self.adcs.read(ad7172::Register::ID);
        // check that ID is 0x00DX, as per datasheet
        if id & 0xfff0 != 0x00d0 {
            log::error!("invalid ID: {:#x}", id);
            return Err(Error::Ident);
        }

        self.report(delay);

        self.adcs.write(
            ad7172::Register::ADCMODE,
            ad7172::AdcMode::DEFAULT
                .with_clocksel(ad7172::ClockSel::ExternalClock)
                .raw_value() as _,
        );

        self.adcs.write(
            ad7172::Register::IFMODE,
            ad7172::IfMode::DEFAULT.with_data_stat(true).raw_value() as _,
        );

        self.adcs.write(
            ad7172::Register::GPIOCON,
            ad7172::GpioCon::DEFAULT.with_sync_en(true).raw_value() as _,
        );

        log::info!("Input configuration: {:?}", input_config);

        for (cfg, channel) in input_config.iter().zip([
            ad7172::Register::CH0,
            ad7172::Register::CH1,
            ad7172::Register::CH2,
            ad7172::Register::CH3,
        ]) {
            let ch = ad7172::Channel::DEFAULT;
            let ch = if let Some(cfg) = cfg {
                ch.with_ainneg(cfg.1)
                    .with_ainpos(cfg.0)
                    .with_setup_sel(u2::new(0)) // only Setup 0 for now
                    .with_en(true)
            } else {
                ch.with_ainneg(ad7172::Mux::Ain4)
                    .with_ainpos(ad7172::Mux::Ain4)
                    .with_setup_sel(u2::new(0))
                    .with_en(false)
            };
            self.adcs.write(channel, ch.raw_value() as _);
        }

        self.adcs.write(
            ad7172::Register::SETUPCON0,
            ad7172::SetupCon::DEFAULT
                .with_ainbufn(true)
                .with_ainbufp(true)
                .with_refbufn(true)
                .with_refbufp(true)
                .with_bipolar(false)
                .raw_value() as _,
        );

        self.adcs.write(
            ad7172::Register::FILTCON0,
            ad7172::FiltCon::DEFAULT
                .with_odr(ad7172::Odr::_1007)
                .raw_value() as _,
        );

        Ok(())
    }

    /// Read the data from the ADC and return the raw data and the status information.
    pub fn read_data(&mut self) -> (AdcCode, ad7172::Status) {
        let (data, status) = self.adcs.read_data();
        (data.into(), status)
    }
}

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            *Stopped + Start / start = Selected(AdcPhy),
            Selected(AdcPhy) + Read / next = Selected(AdcPhy),
            Selected(AdcPhy) + Stop / stop = Stopped,
        }
    }
}

impl sm::StateMachineContext for Adc {
    /// The data readout has to be initiated by selecting the first ADC.
    fn start(&mut self) -> Result<AdcPhy, ()> {
        // set up sampling sequence by selecting the first ADC according to schedule
        self.rdyn.clear_interrupt_pending_bit();
        self.cs[AdcPhy::Zero as usize].set_state(PinState::Low);
        Ok(AdcPhy::Zero)
    }

    /// Clears the interrupt pending flag (which does not trigger an interrupt right away since the currently
    /// selected ADC does not have new data), deselects the current ADC and selects the next in line.
    /// The next ADC will then trigger the interrupt again once it has finished sampling (or when it is
    /// selected if it is done at this point) and the routine will start again.
    fn next(&mut self, phy: &AdcPhy) -> Result<AdcPhy, ()> {
        self.cs[*phy as usize].set_state(PinState::High);
        self.rdyn.clear_interrupt_pending_bit();
        let next = phy.next();
        self.cs[next as usize].set_state(PinState::Low);
        Ok(next)
    }

    fn stop(&mut self, phy: &AdcPhy) -> Result<(), ()> {
        self.cs[*phy as usize].set_state(PinState::High);
        self.rdyn.clear_interrupt_pending_bit();
        Ok(())
    }
}

impl sm::StateMachine<Adc> {
    /// Set up the RDY pin, start generating interrupts, and start the state machine.
    pub fn start(&mut self, exti: &mut device::EXTI, syscfg: &mut device::SYSCFG) {
        let adc = self.context_mut();
        adc.rdyn.make_interrupt_source(syscfg);
        adc.rdyn.trigger_on_edge(exti, gpio::Edge::Falling);
        adc.rdyn.enable_interrupt(exti);
        self.process_event(sm::Events::Start).unwrap();
    }

    /// Handle ADC RDY interrupt.
    ///
    /// This routine is called every time the currently selected ADC on Thermostat reports that it has data ready
    /// to be read out by pulling the dout line low. It then reads out the ADC data via SPI.
    pub fn handle_interrupt(&mut self) -> (AdcPhy, usize, AdcCode) {
        if let sm::States::Selected(phy) = *self.state() {
            let (code, status) = self.context_mut().read_data();
            let adc_ch = status.channel().value() as _;
            self.process_event(sm::Events::Read).unwrap();
            (phy, adc_ch, code)
        } else {
            panic!("Unexpected State")
        }
    }
}
