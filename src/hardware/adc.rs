// Thermostat ADC struct.

use core::convert::Infallible;

use arbitrary_int::u2;
use embedded_hal_1::{digital::ErrorType, digital::OutputPin};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_hal_compat::{Forward, ForwardCompat};
use smlang::statemachine;
use strum::IntoEnumIterator;

use ad7172;

use super::hal::{
    self, device,
    gpio::{self, ExtiPin, gpiob, gpioc, gpioe},
    hal_02::blocking::delay::DelayUs,
    hal_02::digital::v2::PinState,
    prelude::*,
    rcc, spi, stm32,
};

use crate::convert::{AdcCode, AdcPhy};

struct DummyPin;

impl ErrorType for DummyPin {
    type Error = Infallible;
}

impl OutputPin for DummyPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
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

#[derive(Clone, Copy, Debug)]
pub struct Mux {
    pub ainpos: ad7172::Mux,
    pub ainneg: ad7172::Mux,
}

impl Mux {
    pub fn is_single_ended(&self) -> bool {
        const REF: [ad7172::Mux; 2] = [ad7172::Mux::RefN, ad7172::Mux::RefP];
        REF.contains(&self.ainpos) || REF.contains(&self.ainneg)
    }
}

pub type AdcConfig = [[Option<Mux>; 4]; 4];

/// Full Adc structure which holds all the ADC peripherals and auxillary pins on Thermostat-EEM and the configuration.
pub struct Adc {
    adcs: ad7172::Ad7172<
        ExclusiveDevice<
            Forward<hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled>>,
            DummyPin,
            NoDelay,
        >,
    >,
    cs: [gpio::ErasedPin<gpio::Output>; 4],
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
        config: &AdcConfig,
    ) -> Result<Self, Error> {
        let rdyn_pullup = pins.rdyn.internal_pull_up(true);
        // SPI MODE_3: idle high, capture on second transition
        let spi = spi4.spi(pins.spi, spi::MODE_3, 12500.kHz(), spi4_rec, clocks);

        let dev = ExclusiveDevice::new_no_delay(spi.forward(), DummyPin).unwrap();

        let mut adc = Self {
            adcs: ad7172::Ad7172::new(dev),
            cs: pins.cs,
            rdyn: rdyn_pullup,
            sync: pins.sync,
        };

        adc.setup(delay, config)?;
        Ok(adc)
    }

    /// Setup all ADCs to the specifies [AdcConfig].
    fn setup(&mut self, delay: &mut impl DelayUs<u16>, config: &AdcConfig) -> Result<(), Error> {
        // deassert all CS first
        for pin in self.cs.iter_mut() {
            pin.set_state(PinState::High);
        }

        // set sync low first for synchronization at rising edge
        self.sync.set_low();

        for phy in AdcPhy::iter() {
            log::info!("AD7172 {:?}", phy);
            self.selected(phy, |adc| adc.setup_adc(delay, &config[phy as usize]))?;
        }

        // set sync high after initialization of all ADCs
        self.sync.set_high();
        Ok(())
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
                "Temperature (K) (vs internal reference)",
                ad7172::RefSel::Internal,
                (ad7172::Mux::TempP, ad7172::Mux::TempN),
                2.5 / 477e-6,
            ),
            (
                "Avdd-Avss vs internal reference",
                ad7172::RefSel::Internal,
                (ad7172::Mux::AvddAvss5P, ad7172::Mux::AvddAvss5N),
                5.0 * 2.5,
            ),
            (
                "Avdd-Avss vs external referenve (5V)",
                ad7172::RefSel::External,
                (ad7172::Mux::AvddAvss5P, ad7172::Mux::AvddAvss5N),
                5.0 * 5.0,
            ),
            (
                "RefP-RefN vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::RefP, ad7172::Mux::RefN),
                5.0,
            ),
            (
                "RefN-Ain4 vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::RefN, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "RefP-Ain4 vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::RefP, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain0-Ain4 vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::Ain0, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain1-Ain4 vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::Ain1, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain2-Ain4 vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::Ain2, ad7172::Mux::Ain4),
                5.0,
            ),
            (
                "Ain3-Ain4 vs AvddAvss (5V)",
                ad7172::RefSel::AvddAvss,
                (ad7172::Mux::Ain3, ad7172::Mux::Ain4),
                5.0,
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
        input_config: &[Option<Mux>; 4],
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
            let ch = if let Some(mux) = cfg {
                ch.with_ainneg(mux.ainneg)
                    .with_ainpos(mux.ainpos)
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
