// Thermostat DAC/TEC driver
//
// This file contains all of the drivers to convert an 18 bit word to an analog current.
// On Thermostat this used the ad5680 DAC and the MAX1968 PWM TEC driver. The (analog voltage)
// max output voltages/current settings are driven by PWMs of the STM32.
// SingularitySurfer 2022

use super::hal::{
    gpio::{gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiog::*, Alternate, Output, PushPull, AF5},
    hal::{blocking::spi::Transfer, digital::v2::OutputPin, PwmPin, Pwm},
    pwm::*,
    prelude::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::{NoMiso, Spi},
    stm32::{SPI3, TIM1, TIM3},
    time::{MegaHertz, U32Ext},
};

use crate::unit_conversion::{i_to_dac, i_to_pwm, v_to_pwm};

/// SPI Mode 1
pub const SPI_MODE: spi::Mode = spi::Mode {
    polarity: spi::Polarity::IdleLow,
    phase: spi::Phase::CaptureOnSecondTransition,
};

pub const SPI_CLOCK: MegaHertz = MegaHertz(30); // DAC SPI clock speed
pub const MAX_VALUE: u32 = 0x3FFFF; // Maximum DAC output value
pub const F_PWM: u32 = 20; // PWM freq in kHz

pub type DacSpi = Spi<SPI3, (PC10<Alternate<AF5>>, NoMiso, PC12<Alternate<AF5>>)>;

pub struct DacPins {
    pub sck: PC10<Alternate<AF5>>,
    pub mosi: PC12<Alternate<AF5>>,
    pub sync0: PG3<Output<PushPull>>,
    pub sync1: PG2<Output<PushPull>>,
    pub sync2: PG1<Output<PushPull>>,
    pub sync3: PG0<Output<PushPull>>,
    pub shdn0: PG4<Output<PushPull>>,
    pub shdn1: PG5<Output<PushPull>>,
    pub shdn2: PG6<Output<PushPull>>,
    pub shdn3: PG7<Output<PushPull>>,
}

pub struct Pwms {
    pub max_v0: Pwm<TIM1, C1, ComplementaryDisabled, ActiveHigh, ActiveHigh>,
    pub max_v1: PwmChannels<TIM1, pwm::C2>,
    pub max_v2: PwmChannels<TIM1, pwm::C3>,
    pub max_v3: PwmChannels<TIM1, pwm::C4>,
    pub max_i_pos0: PwmChannels<TIM4, pwm::C1>,
    pub max_i_pos1: PwmChannels<TIM4, pwm::C2>,
    pub max_i_pos2: PwmChannels<TIM4, pwm::C3>,
    pub max_i_pos3: PwmChannels<TIM4, pwm::C4>,
    pub max_i_neg0: PwmChannels<TIM3, pwm::C1>,
    pub max_i_neg1: PwmChannels<TIM3, pwm::C2>,
    pub max_i_neg2: PwmChannels<TIM3, pwm::C3>,
    pub max_i_neg3: PwmChannels<TIM3, pwm::C4>,
}

impl Pwms {
    pub fn new<M1, M2, M3, M4, M5, M6, M7, M8, M9, M10, M11, M12>(
        ccdr: Ccdr,
        tim1: TIM1,
        tim3: TIM3,
        tim4: TIM4,
        max_v0: PE9<M1>,
        max_v1: PE11<M2>,
        max_v2: PE13<M3>,
        max_v3: PE14<M4>,
        max_i_pos0: PD12<M5>,
        max_i_pos1: PD13<M6>,
        max_i_pos2: PD14<M7>,
        max_i_pos3: PD15<M8>,
        max_i_neg0: PC6<M9>,
        max_i_neg1: PB5<M10>,
        max_i_neg2: PC8<M11>,
        max_i_neg3: PC9<M12>,
    ) -> Pwms {
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }

        // setup max_v channels on timer 1
        let channels = (
            max_v0.into_alternate_af1(),
            max_v1.into_alternate_af1(),
            max_v2.into_alternate_af1(),
            max_v3.into_alternate_af1(),
        );
        let (mut max_v0, mut max_v1, mut max_v2, mut max_v3) =
            tim1.pwm(channels, F_PWM.khz(), ccdr.peripheral.TIM1, &ccdr.clocks);
        init_pwm_pin(&mut max_v0);
        init_pwm_pin(&mut max_v1);
        init_pwm_pin(&mut max_v2);
        init_pwm_pin(&mut max_v3);

        // setup max_i_pos channels on timer 4
        let channels = (
            max_i_pos0.into_alternate_af1(),
            max_i_pos1.into_alternate_af1(),
            max_i_pos2.into_alternate_af1(),
            max_i_pos3.into_alternate_af1(),
        );
        let (mut max_i_pos0, mut max_i_pos1, mut max_i_pos2, mut max_i_pos3) =
            tim4.pwm(channels, F_PWM.khz(), ccdr.peripheral.TIM4, &ccdr.clocks);
        init_pwm_pin(&mut max_i_pos0);
        init_pwm_pin(&mut max_i_pos1);
        init_pwm_pin(&mut max_i_pos2);
        init_pwm_pin(&mut max_i_pos3);

        // setup max_i_neg channels on timer 3
        let channels = (
            max_i_neg0.into_alternate_af1(),
            max_i_neg1.into_alternate_af1(),
            max_i_neg2.into_alternate_af1(),
            max_i_neg3.into_alternate_af1(),
        );
        let (mut max_i_neg0, mut max_i_neg1, mut max_i_neg2, mut max_i_neg3) =
            tim4.pwm(channels, F_PWM.khz(), ccdr.peripheral.TIM4, &ccdr.clocks);
        init_pwm_pin(&mut max_i_neg0);
        init_pwm_pin(&mut max_i_neg1);
        init_pwm_pin(&mut max_i_neg2);
        init_pwm_pin(&mut max_i_neg3);

        Pwms {
            max_v0,
            max_v1,
            max_v2,
            max_v3,
            max_i_pos0,
            max_i_pos1,
            max_i_pos2,
            max_i_pos3,
            max_i_neg0,
            max_i_neg1,
            max_i_neg2,
            max_i_neg3,
        }
    }

    /// Set PWM channel to relative dutycycle.
    pub fn set(&mut self, duty: f32, ch: u8) {
        fn set<P: PwmPin<Duty = u16>>(pin: &mut P, duty: f32) {
            let duty = i_to_pwm(duty);
            let max = pin.get_max_duty();
            let value = ((duty * (max as f32)) as u16).min(max);
            pin.set_duty(value);
        }
        match ch {
            0 => set(&mut self.max_v0, duty),
            1 => set(&mut self.max_v1, duty),
            2 => set(&mut self.max_i_pos0, duty),
            3 => set(&mut self.max_i_neg0, duty),
            4 => set(&mut self.max_i_pos1, duty),
            5 => set(&mut self.max_i_neg1, duty),
            _ => unreachable!(),
        }
    }

    /// set all PWM oututs to specified min/max currents
    pub fn set_all(
        &mut self,
        min_i0: f32,
        max_i0: f32,
        max_v0: f32,
        min_i1: f32,
        max_i1: f32,
        max_v1: f32,
    ) {
        self.set(v_to_pwm(max_v0), 0);
        self.set(v_to_pwm(max_v1), 1);
        self.set(i_to_pwm(max_i0), 2);
        self.set(i_to_pwm(min_i0), 3);
        self.set(i_to_pwm(max_i1), 4);
        self.set(i_to_pwm(min_i1), 5);
    }
}

/// DAC: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5680.pdf
/// Peltier Driver: https://datasheets.maximintegrated.com/en/ds/MAX1968-MAX1969.pdf
pub struct Dac {
    spi: DacSpi,
    pub val: [u32; 2],
    sync0: PG3<Output<PushPull>>,
    sync1: PG2<Output<PushPull>>,
    sync2: PG1<Output<PushPull>>,
    sync3: PG0<Output<PushPull>>,
    shdn0: PG4<Output<PushPull>>,
    shdn1: PG5<Output<PushPull>>,
    shdn2: PG6<Output<PushPull>>,
    shdn3: PG7<Output<PushPull>>,
}

impl Dac {
    pub fn new(clocks: Clocks, spi3: SPI3, pins: DacPins) -> Self {
        let spi = Spi::spi3(
            spi3,
            (pins.sck, NoMiso, pins.mosi),
            SPI_MODE,
            SPI_CLOCK.into(),
            clocks,
        );

        let mut dac = Dac {
            spi,
            val: [0, 0],
            sync0: pins.sync0,
            sync1: pins.sync1,
            sync2: pins.sync2,
            sync3: pins.sync3,
            shdn0: pins.shdn0,
            shdn1: pins.shdn1,
            shdn2: pins.shdn2,
            shdn3: pins.shdn3,
        };
        dac.dis_ch(0);

        dac.sync0.set_high().unwrap();

        // default to zero amps
        dac.set(i_to_dac(0.0), 0);
        dac
    }

    /// Set the DAC output to value on a channel.
    pub fn set(&mut self, value: u32, ch: u8) {
        let value = value.min(MAX_VALUE);
        // 24 bit transfer. First 6 bit and last 2 bit are low.
        let mut buf = [(value >> 14) as u8, (value >> 6) as u8, (value << 2) as u8];
        if ch == 0 {
            self.sync0.set_low().unwrap();
            self.spi0.transfer(&mut buf).unwrap();
            self.val[0] = value;
            self.sync0.set_high().unwrap();
        } else {
            // self.sync1.set_high().unwrap();
            // // must be high for >= 33 ns
            // delay(100); // 100 * 5.95ns
            // self.sync1.set_low().unwrap();
            // self.spi1.transfer(&mut buf).unwrap();
            // self.val[1] = value;
        }
    }

    /// enable a TEC channel via shutdown pin.
    pub fn en_ch(&mut self, ch: u8) {
        if ch == 0 {
            self.shdn0.set_high().unwrap();
        } else {
            // self.shdn1.set_high().unwrap();
        }
    }

    /// disable a TEC channel via shutdown pin.
    pub fn dis_ch(&mut self, ch: u8) {
        if ch == 0 {
            self.shdn0.set_low().unwrap();
        } else {
            // self.shdn1.set_low().unwrap();
        }
    }
}

macro_rules! setup_tim {
    ($) => {};
}
