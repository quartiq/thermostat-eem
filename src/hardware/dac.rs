// Thermostat DAC/TEC driver
//
// This file contains all of the drivers to convert an 18 bit word to an analog current.
// On Thermostat this used the ad5680 DAC and the MAX1968 PWM TEC driver. The (analog voltage)
// max output voltages/current settings are driven by PWMs of the STM32.

use num_enum::TryFromPrimitive;

use super::hal::{
    gpio::{
        gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiog::*, Alternate, Output, PushPull, AF1, AF2,
        AF6,
    },
    hal::{blocking::spi::Transfer, digital::v2::OutputPin, PwmPin},
    prelude::*,
    pwm,
    pwm::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::{Enabled, NoMiso, Spi},
    stm32::{SPI3, TIM1, TIM3, TIM4},
    time::{MegaHertz, U32Ext},
};

use super::unit_conversion::{i_to_dac, i_to_pwm, v_to_pwm};

pub const SPI_CLOCK: MegaHertz = MegaHertz(30); // DAC SPI clock speed
pub const MAX_VALUE: u32 = 0x3FFFF; // Maximum DAC output value
pub const F_PWM: u32 = 20; // PWM freq in kHz

#[derive(Clone, Copy, TryFromPrimitive)]
#[repr(usize)]
pub enum Channel {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
}

pub enum Limit {
    MaxV,
    MaxIPos,
    MaxINeg,
}

pub struct PwmPins {
    pub max_v0_pin: PE9<Alternate<AF1>>,
    pub max_v1_pin: PE11<Alternate<AF1>>,
    pub max_v2_pin: PE13<Alternate<AF1>>,
    pub max_v3_pin: PE14<Alternate<AF1>>,
    pub max_i_pos0_pin: PD12<Alternate<AF2>>,
    pub max_i_pos1_pin: PD13<Alternate<AF2>>,
    pub max_i_pos2_pin: PD14<Alternate<AF2>>,
    pub max_i_pos3_pin: PD15<Alternate<AF2>>,
    pub max_i_neg0_pin: PC6<Alternate<AF2>>,
    pub max_i_neg1_pin: PB5<Alternate<AF2>>,
    pub max_i_neg2_pin: PC8<Alternate<AF2>>,
    pub max_i_neg3_pin: PC9<Alternate<AF2>>,
}

type Pt0<T, S> = pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

pub struct Pwm {
    max_v0: Pt0<TIM1, C1>,
    max_v1: Pt0<TIM1, C2>,
    max_v2: Pt0<TIM1, C3>,
    max_v3: Pt1<TIM1, C4>,
    max_i_pos0: Pt1<TIM4, C1>,
    max_i_pos1: Pt1<TIM4, C2>,
    max_i_pos2: Pt1<TIM4, C3>,
    max_i_pos3: Pt1<TIM4, C4>,
    max_i_neg0: Pt1<TIM3, C1>,
    max_i_neg1: Pt1<TIM3, C2>,
    max_i_neg2: Pt1<TIM3, C3>,
    max_i_neg3: Pt1<TIM3, C4>,
}

impl Pwm {
    pub fn new(
        clocks: &CoreClocks,
        tim1_rcc: rec::Tim1,
        tim3_rcc: rec::Tim3,
        tim4_rcc: rec::Tim4,
        tim1: TIM1,
        tim3: TIM3,
        tim4: TIM4,
        pins: PwmPins,
    ) -> Pwm {
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }

        let channels = (
            pins.max_v0_pin,
            pins.max_v1_pin,
            pins.max_v2_pin,
            pins.max_v3_pin,
        );
        let (mut max_v0, mut max_v1, mut max_v2, mut max_v3) =
            tim1.pwm(channels, F_PWM.khz(), tim1_rcc, clocks);
        init_pwm_pin(&mut max_v0);
        init_pwm_pin(&mut max_v1);
        init_pwm_pin(&mut max_v2);
        init_pwm_pin(&mut max_v3);

        let channels = (
            pins.max_i_pos0_pin,
            pins.max_i_pos1_pin,
            pins.max_i_pos2_pin,
            pins.max_i_pos3_pin,
        );
        let (mut max_i_pos0, mut max_i_pos1, mut max_i_pos2, mut max_i_pos3) =
            tim4.pwm(channels, F_PWM.khz(), tim4_rcc, clocks);
        init_pwm_pin(&mut max_i_pos0);
        init_pwm_pin(&mut max_i_pos1);
        init_pwm_pin(&mut max_i_pos2);
        init_pwm_pin(&mut max_i_pos3);

        let channels = (
            pins.max_i_neg0_pin,
            pins.max_i_neg1_pin,
            pins.max_i_neg2_pin,
            pins.max_i_neg3_pin,
        );
        let (mut max_i_neg0, mut max_i_neg1, mut max_i_neg2, mut max_i_neg3) =
            tim3.pwm(channels, F_PWM.khz(), tim3_rcc, clocks);
        init_pwm_pin(&mut max_i_neg0);
        init_pwm_pin(&mut max_i_neg1);
        init_pwm_pin(&mut max_i_neg2);
        init_pwm_pin(&mut max_i_neg3);

        Pwm {
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

    /// Set PWM to limit current.
    pub fn set(&mut self, ch: Channel, lim: Limit, val: f32) {
        fn set_pwm<P: PwmPin<Duty = u16>>(pin: &mut P, duty: f32) {
            let max = pin.get_max_duty();
            let value = ((duty * (max as f32)) as u16).min(max);
            pin.set_duty(value);
        }
        match (ch, lim) {
            (Channel::Ch0, Limit::MaxV) => set_pwm(&mut self.max_v0, v_to_pwm(val)),
            (Channel::Ch1, Limit::MaxV) => set_pwm(&mut self.max_v1, v_to_pwm(val)),
            (Channel::Ch2, Limit::MaxV) => set_pwm(&mut self.max_v2, v_to_pwm(val)),
            (Channel::Ch3, Limit::MaxV) => set_pwm(&mut self.max_v3, v_to_pwm(val)),
            (Channel::Ch0, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos0, i_to_pwm(val)),
            (Channel::Ch1, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos1, i_to_pwm(val)),
            (Channel::Ch2, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos2, i_to_pwm(val)),
            (Channel::Ch3, Limit::MaxIPos) => set_pwm(&mut self.max_i_pos3, i_to_pwm(val)),
            (Channel::Ch0, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg0, i_to_pwm(val)),
            (Channel::Ch1, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg1, i_to_pwm(val)),
            (Channel::Ch2, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg2, i_to_pwm(val)),
            (Channel::Ch3, Limit::MaxINeg) => set_pwm(&mut self.max_i_neg3, i_to_pwm(val)),
        }
    }
}

pub struct DacGpio {
    pub sync0: PG3<Output<PushPull>>,
    pub sync1: PG2<Output<PushPull>>,
    pub sync2: PG1<Output<PushPull>>,
    pub sync3: PG0<Output<PushPull>>,
    pub shdn0: PG4<Output<PushPull>>,
    pub shdn1: PG5<Output<PushPull>>,
    pub shdn2: PG6<Output<PushPull>>,
    pub shdn3: PG7<Output<PushPull>>,
}

/// DAC: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5680.pdf
/// Peltier Driver: https://datasheets.maximintegrated.com/en/ds/MAX1968-MAX1969.pdf
pub struct Dac {
    spi: Spi<SPI3, Enabled, u8>,
    pub val: [u32; 4],
    gpio: DacGpio,
}

impl Dac {
    pub fn new(
        clocks: &CoreClocks,
        prec: rec::Spi3,
        spi3: SPI3,
        sck: PC10<Alternate<AF6>>,
        mosi: PC12<Alternate<AF6>>,
        gpio: DacGpio,
    ) -> Self {
        let spi = spi3.spi(
            (sck, NoMiso, mosi),
            spi::MODE_1,
            SPI_CLOCK,
            prec,
            clocks, // default pll1_q clock source
        );

        let mut dac = Dac {
            spi,
            val: [0, 0, 0, 0],
            gpio,
        };

        dac.gpio.sync0.set_high().unwrap();
        dac.gpio.sync1.set_high().unwrap();
        dac.gpio.sync2.set_high().unwrap();
        dac.gpio.sync3.set_high().unwrap();

        // default to zero amps
        dac.set(0.0, Channel::Ch0);
        dac.set(0.0, Channel::Ch1);
        dac.set(0.0, Channel::Ch2);
        dac.set(0.0, Channel::Ch3);
        dac
    }

    /// Set the DAC output to current on a channel.
    pub fn set(&mut self, curr: f32, ch: Channel) {
        let value = i_to_dac(curr);
        let value = value.min(MAX_VALUE);
        // 24 bit transfer. First 4 bit and last 2 bit are low.
        let mut buf = [(value >> 14) as u8, (value >> 6) as u8, (value << 2) as u8];

        match ch {
            Channel::Ch0 => {
                self.gpio.sync0.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[0] = value;
                self.gpio.sync0.set_high().unwrap();
            }
            Channel::Ch1 => {
                self.gpio.sync1.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[1] = value;
                self.gpio.sync1.set_high().unwrap();
            }
            Channel::Ch2 => {
                self.gpio.sync2.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[2] = value;
                self.gpio.sync2.set_high().unwrap();
            }
            Channel::Ch3 => {
                self.gpio.sync3.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[3] = value;
                self.gpio.sync3.set_high().unwrap();
            }
        }
    }

    /// enable or disable a TEC channel via shutdown pin.
    pub fn en_dis_ch(&mut self, ch: Channel, en: bool) {
        match (ch, en) {
            (Channel::Ch0, true) => self.gpio.shdn0.set_high().unwrap(),
            (Channel::Ch1, true) => self.gpio.shdn1.set_high().unwrap(),
            (Channel::Ch2, true) => self.gpio.shdn2.set_high().unwrap(),
            (Channel::Ch3, true) => self.gpio.shdn3.set_high().unwrap(),
            (Channel::Ch0, false) => self.gpio.shdn0.set_low().unwrap(),
            (Channel::Ch1, false) => self.gpio.shdn1.set_low().unwrap(),
            (Channel::Ch2, false) => self.gpio.shdn2.set_low().unwrap(),
            (Channel::Ch3, false) => self.gpio.shdn3.set_low().unwrap(),
        }
    }
}
