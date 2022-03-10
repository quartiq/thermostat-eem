// Thermostat DAC/TEC driver
//
// This file contains all of the drivers to convert an 18 bit word to an analog current.
// On Thermostat this used the ad5680 DAC and the MAX1968 PWM TEC driver. The (analog voltage)
// max output voltages/current settings are driven by PWMs of the STM32.

use num_enum::TryFromPrimitive;

use super::hal::{
    gpio::{gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiog::*, Alternate, Output, PushPull, AF6},
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

macro_rules! setup_tim {
    ($tim:ident, $clocks:ident, $timp:ident, $af:ident, $pin0:ident, $pin1:ident, $pin2:ident, $pin3:ident) => {{
        let channels = ($pin0.$af(), $pin1.$af(), $pin2.$af(), $pin3.$af());
        let (mut pwm_0, mut pwm_1, mut pwm_2, mut pwm_3) =
            $tim.pwm(channels, F_PWM.khz(), $timp, $clocks);
        init_pwm_pin(&mut pwm_0);
        init_pwm_pin(&mut pwm_1);
        init_pwm_pin(&mut pwm_2);
        init_pwm_pin(&mut pwm_3);
        (pwm_0, pwm_1, pwm_2, pwm_3)
    }};
}

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

type Pt0<T, S> = pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

pub struct Pwm {
    max_v0: Pt0<TIM1, C1>,
    max_v1: Pt0<TIM1, C2>,
    max_v2: Pt0<TIM1, C3>,
    max_v3: Pt1<TIM1, C4>,
    max_i0: Pt1<TIM4, C1>,
    max_i1: Pt1<TIM4, C2>,
    max_i2: Pt1<TIM4, C3>,
    max_i3: Pt1<TIM4, C4>,
    min_i0: Pt1<TIM3, C1>,
    min_i1: Pt1<TIM3, C2>,
    min_i2: Pt1<TIM3, C3>,
    min_i3: Pt1<TIM3, C4>,
}

impl Pwm {
    pub fn new<M1, M2, M3, M4, M5, M6, M7, M8, M9, M10, M11, M12>(
        clocks: &CoreClocks,
        tim1_rcc: rec::Tim1,
        tim3_rcc: rec::Tim3,
        tim4_rcc: rec::Tim4,
        tim1: TIM1,
        tim3: TIM3,
        tim4: TIM4,
        max_v0_pin: PE9<M1>,
        max_v1_pin: PE11<M2>,
        max_v2_pin: PE13<M3>,
        max_v3_pin: PE14<M4>,
        max_i0_pin: PD12<M5>,
        max_i1_pin: PD13<M6>,
        max_i2_pin: PD14<M7>,
        max_i3_pin: PD15<M8>,
        min_i0_pin: PC6<M9>,
        min_i1_pin: PB5<M10>,
        min_i2_pin: PC8<M11>,
        min_i3_pin: PC9<M12>,
    ) -> Pwm {
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }

        let (max_v0, max_v1, max_v2, max_v3) = setup_tim!(
            tim1,
            clocks,
            tim1_rcc,
            into_alternate_af1,
            max_v0_pin,
            max_v1_pin,
            max_v2_pin,
            max_v3_pin
        );

        let (max_i0, max_i1, max_i2, max_i3) = setup_tim!(
            tim4,
            clocks,
            tim4_rcc,
            into_alternate_af2,
            max_i0_pin,
            max_i1_pin,
            max_i2_pin,
            max_i3_pin
        );

        let (min_i0, min_i1, min_i2, min_i3) = setup_tim!(
            tim3,
            clocks,
            tim3_rcc,
            into_alternate_af2,
            min_i0_pin,
            min_i1_pin,
            min_i2_pin,
            min_i3_pin
        );

        Pwm {
            max_v0,
            max_v1,
            max_v2,
            max_v3,
            max_i0,
            max_i1,
            max_i2,
            max_i3,
            min_i0,
            min_i1,
            min_i2,
            min_i3,
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
            (Channel::Ch0, Limit::MaxIPos) => set_pwm(&mut self.max_i0, i_to_pwm(val)),
            (Channel::Ch1, Limit::MaxIPos) => set_pwm(&mut self.max_i1, i_to_pwm(val)),
            (Channel::Ch2, Limit::MaxIPos) => set_pwm(&mut self.max_i2, i_to_pwm(val)),
            (Channel::Ch3, Limit::MaxIPos) => set_pwm(&mut self.max_i3, i_to_pwm(val)),
            (Channel::Ch0, Limit::MaxINeg) => set_pwm(&mut self.min_i0, i_to_pwm(val)),
            (Channel::Ch1, Limit::MaxINeg) => set_pwm(&mut self.min_i1, i_to_pwm(val)),
            (Channel::Ch2, Limit::MaxINeg) => set_pwm(&mut self.min_i2, i_to_pwm(val)),
            (Channel::Ch3, Limit::MaxINeg) => set_pwm(&mut self.min_i3, i_to_pwm(val)),
        }
    }
}

pub struct DacPins {
    pub sck: PC10<Alternate<AF6>>,
    pub mosi: PC12<Alternate<AF6>>,
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
    pub fn new(clocks: &CoreClocks, prec: rec::Spi3, spi3: SPI3, pins: DacPins) -> Self {
        let spi = spi3.spi(
            (pins.sck, NoMiso, pins.mosi),
            spi::MODE_1,
            SPI_CLOCK,
            prec,
            clocks, // default pll1_q clock source
        );

        let mut dac = Dac {
            spi,
            val: [0, 0, 0, 0],
            sync0: pins.sync0,
            sync1: pins.sync1,
            sync2: pins.sync2,
            sync3: pins.sync3,
            shdn0: pins.shdn0,
            shdn1: pins.shdn1,
            shdn2: pins.shdn2,
            shdn3: pins.shdn3,
        };

        dac.sync0.set_high().unwrap();
        dac.sync1.set_high().unwrap();
        dac.sync2.set_high().unwrap();
        dac.sync3.set_high().unwrap();

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
                self.sync0.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[0] = value;
                self.sync0.set_high().unwrap();
            }
            Channel::Ch1 => {
                self.sync1.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[1] = value;
                self.sync1.set_high().unwrap();
            }
            Channel::Ch2 => {
                self.sync2.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[2] = value;
                self.sync2.set_high().unwrap();
            }
            Channel::Ch3 => {
                self.sync3.set_low().unwrap();
                self.spi.transfer(&mut buf).unwrap();
                self.val[3] = value;
                self.sync3.set_high().unwrap();
            }
        }
    }

    /// enable or disable a TEC channel via shutdown pin.
    pub fn en_dis_ch(&mut self, ch: Channel, en: bool) {
        match (ch, en) {
            (Channel::Ch0, true) => self.shdn0.set_high().unwrap(),
            (Channel::Ch1, true) => self.shdn1.set_high().unwrap(),
            (Channel::Ch2, true) => self.shdn2.set_high().unwrap(),
            (Channel::Ch3, true) => self.shdn3.set_high().unwrap(),
            (Channel::Ch0, false) => self.shdn0.set_low().unwrap(),
            (Channel::Ch1, false) => self.shdn1.set_low().unwrap(),
            (Channel::Ch2, false) => self.shdn2.set_low().unwrap(),
            (Channel::Ch3, false) => self.shdn3.set_low().unwrap(),
        }
    }
}
