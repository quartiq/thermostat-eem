// Thermostat DAC/TEC driver
//
// This file contains all of the drivers to convert an 18 bit word to an analog current.
// On Thermostat this used the ad5680 DAC and the MAX1968 PWM TEC driver. The (analog voltage)
// max output voltages/current settings are driven by PWMs of the STM32.
// SingularitySurfer 2022

use super::hal::{
    gpio::{gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiog::*, Alternate, Output, PushPull, AF5},
    hal::{blocking::spi::Transfer, digital::v2::OutputPin, PwmPin},
    prelude::*,
    pwm,
    pwm::*,
    rcc::{rec, CoreClocks},
    spi,
    spi::{NoMiso, Spi},
    stm32::{SPI3, TIM1, TIM3, TIM4},
    time::{MegaHertz, U32Ext},
};

use log::info;

use super::unit_conversion::{i_to_dac, i_to_pwm, v_to_pwm};

/// SPI Mode 1
pub const SPI_MODE: spi::Mode = spi::Mode {
    polarity: spi::Polarity::IdleLow,
    phase: spi::Phase::CaptureOnSecondTransition,
};

pub const SPI_CLOCK: MegaHertz = MegaHertz(30); // DAC SPI clock speed
pub const MAX_VALUE: u32 = 0x3FFFF; // Maximum DAC output value
pub const F_PWM: u32 = 20; // PWM freq in kHz

macro_rules! setup_tim {
    ($tim:ident, $ccdr:ident, $timp:ident, $pin0:ident, $pin1:ident, $pin2:ident, $pin3:ident) => {{
        let channels = (
            $pin0.into_alternate_af1(),
            $pin1.into_alternate_af1(),
            $pin2.into_alternate_af1(),
            $pin3.into_alternate_af1(),
        );
        let (mut pwm_0, mut pwm_1, mut pwm_2, mut pwm_3) =
            $tim.pwm(channels, F_PWM.khz(), $peripheral.$timp, &$clocks);
        init_pwm_pin(&mut pwm_0);
        init_pwm_pin(&mut pwm_1);
        init_pwm_pin(&mut pwm_2);
        init_pwm_pin(&mut pwm_3);
        (pwm_0, pwm_1, pwm_2, pwm_3)
    }};
}

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

type Pt0<T, S> = pwm::Pwm<T, S, ComplementaryDisabled, ActiveHigh, ActiveHigh>;
type Pt1<T, S> = pwm::Pwm<T, S, ComplementaryImpossible, ActiveHigh, ActiveHigh>;

pub struct Pwms {
    pub max_v0: Pt0<TIM1, C1>,
    pub max_v1: Pt0<TIM1, C2>,
    pub max_v2: Pt0<TIM1, C3>,
    pub max_v3: Pt1<TIM1, C4>,
    pub max_i_pos0: Pt1<TIM4, C1>,
    pub max_i_pos1: Pt1<TIM4, C2>,
    pub max_i_pos2: Pt1<TIM4, C3>,
    pub max_i_pos3: Pt1<TIM4, C4>,
    pub max_i_neg0: Pt1<TIM3, C1>,
    pub max_i_neg1: Pt1<TIM3, C2>,
    pub max_i_neg2: Pt1<TIM3, C3>,
    pub max_i_neg3: Pt1<TIM3, C4>,
}

impl Pwms {
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
        max_i_pos0_pin: PD12<M5>,
        max_i_pos1_pin: PD13<M6>,
        max_i_pos2_pin: PD14<M7>,
        max_i_pos3_pin: PD15<M8>,
        max_i_neg0_pin: PC6<M9>,
        max_i_neg1_pin: PB5<M10>,
        max_i_neg2_pin: PC8<M11>,
        max_i_neg3_pin: PC9<M12>,
    ) -> Pwms {
        fn init_pwm_pin<P: PwmPin<Duty = u16>>(pin: &mut P) {
            pin.set_duty(0);
            pin.enable();
        }

        // let (mut max_v0, mut max_v1, mut max_v2, mut max_v3) =
        //     setup_tim!(tim1, ccdr, TIM1, max_v0_pin, max_v1_pin, max_v2_pin, max_v3_pin);

        let channels = (
            max_v0_pin.into_alternate_af1(),
            max_v1_pin.into_alternate_af1(),
            max_v2_pin.into_alternate_af1(),
            max_v3_pin.into_alternate_af1(),
        );
        let (mut max_v0, mut max_v1, mut max_v2, mut max_v3) =
            tim1.pwm(channels, F_PWM.khz(), tim1_rcc, &clocks);
        init_pwm_pin(&mut max_v0);
        init_pwm_pin(&mut max_v1);
        init_pwm_pin(&mut max_v2);
        init_pwm_pin(&mut max_v3);

        // let (mut max_i_pos0, mut max_i_pos1, mut max_i_pos2, mut max_i_pos3) =
        //     setup_tim!(tim4, ccdr, TIM4, max_i_pos0_pin, max_i_pos1_pin, max_i_pos2_pin, max_i_pos3_pin);

        let channels = (
            max_i_pos0_pin.into_alternate_af2(),
            max_i_pos1_pin.into_alternate_af2(),
            max_i_pos2_pin.into_alternate_af2(),
            max_i_pos3_pin.into_alternate_af2(),
        );
        let (mut max_i_pos0, mut max_i_pos1, mut max_i_pos2, mut max_i_pos3) =
            tim4.pwm(channels, F_PWM.khz(), tim4_rcc, &clocks);
        init_pwm_pin(&mut max_i_pos0);
        init_pwm_pin(&mut max_i_pos1);
        init_pwm_pin(&mut max_i_pos2);
        init_pwm_pin(&mut max_i_pos3);

        // let (mut max_i_neg0, mut max_i_neg1, mut max_i_neg2, mut max_i_neg3) =
        //     setup_tim!(tim3, ccdr, TIM3, max_i_neg0_pin, max_i_neg1_pin, max_i_neg2_pin, max_i_neg3_pin);
        let channels = (
            max_i_neg0_pin.into_alternate_af2(),
            max_i_neg1_pin.into_alternate_af2(),
            max_i_neg2_pin.into_alternate_af2(),
            max_i_neg3_pin.into_alternate_af2(),
        );
        let (mut max_i_neg0, mut max_i_neg1, mut max_i_neg2, mut max_i_neg3) =
            tim3.pwm(channels, F_PWM.khz(), tim3_rcc, &clocks);
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
    pub fn set(&mut self, ch: u8, duty: f32) {
        fn set<P: PwmPin<Duty = u16>>(pin: &mut P, duty: f32) {
            let duty = i_to_pwm(duty);
            let max = pin.get_max_duty();
            let value = ((duty * (max as f32)) as u16).min(max);
            pin.set_duty(value);
        }
        info!("ch: {:?} duty: {:?}", ch, duty);
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
        self.set( 0,v_to_pwm(max_v0));
        self.set( 1,v_to_pwm(max_v1));
        self.set( 2,i_to_pwm(max_i0));
        self.set( 3,i_to_pwm(min_i0));
        self.set( 4,i_to_pwm(max_i1));
        self.set( 5,i_to_pwm(min_i1));
    }
}

// / DAC: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5680.pdf
// / Peltier Driver: https://datasheets.maximintegrated.com/en/ds/MAX1968-MAX1969.pdf
// pub struct Dac {
//     spi: DacSpi,
//     pub val: [u32; 2],
//     sync0: PG3<Output<PushPull>>,
//     sync1: PG2<Output<PushPull>>,
//     sync2: PG1<Output<PushPull>>,
//     sync3: PG0<Output<PushPull>>,
//     shdn0: PG4<Output<PushPull>>,
//     shdn1: PG5<Output<PushPull>>,
//     shdn2: PG6<Output<PushPull>>,
//     shdn3: PG7<Output<PushPull>>,
// }

// impl Dac {
//     pub fn new(clocks: Clocks, spi3: SPI3, pins: DacPins) -> Self {
//         let spi = Spi::spi3(
//             spi3,
//             (pins.sck, NoMiso, pins.mosi),
//             SPI_MODE,
//             SPI_CLOCK.into(),
//             clocks,
//         );

//         let mut dac = Dac {
//             spi,
//             val: [0, 0],
//             sync0: pins.sync0,
//             sync1: pins.sync1,
//             sync2: pins.sync2,
//             sync3: pins.sync3,
//             shdn0: pins.shdn0,
//             shdn1: pins.shdn1,
//             shdn2: pins.shdn2,
//             shdn3: pins.shdn3,
//         };
//         dac.dis_ch(0);

//         dac.sync0.set_high().unwrap();

//         // default to zero amps
//         dac.set(i_to_dac(0.0), 0);
//         dac
//     }

//     /// Set the DAC output to value on a channel.
//     pub fn set(&mut self, value: u32, ch: u8) {
//         let value = value.min(MAX_VALUE);
//         // 24 bit transfer. First 6 bit and last 2 bit are low.
//         let mut buf = [(value >> 14) as u8, (value >> 6) as u8, (value << 2) as u8];
//         if ch == 0 {
//             self.sync0.set_low().unwrap();
//             self.spi0.transfer(&mut buf).unwrap();
//             self.val[0] = value;
//             self.sync0.set_high().unwrap();
//         } else {
//             // self.sync1.set_high().unwrap();
//             // // must be high for >= 33 ns
//             // delay(100); // 100 * 5.95ns
//             // self.sync1.set_low().unwrap();
//             // self.spi1.transfer(&mut buf).unwrap();
//             // self.val[1] = value;
//         }
//     }

//     /// enable a TEC channel via shutdown pin.
//     pub fn en_ch(&mut self, ch: u8) {
//         if ch == 0 {
//             self.shdn0.set_high().unwrap();
//         } else {
//             // self.shdn1.set_high().unwrap();
//         }
//     }

//     /// disable a TEC channel via shutdown pin.
//     pub fn dis_ch(&mut self, ch: u8) {
//         if ch == 0 {
//             self.shdn0.set_low().unwrap();
//         } else {
//             // self.shdn1.set_low().unwrap();
//         }
//     }
// }
