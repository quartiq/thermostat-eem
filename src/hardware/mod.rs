//! # Thermostat_EEM Firmware
//!
//! Hardware specific setup etc.

use num_enum::TryFromPrimitive;
pub use stm32h7xx_hal as hal;

pub mod adc_internal;
pub mod dac;
pub mod pwm;
pub mod setup;
pub mod system_timer;

// Thermostat MAC definition
// ToDo: implement eeprom MAC
const SRC_MAC: [u8; 6] = [0x80, 0x1f, 0x12, 0x63, 0x84, 0x1b];

// Number of TX descriptors in the ethernet descriptor ring.
const TX_DESRING_CNT: usize = 4;

// Number of RX descriptors in the ethernet descriptor ring.
const RX_DESRING_CNT: usize = 4;

// // ADC constants
// const GAIN: u32 = 0x555555; // default ADC gain from datasheet
// const R_INNER: f32 = 2.0 * 5100.0; // ratiometric resistor setup. 5.1k high and low side.

// // Steinhart-Hart Parameters
// const ZEROK: f32 = 273.15; // 0°C in °K
// const B: f32 = 3988.0; // NTC beta value
// const T_N_INV: f32 = 1.0 / (25.0 + ZEROK); // T_n = 25°C
// const R_N: f32 = 10000.0; // TEC resistance at 25°C





pub type NetworkStack = smoltcp_nal::NetworkStack<
    'static,
    hal::ethernet::EthernetDMA<'static, TX_DESRING_CNT, RX_DESRING_CNT>,
    system_timer::SystemTimer,
>;

pub type NetworkManager = smoltcp_nal::shared::NetworkManager<
    'static,
    hal::ethernet::EthernetDMA<'static, TX_DESRING_CNT, RX_DESRING_CNT>,
    system_timer::SystemTimer,
>;

pub type EthernetPhy = hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>;

// Front LEDs.
pub struct LEDs {
    pub led0: hal::gpio::gpiog::PG9<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led1: hal::gpio::gpiog::PG10<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led2: hal::gpio::gpioe::PE8<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led3: hal::gpio::gpioe::PE10<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led4: hal::gpio::gpioe::PE12<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led5: hal::gpio::gpiog::PG15<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led6: hal::gpio::gpioe::PE15<hal::gpio::Output<hal::gpio::PushPull>>,
    pub led7: hal::gpio::gpiog::PG8<hal::gpio::Output<hal::gpio::PushPull>>,
}

#[derive(Clone, Copy, TryFromPrimitive)]
#[repr(usize)]
pub enum Channel {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
}
