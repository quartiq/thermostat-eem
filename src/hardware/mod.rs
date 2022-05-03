//! # Thermostat_EEM Firmware
//!
//! Hardware specific setup etc.

use enum_iterator::IntoEnumIterator;
use num_enum::TryFromPrimitive;
pub use stm32h7xx_hal as hal;

pub mod ad7172;
pub mod adc;
pub mod adc_internal;
pub mod dac;
pub mod delay;
pub mod fan;
pub mod gpio;
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

#[derive(Clone, Copy, TryFromPrimitive, IntoEnumIterator, defmt::Format)]
#[repr(usize)]
pub enum OutputChannel {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}
