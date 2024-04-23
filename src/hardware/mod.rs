//! # Thermostat_EEM Firmware
//!
//! Hardware specific setup etc.

pub use stm32h7xx_hal as hal;

pub mod ad7172;
pub mod adc;
pub mod adc_internal;
pub mod dac;
pub mod delay;
pub mod fan;
pub mod flash;
pub mod gpio;
pub mod metadata;
pub mod platform;
pub mod pwm;
pub mod setup;
pub mod system_timer;

// Number of TX descriptors in the ethernet descriptor ring.
const TX_DESRING_CNT: usize = 4;

// Number of RX descriptors in the ethernet descriptor ring.
const RX_DESRING_CNT: usize = 4;

pub type NetworkStack = smoltcp_nal::NetworkStack<
    'static,
    hal::ethernet::EthernetDMA<TX_DESRING_CNT, RX_DESRING_CNT>,
    SystemTimer,
>;

pub type NetworkManager = smoltcp_nal::shared::NetworkManager<
    'static,
    hal::ethernet::EthernetDMA<TX_DESRING_CNT, RX_DESRING_CNT>,
    SystemTimer,
>;

pub type EthernetPhy = hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>;

#[derive(Clone, Copy, strum::EnumIter, Debug)]
#[repr(usize)]
pub enum OutputChannelIdx {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

/// System timer (RTIC Monotonic) tick frequency
pub const MONOTONIC_FREQUENCY: u32 = 1_000;
pub type Systick = rtic_monotonics::systick::Systick;
pub type SystemTimer = mono_clock::MonoClock<u32, MONOTONIC_FREQUENCY>;

pub type SerialPort = usbd_serial::SerialPort<
    'static,
    crate::hardware::UsbBus,
    &'static mut setup::SerialBufferStore,
    &'static mut setup::SerialBufferStore,
>;

pub type SerialTerminal<C, const Y: usize> =
    serial_settings::Runner<'static, crate::settings::SerialSettingsPlatform<C, Y>, Y>;

pub type UsbBus = stm32h7xx_hal::usb_hs::UsbBus<stm32h7xx_hal::usb_hs::USB2>;

// Type alias for the USB device.
pub type UsbDevice = usb_device::device::UsbDevice<'static, UsbBus>;
