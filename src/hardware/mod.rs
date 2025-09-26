//! # Thermostat_EEM Firmware
//!
//! Hardware specific setup etc.

use hal::flash::{LockedFlashBank, UnlockedFlashBank};
use platform::{AsyncFlash, UnlockFlash};
pub use stm32h7xx_hal as hal;

pub mod adc;
pub mod adc_internal;
pub mod dac;
pub mod fan;
pub mod gpio;
pub mod metadata;
pub mod net;
pub mod pwm;
pub mod setup;

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

/// System timer (RTIC Monotonic) tick frequency
pub const MONOTONIC_FREQUENCY: u32 = 1_000;
rtic_monotonics::systick_monotonic!(Systick, MONOTONIC_FREQUENCY);
pub type SystemTimer = mono_clock::MonoClock<u32, MONOTONIC_FREQUENCY>;

pub type UsbBus = stm32h7xx_hal::usb_hs::UsbBus<stm32h7xx_hal::usb_hs::USB2>;

// Type alias for the USB device.
pub type UsbDevice = usb_device::device::UsbDevice<'static, UsbBus>;

pub type SerialPort =
    usbd_serial::SerialPort<'static, UsbBus, &'static mut [u8], &'static mut [u8]>;

pub type SerialTerminal<C> = serial_settings::Runner<
    'static,
    platform::SerialSettingsPlatform<C, AsyncFlash<Flash>, SerialPort>,
>;

pub struct Flash(LockedFlashBank);

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error = <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn capacity(&self) -> usize {
        self.0.capacity()
    }

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }
}

impl UnlockFlash for Flash {
    type Unlocked<'a> = UnlockedFlashBank<'a>;
    fn unlock(&mut self) -> Self::Unlocked<'_> {
        self.0.unlocked()
    }
}
