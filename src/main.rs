//! # Driver Firmware
//!
//! Firmware repository for "Driver", an intelligent, high performance laser current driver.
//! Driver is a mezzanine module to Stabilizer, on which the firmware is deployed.
//!
//! This software is currently just the groundwork for the future developments.

#![no_std]
#![no_main]

extern crate panic_halt;
pub extern crate stm32h7xx_hal;

use cortex_m_rt::entry;
use rtt_logger::RTTLogger;
pub use stm32h7xx_hal as hal;

use log::info;

/// Main program function. The computer will start program execution here. duh
#[entry]
fn main() -> ! {
    static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Trace);
    rtt_target::rtt_init_print!();
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
    info!("---Hello World");
    loop {}
}
