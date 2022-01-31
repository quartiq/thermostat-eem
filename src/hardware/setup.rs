use rtt_logger::RTTLogger;
use stm32h7xx_hal::hal::digital::v2::OutputPin;

use super::hal::{gpio::GpioExt, prelude::*};

use log::info;
pub struct ThermostatDevices {
    pub ph: bool,
}

pub fn setup(
    mut core: rtic::export::Peripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
) -> (ThermostatDevices) {
    let pwr = device.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .use_hse(8.mhz())
        .sysclk(400.mhz())
        .hclk(200.mhz())
        .per_ck(100.mhz())
        .pll2_p_ck(100.mhz())
        .pll2_q_ck(100.mhz())
        .freeze(vos, &device.SYSCFG);

    static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Trace);
    rtt_target::rtt_init_print!();
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
    info!("---Starting Hardware Setup");

    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    let mut leds = [gpiog.pg9.into_push_pull_output()];

    leds[0].set_high().unwrap();

    (ThermostatDevices { ph: false })
}
