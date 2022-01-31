use rtt_logger::RTTLogger;

use log::info;
pub struct ThermostatDevices {
    pub ph: bool,
}

pub fn setup(
    mut core: rtic::export::Peripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
) -> (ThermostatDevices) {
    static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Trace);
    rtt_target::rtt_init_print!();
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
    info!("---Starting Hardware Setup");

    (ThermostatDevices { ph: false })
}
