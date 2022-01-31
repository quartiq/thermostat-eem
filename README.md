# thermostat-eem
Embedded software for the [Thermostat-EEM](https://github.com/sinara-hw/Thermostat_EEM) multichannel temperature controller.

- developed in Rust
- using [STM32H7 hal](https://github.com/stm32-rs/stm32h7xx-hal) 
- [RTIC](https://github.com/rtic-rs/cortex-m-rtic) based task scheduling
- [MQTT](https://mqtt.org/) networking using the [smoltcp](https://github.com/smoltcp-rs/smoltcp) tcp/ip stack, [minimq](https://github.com/quartiq/minimq) for embedded MQTT and [miniconf](https://github.com/quartiq/miniconf) for settings
- signal processing and control based on biquad IIR filters from [idsp](https://github.com/quartiq/idsp)
