use core::sync::atomic::{AtomicBool, Ordering};

use crate::hardware::{
    adc::{AdcConfig, AdcInput},
    system_timer,
};
use smoltcp_nal::smoltcp;

use super::hal::{
    self as hal,
    ethernet::{self, PHY},
    gpio::{GpioExt, Speed},
    prelude::*,
};

use super::{
    adc::{sm::StateMachine, Adc, AdcPins},
    adc_internal::{AdcInternal, AdcInternalPins},
    dac::{Dac, DacPins},
    delay,
    fan::{Fan, FanPins},
    gpio::{Gpio, GpioPins},
    pwm::{Pwm, PwmPins},
    EthernetPhy, NetworkStack,
};

use log::info;

const NUM_TCP_SOCKETS: usize = 4;
const NUM_UDP_SOCKETS: usize = 1;
const NUM_SOCKETS: usize = NUM_UDP_SOCKETS + NUM_TCP_SOCKETS;

pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],

    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_SOCKETS + 1],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub udp_socket_storage: [UdpSocketStorage; NUM_UDP_SOCKETS],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
}

pub struct UdpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 2048],
    tx_metadata: [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
    rx_metadata: [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
}

impl UdpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 2048],
            tx_metadata: [smoltcp::storage::PacketMetadata::<smoltcp::wire::IpEndpoint>::EMPTY; 10],
            rx_metadata: [smoltcp::storage::PacketMetadata::<smoltcp::wire::IpEndpoint>::EMPTY; 10],
        }
    }
}

#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 1024],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 1024],
        }
    }
}

impl Default for NetStorage {
    fn default() -> Self {
        NetStorage {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
                smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
            )],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_SOCKETS + 1],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
            udp_socket_storage: [UdpSocketStorage::new(); NUM_UDP_SOCKETS],
        }
    }
}

/// The available networking devices on Thermostat.
pub struct NetworkDevices {
    pub stack: NetworkStack,
    pub phy: EthernetPhy,
    pub mac_address: smoltcp::wire::EthernetAddress,
}

/// The available hardware interfaces on Thermostat.
pub struct ThermostatDevices {
    pub clocks: hal::rcc::CoreClocks,
    pub net: NetworkDevices,
    pub dac: Dac,
    pub pwm: Pwm,
    pub gpio: Gpio,
    pub fan: Fan,
    pub adc_internal: AdcInternal,
    pub adc_sm: StateMachine<Adc>,
    pub adc_channels: [[bool; 4]; 4],
}

#[link_section = ".sram3.eth"]
/// Static storage for the ethernet DMA descriptor ring.
static mut DES_RING: ethernet::DesRing<{ super::TX_DESRING_CNT }, { super::RX_DESRING_CNT }> =
    ethernet::DesRing::new();

pub fn setup(
    mut device: stm32h7xx_hal::stm32::Peripherals,
    clock: system_timer::SystemTimer,
) -> ThermostatDevices {
    // Set up RTT logging
    {
        // Enable debug during WFE/WFI-induced sleep
        device.DBGMCU.cr.modify(|_, w| w.dbgsleep_d1().set_bit());

        // Set up RTT channel to use for `rprintln!()` as "best effort".
        // This removes a critical section around the logging and thus allows
        // high-prio tasks to always interrupt at low latency.
        // It comes at a cost:
        // If a high-priority tasks preempts while we are logging something,
        // and if we then also want to log from within that high-preiority task,
        // the high-prio log message will be lost.

        let channels = rtt_target::rtt_init_default!();
        // Note(unsafe): The closure we pass does not establish a critical section
        // as demanded but it does ensure synchronization and implements a lock.
        unsafe {
            rtt_target::set_print_channel_cs(
                channels.up.0,
                &((|arg, f| {
                    static LOCKED: AtomicBool = AtomicBool::new(false);
                    if LOCKED.compare_exchange_weak(
                        false,
                        true,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    ) == Ok(false)
                    {
                        f(arg);
                        LOCKED.store(false, Ordering::Release);
                    }
                }) as rtt_target::CriticalSectionFunc),
            );
        }

        static LOGGER: rtt_logger::RTTLogger = rtt_logger::RTTLogger::new(log::LevelFilter::Debug);
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();
        log::info!("Start logging");
    }
    let pwr = device.PWR.constrain();
    let vos = pwr.freeze();

    // Enable SRAM3 for the ethernet descriptor ring.
    device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    device.RCC.d2ccip1r.modify(|_, w| w.spi123sel().per());

    // Clear reset flags.
    device.RCC.rsr.write(|w| w.rmvf().set_bit());

    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .use_hse(8.MHz())
        .sysclk(400.MHz())
        .hclk(200.MHz())
        .per_ck(100.MHz())
        .pll2_p_ck(80.MHz())
        .pll2_q_ck(100.MHz())
        .mco1_from_hse(2.MHz())
        .freeze(vos, &device.SYSCFG);

    info!("--- Starting hardware setup");

    let mut delay = delay::AsmDelay::new(ccdr.clocks.c_ck().to_Hz());

    // Take GPIOs
    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = device.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    info!("Setup GPIO");
    let gpio_pins = GpioPins {
        hwrev: [
            gpiod.pd8.into_floating_input().erase(),
            gpiod.pd9.into_floating_input().erase(),
            gpiod.pd10.into_floating_input().erase(),
            gpiod.pd11.into_floating_input().erase(),
        ],
        led: [
            gpiog.pg9.into_push_pull_output().erase(),
            gpiog.pg10.into_push_pull_output().erase(),
            gpioe.pe8.into_push_pull_output().erase(),
            gpioe.pe10.into_push_pull_output().erase(),
            gpioe.pe12.into_push_pull_output().erase(),
            gpiog.pg15.into_push_pull_output().erase(),
            gpioe.pe15.into_push_pull_output().erase(),
            gpiog.pg8.into_push_pull_output().erase(),
        ],
        shdn: [
            gpiog.pg4.into_push_pull_output().erase(),
            gpiog.pg5.into_push_pull_output().erase(),
            gpiog.pg6.into_push_pull_output().erase(),
            gpiog.pg7.into_push_pull_output().erase(),
        ],
        poe_pwr: gpiof.pf2.into_floating_input(),
        at_event: gpioe.pe7.into_floating_input(),
        eem_pwr: gpiod.pd0.into_push_pull_output(),
        tec_freq: gpiod.pd2.into_push_pull_output(),
        overtemp: gpiog.pg12.into_floating_input(),
    };
    let gpio = Gpio::new(gpio_pins);
    info!("HWREV: {:?}", gpio.hwrev());
    info!("PoE Power: {:?}", gpio.poe());
    info!("Overtemp: {:?}", gpio.overtemp());

    info!("Setup fan");
    let fan = Fan::new(
        &ccdr.clocks,
        (ccdr.peripheral.TIM2, ccdr.peripheral.TIM8),
        (device.TIM2, device.TIM8),
        FanPins {
            tacho: gpiob.pb10.into_alternate(),
            pwm: gpioc.pc7.into_alternate(),
        },
    );

    info!("Setup TEC limit PWM");
    let pwm_pins = PwmPins {
        voltage: (
            gpioe.pe9.into_alternate(),
            gpioe.pe11.into_alternate(),
            gpioe.pe13.into_alternate(),
            gpioe.pe14.into_alternate(),
        ),
        negative_current: (
            gpioc.pc6.into_alternate(),
            gpiob.pb5.into_alternate(),
            gpioc.pc8.into_alternate(),
            gpioc.pc9.into_alternate(),
        ),
        positive_current: (
            gpiod.pd12.into_alternate(),
            gpiod.pd13.into_alternate(),
            gpiod.pd14.into_alternate(),
            gpiod.pd15.into_alternate(),
        ),
    };

    let pwm = Pwm::new(
        &ccdr.clocks,
        (
            ccdr.peripheral.TIM1,
            ccdr.peripheral.TIM3,
            ccdr.peripheral.TIM4,
        ),
        (device.TIM1, device.TIM3, device.TIM4),
        pwm_pins,
    );

    info!("Setup TEC driver DACs");
    let dac = Dac::new(
        &ccdr.clocks,
        ccdr.peripheral.SPI3,
        device.SPI3,
        gpioc.pc10.into_alternate(),
        gpioc.pc12.into_alternate(),
        DacPins {
            sync: [
                gpiog.pg3.into_push_pull_output().erase(),
                gpiog.pg2.into_push_pull_output().erase(),
                gpiog.pg1.into_push_pull_output().erase(),
                gpiog.pg0.into_push_pull_output().erase(),
            ],
        },
    );

    info!("Setup CPU ADCs");
    let adc_internal_pins = AdcInternalPins {
        output_voltage: (
            gpioc.pc3.into_analog(),
            gpioa.pa0.into_analog(),
            gpioa.pa3.into_analog(),
            gpioa.pa4.into_analog(),
        ),
        output_current: (
            gpioa.pa5.into_analog(),
            gpioa.pa6.into_analog(),
            gpiob.pb0.into_analog(),
            gpiob.pb1.into_analog(),
        ),
        output_vref: (
            gpiof.pf3.into_analog(),
            gpiof.pf4.into_analog(),
            gpiof.pf5.into_analog(),
            gpiof.pf6.into_analog(),
        ),
        p3v3_voltage: gpiof.pf7.into_analog(),
        p5v_voltage: gpioc.pc0.into_analog(),
        p12v_voltage: gpioc.pc2.into_analog(),
        p12v_current: gpiof.pf8.into_analog(),
    };

    let mut adc_internal = AdcInternal::new(
        &mut delay,
        &ccdr.clocks,
        (ccdr.peripheral.ADC12, ccdr.peripheral.ADC3),
        (
            device.ADC1,
            device.ADC2,
            device.ADC3,
            device.ADC12_COMMON,
            device.ADC3_COMMON,
        ),
        adc_internal_pins,
    );

    info!("P3V3: {} V", adc_internal.read_p3v3_voltage());
    info!("P5V: {} V", adc_internal.read_p5v_voltage());
    info!(
        "P12V: {} V, {} A",
        adc_internal.read_p12v_voltage(),
        adc_internal.read_p12v_current()
    );

    info!("Setup ADC");

    // enable MCO 2MHz clock output to ADCs
    gpioa.pa8.into_alternate::<0>();

    // Each ADC has two differential inputs
    #[cfg(feature = "all_differential")]
    #[allow(unused_variables)]
    let adc_input_config = AdcConfig {
        input_config: [
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
        ],
    };

    #[cfg(feature = "ai_artiq")]
    let adc_input_config = AdcConfig {
        input_config: [
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
            [
                Some((AdcInput::Ain0, AdcInput::Ain1)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
                None,
                None,
            ],
            // This is a hack to get the single ended inputs to work. Somehow the ADC sometimes went
            // into an error state and did not recover if the single ended channels were referenced to the positive reference input.
            // The workaround is to connect the positive reference to an unused single ended input (Ain2) and
            // reference the single ended input to this input. The ADC seems to not have a problem recovering here.
            [
                Some((AdcInput::Ain2, AdcInput::Ain0)),
                Some((AdcInput::Ain2, AdcInput::Ain1)),
                Some((AdcInput::Ain0, AdcInput::Ain2)),
                Some((AdcInput::Ain2, AdcInput::Ain3)),
            ],
        ],
    };

    for (i, config) in adc_input_config.input_config.iter().enumerate() {
        log::info!("ADC{} input configuration: {:?}", i, config);
    }

    let adc = Adc::new(
        &mut delay,
        &ccdr.clocks,
        ccdr.peripheral.SPI4,
        device.SPI4,
        AdcPins {
            spi: (
                gpioe.pe2.into_alternate(),
                gpioe.pe5.into_alternate(),
                gpioe.pe6.into_alternate(),
            ),
            cs: [
                gpioe.pe0.into_push_pull_output().erase(),
                gpioe.pe1.into_push_pull_output().erase(),
                gpioe.pe3.into_push_pull_output().erase(),
                gpioe.pe4.into_push_pull_output().erase(),
            ],
            rdyn: gpioc.pc11.into_pull_up_input(),
            sync: gpiob.pb11.into_push_pull_output(),
        },
        adc_input_config,
    )
    .unwrap();

    let adc_channels = adc.channels();

    let mut adc_sm = StateMachine::new(adc);
    adc_sm.start(&mut device.EXTI, &mut device.SYSCFG);

    let mut eeprom_i2c = {
        let sda = gpiob.pb9.into_alternate().set_open_drain();
        let scl = gpiob.pb8.into_alternate().set_open_drain();
        device
            .I2C1
            .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks)
    };

    // The EEPROM is a variant without address bits, so the 3 LSB of this word are "dont-cares".
    const I2C_ADDR: u8 = 0x50;
    // The MAC address is stored in the last 6 bytes of the 256 byte address space.
    const MAC_POINTER: u8 = 0xFA;

    let mut buffer = [0u8; 6];
    eeprom_i2c
        .write_read(I2C_ADDR, &[MAC_POINTER], &mut buffer)
        .unwrap();
    let mac_addr = smoltcp::wire::EthernetAddress(buffer);
    log::info!("EUI48: {}", mac_addr);

    info!("Setup Ethernet");

    // Setup network
    let net = {
        let ethernet_pins = {
            // Reset the PHY before configuring pins.
            let mut eth_phy_nrst = gpiog.pg14.into_push_pull_output();
            eth_phy_nrst.set_low();
            delay.delay_us(200u8);
            eth_phy_nrst.set_high();

            eth_phy_nrst.set_high();

            let ref_clk = gpioa.pa1.into_alternate().speed(Speed::VeryHigh);
            let mdio = gpioa.pa2.into_alternate().speed(Speed::VeryHigh);
            let mdc = gpioc.pc1.into_alternate().speed(Speed::VeryHigh);
            let crs_dv = gpioa.pa7.into_alternate().speed(Speed::VeryHigh);
            let rxd0 = gpioc.pc4.into_alternate().speed(Speed::VeryHigh);
            let rxd1 = gpioc.pc5.into_alternate().speed(Speed::VeryHigh);
            let tx_en = gpiog.pg11.into_alternate().speed(Speed::VeryHigh);
            let txd0 = gpiog.pg13.into_alternate().speed(Speed::VeryHigh);
            let txd1 = gpiob.pb13.into_alternate().speed(Speed::VeryHigh);

            (ref_clk, mdio, mdc, crs_dv, rxd0, rxd1, tx_en, txd0, txd1)
        };

        // Configure the ethernet controller
        let (eth_dma, eth_mac) = ethernet::new(
            device.ETHERNET_MAC,
            device.ETHERNET_MTL,
            device.ETHERNET_DMA,
            ethernet_pins,
            // Note(unsafe): We only call this function once to take ownership of the
            // descriptor ring.
            unsafe { &mut DES_RING },
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        );

        // Reset and initialize the ethernet phy.
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        unsafe { ethernet::enable_interrupt() };

        // Note(unwrap): The hardware configuration function is only allowed to be called once.
        // Unwrapping is intended to panic if called again to prevent re-use of global memory.
        let store = cortex_m::singleton!(: NetStorage = NetStorage::default()).unwrap();

        store.ip_addrs[0] = smoltcp::wire::IpCidr::new(
            smoltcp::wire::IpAddress::Ipv4(smoltcp::wire::Ipv4Address::UNSPECIFIED),
            0,
        );

        let mut routes = smoltcp::iface::Routes::new(&mut store.routes_cache[..]);
        routes
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .unwrap();

        let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

        let mut interface = smoltcp::iface::InterfaceBuilder::new(eth_dma, &mut store.sockets[..])
            .hardware_addr(smoltcp::wire::HardwareAddress::Ethernet(mac_addr))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();

        interface.add_socket(smoltcp::socket::Dhcpv4Socket::new());

        for storage in store.tcp_socket_storage[..].iter_mut() {
            let tcp_socket = {
                let rx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut storage.rx_storage[..]);
                let tx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut storage.tx_storage[..]);

                smoltcp::socket::TcpSocket::new(rx_buffer, tx_buffer)
            };

            interface.add_socket(tcp_socket);
        }

        for storage in store.udp_socket_storage[..].iter_mut() {
            let udp_socket = {
                let rx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                    &mut storage.rx_metadata[..],
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                    &mut storage.tx_metadata[..],
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::UdpSocket::new(rx_buffer, tx_buffer)
            };

            interface.add_socket(udp_socket);
        }

        let random_seed = {
            let mut rng = device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
            let mut data = [0u8; 4];
            rng.fill(&mut data).unwrap();
            data
        };

        let mut stack = smoltcp_nal::NetworkStack::new(interface, clock);

        stack.seed_random_port(&random_seed);

        NetworkDevices {
            stack,
            phy: lan8742a,
            mac_address: mac_addr,
        }
    };

    info!("--- Hardware setup done");

    ThermostatDevices {
        clocks: ccdr.clocks,
        net,
        dac,
        pwm,
        gpio,
        fan,
        adc_internal,
        adc_sm,
        adc_channels,
    }
}
