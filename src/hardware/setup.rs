use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::hardware::adc::Mux;
use heapless::String;
use smoltcp_nal::smoltcp;

use super::adc::AdcConfig;
use super::hal::{
    self as hal,
    ethernet::{self, PHY},
    gpio::{GpioExt, Speed},
    prelude::*,
};

use super::{
    Systick,
    adc::{Adc, AdcPins, sm::StateMachine},
    adc_internal::{AdcInternal, AdcInternalPins},
    dac::{Dac, DacPins},
    fan::{Fan, FanPins},
    gpio::Gpio,
    net::{EthernetPhy, NetworkStack, RX_DESRING_CNT, TX_DESRING_CNT},
    pwm::{Pwm, PwmPins},
};
use platform::ApplicationMetadata;

use log::info;

const NUM_TCP_SOCKETS: usize = 4;
const NUM_UDP_SOCKETS: usize = 1;
const NUM_SOCKETS: usize = NUM_UDP_SOCKETS + NUM_TCP_SOCKETS;

pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_SOCKETS + 2],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub udp_socket_storage: [UdpSocketStorage; NUM_UDP_SOCKETS],
    pub dns_storage: [Option<smoltcp::socket::dns::DnsQuery>; 1],
}

pub struct UdpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 2048],
    tx_metadata: [smoltcp::storage::PacketMetadata<
        smoltcp::socket::udp::UdpMetadata,
    >; 10],
    rx_metadata: [smoltcp::storage::PacketMetadata<
        smoltcp::socket::udp::UdpMetadata,
    >; 10],
}

impl UdpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 2048],
            tx_metadata: [smoltcp::storage::PacketMetadata::EMPTY; 10],
            rx_metadata: [smoltcp::storage::PacketMetadata::EMPTY; 10],
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
            // Placeholder for the real IP address, which is initialized at runtime.
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_SOCKETS + 2],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
            udp_socket_storage: [UdpSocketStorage::new(); NUM_UDP_SOCKETS],
            dns_storage: [None; 1],
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
pub struct ThermostatDevices<
    C: serial_settings::Settings + 'static,
    const Y: usize,
> {
    pub clocks: hal::rcc::CoreClocks,
    pub net: NetworkDevices,
    pub dac: Dac,
    pub pwm: Pwm,
    pub gpio: Gpio,
    pub fan: Fan,
    pub adc_internal: AdcInternal,
    pub adc_sm: StateMachine<Adc>,
    pub adc_input_config: AdcConfig,
    pub usb_serial: super::SerialTerminal<C>,
    pub usb: super::UsbDevice,
    pub metadata: &'static ApplicationMetadata,
    pub settings: C,
}

#[unsafe(link_section = ".sram3.eth")]
/// Static storage for the ethernet DMA descriptor ring.
static DES_RING: grounded::uninit::GroundedCell<
    ethernet::DesRing<{ TX_DESRING_CNT }, { RX_DESRING_CNT }>,
> = grounded::uninit::GroundedCell::uninit();

pub fn setup<C, const Y: usize>(
    mut core: stm32h7xx_hal::stm32::CorePeripherals,
    mut device: stm32h7xx_hal::stm32::Peripherals,
    clock: crate::hardware::SystemTimer,
) -> ThermostatDevices<C, Y>
where
    C: serial_settings::Settings + platform::AppSettings,
{
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

        static LOGGER: rtt_logger::RTTLogger =
            rtt_logger::RTTLogger::new(log::LevelFilter::Debug);
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();
        log::info!("Start logging");
    }

    // Check for a reboot to DFU before doing any system configuration.
    if platform::dfu_flag_is_set() {
        platform::bootload_dfu();
    }

    let pwr = device.PWR.constrain();
    let vos = pwr.freeze();

    // Enable SRAM3 for the ethernet descriptor ring.
    device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    device.RCC.d2ccip1r.modify(|_, w| w.spi123sel().per());

    device.RCC.d3ccipr.modify(|_, w| w.adcsel().per());

    // Clear reset flags.
    device.RCC.rsr.write(|w| w.rmvf().set_bit());

    let rcc = device.RCC.constrain();
    let mut ccdr = rcc
        .use_hse(8.MHz())
        .sysclk(400.MHz())
        .hclk(200.MHz())
        .per_ck(64.MHz())
        .pll2_p_ck(80.MHz())
        .pll2_q_ck(100.MHz())
        .mco1_from_hse(2.MHz())
        .freeze(vos, &device.SYSCFG);

    // Set up USB clocks.
    ccdr.clocks.hsi48_ck().unwrap();
    ccdr.peripheral
        .kernel_usb_clk_mux(stm32h7xx_hal::rcc::rec::UsbClkSel::Hsi48);

    Systick::start(core.SYST, ccdr.clocks.sysclk().to_Hz());

    // After ITCM loading.
    core.SCB.enable_icache();

    info!("--- Starting hardware setup");

    // Note: Frequencies are scaled by 2 to account for the M7 dual instruction pipeline.
    let mut delay = platform::AsmDelay::new(ccdr.clocks.c_ck().to_Hz() * 2);

    // Take GPIOs
    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = device.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    info!("Setup GPIO");
    let mut gpio = Gpio {
        hwrev: [
            gpiod.pd8.into_floating_input().erase(),
            gpiod.pd9.into_floating_input().erase(),
            gpiod.pd10.into_floating_input().erase(),
            gpiod.pd11.into_floating_input().erase(),
        ],
        led_red: [
            gpioe.pe8.into_push_pull_output().erase(),
            gpioe.pe10.into_push_pull_output().erase(),
            gpioe.pe15.into_push_pull_output().erase(),
            gpiog.pg8.into_push_pull_output().erase(),
        ],
        led_green: [
            gpiog.pg9.into_push_pull_output().erase(),
            gpiog.pg10.into_push_pull_output().erase(),
            gpioe.pe12.into_push_pull_output().erase(),
            gpiog.pg15.into_push_pull_output().erase(),
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
    gpio.init();
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

    #[cfg(feature = "all_differential")]
    let adc_input_config = [[
        Some(Mux {
            ainpos: ad7172::Mux::Ain0,
            ainneg: ad7172::Mux::Ain1,
        }),
        Some(Mux {
            ainpos: ad7172::Mux::Ain2,
            ainneg: ad7172::Mux::Ain3,
        }),
        None,
        None,
    ]; 4];

    #[cfg(feature = "all_single_ended")]
    let adc_input_config = [[
        Some(Mux {
            ainpos: ad7172::Mux::Ain0,
            ainneg: ad7172::Mux::RefN,
        }),
        Some(Mux {
            ainpos: ad7172::Mux::RefP,
            ainneg: ad7172::Mux::Ain1,
        }),
        Some(Mux {
            ainpos: ad7172::Mux::Ain2,
            ainneg: ad7172::Mux::RefN,
        }),
        Some(Mux {
            ainpos: ad7172::Mux::RefP,
            ainneg: ad7172::Mux::Ain3,
        }),
    ]; 4];

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
        &adc_input_config,
    )
    .unwrap();

    let mut adc_sm = StateMachine::new(adc);
    adc_sm.start(&mut device.EXTI, &mut device.SYSCFG);

    let mut afe_i2c = {
        let sda = gpiof.pf0.into_alternate_open_drain();
        let scl = gpiof.pf1.into_alternate_open_drain();
        device.I2C2.i2c(
            (scl, sda),
            100.kHz(),
            ccdr.peripheral.I2C2,
            &ccdr.clocks,
        )
    };

    let mut i2c = {
        let sda = gpiob.pb9.into_alternate_open_drain();
        let scl = gpiob.pb8.into_alternate_open_drain();
        device.I2C1.i2c(
            (scl, sda),
            100.kHz(),
            ccdr.peripheral.I2C1,
            &ccdr.clocks,
        )
    };

    let mut eui48 = [0; 6];
    if i2c.write_read(0x50, &[0xFA], &mut eui48).is_err() {
        // wrong ESD protection https://github.com/sinara-hw/Thermostat_EEM/issues/51
        log::warn!("I2C failure, using default MAC");
        eui48 = [0x02, 0x00, 0x00, 0x00, 0x00, 0xd3];
    } else {
        let mut lm75 = lm75::Lm75::new(i2c, lm75::Address::default());
        log::info!("LM75 Temperature: {}°C", lm75.read_temperature().unwrap());
        if let Ok(()) = afe_i2c.write_read(0x50, &[0xFA], &mut eui48) {
            log::info!("AFE EUI48: {}", smoltcp::wire::EthernetAddress(eui48));
        } else {
            log::warn!("AFE EUI48 read failure.");
        }
    }
    let mac_addr = smoltcp::wire::EthernetAddress(eui48);
    log::info!("EUI48: {}", mac_addr);

    let (flash, mut settings) = {
        let mut flash = {
            let (_, flash_bank2) = device.FLASH.split();
            platform::AsyncFlash(super::Flash(flash_bank2.unwrap()))
        };

        let mut settings = C::new(platform::NetSettings::new(mac_addr));
        platform::SerialSettingsPlatform::<_, _, ()>::load(
            &mut settings,
            &mut flash,
        );
        (flash, settings)
    };

    info!("Setup Ethernet");

    // Setup network
    let net = {
        let ethernet_pins = {
            // Reset the PHY before configuring pins.
            let mut eth_phy_nrst = gpiog.pg14.into_push_pull_output();
            eth_phy_nrst.set_low();
            delay.delay_us(200u8);
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

        let ring = unsafe {
            let p = DES_RING.get();
            core::ptr::write(p, ethernet::DesRing::new());
            &mut *p
        };

        // Configure the ethernet controller
        let (mut eth_dma, eth_mac) = ethernet::new(
            device.ETHERNET_MAC,
            device.ETHERNET_MTL,
            device.ETHERNET_DMA,
            ethernet_pins,
            // Note(unsafe): We only call this function once to take ownership of the
            // descriptor ring.
            ring,
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        );

        // Reset and initialize the ethernet phy.
        let mut lan8742a =
            ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        unsafe { ethernet::enable_interrupt() };

        // Configure IP address according to DHCP socket availability
        let ip_addrs: smoltcp::wire::IpAddress = match settings.net().ip.parse()
        {
            Ok(addr) => addr,
            Err(e) => {
                log::warn!(
                    "Invalid IP address in settings: {e:?}. Defaulting to 0.0.0.0 (DHCP)"
                );
                "0.0.0.0".parse().unwrap()
            }
        };

        let random_seed = {
            let mut rng =
                device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
            let mut data = [0u8; 8];
            rng.fill(&mut data).unwrap();
            data
        };

        // Note(unwrap): The hardware configuration function is only allowed to be called once.
        // Unwrapping is intended to panic if called again to prevent re-use of global memory.
        let store =
            cortex_m::singleton!(: NetStorage = NetStorage::default()).unwrap();

        store.ip_addrs[0] = smoltcp::wire::IpCidr::new(ip_addrs, 24);

        let mut ethernet_config = smoltcp::iface::Config::new(
            smoltcp::wire::HardwareAddress::Ethernet(mac_addr),
        );
        ethernet_config.random_seed = u64::from_be_bytes(random_seed);

        let mut interface = smoltcp::iface::Interface::new(
            ethernet_config,
            &mut eth_dma,
            smoltcp::time::Instant::ZERO,
        );

        interface
            .routes_mut()
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .unwrap();

        interface.update_ip_addrs(|ref mut addrs| {
            if !ip_addrs.is_unspecified() {
                addrs
                    .push(smoltcp::wire::IpCidr::new(ip_addrs, 24))
                    .unwrap();
            }
        });

        let mut sockets =
            smoltcp::iface::SocketSet::new(&mut store.sockets[..]);
        for storage in store.tcp_socket_storage[..].iter_mut() {
            let tcp_socket = {
                let rx_buffer = smoltcp::socket::tcp::SocketBuffer::new(
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::tcp::SocketBuffer::new(
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::tcp::Socket::new(rx_buffer, tx_buffer)
            };

            sockets.add(tcp_socket);
        }

        if ip_addrs.is_unspecified() {
            sockets.add(smoltcp::socket::dhcpv4::Socket::new());
        }

        sockets.add(smoltcp::socket::dns::Socket::new(
            &[],
            &mut store.dns_storage[..],
        ));

        for storage in store.udp_socket_storage[..].iter_mut() {
            let udp_socket = {
                let rx_buffer = smoltcp::socket::udp::PacketBuffer::new(
                    &mut storage.rx_metadata[..],
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::udp::PacketBuffer::new(
                    &mut storage.tx_metadata[..],
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::udp::Socket::new(rx_buffer, tx_buffer)
            };

            sockets.add(udp_socket);
        }

        let mut stack =
            smoltcp_nal::NetworkStack::new(interface, eth_dma, sockets, clock);

        stack.seed_random_port(&random_seed);

        NetworkDevices {
            stack,
            phy: lan8742a,
            mac_address: mac_addr,
        }
    };

    let (usb_device, usb_serial) = {
        let _usb_id = gpioa.pa10.into_alternate::<10>();
        let usb_n = gpioa.pa11.into_alternate();
        let usb_p = gpioa.pa12.into_alternate();
        let usb = stm32h7xx_hal::usb_hs::USB2::new(
            device.OTG2_HS_GLOBAL,
            device.OTG2_HS_DEVICE,
            device.OTG2_HS_PWRCLK,
            usb_n,
            usb_p,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        let endpoint_memory =
            cortex_m::singleton!(: Option<&'static mut [u32]> = None).unwrap();
        endpoint_memory.replace(
            &mut cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap()[..],
        );
        let usb_bus = cortex_m::singleton!(: usb_device::bus::UsbBusAllocator<super::UsbBus> =
        stm32h7xx_hal::usb_hs::UsbBus::new(
            usb,
            endpoint_memory.take().unwrap(),
        ))
        .unwrap();

        let read_store = cortex_m::singleton!(: [u8; 128] = [0; 128]).unwrap();
        let write_store =
            cortex_m::singleton!(: [u8; 1024] = [0; 1024]).unwrap();
        let serial = usbd_serial::SerialPort::new_with_store(
            usb_bus,
            &mut read_store[..],
            &mut write_store[..],
        );

        // Generate a device serial number from the MAC address.
        let serial_number = cortex_m::singleton!(: String<17> = {
            let mut s = String::new();
            write!(s, "{mac_addr}").unwrap();
            s
        })
        .unwrap();

        let usb_device = usb_device::device::UsbDeviceBuilder::new(
            usb_bus,
            usb_device::device::UsbVidPid(0x1209, 0x391A),
        )
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("ARTIQ/Sinara")
            .product("Thermostat-EEM")
            .serial_number(serial_number)])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        (usb_device, serial)
    };

    let metadata = super::metadata::metadata(gpio.hwrev_str());

    let usb_terminal = {
        let input_buffer =
            cortex_m::singleton!(: [u8; 128] = [0u8; 128]).unwrap();
        let serialize_buffer =
            cortex_m::singleton!(: [u8; 512] = [0u8; 512]).unwrap();

        serial_settings::Runner::new(
            platform::SerialSettingsPlatform {
                interface: serial_settings::BestEffortInterface::new(
                    usb_serial,
                ),
                storage: flash,
                metadata,
                _settings_marker: core::marker::PhantomData,
            },
            input_buffer,
            serialize_buffer,
            &mut settings,
        )
        .unwrap()
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
        adc_input_config,
        usb_serial: usb_terminal,
        settings,
        usb: usb_device,
        metadata,
    }
}
