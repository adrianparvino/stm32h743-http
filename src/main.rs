// #![deny(unsafe_code)]
#![deny(warnings)]
#![allow(unused_imports)]
#![allow(unused_must_use)]
#![allow(dead_code)]
#![no_std]
#![no_main]

use rtic::app;

mod cdc_ecm;
mod veth;
mod response_builder;
mod request;
mod http_server;

use defmt_rtt as _;
use panic_probe as _;

#[app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use rtic::Monotonic;
    use rtic::time::duration::Seconds;
    use rtic::time::duration::Milliseconds;
    use rtic::time::duration::Microseconds;
    use stm32h7xx_hal::{
        rcc::rec::UsbClkSel,
        gpio::{gpioa::PA1, Input, Output, PullUp, PushPull},
        usb_hs::{UsbBus, USB2},
        prelude::*,
    };
    use embedded_hal::digital::v2::InputPin;
    use embedded_hal::digital::v2::OutputPin;

    use usb_device::prelude::*;
    use usb_device::class_prelude::UsbBusAllocator;

    use smoltcp::time::Duration;
    use smoltcp::iface::{Neighbor, NeighborCache, InterfaceBuilder, Interface as EthernetInterface};
    use smoltcp::wire::{
        IpCidr,
        IpAddress,
        Ipv6Repr,
        EthernetAddress,
        IpVersion,
        IpProtocol::*,
        Icmpv6Repr::Ndisc,
        NdiscRouterFlags,
        NdiscPrefixInformation,
        NdiscPrefixInfoFlags,
        NdiscRepr::*
    };
    use smoltcp::socket::{
        SocketHandle,
        SocketSet,
        SocketSetItem,
        TcpSocket, TcpSocketBuffer,
        UdpSocket, UdpSocketBuffer, UdpPacketMetadata,
        RawSocket, RawSocketBuffer, RawPacketMetadata,
    };

    use dwt_systick_monotonic::DwtSystick;

    use replace_with::replace_with_or_abort;

    use core::mem::MaybeUninit;

    use crate::cdc_ecm::*;
    use crate::veth::*;
    use crate::http_server::{self, State as HttpState};

    use heapless::Vec;

    #[derive(Debug)]
    pub enum TcpContinuation {
        Listen(&'static mut Vec<u8, 1024>),
        Receive(&'static mut Vec<u8, 1024>),
        Transmit(&'static mut Vec<u8, 1024>, &'static [u8]),
    }

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBus<USB2>>,
        iface: EthernetInterface<'static, Veth<'static, UsbBus<USB2>>>,
        sockets: SocketSet<'static>,
        // rtc: Rtc,
        // button: PA0<Input<PullUp>>,
        led: PA1<Output<PushPull>>,
    }

    #[local]
    struct Local {
        http_servers: [http_server::State; 2]
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<400_000_000>; // 400 MHz

    #[init(local = [
        ETH_RX_BUFFER: [u8; 1514] = [0u8; 1514],
        ETH_TX_BUFFER: [u8; 1514] = [0u8; 1514],

        TCP_RX_BUFFER0: [u8; 1024] = [0u8; 1024],
        TCP_TX_BUFFER0: [u8; 2048] = [0u8; 2048],
        HTTP_STATE0: Vec<u8, 1024> = Vec::new(),

        TCP_RX_BUFFER1: [u8; 1024] = [0u8; 1024],
        TCP_TX_BUFFER1: [u8; 2048] = [0u8; 2048],
        HTTP_STATE1: Vec<u8, 1024> = Vec::new(),

        MDNS_RX_BUFFER: [u8; 512] = [0u8; 512],
        MDNS_TX_BUFFER: [u8; 512] = [0u8; 512],
        MDNS_RX_UDP_PACKET_METADATA: [UdpPacketMetadata; 4] = [UdpPacketMetadata::EMPTY; 4],
        MDNS_TX_UDP_PACKET_METADATA: [UdpPacketMetadata; 4] = [UdpPacketMetadata::EMPTY; 4],

        ICMP_RX_BUFFER: [u8; 512] = [0u8; 512],
        ICMP_TX_BUFFER: [u8; 512] = [0u8; 512],
        ICMP_RX_PACKET_METADATA: [RawPacketMetadata; 4] = [RawPacketMetadata::EMPTY; 4],
        ICMP_TX_PACKET_METADATA: [RawPacketMetadata; 4] = [RawPacketMetadata::EMPTY; 4],

        EP_MEMORY: [u32; 2048] = [0; 2048],
        USB_BUS: MaybeUninit<UsbBusAllocator<UsbBus<USB2>>> = MaybeUninit::uninit(),
        IP_ADDRS: MaybeUninit<[IpCidr; 2]> = MaybeUninit::uninit(),
        NEIGHBOR_CACHE: [Option<(IpAddress, Neighbor)>; 8] = [None; 8],
        SOCKET_STORE: [Option<SocketSetItem<'static>>; 8] = [None, None, None, None, None, None, None, None]
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let init::LocalResources {
            ETH_RX_BUFFER,
            ETH_TX_BUFFER,

            TCP_RX_BUFFER0,
            TCP_TX_BUFFER0,
            HTTP_STATE0,

            TCP_RX_BUFFER1,
            TCP_TX_BUFFER1,
            HTTP_STATE1,

            MDNS_RX_BUFFER,
            MDNS_TX_BUFFER,
            MDNS_RX_UDP_PACKET_METADATA,
            MDNS_TX_UDP_PACKET_METADATA,

            ICMP_RX_BUFFER,
            ICMP_TX_BUFFER,
            ICMP_RX_PACKET_METADATA,
            ICMP_TX_PACKET_METADATA,

            EP_MEMORY,
            USB_BUS,
            IP_ADDRS,
            NEIGHBOR_CACHE,
            SOCKET_STORE
        } = ctx.local;

        let ip_addrs: &'static mut [IpCidr] = unsafe {
            IP_ADDRS.as_mut_ptr().write([
                IpCidr::new(IpAddress::v6(0xfe80, 0, 0, 0, 0, 0xff, 0xfe00, 1), 64),
                IpCidr::new(IpAddress::v6(0xfd00, 0, 0, 0, 0, 0, 0, 1), 64)
            ]);
            IP_ADDRS.assume_init_mut()
        };

        let neighbor_cache = NeighborCache::new(NEIGHBOR_CACHE as &'static mut [Option<(IpAddress, Neighbor)>] );


        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = ctx.device.RCC.constrain();
        let mut ccdr = {
            rcc.use_hse(25.mhz())
               .sysclk(400.mhz())
               .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
               .freeze(pwrcfg, &ctx.device.SYSCFG)
        };

        // 48MHz CLOCK
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);

        let dwt_systick = MyMono::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, 400_000_000);

        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let led = gpioa.pa1.into_push_pull_output();

        let pin_dm = gpioa.pa11.into_alternate_af10();
        let pin_dp = gpioa.pa12.into_alternate_af10();

        let usb = USB2::new(
            ctx.device.OTG2_HS_GLOBAL,
            ctx.device.OTG2_HS_DEVICE,
            ctx.device.OTG2_HS_PWRCLK,
            pin_dm,
            pin_dp,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks
        );

        let usb_bus = unsafe {
            USB_BUS.as_mut_ptr().write(UsbBus::new(usb, &mut *EP_MEMORY));
            USB_BUS.assume_init_ref()
        };

        let ecm = CdcEcmClass::new(&usb_bus, ETH_RX_BUFFER, ETH_TX_BUFFER);
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .product("Cat port")
            .device_class(USB_CLASS_CDC)
            .build();

        let veth = ecm.jack_in();

        let iface = InterfaceBuilder::new(veth)
            .ethernet_addr(EthernetAddress::from_bytes(&[2,0,0,0,0,1]))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(ip_addrs)
            .finalize();

        let mut sockets = SocketSet::new(SOCKET_STORE as &'static mut [Option<SocketSetItem<'static>>]);

        let tcp_rx_buffer0 = TcpSocketBuffer::new(TCP_RX_BUFFER0 as &'static mut [u8]);
        let tcp_tx_buffer0 = TcpSocketBuffer::new(TCP_TX_BUFFER0 as &'static mut [u8]);
        let tcp_socket0 = TcpSocket::new(tcp_rx_buffer0, tcp_tx_buffer0);
        let tcp_handle0 = sockets.add(tcp_socket0);
        let http_server0 = HttpState::new(tcp_handle0, HTTP_STATE0);

        let tcp_rx_buffer1 = TcpSocketBuffer::new(TCP_RX_BUFFER1 as &'static mut [u8]);
        let tcp_tx_buffer1 = TcpSocketBuffer::new(TCP_TX_BUFFER1 as &'static mut [u8]);
        let tcp_socket1 = TcpSocket::new(tcp_rx_buffer1, tcp_tx_buffer1);
        let tcp_handle1 = sockets.add(tcp_socket1);
        let http_server1 = HttpState::new(tcp_handle1, HTTP_STATE1);

        let mdns_rx_buffer = UdpSocketBuffer::new(MDNS_RX_UDP_PACKET_METADATA as &'static mut [UdpPacketMetadata], MDNS_RX_BUFFER as &'static mut [u8]);
        let mdns_tx_buffer = UdpSocketBuffer::new(MDNS_TX_UDP_PACKET_METADATA as &'static mut [UdpPacketMetadata], MDNS_TX_BUFFER as &'static mut [u8]);
        let mdns_socket = UdpSocket::new(mdns_rx_buffer, mdns_tx_buffer);
        let mdns_handle = sockets.add(mdns_socket);
        sockets.get::<UdpSocket>(mdns_handle).bind(5353).unwrap();

        let icmp_rx_buffer = RawSocketBuffer::new(ICMP_RX_PACKET_METADATA as &'static mut [RawPacketMetadata], ICMP_RX_BUFFER as &'static mut [u8]);
        let icmp_tx_buffer = RawSocketBuffer::new(ICMP_TX_PACKET_METADATA as &'static mut [RawPacketMetadata], ICMP_TX_BUFFER as &'static mut [u8]);
        let icmp_socket = RawSocket::new(IpVersion::Ipv6, Icmpv6, icmp_rx_buffer, icmp_tx_buffer);
        let icmp_handle = sockets.add(icmp_socket);

        raw_recv::spawn_after(Milliseconds(3000u32), icmp_handle).unwrap();
        mdns_recv::spawn_after(Seconds(3u32), mdns_handle).unwrap();
        usb_poll::spawn().unwrap();

        (Shared {
            usb_dev,
            iface,
            sockets,
            // button,
            // rtc,
            led
        }, Local {
            http_servers: [http_server0, http_server1]
        }, init::Monotonics(
            dwt_systick
        ))
    }

    #[task(shared = [led])]
    fn set_led(ctx: set_led::Context, state: bool) {
        let set_led::SharedResources {mut led} = ctx.shared;

        led.lock(|led| {
            if state {
                led.set_low().unwrap();
            } else {
                led.set_high().unwrap();
            }
        });
    }

    #[task(priority = 2, shared = [sockets])]
    fn mdns_recv(ctx: mdns_recv::Context, mdns_handle: SocketHandle) {
        let mdns_recv::SharedResources {mut sockets} = ctx.shared;

        sockets.lock(|sockets| {
            let mut socket = sockets.get::<UdpSocket>(mdns_handle);

            let reply = [
                0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                4, 116, 101, 115, 116, 5, 108, 111, 99, 97, 108, 0,
                0x00, 28, 0x80, 0x01, 0x00, 0x00, 0x00, 0xff, 0x00, 0x10,
                0xfd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
            ];

            socket.send_slice(&reply, smoltcp::wire::IpEndpoint::new(IpAddress::v6(0xff02, 0, 0, 0, 0, 0, 0, 0xfb), 5353));
        });

        mdns_recv::spawn_after(Seconds(1u32), mdns_handle).unwrap();
    }

    #[task(priority = 2, shared = [sockets])]
    fn raw_recv(ctx: raw_recv::Context, raw_handle: SocketHandle) {
        let raw_recv::SharedResources {mut sockets} = ctx.shared;

        sockets.lock(|sockets| {
            let mut socket = sockets.get::<RawSocket>(raw_handle);

            let advert = Ndisc(RouterAdvert {
                hop_limit: 0,
                flags: NdiscRouterFlags::empty(),
                router_lifetime: Duration::from_secs(9000),
                reachable_time: Duration::from_secs(0),
                retrans_time: Duration::from_secs(0),
                lladdr: Some(EthernetAddress::from_bytes(&[2,0,0,0,0,1])),
                mtu: None,
                prefix_info: Some(NdiscPrefixInformation {
                    prefix_len: 64,
                    flags: NdiscPrefixInfoFlags::ADDRCONF | NdiscPrefixInfoFlags::ON_LINK,
                    valid_lifetime: Duration::from_secs(0xffffffff),
                    preferred_lifetime: Duration::from_secs(0xffffffff),
                    prefix: smoltcp::wire::Ipv6Address([0xfd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
                })
            });

            let ipv6_header = Ipv6Repr {
                src_addr: smoltcp::wire::Ipv6Address([0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xfe, 0, 0, 1]),
                dst_addr: smoltcp::wire::Ipv6Address::LINK_LOCAL_ALL_NODES,
                hop_limit: 255,
                next_header: Icmpv6,
                payload_len: advert.buffer_len()
            };

            if let Ok(buffer) = socket.send(ipv6_header.buffer_len() + advert.buffer_len()) {
                let mut packet = smoltcp::wire::Ipv6Packet::new_unchecked(buffer);
                ipv6_header.emit(&mut packet);
                let mut packet = smoltcp::wire::Icmpv6Packet::new_unchecked(packet.payload_mut());
                advert.emit(
                    &IpAddress::from(ipv6_header.src_addr),
                    &IpAddress::from(ipv6_header.dst_addr),
                    &mut packet,
                    &smoltcp::phy::ChecksumCapabilities::default()
                );
            }
        });

        raw_recv::spawn_after(Seconds(1u32), raw_handle).unwrap();
    }

    #[task(shared = [sockets, led], local = [http_servers])]
    fn http_step(ctx: http_step::Context) {
        let http_step::SharedResources { mut sockets, mut led } = ctx.shared;
        let http_step::LocalResources { http_servers } = ctx.local;

        sockets.lock(|sockets| {
            for http_server in http_servers {
                replace_with_or_abort(http_server, |http_server| {
                    match http_server {
                        HttpState::Init(x) => x.transition(sockets),
                        HttpState::Listen(x) => x.transition(sockets),
                        HttpState::Send(x) => x.transition(sockets),
                        HttpState::Receive(x) => x.transition(sockets, led.lock(|led| led.is_low().unwrap()))
                    }
                });
            };
        });
    }

    #[task(priority = 1, shared = [usb_dev, iface, sockets])]
    fn usb_poll(ctx: usb_poll::Context) {
        let usb_poll::SharedResources {usb_dev, mut iface, sockets} = ctx.shared;

        (usb_dev, &mut iface).lock(|usb_dev, iface| {
            usb_dev.poll(&mut [iface.device_mut().inner()])
        });

        (&mut iface, sockets).lock(|iface, sockets| {
            let instant = smoltcp::time::Instant::from_millis(
                monotonics::MyMono::now().duration_since_epoch().integer()/400_000_000
            );
            let _ = iface.poll(sockets, instant);
        });

        http_step::spawn().unwrap();
        usb_poll::spawn_after(Microseconds(1_000_000u32/48_000)).unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
        }
    }
}
