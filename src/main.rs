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
use panic_halt as _;

#[app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use rtic::Monotonic;
    use rtic::time::duration::Seconds;
    use rtic::time::duration::Milliseconds;
    use rtic::time::duration::Microseconds;
    use stm32f4xx_hal::{
        gpio::{gpioa::PA0, gpioc::PC13, Input, Output, PullUp, PushPull},
        otg_fs::{UsbBus, USB, UsbBusType},
        prelude::*,
    };

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

    #[resources]
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        iface: EthernetInterface<'static, Veth<'static, UsbBus<USB>>>,
        sockets: SocketSet<'static>,
        // rtc: Rtc,
        button: PA0<Input<PullUp>>,
        led: PC13<Output<PushPull>>,

        #[lock_free]
        http_servers: [http_server::State; 2]
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<108_000_000>; // 108 MHz

    #[init]
    fn init(mut ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        static mut ETH_RX_BUFFER: [u8; 1514] = [0u8; 1514];
        static mut ETH_TX_BUFFER: [u8; 1514] = [0u8; 1514];

        static mut TCP_RX_BUFFER0: [u8; 128] = [0u8; 128];
        static mut TCP_TX_BUFFER0: [u8; 1024] = [0u8; 1024];
        static mut HTTP_STATE0: Vec<u8, 1024> = Vec::new();

        static mut TCP_RX_BUFFER1: [u8; 128] = [0u8; 128];
        static mut TCP_TX_BUFFER1: [u8; 1024] = [0u8; 1024];
        static mut HTTP_STATE1: Vec<u8, 1024> = Vec::new();

        static mut MDNS_RX_BUFFER: [u8; 512] = [0u8; 512];
        static mut MDNS_TX_BUFFER: [u8; 512] = [0u8; 512];
        static mut MDNS_RX_UDP_PACKET_METADATA: [UdpPacketMetadata; 4] = [UdpPacketMetadata::EMPTY; 4];
        static mut MDNS_TX_UDP_PACKET_METADATA: [UdpPacketMetadata; 4] = [UdpPacketMetadata::EMPTY; 4];

        static mut ICMP_RX_BUFFER: [u8; 512] = [0u8; 512];
        static mut ICMP_TX_BUFFER: [u8; 512] = [0u8; 512];
        static mut ICMP_RX_PACKET_METADATA: [RawPacketMetadata; 4] = [RawPacketMetadata::EMPTY; 4];
        static mut ICMP_TX_PACKET_METADATA: [RawPacketMetadata; 4] = [RawPacketMetadata::EMPTY; 4];

        static mut EP_MEMORY: [u32; 2048] = [0; 2048];
        static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBusType>> = MaybeUninit::uninit();
        static mut IP_ADDRS: MaybeUninit<[IpCidr; 2]> = MaybeUninit::uninit();
        static mut NEIGHBOR_CACHE: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
        static mut SOCKET_STORE: [Option<SocketSetItem<'static>>; 8] = [None, None, None, None, None, None, None, None];

        let ip_addrs: &'static mut [IpCidr] = unsafe {
            IP_ADDRS.as_mut_ptr().write([
                IpCidr::new(IpAddress::v6(0xfe80, 0, 0, 0, 0, 0xff, 0xfe00, 1), 64),
                IpCidr::new(IpAddress::v6(0xfd00, 0, 0, 0, 0, 0, 0, 1), 64)
            ]);
            IP_ADDRS.assume_init_mut()
        };

        let neighbor_cache = NeighborCache::new(NEIGHBOR_CACHE as &'static mut [Option<(IpAddress, Neighbor)>] );

        let rcc = ctx.device.RCC.constrain();
        let clocks = unsafe {
            rcc.cfgr
                .use_hse(25.mhz())
                .sysclk(108.mhz())
                .pclk1(24.mhz())
                .require_pll48clk()
                .freeze_unchecked()
        };

        let dwt_systick = MyMono::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, 108_000_000);

        let gpioa = ctx.device.GPIOA.split();
        let button = gpioa.pa0.into_pull_up_input();
        let pin_dp = gpioa.pa12.into_alternate_af10();
        let pin_dm = gpioa.pa11.into_alternate_af10();

        let gpioc = ctx.device.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output();

        let usb = USB {
            usb_global: ctx.device.OTG_FS_GLOBAL,
            usb_device: ctx.device.OTG_FS_DEVICE,
            usb_pwrclk: ctx.device.OTG_FS_PWRCLK,
            hclk: clocks.hclk(),
            pin_dm,
            pin_dp
        };

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

        raw_recv::spawn_after(Milliseconds(4500u32), icmp_handle).unwrap();
        mdns_recv::spawn_after(Seconds(5u32), mdns_handle).unwrap();
        usb_poll::spawn().unwrap();

        (init::LateResources {
            usb_dev,
            iface,
            sockets,
            button,
            http_servers: [http_server0, http_server1],
            // rtc,
            led
        }, init::Monotonics(dwt_systick))
    }

    #[task(resources = [led])]
    fn set_led(ctx: set_led::Context, state: bool) {
        let set_led::Resources {mut led} = ctx.resources;

        led.lock(|led| {
            if state {
                led.set_low().unwrap();
            } else {
                led.set_high().unwrap();
            }
        });
    }

    #[task(priority = 2, resources = [sockets])]
    fn mdns_recv(ctx: mdns_recv::Context, mdns_handle: SocketHandle) {
        let mdns_recv::Resources {mut sockets} = ctx.resources;

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

    #[task(priority = 2, resources = [sockets])]
    fn raw_recv(ctx: raw_recv::Context, raw_handle: SocketHandle) {
        let raw_recv::Resources {mut sockets} = ctx.resources;

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

    #[task(resources = [sockets, led, http_servers])]
    fn http_step(ctx: http_step::Context) {
        let http_step::Resources {mut sockets, mut led, http_servers} = ctx.resources;

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

    #[task(priority = 1, resources = [usb_dev, iface, sockets])]
    fn usb_poll(ctx: usb_poll::Context) {
        let usb_poll::Resources {usb_dev, mut iface, sockets} = ctx.resources;

        (usb_dev, &mut iface).lock(|usb_dev, iface| {
            usb_dev.poll(&mut [iface.device_mut().inner()])
        });

        (&mut iface, sockets).lock(|iface, sockets| {
            let instant = smoltcp::time::Instant::from_millis(
                *monotonics::MyMono::now().duration_since_epoch().integer()/84_000
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
