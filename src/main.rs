#![feature(maybe_uninit_ref)]
// #![deny(unsafe_code)]
#![deny(warnings)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![no_std]
#![no_main]

use rtic::app;

mod cdc_ecm;
mod veth;
mod response_builder;
mod request;

extern crate managed;

use defmt_rtt as _;
use panic_halt as _;

#[app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use rtic::Monotonic;
    use rtic::time::duration::Milliseconds;
    use stm32f4xx_hal::{
        gpio::{gpioa::PA0, gpioc::PC13, Input, Output, PullUp, PushPull},
        otg_fs::{UsbBus, USB, UsbBusType},
        prelude::*,
        rtc::Rtc,
    };

    use usb_device::prelude::*;
    use usb_device::class_prelude::UsbBusAllocator;

    use smoltcp::iface::{Neighbor, NeighborCache, InterfaceBuilder, Interface as EthernetInterface};
    use smoltcp::wire::{IpCidr, IpAddress, EthernetAddress};
    use smoltcp::socket::{SocketHandle, TcpState, TcpSocket, TcpSocketBuffer, SocketSet, SocketSetItem};

    use dwt_systick_monotonic::DwtSystick;

    use rtcc::Rtcc;

    use core::mem::MaybeUninit;

    // use crate::hd44780_i2c::de;
    use crate::cdc_ecm::*;
    use crate::veth::*;
    use crate::response_builder::*;
    use crate::request::*;

    use core_io::Write;

    #[resources]
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        iface: EthernetInterface<'static, Veth<'static, UsbBus<USB>>>,
        sockets: SocketSet<'static>,
        // rtc: Rtc,
        tcp_handle: SocketHandle,
        button: PA0<Input<PullUp>>,
        led: PC13<Output<PushPull>>
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<84_000_000>; // 84 MHz

    #[cfg(not(debug_assertions))]
    const INDEX_HTML: &[u8] = include_bytes!("index.html");
    #[cfg(not(debug_assertions))]
    const APP_JS: &[u8] = include_bytes!("app.js.gz");
    #[cfg(not(debug_assertions))]
    const SITE_CSS: &[u8] = include_bytes!("site.css");

    #[init]
    fn init(mut ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        static mut EP_MEMORY: [u32; 2048] = [0; 2048];
        static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBusType>> = MaybeUninit::uninit();
        static mut IP_ADDRS: MaybeUninit<[IpCidr; 1]> = MaybeUninit::uninit();
        static mut NEIGHBOR_CACHE: [Option<(IpAddress, Neighbor)>; 8] = [None; 8];
        static mut SOCKET_STORE: [Option<SocketSetItem<'static>>; 8] = [None, None, None, None, None, None, None, None];

        static mut ETH_RX_BUFFER: [u8; 1514] = [0u8; 1514];
        static mut ETH_TX_BUFFER: [u8; 1514] = [0u8; 1514];

        static mut TCP_RX_BUFFER: [u8; 1024] = [0u8; 1024];
        static mut TCP_TX_BUFFER: [u8; 1024] = [0u8; 1024];

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(25.mhz())
            .sysclk(84.mhz())
            .pclk1(24.mhz())
            .require_pll48clk()
            .freeze();

        // let rtc = Rtc::new(ctx.device.RTC, 0, 0, false, &mut ctx.device.PWR);

        let dwt_systick = MyMono::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, 84_000_000);

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

        let ip_addrs: &'static mut [IpCidr] = unsafe {
            IP_ADDRS.as_mut_ptr().write([IpCidr::new(IpAddress::v4(169, 254, 12, 34), 7)]);
            IP_ADDRS.assume_init_mut()
        };

        let neighbor_cache = NeighborCache::new(NEIGHBOR_CACHE as &'static mut [Option<(IpAddress, Neighbor)>] );

        let iface = InterfaceBuilder::new(veth)
            .ethernet_addr(EthernetAddress::from_bytes(&[2,0,0,0,0,1]))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(ip_addrs)
            .finalize();

        let mut sockets = SocketSet::new(SOCKET_STORE as &'static mut [Option<SocketSetItem<'static>>]);

        let tcp_rx_buffer = TcpSocketBuffer::new(TCP_RX_BUFFER as &'static mut [u8]);
        let tcp_tx_buffer = TcpSocketBuffer::new(TCP_TX_BUFFER as &'static mut [u8]);
        let tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

        let tcp_handle = sockets.add(tcp_socket);

        sockets.get::<TcpSocket>(tcp_handle).listen((IpAddress::v4(169, 254, 12, 34), 8080)).unwrap();

        handle_ethernet::spawn_after(Milliseconds(1u32)).unwrap();

        (init::LateResources {
            usb_dev,
            iface,
            sockets,
            tcp_handle,
            button,
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

    #[task(resources = [sockets, tcp_handle, led])]
    fn handle_ethernet(ctx: handle_ethernet::Context) {
        static mut RESPONSE: Option<&'static [u8]> = None;
        let handle_ethernet::Resources {mut sockets, mut tcp_handle, mut led} = ctx.resources;

        (&mut tcp_handle, &mut sockets).lock(|tcp_handle, sockets| {
            let mut socket = sockets.get::<TcpSocket>(*tcp_handle);

            match socket.state() {
                TcpState::Closed => {
                    let _ = socket.listen((IpAddress::v4(169, 254, 12, 34), 8080));
                    return;
                }

                TcpState::CloseWait => {
                    socket.close();
                    return;
                }

                TcpState::Established => {
                    let mut buffer = [0u8; 512];


                    match RESPONSE {
                        None => {
                            let result = socket.recv(|data| {
                                let mut headers = [httparse::EMPTY_HEADER; 16];
                                let mut req = httparse::Request::new(&mut headers);

                                match req.parse(data) {
                                    Ok(httparse::Status::Complete(x)) => {
                                        let ref request_path = req.path.unwrap();
                                        let method = match req.method.unwrap() {
                                            "GET" => Method::Get,
                                            "POST" => Method::Post,
                                            "OPTIONS" => Method::Options,
                                            _ => Method::Unknown
                                        };
                                        let path_length = request_path.len();
                                        let content_length = headers.iter().find_map(|&httparse::Header {name, value}| {
                                            if name.chars().zip("content-length".chars()).all(|(x, y)| x.to_ascii_lowercase() == y) {
                                                core::str::from_utf8(value)
                                                    .ok()
                                                    .and_then(|x| str::parse::<usize>(x).ok())
                                            } else {
                                                None
                                            }
                                        }).unwrap_or(0);

                                        write!(&mut buffer[..path_length], "{}", request_path).unwrap();
                                        (&mut buffer[path_length..]).write(&data[x..x + content_length]).unwrap();
                                        (x + content_length,
                                         Some((
                                             method,
                                             &buffer[..path_length],
                                             &buffer[path_length..path_length + content_length]
                                         )))
                                    },
                                    _ => {
                                        (0, None)
                                    }
                                }
                            }).unwrap();

                            match result {
                                #[cfg(not(debug_assertions))]
                                Some((Method::Get, b"/", _)) => {
                                    let response = INDEX_HTML;
                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(200)
                                            .access_control_allow_headers("content-type")
                                            .content_length(response.len())
                                            .access_control_allow_origin("*")
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();

                                    *RESPONSE = Some(response);
                                },
                                #[cfg(not(debug_assertions))]
                                Some((Method::Get, b"/js/app.js", _)) => {
                                    let response = APP_JS;
                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(200)
                                            .access_control_allow_headers("content-type")
                                            .content_length(response.len())
                                            .content_encoding("gzip")
                                            .access_control_allow_origin("*")
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();

                                    *RESPONSE = Some(response);
                                },
                                #[cfg(not(debug_assertions))]
                                Some((Method::Get, b"/css/site.css", _)) => {
                                    let response = SITE_CSS;
                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(200)
                                            .access_control_allow_headers("content-type")
                                            .content_length(response.len())
                                            .access_control_allow_origin("*")
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();

                                    *RESPONSE = Some(response);
                                },
                                Some((Method::Get, b"/led", _)) => {
                                    let response = if led.lock(|led| led.is_low().unwrap()) { &b"TRUE"[..] } else { &b"FALSE"[..] };

                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(200)
                                            .access_control_allow_origin("*")
                                            .content_length(response.len())
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();

                                    *RESPONSE = Some(response);
                                },
                                Some((Method::Post, b"/led", body)) => {
                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(204)
                                            .access_control_allow_origin("*")
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();

                                    match body {
                                        b"set=true" => set_led::spawn(true).unwrap(),
                                        b"set=false" => set_led::spawn(false).unwrap(),
                                        _ => {}
                                    }
                                },
                                Some((Method::Options, _, _)) => {
                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(204)
                                            .access_control_allow_origin("*")
                                            .access_control_allow_headers("content-type")
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();
                                },
                                Some((_, _, _)) => {
                                    socket.send(|buf| {
                                        let response = ResponseBuilder::new(buf)
                                            .status(404)
                                            .content_length(0)
                                            .finalize();
                                        (response.len(), ())
                                    }).unwrap();
                                },
                                _ => {}
                            }
                        },

                        Some(response) => {
                            match socket.send_slice(response) {
                                Ok(bytes_sent) => {
                                    let new_response = &response[bytes_sent..];

                                    if new_response.len() == 0 {
                                        *RESPONSE = None;
                                    } else {
                                        *RESPONSE = Some(new_response);
                                    }
                                }
                                _ => {}
                            };
                        }
                    }
                }
                _ => {}
            };
        });

        handle_ethernet::spawn_after(Milliseconds(1u32)).unwrap();
    }

    #[idle(resources = [usb_dev, sockets, iface])]
    fn idle(ctx: idle::Context) -> ! {
        let idle::Resources {mut usb_dev, mut iface, mut sockets} = ctx.resources;

        loop {
            (&mut usb_dev, &mut iface, &mut sockets).lock(|usb_dev, iface, sockets| {
                usb_dev.poll(&mut [iface.device_mut().inner()]);

                let instant = smoltcp::time::Instant::from_millis(
                    monotonics::MyMono::now().duration_since_epoch().integer()/84_000
                );
                let _ = iface.poll(sockets, instant);
            });
        }
    }
}
