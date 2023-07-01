use heapless::Vec;
use smoltcp::socket::{SocketHandle, SocketSet, TcpSocket, TcpState};
use smoltcp::wire::IpAddress;

use crate::app::set_led;
use crate::response_builder::*;
use defmt::*;

#[derive(Debug)]
pub enum Method {
    Get,
    Post,
    Options,
    Unknown,
}

pub struct Init {
    handle: SocketHandle,
    receive_buffer: &'static mut Vec<u8, 1024>,
}

pub struct Listen {
    handle: SocketHandle,
    receive_buffer: &'static mut Vec<u8, 1024>,
}

pub struct Receive {
    handle: SocketHandle,
    receive_buffer: &'static mut Vec<u8, 1024>,
}

pub struct Send {
    handle: SocketHandle,
    receive_buffer: &'static mut Vec<u8, 1024>,
    body: &'static [u8],
}

pub enum State {
    Init(Init),
    Listen(Listen),
    Receive(Receive),
    Send(Send),
}

static INDEX_HTML: &[u8] = include_bytes!("index.html");
static APP_JS_GZ: &[u8] = include_bytes!("app.js.gz");
static SITE_CSS_GZ: &[u8] = include_bytes!("site.css.gz");

impl State {
    pub fn new(handle: SocketHandle, receive_buffer: &'static mut Vec<u8, 1024>) -> Self {
        State::Init(Init::new(handle, receive_buffer))
    }
}

impl Init {
    fn new(handle: SocketHandle, receive_buffer: &'static mut Vec<u8, 1024>) -> Self {
        Init {
            handle,
            receive_buffer,
        }
    }

    pub fn transition(self, sockets: &mut SocketSet) -> State {
        let Init {
            handle,
            receive_buffer,
        } = self;
        let mut socket = sockets.get::<TcpSocket>(handle);
        socket.listen((IpAddress::v6(0xfd00, 0, 0, 0, 0, 0, 0, 1), 8080));

        info!("Init -> Listen");
        flush();

        State::Listen(Listen {
            handle,
            receive_buffer,
        })
    }
}

impl Listen {
    pub fn transition(self, sockets: &mut SocketSet) -> State {
        let Listen {
            handle,
            receive_buffer,
        } = self;
        let mut socket = sockets.get::<TcpSocket>(handle);

        match socket.state() {
            TcpState::Closed => State::Init(Init {
                handle,
                receive_buffer,
            }),
            TcpState::CloseWait => {
                socket.close();
                State::Init(Init {
                    handle,
                    receive_buffer,
                })
            }
            TcpState::Established => {
                receive_buffer.clear();

                State::Receive(Receive {
                    handle,
                    receive_buffer,
                })
            }
            _ => State::Listen(Listen {
                handle,
                receive_buffer,
            }),
        }
    }
}

impl Receive {
    pub fn transition(self, sockets: &mut SocketSet, led: bool) -> State {
        let Receive {
            handle,
            receive_buffer,
        } = self;
        let mut socket = sockets.get::<TcpSocket>(handle);

        let mut buffer = [0u8; 256];

        let result = socket.recv(|data| {
            let mut headers = [httparse::EMPTY_HEADER; 16];
            let mut req = httparse::Request::new(&mut headers);

            let original_len = receive_buffer.len();

            let slice_len =
                core::cmp::min(data.len(), receive_buffer.capacity() - receive_buffer.len());
            receive_buffer
                .extend_from_slice(&data[..slice_len])
                .unwrap();

            match req.parse(receive_buffer) {
                Ok(httparse::Status::Complete(x)) => {
                    let ref request_path = req.path.unwrap();
                    let method = match req.method.unwrap() {
                        "GET" => Method::Get,
                        "POST" => Method::Post,
                        "OPTIONS" => Method::Options,
                        _ => Method::Unknown,
                    };
                    let path_length = request_path.len();
                    let content_length = headers
                        .iter()
                        .find_map(|&httparse::Header { name, value }| {
                            if name
                                .chars()
                                .zip("content-length".chars())
                                .all(|(x, y)| x.to_ascii_lowercase() == y)
                            {
                                core::str::from_utf8(value)
                                    .ok()
                                    .and_then(|x| str::parse::<usize>(x).ok())
                            } else {
                                None
                            }
                        })
                        .unwrap_or(0);

                    let (buffer_path, rest) = buffer.split_at_mut(path_length);
                    buffer_path.copy_from_slice(request_path.as_bytes());
                    rest[..content_length].copy_from_slice(&receive_buffer[x..][..content_length]);

                    receive_buffer.clear();

                    (
                        x + content_length - original_len,
                        Some((
                            method,
                            &buffer[..path_length],
                            &buffer[path_length..path_length + content_length],
                        )),
                    )
                }
                Ok(httparse::Status::Partial) => (data.len(), None),
                _ => (0, None),
            }
        });

        match result {
            Ok(Some((Method::Get, b"/index.html", _))) => {
                let body = INDEX_HTML;

                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(200)
                        .access_control_allow_origin("*")
                        .content_length(body.len())
                        .finalize();
                    (response.len(), ())
                });

                State::Send(Send {
                    handle,
                    receive_buffer,
                    body,
                })
            }
            Ok(Some((Method::Get, b"/js/app.js", _))) => {
                let body = APP_JS_GZ;

                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(200)
                        .access_control_allow_origin("*")
                        .content_length(body.len())
                        .content_encoding("gzip")
                        .finalize();
                    (response.len(), ())
                });

                State::Send(Send {
                    handle,
                    receive_buffer,
                    body,
                })
            }
            Ok(Some((Method::Get, b"/css/site.css", _))) => {
                let body = SITE_CSS_GZ;

                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(200)
                        .access_control_allow_origin("*")
                        .content_length(body.len())
                        .content_encoding("gzip")
                        .finalize();
                    (response.len(), ())
                });

                State::Send(Send {
                    handle,
                    receive_buffer,
                    body,
                })
            }
            Ok(Some((Method::Get, b"/led", _))) => {
                let body = if led { &b"TRUE"[..] } else { &b"FALSE"[..] };

                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(200)
                        .access_control_allow_origin("*")
                        .content_length(body.len())
                        .finalize();
                    (response.len(), ())
                });

                State::Send(Send {
                    handle,
                    receive_buffer,
                    body,
                })
            }
            Ok(Some((Method::Post, b"/led", body))) => {
                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(204)
                        .access_control_allow_origin("*")
                        .finalize();
                    (response.len(), ())
                });

                match body {
                    b"set=true" => set_led::spawn(true).unwrap(),
                    b"set=false" => set_led::spawn(false).unwrap(),
                    _ => {}
                }

                State::Receive(Receive {
                    handle,
                    receive_buffer,
                })
            }
            Ok(Some((Method::Options, _, _))) => {
                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(204)
                        .access_control_allow_origin("*")
                        .access_control_allow_headers("content-type")
                        .finalize();
                    (response.len(), ())
                });

                State::Receive(Receive {
                    handle,
                    receive_buffer,
                })
            }
            Ok(Some((_, _, _))) => {
                socket.send(|buf| {
                    let response = ResponseBuilder::new(buf)
                        .status(404)
                        .access_control_allow_origin("*")
                        .access_control_allow_headers("content-type")
                        .content_length(0)
                        .finalize();
                    (response.len(), ())
                });

                State::Receive(Receive {
                    handle,
                    receive_buffer,
                })
            }
            Ok(None) => State::Receive(Receive {
                handle,
                receive_buffer,
            }),
            Err(_) => State::Init(Init {
                handle,
                receive_buffer,
            }),
        }
    }
}

impl Send {
    pub fn transition(self, sockets: &mut SocketSet) -> State {
        let Send {
            handle,
            receive_buffer,
            body,
        } = self;
        let mut socket = sockets.get::<TcpSocket>(handle);

        match socket.send_slice(body) {
            Ok(bytes_sent) => {
                let new_body = &body[bytes_sent..];

                if new_body.len() == 0 {
                    State::Receive(Receive {
                        handle,
                        receive_buffer,
                    })
                } else {
                    State::Send(Send {
                        handle,
                        receive_buffer,
                        body: new_body,
                    })
                }
            }
            _ => State::Init(Init {
                handle,
                receive_buffer,
            }),
        }
    }
}
