use usb_device::class_prelude::*;
use usb_device::Result;

pub const USB_CLASS_CDC: u8 = 0x02;

const CDC_SUBCLASS_ECM: u8 = 0x06;

const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CS_INTERFACE: u8 = 0x24;
const CDC_TYPE_HEADER: u8 = 0x00;
const CDC_TYPE_ETHERNET_FUNCTION: u8 = 0x0f;
const CDC_TYPE_UNION: u8 = 0x06;

#[derive(Copy, Clone, Debug)]
enum RxState {
    Receiving(usize),
    Eof(usize)
}

#[derive(Copy, Clone, Debug)]
enum TxState {
    Transmitting(usize),
    Zlp,
    Bof
}

use RxState::*;
use TxState::*;

use core::cell::RefCell;
use core::cell::Cell;

pub struct CdcEcmClass<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    mac_address_index: StringIndex,
    link_initialized: bool,

    rx_buffer: RefCell<&'static mut [u8; 1514]>,
    rx_state: Cell<RxState>,

    tx_buffer: RefCell<&'static mut [u8; 1514]>,
    tx_state: Cell<TxState>
}

impl<'a, B: UsbBus> CdcEcmClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>,
               eth_rx_buffer: &'static mut [u8; 1514],
               eth_tx_buffer: &'static mut [u8; 1514]) -> CdcEcmClass<'a, B> {
        CdcEcmClass {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(64, 1),
            data_if: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
            mac_address_index: alloc.string(),
            link_initialized: false,

            rx_buffer: RefCell::new(eth_rx_buffer),
            rx_state: Cell::new(Receiving(0)),

            tx_buffer: RefCell::new(eth_tx_buffer),
            tx_state: Cell::new(Bof),
        }
    }

    pub fn set_link_state(&mut self, initialized: bool) {
        self.link_initialized = initialized;
    }

    pub fn maybe_rx<R, F>(&self, f: F) -> Option<R>
    where
    F: FnOnce(&mut [u8]) -> R {
        match self.rx_state.get() {
            Eof(offset) => {
                self.rx_state.set(Receiving(0));
                Some(f(&mut self.rx_buffer.borrow_mut()[..offset]))
            },
            Receiving(_) => None
        }
    }

    fn flush(&self) {
        match self.tx_state.get() {
            Transmitting(offset) => {
                let bytes_to_write = core::cmp::min(offset, 64);
                match self.write_ep.write(&mut self.tx_buffer.borrow_mut()[1514 - offset..][..bytes_to_write]) {
                    Ok(bytes_written) => {
                        assert!(bytes_to_write == bytes_written);

                        if offset < 64 {
                            self.tx_state.set(Bof);
                        } else if offset == 64 {
                            self.tx_state.set(Zlp);
                        } else {
                            self.tx_state.set(Transmitting(offset - bytes_written));
                        }
                    }
                    Err(_) => {}
                };
            },
            Zlp => {
                let _ = self.write_ep.write(&[]);
            }
            _ => {}
        }
    }

    pub fn maybe_tx<R, F>(&self, len: usize, f: F) -> Option<R>
    where
    F: FnOnce(&mut [u8]) -> R {
        match self.tx_state.get() {
            Bof => {
                let ret = Some(f(&mut self.tx_buffer.borrow_mut()[1514 - len..]));
                self.tx_state.set(Transmitting(len));
                self.flush();
                ret
            },
            _ => None
        }
    }

    pub fn has_rx(&self) -> bool {
        match self.rx_state.get() {
            Eof(_) => true,
            _ => false
        }
    }

    pub fn jack_in(self) -> crate::veth::Veth<'a, B> {
        self.comm_ep.write(&[
            0b10100001,
            0x00,
            1, 0,
            u8::from(self.data_if), 0,
            0, 0
        ]).unwrap();
        crate::veth::Veth::new(self)
    }
}

impl<B: UsbBus> UsbClass<B> for CdcEcmClass<'_, B> {
    fn get_string(&self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.mac_address_index {
            return Some(&"020000000000");
        }
        None
    }

    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.iad(self.comm_if, 2, USB_CLASS_CDC, CDC_SUBCLASS_ECM, 0)?;
        writer.interface(self.comm_if, USB_CLASS_CDC, CDC_SUBCLASS_ECM, 0)?;
        writer.write(CS_INTERFACE, &[CDC_TYPE_HEADER, 0x10, 0x01])?;
        writer.write(CS_INTERFACE, &[CDC_TYPE_UNION, self.comm_if.into(), self.data_if.into()])?;
        writer.write(CS_INTERFACE, &[
            CDC_TYPE_ETHERNET_FUNCTION,
            self.mac_address_index.into(),
            0, 0, 0, 0,
            0xea, 0x05, // 1514
            0, 0,
            0
        ])?;
        writer.endpoint(&self.comm_ep)?;
        writer.interface(self.data_if, USB_CLASS_CDC_DATA, 0, 0)?;
        writer.interface_alt(self.data_if, 1, USB_CLASS_CDC_DATA, CDC_SUBCLASS_ECM, 0, None)?;
        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;

        Ok(())
    }

    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        if interface == self.data_if {
            return Some(if self.link_initialized { 1 } else { 0 })
        }
        None
    }

    fn set_alt_setting(&mut self, interface: InterfaceNumber, alternative: u8) -> bool {
        if interface == self.data_if {
            self.set_link_state(alternative == 1);
            return true
        }
        return false
    }

    fn control_out(&mut self, xfer: ControlOut<'_, '_, '_, B>) {
        let request = xfer.request();
        if request.recipient == control::Recipient::Interface && request.index == u8::from(self.comm_if) as u16 {
            xfer.accept().ok();
        }
    }


    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.read_ep.address() {
            match (self.rx_state.get(), self.rx_buffer.borrow_mut()) {
                (Receiving(offset), mut buffer) if buffer[offset..].len() >= 64 => {
                    match self.read_ep.read(&mut buffer[offset..]) {
                        Ok(64) => {
                            self.rx_state.set(Receiving(offset + 64));
                        }
                        Ok(bytes_read) => {
                            self.rx_state.set(Eof(offset + bytes_read))
                        }
                        Err(_) => {
                        }
                    };
                }
                _ => {}
            }
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.write_ep.address() {
            self.flush()
        }
    }
}
