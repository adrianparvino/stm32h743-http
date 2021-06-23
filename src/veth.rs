use smoltcp::Result;
use smoltcp::time::Instant;
use smoltcp::phy::{
    self,
    DeviceCapabilities,
    Device,
};
use usb_device::class_prelude::UsbBus;
use crate::cdc_ecm::CdcEcmClass;

pub struct Veth<'a, B: UsbBus> {
    phy: CdcEcmClass<'a, B>
}

impl<'a, B: UsbBus> Veth<'a, B> {
    pub fn new(phy: CdcEcmClass<'a, B>) -> Self {
        Veth { phy }
    }

    pub fn inner(&'_ mut self) -> &'_ mut CdcEcmClass<'a, B> {
        &mut self.phy
    }
}

pub struct VethTxToken<'a, B: UsbBus> {
    veth: &'a  Veth<'a, B>
}

impl<'a, 'rx, 'tx, B: UsbBus> phy::TxToken for VethTxToken<'a, B> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
    F: FnOnce(&mut [u8]) -> Result<R> {
        match self.veth.phy.maybe_tx(len, f) {
            Some(x) => x,
            _ => Err(smoltcp::Error::Dropped)
        }
    }
}

pub struct VethRxToken<'a, B: UsbBus> {
    veth: &'a Veth<'a, B>
}

impl<'a, 'rx, 'tx, B: UsbBus> phy::RxToken for VethRxToken<'a, B> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R>
    where
    F: FnOnce(&mut [u8]) -> Result<R> {
        match self.veth.phy.maybe_rx(f) {
            Some(x) => x,
            _ => Err(smoltcp::Error::Dropped)
        }
    }
}

impl<'a, 'b, B: UsbBus + 'a> Device<'a> for Veth<'b, B> {
    type RxToken = VethRxToken<'a, B>;
    type TxToken = VethTxToken<'a, B>;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1514;
        caps
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(VethTxToken {veth: self})
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        if self.phy.has_rx() {
            Some((VethRxToken {veth: self}, VethTxToken {veth: self}))
        } else {
            None
        }
    }
}
