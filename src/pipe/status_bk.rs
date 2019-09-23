use super::register::{Readable, Register, Writable, R as GenR, W as GenW};

/// §32.8.7.5
/// Host Status Bank.
///
/// Offset: 0x0a & 0x1a
/// Reset: 0xxxxxxx
/// Property: NA

pub type StatusBk = Register<u8, _StatusBk>;
impl Readable for StatusBk {}
impl Writable for StatusBk {}

pub type R = GenR<u8, StatusBk>;
pub type W = GenW<u8, StatusBk>;

pub struct _StatusBk;

impl R {
    pub fn errorflow(&self) -> ErrorFlowR {
        let bits = {
            const POS: u8 = 1;
            const MASK: u8 = 1;
            ((self.bits >> POS) & MASK) == 1
        };

        ErrorFlowR(bits)
    }

    pub fn crcerr(&self) -> CRCErrR {
        let bits = {
            const POS: u8 = 0;
            const MASK: u8 = 1;
            ((self.bits >> POS) & MASK) == 1
        };

        CRCErrR(bits)
    }
}

/// Error Flow Status
///
/// This bit defines the Error Flow Status.
///
/// This bit is set when a Error Flow has been detected during
/// transfer from/towards this bank.
///
/// For IN transfer, a NAK handshake has been received. For OUT
/// transfer, a NAK handshake has been received. For Isochronous IN
/// transfer, an overrun condition has occurred. For Isochronous OUT
/// transfer, an underflow condition has occurred.
pub struct ErrorFlowR(bool);
impl ErrorFlowR {
    pub fn bit(&self) -> bool {
        self.0
    }

    pub fn bit_is_set(&self) -> bool {
        self.bit()
    }

    pub fn bit_is_clear(&self) -> bool {
        !self.bit_is_set()
    }
}

/// CRC Error
///
/// This bit defines the CRC Error Status.
///
/// This bit is set when a CRC error has been detected in an
/// isochronous IN endpoint bank.
pub struct CRCErrR(bool);
impl CRCErrR {
    pub fn bit(&self) -> bool {
        self.0
    }

    pub fn bit_is_set(&self) -> bool {
        self.bit()
    }

    pub fn bit_is_clear(&self) -> bool {
        !self.bit_is_set()
    }
}

impl W {
    pub fn errorflow(&mut self) -> ErrorFlowW {
        ErrorFlowW { w: self }
    }

    pub fn crcerr(&mut self) -> CRCErrW {
        CRCErrW { w: self }
    }
}

pub struct ErrorFlowW<'a> {
    w: &'a mut W,
}
impl<'a> ErrorFlowW<'a> {
    pub fn bit(self, v: bool) -> &'a mut W {
        const POS: u8 = 1;
        const MASK: bool = true;
        self.w.bits &= !((MASK as u8) << POS);
        self.w.bits |= ((v & MASK) as u8) << POS;
        self.w
    }

    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }

    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
}

pub struct CRCErrW<'a> {
    w: &'a mut W,
}
impl<'a> CRCErrW<'a> {
    pub fn bit(self, v: bool) -> &'a mut W {
        const POS: u8 = 0;
        const MASK: bool = true;
        self.w.bits &= !((MASK as u8) << POS);
        self.w.bits |= ((v & MASK) as u8) << POS;
        self.w
    }

    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }

    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
}
