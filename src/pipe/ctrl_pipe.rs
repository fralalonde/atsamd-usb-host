use super::register::{Readable, Register, Writable, R as GenR, W as GenW};

/// Host Control Pipe.
///
/// Offset: 0x0c
/// Reset: 0xXXXX
/// Property: PAC Write-Protection, Write-Synchronized, Read-Synchronized
pub type CtrlPipe = Register<u16, _CtrlPipe>;
impl Readable for CtrlPipe {}
impl Writable for CtrlPipe {}

pub type R = GenR<u16, CtrlPipe>;
pub type W = GenW<u16, CtrlPipe>;

pub struct _CtrlPipe;

impl R {
    pub fn permax(&self) -> PErMaxR {
        let bits = {
            const POS: u8 = 12;
            const MASK: u16 = 0xf;
            ((self.bits >> POS) & MASK) as u8
        };

        PErMaxR(bits)
    }

    pub fn pepnum(&self) -> PEpNumR {
        let bits = {
            const POS: u8 = 8;
            const MASK: u16 = 0xf;
            ((self.bits >> POS) & MASK) as u8
        };

        PEpNumR(bits)
    }

    pub fn pdaddr(&self) -> PDAddrR {
        let bits = {
            const POS: u8 = 0;
            const MASK: u16 = 0x3f;
            ((self.bits >> POS) & MASK) as u8
        };

        PDAddrR(bits)
    }
}

/// Pipe Error Max Number
///
/// These bits define the maximum number of error for this Pipe before
/// freezing the pipe automatically.
pub struct PErMaxR(u8);
impl PErMaxR {
    pub fn max(&self) -> u8 {
        self.0
    }
}

/// Pipe EndPoint Number
///
/// These bits define the number of endpoint for this Pipe.
pub struct PEpNumR(u8);
impl PEpNumR {
    pub fn epnum(&self) -> u8 {
        self.0
    }
}

/// Pipe Device Address
///
/// These bits define the Device Address for this pipe.
pub struct PDAddrR(u8);
impl PDAddrR {
    pub fn addr(&self) -> u8 {
        self.0
    }
}

impl W {
    pub fn permax(&mut self) -> PErMaxW {
        PErMaxW { w: self }
    }

    pub fn pepnum(&mut self) -> PEpNumW {
        PEpNumW { w: self }
    }

    pub fn pdaddr(&mut self) -> PDAddrW {
        PDAddrW { w: self }
    }
}

pub struct PErMaxW<'a> {
    w: &'a mut W,
}
impl<'a> PErMaxW<'a> {
    pub unsafe fn bits(self, v: u8) -> &'a mut W {
        const POS: u8 = 12;
        const MASK: u8 = 0xf;
        self.w.bits &= !(u16::from(MASK) << POS);
        self.w.bits |= u16::from(v & MASK) << POS;
        self.w
    }

    pub fn set_max(self, v: u8) -> &'a mut W {
        unsafe { self.bits(v) }
    }
}

pub struct PEpNumW<'a> {
    w: &'a mut W,
}
impl<'a> PEpNumW<'a> {
    pub unsafe fn bits(self, v: u8) -> &'a mut W {
        const POS: u8 = 8;
        const MASK: u8 = 0xf;
        self.w.bits &= !(u16::from(MASK) << POS);
        self.w.bits |= u16::from(v & MASK) << POS;
        self.w
    }

    pub fn set_epnum(self, v: u8) -> &'a mut W {
        unsafe { self.bits(v) }
    }
}

pub struct PDAddrW<'a> {
    w: &'a mut W,
}
impl<'a> PDAddrW<'a> {
    pub unsafe fn bits(self, v: u8) -> &'a mut W {
        const POS: u8 = 0;
        const MASK: u8 = 0x3f;
        self.w.bits &= !(u16::from(MASK) << POS);
        self.w.bits |= u16::from(v & MASK) << POS;
        self.w
    }

    pub fn set_addr(self, v: u8) -> &'a mut W {
        unsafe { self.bits(v) }
    }
}
