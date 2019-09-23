use super::register::{Readable, Register, Writable, R as GenR, W as GenW};

/// § 32.8.7.2
/// Address of the Data Buffer.
///
/// Offset: 0x00 & 0x10
/// Reset: 0xxxxxxxxx
/// Property: NA
pub type Addr = Register<u32, _Addr>;
impl Readable for Addr {}
impl Writable for Addr {}

pub type R = GenR<u32, Addr>;
pub type W = GenW<u32, Addr>;

pub struct _Addr;

impl R {
    pub fn addr(&self) -> AddrR {
        AddrR::new(self.bits)
    }
}

impl W {
    pub fn addr(&mut self) -> AddrW {
        AddrW { w: self }
    }
}

/// Data Pointer Address Value
///
/// These bits define the data pointer address as an absolute double
/// word address in RAM. The two least significant bits must be zero
/// to ensure the descriptor is 32-bit aligned.
pub type AddrR = GenR<u32, Addr>;

pub struct AddrW<'a> {
    w: &'a mut W,
}

impl<'a> AddrW<'a> {
    pub unsafe fn bits(self, bits: u32) -> &'a mut W {
        self.w.bits(bits)
    }
}
