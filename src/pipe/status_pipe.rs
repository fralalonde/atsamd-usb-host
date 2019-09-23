use super::register::{Readable, Register, Writable, R as GenR, W as GenW};

/// Host Status Pipe.
///
/// Offset: 0x0e & 0x1e
/// Reset: 0xxxxxx
/// Property: PAC Write-Protection, Write-Synchronized, Read-Synchronized

pub type StatusPipe = Register<u8, _StatusPipe>;
impl Readable for StatusPipe {}
impl Writable for StatusPipe {}

pub type R = GenR<u8, StatusPipe>;
pub type W = GenW<u8, StatusPipe>;

pub struct _StatusPipe;
impl R {
    pub fn ercnt(&self) -> ErCntR {
        let bits = {
            const POS: u8 = 5;
            const MASK: u8 = 0x7;
            ((self.bits >> POS) & MASK) as u8
        };

        ErCntR(bits)
    }

    pub fn crc16er(&self) -> CRC16ErR {
        let bits = {
            const POS: u8 = 4;
            const MASK: u8 = 1;
            ((self.bits >> POS) & MASK) == 1
        };

        CRC16ErR(bits)
    }

    pub fn touter(&self) -> TOutErrR {
        let bits = {
            const POS: u8 = 3;
            const MASK: u8 = 1;

            ((self.bits >> POS) & MASK) == 1
        };

        TOutErrR(bits)
    }

    pub fn pider(&self) -> PIDErR {
        let bits = {
            const POS: u8 = 2;
            const MASK: u8 = 1;

            ((self.bits >> POS) & MASK) == 1
        };

        PIDErR(bits)
    }

    pub fn dapider(&self) -> DaPIDErR {
        let bits = {
            const POS: u8 = 1;
            const MASK: u8 = 1;

            ((self.bits >> POS) & MASK) == 1
        };

        DaPIDErR(bits)
    }

    pub fn dtgler(&self) -> DTglErR {
        let bits = {
            const POS: u8 = 0;
            const MASK: u8 = 1;

            ((self.bits >> POS) & MASK) == 1
        };

        DTglErR(bits)
    }
}

/// Pipe Error Counter
///
/// The number of errors detected on the pipe.
pub struct ErCntR(u8);
impl ErCntR {
    pub fn bits(&self) -> u8 {
        self.0
    }
}

/// CRC16 ERROR
///
/// This bit defines the CRC16 Error Status.
///
/// This bit is set when a CRC 16 error has been detected during a IN
/// transactions.
pub struct CRC16ErR(bool);
impl CRC16ErR {
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

/// TIME OUT ERROR
///
/// This bit defines the Time Out Error Status.
///
/// This bit is set when a Time Out error has been detected during a
/// USB transaction.
pub struct TOutErrR(bool);
impl TOutErrR {
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

/// PID ERROR
///
/// This bit defines the PID Error Status.
///
/// This bit is set when a PID error has been detected during a USB
/// transaction.
pub struct PIDErR(bool);
impl PIDErR {
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

/// Data PID ERROR
///
/// This bit defines the PID Error Status.
///
/// This bit is set when a Data PID error has been detected during a
/// USB transaction.
pub struct DaPIDErR(bool);
impl DaPIDErR {
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

/// Data Toggle Error
///
/// This bit defines the Data Toggle Error Status.
///
/// This bit is set when a Data Toggle Error has been detected.
pub struct DTglErR(bool);
impl DTglErR {
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
    pub fn ercnt(&mut self) -> ErCntW {
        ErCntW { w: self }
    }

    pub fn crc16er(&mut self) -> CRC16ErW {
        CRC16ErW { w: self }
    }

    pub fn touter(&mut self) -> TOutErW {
        TOutErW { w: self }
    }

    pub fn pider(&mut self) -> PIDErW {
        PIDErW { w: self }
    }

    pub fn dapider(&mut self) -> DaPIDErW {
        DaPIDErW { w: self }
    }

    pub fn dtgler(&mut self) -> DTglErW {
        DTglErW { w: self }
    }
}

/// Pipe Error Counter
///
/// The number of errors detected on the pipe.
pub struct ErCntW<'a> {
    w: &'a mut W,
}
impl<'a> ErCntW<'a> {
    pub unsafe fn bits(self, v: u8) -> &'a mut W {
        const POS: u8 = 5;
        const MASK: u8 = 0x7;
        self.w.bits &= !((MASK as u8) << POS);
        self.w.bits |= ((v & MASK) as u8) << POS;
        self.w
    }

    pub fn set_count(self, v: u8) -> &'a mut W {
        unsafe { self.bits(v) }
    }
}

/// CRC16 ERROR
///
/// This bit defines the CRC16 Error Status.
///
/// This bit is set when a CRC 16 error has been detected during a IN
/// transactions.
pub struct CRC16ErW<'a> {
    w: &'a mut W,
}
impl<'a> CRC16ErW<'a> {
    pub fn bit(self, v: bool) -> &'a mut W {
        const POS: u8 = 4;
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

/// TIME OUT ERROR
///
/// This bit defines the Time Out Error Status.
///
/// This bit is set when a Time Out error has been detected during a
/// USB transaction.
pub struct TOutErW<'a> {
    w: &'a mut W,
}
impl<'a> TOutErW<'a> {
    pub fn bit(self, v: bool) -> &'a mut W {
        const POS: u8 = 3;
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

/// PID ERROR
///
/// This bit defines the PID Error Status.
///
/// This bit is set when a PID error has been detected during a USB
/// transaction.
pub struct PIDErW<'a> {
    w: &'a mut W,
}
impl<'a> PIDErW<'a> {
    pub fn bit(self, v: bool) -> &'a mut W {
        const POS: u8 = 2;
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

/// Data PID ERROR
///
/// This bit defines the PID Error Status.
///
/// This bit is set when a Data PID error has been detected during a
/// USB transaction.
pub struct DaPIDErW<'a> {
    w: &'a mut W,
}
impl<'a> DaPIDErW<'a> {
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

/// Data Toggle Error
///
/// This bit defines the Data Toggle Error Status.
///
/// This bit is set when a Data Toggle Error has been detected.
pub struct DTglErW<'a> {
    w: &'a mut W,
}
impl<'a> DTglErW<'a> {
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
