use crate::rtc;
use starb::{Reader, RingBuffer, Writer};

use core::cell::UnsafeCell;
use core::fmt::{self, Write};
use embedded_hal::{digital::v2::OutputPin, serial};
use log::{Metadata, Record};
use trinket_m0::{
    gpio::{Pa6, Pa7, PfD},
    sercom::{Sercom0Pad2, Sercom0Pad3, UART0},
};

static mut UART0: usize = 0;

struct JoinedRingBuffer<'a> {
    lbr: Reader<'a, u8>,
    lbw: Writer<'a, u8>,
}

impl<'a> JoinedRingBuffer<'a> {
    const fn new(rb: &'a RingBuffer<u8>) -> Self {
        let (lbr, lbw) = rb.split();
        Self { lbr: lbr, lbw: lbw }
    }
}

impl fmt::Write for JoinedRingBuffer<'_> {
    fn write_str(&mut self, s: &str) -> Result<(), fmt::Error> {
        for b in s.bytes() {
            if let Err(_) = self.lbw.unshift(b) {
                // Ignore buffer full errors for logging.
                return Ok(());
            }
        }
        Ok(())
    }
}

static mut LB: RingBuffer<u8> = RingBuffer::<u8>::new();
static mut JRB: JoinedRingBuffer = unsafe { JoinedRingBuffer::new(&LB) };

// The UART isn't necessarily Sync, so wrap it in something that
// is. As long as flush() is only called from one thread, we're fine,
// but this is a guarantee that the logger module doesn't make.
pub struct WriteWrapper<W> {
    w: W,
}
impl<W> WriteWrapper<W>
where
    W: serial::Write<u8>,
{
    pub fn new(writer: W) -> Self {
        Self { w: writer }
    }
}
unsafe impl<W> Sync for WriteWrapper<W> {}

pub struct SerialLogger<W, L> {
    writer: UnsafeCell<WriteWrapper<W>>,
    led: UnsafeCell<L>,
}

impl<W, L> SerialLogger<W, L>
where
    W: serial::Write<u8>,
    L: OutputPin + Send + Sync,
{
    pub fn new(writer: WriteWrapper<W>, led: L) -> Self {
        // Stash this for unsafe usage in case there's an issue with
        // the rest of the logging.
        unsafe { UART0 = core::mem::transmute(&writer.w) };
        Self {
            writer: UnsafeCell::new(writer),
            led: UnsafeCell::new(led),
        }
    }
}
unsafe impl<W, L> Send for SerialLogger<W, L> {}
unsafe impl<W, L> Sync for SerialLogger<W, L> {}

impl<W, L> log::Log for SerialLogger<W, L>
where
    W: serial::Write<u8>,
    L: OutputPin + Send + Sync,
{
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let jrb = unsafe { &mut JRB };
        write!(
            jrb,
            "[{}] {} {} -- {}\r\n",
            rtc::millis(),
            record.level(),
            record.target(),
            record.args()
        )
        .ok();
    }

    fn flush(&self) {
        // Unsafe due to mutable static. We can only deal with the
        // tail position of the buffer here to keep things safe.
        let jrb = unsafe { &mut JRB };
        if jrb.lbr.is_empty() {
            return;
        }

        let led = unsafe { &mut (*self.led.get()) };
        let writer = unsafe { &mut (*self.writer.get()) };

        led.set_high().ok();
        while let Some(b) = jrb.lbr.shift() {
            nb::block!(writer.w.write(b)).ok();
        }
        led.set_low().ok();
    }
}

// Write to the UART right now, instead of putting it on a ring
// buffer. This function is a huge hack, and only useful for debugging
// either before the main loop starts or if the ring buffer is broken.
pub unsafe fn write_fmt_now(args: fmt::Arguments, nl: bool) {
    if UART0 == 0 {
        return;
    }
    let uart: &mut UART0<Sercom0Pad3<Pa7<PfD>>, Sercom0Pad2<Pa6<PfD>>, (), ()> =
        core::mem::transmute(UART0);
    fmt::write(uart, args).expect("writing fmt now to uart");
    if nl {
        uart.write_str("\r\n").expect("writing nl now to uart");
    }
}
