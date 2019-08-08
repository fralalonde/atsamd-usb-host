//! USB Host driver implementation for SAMD* series chips.

#![no_std]
mod pipe;

use pipe::{PipeErr, PipeTable};

use usb_host::{
    DescriptorType, DeviceDescriptor, Direction, Driver, DriverError, Endpoint, RequestCode,
    RequestDirection, RequestKind, RequestRecipient, RequestType, TransferError, TransferType,
    USBHost, WValue,
};

use atsamd_hal::{
    calibration::{usb_transn_cal, usb_transp_cal, usb_trim_cal},
    clock::{ClockGenId, ClockSource, GenericClockController},
    gpio::{self, Floating, Input, OpenDrain, Output},
    target_device::{PM, USB},
};
use embedded_hal::digital::v2::OutputPin;
use log::{debug, error, trace, warn};
use starb::{Reader, RingBuffer, Writer};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    Error,
    Detached,
    Attached,
}
type Events = RingBuffer<Event>;
type EventReader = Reader<'static, Event>;
type EventWriter = Writer<'static, Event>;

const NAK_LIMIT: usize = 15;

// Must be at least 100ms. cf §9.1.2 of USB 2.0.
const SETTLE_DELAY: usize = 205; // Delay in sec/1024

// Ring buffer for sharing events from interrupt context.
static mut EVENTS: Events = Events::new(Event::Error);
// FIXME: this is just for testing. The enum needs to be
// thread-safe if this is the way we're going.
static mut LATEST_EVENT: Event = Event::Detached;

#[derive(Clone, Copy, Debug, PartialEq)]
enum DetachedState {
    Initialize,
    WaitForDevice,
    Illegal,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum AttachedState {
    WaitForSettle(usize),
    WaitResetComplete,
    WaitSOF(usize),
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum SteadyState {
    Configuring,
    Running,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum TaskState {
    Detached(DetachedState),
    Attached(AttachedState),
    Steady(SteadyState),
}

use core::mem::{self, MaybeUninit};
use core::ptr;

const MAX_DEVICES: usize = 4;
struct DeviceTable {
    tbl: [Option<Device>; MAX_DEVICES],
}
impl DeviceTable {
    fn new() -> Self {
        let tbl = {
            let mut devs: [MaybeUninit<Option<Device>>; MAX_DEVICES] =
                unsafe { MaybeUninit::uninit().assume_init() };
            for d in &mut devs[..] {
                unsafe { ptr::write(d.as_mut_ptr(), None) }
            }
            unsafe { mem::transmute(devs) }
        };

        Self { tbl: tbl }
    }

    /// Allocate a device with the next available address.
    // TODO: get rid of the millis argument somehow, but the device
    // does need a way of tracking time for Settle reasons.
    fn next(&mut self) -> Option<&mut Device> {
        for i in 1..self.tbl.len() {
            if self.tbl[i].is_none() {
                let d = Device { addr: i as u8 };
                self.tbl[i] = Some(d);
                return self.tbl[i].as_mut();
            }
        }
        None
    }

    /// Remove the device at address `addr`.
    fn remove(&mut self, addr: u8) -> Option<Device> {
        let v = core::mem::replace(&mut self.tbl[addr as usize], None);
        v
    }
}

struct Device {
    addr: u8,
}

pub struct SAMDHost<'a, F> {
    usb: USB,

    events: EventReader,
    task_state: TaskState,

    // Need chunk of RAM for USB pipes, which gets used with DESCADD
    // register.
    pipe_table: PipeTable,

    devices: DeviceTable,

    // need sof 1kHz pad?
    _sof_pad: gpio::Pa23<gpio::PfG>,
    _dm_pad: gpio::Pa24<gpio::PfG>,
    _dp_pad: gpio::Pa25<gpio::PfG>,
    host_enable_pin: Option<gpio::Pa28<Output<OpenDrain>>>,

    // To get current milliseconds.
    millis: &'a F,
}

impl<'a, F> SAMDHost<'a, F>
where
    F: Fn() -> usize,
{
    pub fn new(
        usb: USB,
        sof_pin: gpio::Pa23<Input<Floating>>,
        dm_pin: gpio::Pa24<Input<Floating>>,
        dp_pin: gpio::Pa25<Input<Floating>>,
        host_enable_pin: Option<gpio::Pa28<Input<Floating>>>,
        port: &mut gpio::Port,
        clocks: &mut GenericClockController,
        pm: &mut PM,
        millis: &'a F,
    ) -> (Self, impl FnMut()) {
        let (eventr, mut eventw) = unsafe { EVENTS.split() };

        let mut rc = Self {
            usb: usb,

            events: eventr,
            task_state: TaskState::Detached(DetachedState::Initialize),

            pipe_table: PipeTable::new(),

            devices: DeviceTable::new(),

            _sof_pad: sof_pin.into_function_g(port),
            _dm_pad: dm_pin.into_function_g(port),
            _dp_pad: dp_pin.into_function_g(port),
            host_enable_pin: None,

            millis: millis,
        };

        if let Some(he_pin) = host_enable_pin {
            rc.host_enable_pin = Some(he_pin.into_open_drain_output(port));
        }

        pm.apbbmask.modify(|_, w| w.usb_().set_bit());

        // Set up USB clock from 48MHz source on generic clock 6.
        clocks.configure_gclk_divider_and_source(ClockGenId::GCLK6, 1, ClockSource::DFLL48M, false);
        let gclk6 = clocks
            .get_gclk(ClockGenId::GCLK6)
            .expect("Could not get clock 6");
        clocks.usb(&gclk6);

        let usbp = &rc.usb as *const _ as usize;
        (rc, move || handler(usbp, &mut eventw))
    }

    pub fn reset_periph(&mut self) {
        debug!("resetting usb");
        // Reset the USB peripheral and wait for sync.
        self.usb.host().ctrla.write(|w| w.swrst().set_bit());
        while self.usb.host().syncbusy.read().swrst().bit_is_set() {}

        // Specify host mode.
        self.usb.host().ctrla.modify(|_, w| w.mode().host());

        // Unsafe due to use of raw bits method.
        unsafe {
            self.usb.host().padcal.write(|w| {
                w.transn().bits(usb_transn_cal());
                w.transp().bits(usb_transp_cal());
                w.trim().bits(usb_trim_cal())
            });
        }

        // Use normal, which is 0 and apparently means low-and-full capable
        self.usb.host().ctrlb.modify(|_, w| w.spdconf().normal());
        // According to docs, 1,2,3 are reserved, but .fs returns 3
        //self.usb.host().ctrlb.modify(|_, w| w.spdconf().fs());

        self.usb.host().ctrla.modify(|_, w| w.runstdby().set_bit()); // keep usb clock running in standby.

        // Set address of USB SRAM.
        // Unsafe due to use of raw bits method.
        unsafe {
            self.usb
                .host()
                .descadd
                .write(|w| w.bits(&self.pipe_table as *const _ as u32));
        }

        if let Some(he_pin) = &mut self.host_enable_pin {
            he_pin.set_high().expect("turning on usb host enable pin");
        }

        self.usb.host().intenset.write(|w| {
            w.wakeup().set_bit();
            w.dconn().set_bit();
            w.ddisc().set_bit()
        });

        self.usb.host().ctrla.modify(|_, w| w.enable().set_bit());
        while self.usb.host().syncbusy.read().enable().bit_is_set() {}

        // Set VBUS OK to allow host operation.
        self.usb.host().ctrlb.modify(|_, w| w.vbusok().set_bit());
        debug!("...done");
    }

    pub fn task(&mut self, drivers: &mut [&mut dyn Driver]) {
        static mut LAST_EVENT: Event = Event::Error;
        unsafe {
            if LAST_EVENT != LATEST_EVENT {
                trace!("new event: {:?}", LATEST_EVENT);
            }
        }

        static mut LAST_TASK_STATE: TaskState = TaskState::Detached(DetachedState::Illegal);
        self.task_state = match unsafe { LATEST_EVENT } {
            Event::Error => TaskState::Detached(DetachedState::Illegal),
            Event::Detached => {
                if let TaskState::Detached(_) = self.task_state {
                    self.task_state
                } else {
                    TaskState::Detached(DetachedState::Initialize)
                }
            }
            Event::Attached => {
                if let TaskState::Detached(_) = self.task_state {
                    TaskState::Attached(AttachedState::WaitForSettle(
                        (self.millis)() + SETTLE_DELAY,
                    ))
                } else {
                    self.task_state
                }
            }
        };

        static mut LAST_CBITS: u16 = 0;
        static mut LAST_FLAGS: u16 = 0;
        let cbits = self.usb.host().ctrlb.read().bits();
        let bits = self.usb.host().intflag.read().bits();
        unsafe {
            if LAST_CBITS != cbits || LAST_FLAGS != bits || LAST_TASK_STATE != self.task_state {
                trace!(
                    "cb: {:x}, f: {:x} changing state {:?} -> {:?}",
                    cbits,
                    bits,
                    LAST_TASK_STATE,
                    self.task_state,
                );
            }
            LAST_CBITS = cbits;
            LAST_FLAGS = bits;
            LAST_TASK_STATE = self.task_state
        };

        if let Some(_event) = self.events.shift() {
            // trace!("Found event: {:?}", event);
            // self.task_state = match event {
            //     Event::None => TaskState::Detached(DetachedState::Illegal),
            //     Event::Detached => {
            //         if let TaskState::Detached(_) = self.task_state {
            //             self.task_state
            //         } else {
            //             TaskState::Detached(DetachedState::Initialize)
            //         }
            //     }
            //     Event::Attached => {
            //         if let TaskState::Detached(_) = self.task_state {
            //             self.delay = self.millis() + SETTLE_DELAY;
            //             TaskState::Attached(AttachedState::WaitForSettle)
            //         } else {
            //             self.task_state
            //         }
            //     }
            // };
        }

        self.fsm(drivers);

        unsafe {
            LAST_EVENT = LATEST_EVENT;
        }
    }
    fn fsm(&mut self, drivers: &mut [&mut dyn Driver]) {
        // respond to events from interrupt.
        match self.task_state {
            TaskState::Detached(s) => self.detached_fsm(s),
            TaskState::Attached(s) => self.attached_fsm(s),
            TaskState::Steady(s) => self.steady_fsm(s, drivers),
        };
    }

    fn detached_fsm(&mut self, s: DetachedState) {
        match s {
            DetachedState::Initialize => {
                self.reset_periph();
                // TODO: Free resources.

                self.task_state = TaskState::Detached(DetachedState::WaitForDevice);
            }

            // Do nothing state. Just wait for an interrupt to come in
            // saying we have a device attached.
            DetachedState::WaitForDevice => {}

            // TODO: should probably reset everything if we end up here somehow.
            DetachedState::Illegal => {}
        }
    }

    fn attached_fsm(&mut self, s: AttachedState) {
        match s {
            AttachedState::WaitForSettle(until) => {
                if (self.millis)() >= until {
                    self.usb.host().ctrlb.modify(|_, w| w.busreset().set_bit());
                    self.task_state = TaskState::Attached(AttachedState::WaitResetComplete);
                }
            }

            AttachedState::WaitResetComplete => {
                if self.usb.host().intflag.read().rst().bit_is_set() {
                    trace!("reset was sent");
                    self.usb.host().intflag.write(|w| w.rst().set_bit());

                    // Seems unneccesary, since SOFE will be set
                    // immediately after reset according to §32.6.3.3.
                    self.usb.host().ctrlb.modify(|_, w| w.sofe().set_bit());
                    // USB spec requires 20ms of SOF after bus reset.
                    self.task_state =
                        TaskState::Attached(AttachedState::WaitSOF((self.millis)() + 20));
                }
            }

            AttachedState::WaitSOF(until) => {
                if self.usb.host().intflag.read().hsof().bit_is_set() {
                    self.usb.host().intflag.write(|w| w.hsof().set_bit());
                    if (self.millis)() >= until {
                        self.task_state = TaskState::Steady(SteadyState::Configuring);
                    }
                }
            }
        }
    }

    fn steady_fsm(&mut self, s: SteadyState, drivers: &mut [&mut dyn Driver]) {
        match s {
            SteadyState::Configuring => {
                self.task_state = match self.configure_dev(drivers) {
                    Ok(_) => TaskState::Steady(SteadyState::Running),
                    Err(e) => {
                        warn!("Enumeration error: {:?}", e);
                        TaskState::Steady(SteadyState::Error)
                    }
                }
            }

            SteadyState::Running => {
                for d in &mut drivers[..] {
                    if let Err(e) = d.tick((self.millis)(), self) {
                        warn!("running driver {:?}: {:?}", d, e);
                        if let DriverError::Permanent(a, _) = e {
                            self.devices.remove(a);
                        }
                    }
                }
            }

            SteadyState::Error => {}
        }
    }

    fn configure_dev(&mut self, drivers: &mut [&mut dyn Driver]) -> Result<(), TransferError> {
        let none: Option<&mut [u8]> = None;
        let mut a0ep0 = Addr0EP0 {
            in_toggle: true,
            out_toggle: true,
        };
        let mut dev_desc: DeviceDescriptor =
            unsafe { MaybeUninit::<DeviceDescriptor>::uninit().assume_init() };
        self.control_transfer(
            &mut a0ep0,
            RequestType::from((
                RequestDirection::DeviceToHost,
                RequestKind::Standard,
                RequestRecipient::Device,
            )),
            RequestCode::GetDescriptor,
            WValue::from((0, DescriptorType::Device as u8)),
            0,
            Some(unsafe { to_slice_mut(&mut dev_desc) }),
        )?;

        trace!(" -- dev_desc: {:?}", dev_desc);

        // TODO: new error for being out of devices.
        let addr = self
            .devices
            .next()
            .ok_or(TransferError::Permanent("out of devices"))?
            .addr;
        debug!("Setting address to {}.", addr);
        self.control_transfer(
            &mut a0ep0,
            RequestType::from((
                RequestDirection::HostToDevice,
                RequestKind::Standard,
                RequestRecipient::Device,
            )),
            RequestCode::SetAddress,
            WValue::from((addr, 0)),
            0,
            none,
        )?;

        // Now that the device is addressed, see if any drivers want
        // it.
        for d in &mut drivers[..] {
            if d.want_device(&dev_desc) {
                let res = d.add_device(dev_desc, addr);
                match res {
                    Ok(_) => return Ok(()),
                    Err(_) => return Err(TransferError::Permanent("out of addresses")),
                }
            }
        }
        Ok(())
    }
}

struct Addr0EP0 {
    in_toggle: bool,
    out_toggle: bool,
}
impl Endpoint for Addr0EP0 {
    fn address(&self) -> u8 {
        0
    }

    fn endpoint_num(&self) -> u8 {
        0
    }

    fn transfer_type(&self) -> TransferType {
        TransferType::Control
    }

    fn direction(&self) -> Direction {
        Direction::In
    }

    fn max_packet_size(&self) -> u16 {
        8
    }

    fn in_toggle(&self) -> bool {
        self.in_toggle
    }

    fn set_in_toggle(&mut self, toggle: bool) {
        self.in_toggle = toggle;
    }

    fn out_toggle(&self) -> bool {
        self.out_toggle
    }

    fn set_out_toggle(&mut self, toggle: bool) {
        self.out_toggle = toggle;
    }
}

pub fn handler(usbp: usize, events: &mut EventWriter) {
    let usb: &mut USB = unsafe { core::mem::transmute(usbp) };
    let flags = usb.host().intflag.read();

    trace!("USB - {:x}", flags.bits());

    let mut unshift_event = |e: Event| {
        unsafe { LATEST_EVENT = e };
        if let Err(_) = events.unshift(e) {
            error!("Couldn't write USB event to queue.");
        }
    };

    if flags.hsof().bit_is_set() {
        trace!(" +hsof");
        usb.host().intflag.write(|w| w.hsof().set_bit());
        unshift_event(Event::Attached);
    }

    if flags.rst().bit_is_set() {
        // We seem to get this whenever a device attaches/detaches.
        trace!(" +rst");
        usb.host().intflag.write(|w| w.rst().set_bit());
        unshift_event(Event::Detached);
    }

    if flags.uprsm().bit_is_set() {
        trace!(" +uprsm");
        usb.host().intflag.write(|w| w.uprsm().set_bit());
        unshift_event(Event::Detached);
    }

    if flags.dnrsm().bit_is_set() {
        trace!(" +dnrsm");
        usb.host().intflag.write(|w| w.dnrsm().set_bit());
        unshift_event(Event::Detached);
    }

    if flags.wakeup().bit_is_set() {
        // §32.8.5.8 - since VBUSOK is set, then this happens when a
        // device is connected.
        trace!(" +wakeup");
        usb.host().intflag.write(|w| w.wakeup().set_bit());
        unshift_event(Event::Attached);
    }

    if flags.ramacer().bit_is_set() {
        trace!(" +ramacer");
        usb.host().intflag.write(|w| w.ramacer().set_bit());
        unshift_event(Event::Detached);
    }

    if flags.dconn().bit_is_set() {
        trace!(" +dconn");
        usb.host().intflag.write(|w| w.dconn().set_bit());
        usb.host().intenclr.write(|w| w.dconn().set_bit());
        usb.host().intflag.write(|w| w.ddisc().set_bit());
        usb.host().intenset.write(|w| w.ddisc().set_bit());
        usb.host().intflag.write(|w| w.dconn().set_bit());
        unshift_event(Event::Attached);
    }

    if flags.ddisc().bit_is_set() {
        trace!(" +ddisc");
        usb.host().intflag.write(|w| w.ddisc().set_bit());
        usb.host().intenclr.write(|w| w.ddisc().set_bit());
        usb.host().intflag.write(|w| w.dconn().set_bit());
        usb.host().intenset.write(|w| w.dconn().set_bit());
        usb.host().intflag.write(|w| w.ddisc().set_bit());
        unshift_event(Event::Detached);
    }
}

impl From<PipeErr> for TransferError {
    fn from(v: PipeErr) -> Self {
        match v {
            PipeErr::TransferFail => Self::Retry("transfer failed"),
            PipeErr::Flow => Self::Retry("data flow"),
            PipeErr::DataToggle => Self::Retry("toggle sequence"),
            PipeErr::ShortPacket => Self::Permanent("short packet"),
            PipeErr::InvalidPipe => Self::Permanent("invalid pipe"),
            PipeErr::InvalidToken => Self::Permanent("invalid token"),
            PipeErr::Stall => Self::Permanent("stall"),
            PipeErr::PipeErr => Self::Permanent("pipe error"),
            PipeErr::HWTimeout => Self::Permanent("hardware timeout"),
            PipeErr::SWTimeout => Self::Permanent("software timeout"),
            PipeErr::Other(s) => Self::Permanent(s),
        }
    }
}

impl<F> USBHost for SAMDHost<'_, F>
where
    F: Fn() -> usize,
{
    fn control_transfer(
        &mut self,
        ep: &mut dyn Endpoint,
        bm_request_type: RequestType,
        b_request: RequestCode,
        w_value: WValue,
        w_index: u16,
        buf: Option<&mut [u8]>,
    ) -> Result<usize, TransferError> {
        let mut pipe = self.pipe_table.pipe_for(self.usb.host_mut(), ep);
        let len = pipe.control_transfer(
            ep,
            bm_request_type,
            b_request,
            w_value,
            w_index,
            buf,
            self.millis,
        )?;
        Ok(len)
    }

    fn in_transfer(
        &mut self,
        ep: &mut dyn Endpoint,
        buf: &mut [u8],
    ) -> Result<usize, TransferError> {
        let mut pipe = self.pipe_table.pipe_for(self.usb.host_mut(), ep);
        let len = pipe.in_transfer(ep, buf, NAK_LIMIT, self.millis)?;
        Ok(len)
    }

    fn out_transfer(&mut self, ep: &mut dyn Endpoint, buf: &[u8]) -> Result<usize, TransferError> {
        let mut pipe = self.pipe_table.pipe_for(self.usb.host_mut(), ep);
        let len = pipe.out_transfer(ep, buf, NAK_LIMIT, self.millis)?;
        Ok(len)
    }
}

unsafe fn to_slice_mut<T>(v: &mut T) -> &mut [u8] {
    let ptr = v as *mut T as *mut u8;
    let len = mem::size_of::<T>();
    core::slice::from_raw_parts_mut(ptr, len)
}
