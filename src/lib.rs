//! USB Host driver implementation for SAMD* series chips.
//! USB Host driver implementation for SAMD* series chips.

#![no_std]

mod pipe;

#[macro_use]
extern crate defmt;

use pipe::{PipeErr, PipeTable};

use usb_host::{AddressPool, DescriptorType, DeviceDescriptor, Direction, Driver, DriverError, Endpoint, RequestCode, RequestDirection, RequestKind, RequestRecipient, RequestType, TransferError, TransferType, USBHost, WValue};

use atsamd_hal::{
    calibration::{usb_transn_cal, usb_transp_cal, usb_trim_cal},
    clock::{ClockGenId, ClockSource, GenericClockController},
    gpio::{self, Floating, Input, OpenDrain, Output},
    target_device::{PM, USB},
};
use embedded_hal::digital::v2::OutputPin;

#[derive(Debug)]
#[derive(defmt::Format)]
pub enum HostEvent {
    Detached,
    Attached,
    RamAccess,
    UpstreamResume,
    DownResume,
    WakeUp,
    Reset,
    HostStartOfFrame,
}

const NAK_LIMIT: usize = 15;

// Ring buffer for sharing events from interrupt context.
// static mut EVENTS: Events = Events::new();

#[derive(Clone, Copy, Debug, PartialEq)]
#[derive(defmt::Format)]
enum DetachedState {
    Initialize,
    WaitForDevice,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[derive(defmt::Format)]
enum AttachedState {
    ResetBus,
    WaitResetComplete,
    WaitSOF(u64),
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[derive(defmt::Format)]
enum SteadyState {
    Configuring,
    Running,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[derive(defmt::Format)]
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

        Self { tbl }
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
        core::mem::replace(&mut self.tbl[addr as usize], None)
    }
}

struct Device {
    addr: u8,
}

pub struct SAMDHost {
    usb: USB,
    task_state: TaskState,

    // Need chunk of RAM for USB pipes, which gets used with DESCADD
    // register.
    pipe_table: PipeTable,

    addr_pool: AddressPool,

    _dm_pad: gpio::Pa24<gpio::PfG>,
    _dp_pad: gpio::Pa25<gpio::PfG>,
    _sof_pad: Option<gpio::Pa23<gpio::PfG>>,
    host_enable_pin: Option<gpio::Pa28<Output<OpenDrain>>>,
    millis: fn() -> u64,
}

pub struct Pins {
    dm_pin: gpio::Pa24<Input<Floating>>,
    dp_pin: gpio::Pa25<Input<Floating>>,
    sof_pin: Option<gpio::Pa23<Input<Floating>>>,
    host_enable_pin: Option<gpio::Pa28<Input<Floating>>>,
}

impl Pins {
    pub fn new(
        dm_pin: gpio::Pa24<Input<Floating>>,
        dp_pin: gpio::Pa25<Input<Floating>>,
        sof_pin: Option<gpio::Pa23<Input<Floating>>>,
        host_enable_pin: Option<gpio::Pa28<Input<Floating>>>,
    ) -> Self {
        Self {
            dm_pin,
            dp_pin,
            sof_pin,
            host_enable_pin,
        }
    }
}

impl SAMDHost {
    pub fn new(
        usb: USB,
        pins: Pins,
        port: &mut gpio::Port,
        clocks: &mut GenericClockController,
        power: &mut PM,
        millis: fn() -> u64,
    ) -> Self {
        power.apbbmask.modify(|_, w| w.usb_().set_bit());

        clocks.configure_gclk_divider_and_source(ClockGenId::GCLK6, 1, ClockSource::DFLL48M, false);
        let gclk6 = clocks.get_gclk(ClockGenId::GCLK6).expect("Could not get clock 6");
        clocks.usb(&gclk6);

        SAMDHost {
            usb,
            task_state: TaskState::Detached(DetachedState::Initialize),
            pipe_table: PipeTable::new(),
            addr_pool: usb_host::AddressPool::new(),

            _dm_pad: pins.dm_pin.into_function_g(port),
            _dp_pad: pins.dp_pin.into_function_g(port),
            _sof_pad: pins.sof_pin.map(|p| p.into_function_g(port)),
            host_enable_pin: pins.host_enable_pin.map(|p| p.into_open_drain_output(port)),
            millis,
        }
    }

    /// Low-Level USB Host Interrupt service method
    /// Any Event returned by should be sent to process_event()
    /// then fsm_tick() should be called for each event or once if no event at all
    pub fn irq_next_event(&self) -> Option<HostEvent> {
        let flags = self.usb.host().intflag.read();

        if flags.ddisc().bit_is_set() {
            self.usb.host().intflag.write(|w| w.ddisc().set_bit());
            Some(HostEvent::Detached)
        } else if flags.dconn().bit_is_set() {
            self.usb.host().intflag.write(|w| w.dconn().set_bit());
            Some(HostEvent::Attached)
        } else if flags.ramacer().bit_is_set() {
            self.usb.host().intflag.write(|w| w.ramacer().set_bit());
            Some(HostEvent::RamAccess)
        } else if flags.uprsm().bit_is_set() {
            self.usb.host().intflag.write(|w| w.uprsm().set_bit());
            Some(HostEvent::UpstreamResume)
        } else if flags.dnrsm().bit_is_set() {
            self.usb.host().intflag.write(|w| w.dnrsm().set_bit());
            Some(HostEvent::DownResume)
        } else if flags.wakeup().bit_is_set() {
            self.usb.host().intflag.write(|w| w.wakeup().set_bit());
            Some(HostEvent::WakeUp)
        } else if flags.rst().bit_is_set() {
            // self.usb.host().intflag.write(|w| w.rst().set_bit());
            Some(HostEvent::Reset)
        } else if flags.hsof().bit_is_set() {
            // self.usb.host().intflag.write(|w| w.hsof().set_bit());
            Some(HostEvent::HostStartOfFrame)
        } else {
            None
        }
    }

    pub fn reset_host(&mut self) {
        debug!("USB Host Resetting");
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
            w.dconn().set_bit();
            w.ddisc().set_bit();
            w.wakeup().set_bit();
            w.ramacer().set_bit();
            w.uprsm().set_bit();
            w.dnrsm().set_bit();
            w.rst().set_bit();
            w.hsof().set_bit()
        });

        self.usb.host().ctrla.modify(|_, w| w.enable().set_bit());
        while self.usb.host().syncbusy.read().enable().bit_is_set() {}

        // Set VBUS OK to allow host operation.
        self.usb.host().ctrlb.modify(|_, w| w.vbusok().set_bit());
        debug!("USB Host Reset");
    }

    pub fn update(&mut self, event: Option<HostEvent>, drivers: &mut dyn Driver) {
        let prev_state = self.task_state;

        if let Some(event) = event {
            match (event, self.task_state) {
                (HostEvent::Detached, TaskState::Attached(_) | TaskState::Steady(_)) => {
                    self.task_state = TaskState::Detached(DetachedState::Initialize)
                }
                (HostEvent::Attached, TaskState::Detached(_)) => {
                    self.task_state = TaskState::Attached(AttachedState::ResetBus)
                }
                _ => {}
            };
        }

        match self.task_state {
            TaskState::Detached(s) => self.detached_fsm(s),
            TaskState::Attached(s) => self.attached_fsm(s),
            TaskState::Steady(s) => self.steady_fsm(s, drivers),
        };

        if prev_state != self.task_state {
            debug!("USB new task state {:?}", self.task_state)
        }
    }

    fn detached_fsm(&mut self, s: DetachedState) {
        match s {
            DetachedState::Initialize => {
                self.reset_host();
                // TODO: Free resources.

                self.task_state = TaskState::Detached(DetachedState::WaitForDevice);
            }

            // Do nothing state. Just wait for an interrupt to come in saying we have a device attached.
            DetachedState::WaitForDevice => {}
        }
    }

    fn attached_fsm(&mut self, s: AttachedState) {
        match s {
            AttachedState::ResetBus => {
                self.usb.host().ctrlb.modify(|_, w| w.busreset().set_bit());
                self.task_state = TaskState::Attached(AttachedState::WaitResetComplete);
            }

            AttachedState::WaitResetComplete => {
                if self.usb.host().intflag.read().rst().bit_is_set() {
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

    fn steady_fsm(&mut self, s: SteadyState, drivers: &mut dyn Driver) {
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
                // for d in &mut drivers[..] {
                if let Err(e) = drivers.tick((self.millis)(), self) {
                    // warn!("running driver {:?}: {:?}", d, e);
                    if let DriverError::Permanent(a, _) = e {
                        drivers.remove_device(a);
                        // self.devices.remove(a);
                    }
                }
                // }
            }

            SteadyState::Error => {}
        }
    }

    fn configure_dev(&mut self, drivers: &mut dyn Driver) -> Result<(), TransferError> {
        let none: Option<&mut [u8]> = None;
        let max_packet_size: u16 = match self.usb.host().status.read().speed().bits() {
            0x0 => 64,
            _ => 8,
        };
        let mut a0ep0 = Addr0EP0 {
            max_packet_size,
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
        let addr = self.addr_pool.take_next().ok_or(TransferError::Permanent("Out of USB addr"))?;
        debug!("Setting address to {}.", addr);
        self.control_transfer(
            &mut a0ep0,
            RequestType::from((
                RequestDirection::HostToDevice,
                RequestKind::Standard,
                RequestRecipient::Device,
            )),
            RequestCode::SetAddress,
            WValue::from((addr.into(), 0)),
            0,
            none,
        )?;

        // Now that the device is addressed, see if any drivers want  it.
        if drivers.want_device(&dev_desc) {
            let res = drivers.add_device(dev_desc, addr.into());
            match res {
                Ok(_) => return Ok(()),
                Err(_) => return Err(TransferError::Permanent("out of addresses")),
            }
        }
        Ok(())
    }
}

struct Addr0EP0 {
    max_packet_size: u16,
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
        self.max_packet_size
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

impl USBHost for SAMDHost {
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
