#![no_std]
#![no_main]
#![feature(const_fn)]
#![feature(const_transmute)]
#![allow(dead_code)]

mod logger;
mod macros;
mod rtc;

use atsamd_usb_host::SAMDHost;
use bootkbd::BootKeyboard;
use clint::HandlerArray;
use cortex_m::asm::wfi;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use embedded_hal::digital::v2::OutputPin;
use log::{debug, info, LevelFilter};
use trinket_m0::{
    self as hal,
    clock::GenericClockController,
    gpio::{OpenDrain, Output, Pa10, Pa6, Pa7, PfD},
    sercom,
    target_device::{interrupt, Interrupt},
    time::*,
    CorePeripherals, Peripherals,
};
use usb_host::Driver;

static HANDLERS: HandlerArray = HandlerArray::new();

static mut LED: usize = 0;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().expect("taking peripherals");
    let mut core = CorePeripherals::take().expect("taking core peripherals");

    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut pins = hal::Pins::new(peripherals.PORT);

    let uart = hal::uart(
        &mut clocks,
        115_200.hz(),
        peripherals.SERCOM0,
        &mut core.NVIC,
        &mut peripherals.PM,
        pins.d3,
        pins.d4,
        &mut pins.port,
    );

    let mut red_led = pins.d13.into_open_drain_output(&mut pins.port);
    red_led.set_low().expect("turning off red LED");
    unsafe { LED = core::mem::transmute(&red_led) }

    // We do the transmute because, while all the underlying data is
    // static, we're unable to get a referecence to the UART or LED
    // until run-time. Another option would be to use Option in the
    // SerialLogger definition, but that requires a check every time
    // they might be used.
    let uart_wrapped = logger::WriteWrapper::new(uart);
    let logger = logger::SerialLogger::new(uart_wrapped, red_led);

    // Wow, would I love to not be annotating this type.
    let logger_ref: &'static logger::SerialLogger<
        sercom::UART0<sercom::Sercom0Pad3<Pa7<PfD>>, sercom::Sercom0Pad2<Pa6<PfD>>, (), ()>,
        Pa10<Output<OpenDrain>>,
    > = unsafe { core::mem::transmute(&logger) };
    unsafe { log::set_logger_racy(logger_ref).expect("couldn't set logger") };
    log::set_max_level(LevelFilter::Info);

    info!("setting up timer");
    let mut rtc_handler = rtc::setup(peripherals.RTC, &mut clocks);

    info!("setting up usb host");
    let (mut usb_host, mut usb_handler) = SAMDHost::new(
        peripherals.USB,
        pins.usb_sof,
        pins.usb_dm,
        pins.usb_dp,
        Some(pins.usb_host_enable),
        &mut pins.port,
        &mut clocks,
        &mut peripherals.PM,
        &rtc::millis,
    );

    let mut bootkbd = BootKeyboard::new();
    let mut drivers: [&mut dyn Driver; 1] = [&mut bootkbd];

    info!("setting up handlers");
    HANDLERS.with_overrides(|hs| {
        hs.register(0, &mut rtc_handler);
        core.NVIC.enable(Interrupt::RTC);

        hs.register(1, &mut usb_handler);
        unsafe { core.NVIC.set_priority(Interrupt::USB, 0) };
        core.NVIC.enable(Interrupt::USB);

        info!("Boot up complete.");

        loop {
            usb_host.task(&mut drivers[..]);
            wfi();
        }
    });
    unreachable!();
}

#[panic_handler]
fn panic_handler(pi: &core::panic::PanicInfo) -> ! {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    logln_now!("~~~ PANIC ~~~");
    logln_now!("{}", pi);
    logln_now!("flushing log");
    loop {
        log::logger().flush();
        wfi()
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    log::logger().flush();
    logln_now!("!!! Hard Fault - ef: {:?} !!!", ef);
    logln_now!("flushing log");
    loop {
        log::logger().flush();
        wfi()
    }
}

#[exception]
fn DefaultHandler(interrupt: i16) {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    debug!("*** Default Handler: {} ***", interrupt);
}

#[exception]
fn NonMaskableInt() {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    debug!("+++ NonMaskableInt +++");
}

#[exception]
fn SVCall() {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    debug!("+++ SVCall +++");
}

#[exception]
fn PendSV() {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    debug!("+++ PendSV +++");
}

#[exception]
fn SysTick() {
    let red_led: &mut Pa10<Output<OpenDrain>> = unsafe { core::mem::transmute(LED) };
    red_led.set_high().ok();

    debug!("+++ SysTick +++");
}

#[interrupt]
fn RTC() {
    HANDLERS.call(0);
}

#[interrupt]
fn USB() {
    HANDLERS.call(1);
}
