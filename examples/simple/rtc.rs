use core::sync::atomic::{AtomicUsize, Ordering};
use log;
use trinket_m0::{clock::GenericClockController, RTC};

struct Clock(AtomicUsize);
impl Clock {
    const fn new() -> Self {
        Self(AtomicUsize::new(0))
    }

    fn set(&self, millis: usize) {
        self.0.store(millis, Ordering::SeqCst)
    }

    // Slightly less than 1ms, due to using a 32,768Hz clock, we can't
    // hit exactly 1ms, so we shoot for a bit under.
    fn millis(&self) -> usize {
        self.0.load(Ordering::SeqCst)
    }
}

static CLOCK: Clock = Clock::new();

// Set to run every ~500µs.
static COUNTER: u32 = 16; // 32 ticks requires 1024 cycles at 32,768Hz for 1 second.

pub fn setup(mut rtc: RTC, clocks: &mut GenericClockController) -> impl FnMut() {
    let rtc_clock = &clocks.gclk1();
    clocks.rtc(&rtc_clock);

    rtc.mode0().ctrl.write(|w| w.swrst().set_bit());
    while rtc.mode0().status.read().syncbusy().bit_is_set() {}

    rtc.mode0().ctrl.write(|w| {
        w.mode().count32();

        // Neither the prescaler nor matchlr values seem to work. Not
        // sure why.
        //w.prescaler().div1024();
        w.matchclr().set_bit() // Reset on match for periodic
    });

    rtc.mode0().comp[0].write(|w| unsafe { w.bits(COUNTER) });
    rtc.mode0().intflag.write(|w| w.cmp0().set_bit());
    rtc.mode0().intenset.write(|w| w.cmp0().set_bit());

    // Enable the RTC and wait for sync.
    rtc.mode0().ctrl.write(|w| w.enable().set_bit());
    while rtc.mode0().status.read().syncbusy().bit_is_set() {}

    move || handler(&mut rtc)
}

pub fn millis() -> usize {
    CLOCK.millis()
}

fn handler(rtc: &mut RTC) {
    // FIXME: matchclr doesn't seem to work to reset the counter?
    rtc.mode0().count.write(|w| unsafe { w.bits(0) });
    rtc.mode0().intflag.write(|w| w.cmp0().set_bit());

    static mut TICKS: usize = 0;
    static mut ADD: bool = false;
    unsafe {
        if ADD {
            TICKS += 1;
            CLOCK.set(TICKS);
        }
        ADD = !ADD;

        log::logger().flush();
    }
}
