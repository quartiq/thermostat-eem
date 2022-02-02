use embedded_time::{clock::Error, fraction::Fraction, Clock, Instant};
///! System timer used for RTIC scheduling
///!
///! # Design
///! The SystemTimer is an RTIC monotonic timer that can be used for scheduling tasks in RTIC.
///! This timer is used in place of the cycle counter to allow the timer to tick at a slower rate
///! than the CPU clock. This allows for longer scheduling periods with less resolution. This is
///! needed for infrequent (e.g. multiple second) telemetry periods.
use hal::prelude::*;
use stm32h7xx_hal as hal;

// A global buffer indicating how many times the internal counter has overflowed.
static mut OVERFLOWS: u32 = 0;

/// System timer used for implementing RTIC scheduling.
///
/// This implementation synchronizes access to the timer peripheral, so it is safe to copy/clone
/// and/or instantiate multiple timers. All timers will reference the same underlying hardware
/// clock.
///
/// This timer supports durations up to (just shy of) 5 days. Any duration larger than that will
/// wrap and behave incorrectly.
///
/// # Note
/// The system timer must be initialized before being used.
#[derive(Copy, Clone, Debug, Default)]
pub struct SystemTimer {}

impl SystemTimer {
    /// Initialize the system timer.
    ///
    /// # Args
    /// * `timer` - The hardware timer used for implementing the RTIC monotonic.
    pub fn initialize(mut timer: hal::timer::Timer<hal::device::TIM15>) {
        timer.pause();
        // Have the system timer operate at a tick rate of 10 KHz (100 µs per tick). With this
        // configuration and a 65535 period, we get an overflow once every 6.5 seconds.
        timer.set_tick_freq(10.khz());
        timer.apply_freq();

        timer.resume();
    }
}

impl Clock for SystemTimer {
    type T = u32;

    // The duration of each tick in seconds.
    const SCALING_FACTOR: Fraction = Fraction::new(1, 10_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        // Note(unsafe): Multiple interrupt contexts have access to the underlying timer, so care
        // is taken when reading and modifying register values.
        let regs = unsafe { &*hal::device::TIM15::ptr() };

        loop {
            if let Some(instant) = cortex_m::interrupt::free(|_cs| {
                // Checking for overflows of the current counter must be performed atomically. Any
                // other task that is accessing the current time could potentially race for the
                // registers. Note that this is only required for writing to global state (e.g. timer
                // registers and overflow counter)
                // Check for overflows and clear the overflow bit atomically. This must be done in
                // a critical section to prevent race conditions on the status register.
                if regs.sr.read().uif().bit_is_set() {
                    regs.sr.modify(|_, w| w.uif().clear_bit());
                    unsafe {
                        OVERFLOWS = OVERFLOWS.wrapping_add(1);
                    }
                }

                let current_value = regs.cnt.read().bits();

                // Check that an overflow didn't occur since we just cleared the overflow bit. If
                // it did, loop around and retry.
                if regs.sr.read().uif().bit_is_clear() {
                    // Note(unsafe): We are in a critical section, so it is safe to read the
                    // global variable.
                    unsafe { Some(Instant::new(((OVERFLOWS << 16) + current_value) as u32)) }
                } else {
                    None
                }
            }) {
                return Ok(instant);
            }
        }
    }
}
