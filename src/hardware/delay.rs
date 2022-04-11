use super::hal::hal::blocking::delay::{DelayMs, DelayUs};

pub struct AsmDelay {
    frequency_us: u32,
    frequency_ms: u32,
}

impl AsmDelay {
    pub fn new(freq: u32) -> AsmDelay {
        AsmDelay {
            // multiply by two to get to the frequency that halt instructions have to be issued on the STM32H7 (dual-issue pipeline)
            frequency_us: (freq / 1_000_000) * 2,
            frequency_ms: (freq / 1_000) * 2,
        }
    }
}

impl<U> DelayUs<U> for AsmDelay
where
    U: Into<u32>,
{
    fn delay_us(&mut self, us: U) {
        cortex_m::asm::delay(self.frequency_us * us.into())
    }
}

impl<U> DelayMs<U> for AsmDelay
where
    U: Into<u32>,
{
    fn delay_ms(&mut self, ms: U) {
        cortex_m::asm::delay(self.frequency_ms * ms.into())
    }
}
