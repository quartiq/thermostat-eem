use super::hal::hal::blocking::delay::DelayUs;

pub struct AsmDelay {
    cyc_per_us_corrected: u32,
}

impl AsmDelay {
    pub fn new(freq: u32) -> AsmDelay {
        AsmDelay {
            // Corrected value for cortex_m::asm::delay cycles per us.
            // See https://github.com/rust-embedded/cortex-m/issues/430
            cyc_per_us_corrected: (freq / 1_000_000) * 2,
        }
    }
}

impl<U> DelayUs<U> for AsmDelay
where
    U: Into<u32>,
{
    fn delay_us(&mut self, us: U) {
        cortex_m::asm::delay(self.cyc_per_us_corrected * us.into())
    }
}
