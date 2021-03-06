//! # Thermostat_EEM temperature telemetry processing

use serde::Serialize;

/// Statistics telemetry struct.
#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Statistics {
    mean: f32,
    min: f32,
    max: f32,
}

impl From<Buffer> for Statistics {
    /// Process temperature buffer. Calculates mean, returns the finalized Statistics type
    /// and resets the buffer.
    fn from(buff: Buffer) -> Statistics {
        let mean = buff.accumulator / buff.counter as f64;
        Statistics {
            mean: mean as f32,
            max: buff.max,
            min: buff.min,
        }
    }
}

/// Statistics buffer for computing min/max/mean of the last telemetry period.
#[derive(Copy, Clone, Debug)]
pub struct Buffer {
    min: f32,
    max: f32,
    accumulator: f64,
    counter: u32,
}

impl Buffer {
    /// Add a new temperature sample to the buffer. This will add it to the accumulator,
    /// update min/max and increment the counter.
    pub fn update(&mut self, temp: f64) {
        self.max = self.max.max(temp as f32);
        self.min = self.min.min(temp as f32);
        self.accumulator += temp;
        self.counter += 1;
    }
}

impl Default for Buffer {
    fn default() -> Self {
        Self {
            counter: 0,
            accumulator: 0.,
            max: f32::NEG_INFINITY,
            min: f32::INFINITY,
        }
    }
}
