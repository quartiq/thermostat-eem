//! # Thermostat_EEM temperature telemetry processing

use serde::Serialize;
use num_traits::Float;

/// Statistics telemetry struct. Contains the mean, min and max temperature in the last telemetry period.
#[derive(Serialize, Copy, Clone, Debug)]
pub struct Statistics {
    mean: f32,
    ptp: f32,
    std: f32,
}

impl From<Buffer> for Option<Statistics> {
    /// Process temperature buffer. Calculates mean, returns the finalized Statistics type
    /// and resets the buffer.
    fn from(buff: Buffer) -> Self {
        if buff.counter > 0 {
            let c = 1.0 / buff.counter as f32;
            let mean = buff.m1 * c;
            let var = buff.m2 * c - mean * mean;
            Some(Statistics {
                mean: mean + buff.x0,
                ptp: buff.max - buff.min,
                std: var.sqrt(),
            })
        } else {
            None
        }
    }
}

/// Statistics buffer for computing min/max/mean of the last telemetry period.
#[derive(Copy, Clone, Debug)]
pub struct Buffer {
    min: f32,
    max: f32,
    m1: f32,
    m2: f32,
    x0: f32,
    counter: u32,
}

impl Buffer {
    /// Add a new temperature sample to the buffer. This will add it to the accumulator,
    /// update min/max and increment the counter.
    pub fn update(&mut self, x: f32) {
        self.max = self.max.max(x);
        self.min = self.min.min(x);
        if self.counter == 0 {
            self.x0 = x;
        };
        self.counter += 1;
        let t = x - self.x0;
        self.m1 += t;
        self.m2 += t * t;
    }
}

impl Default for Buffer {
    fn default() -> Self {
        Self {
            counter: 0,
            m1: 0.,
            m2: 0.,
            x0: 0.,
            max: f32::NEG_INFINITY,
            min: f32::INFINITY,
        }
    }
}
