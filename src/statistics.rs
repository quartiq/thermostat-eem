//! # Thermostat_EEM temperature telemetry processing

use serde::Serialize;

/// Statistics telemetry struct. Contains the mean, min and max temperature in the last telemetry period.
#[derive(Serialize, Copy, Clone, Default, Debug)]
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
            let c = 1.0 / buff.counter as f64;
            let mean = buff.m1 * c;
            let var = buff.m2 * c - mean * mean;
            Some(Statistics {
                mean: (mean + buff.first) as f32,
                ptp: (buff.max - buff.min) as f32,
                std: libm::sqrtf(var as f32),
            })
        } else {
            None
        }
    }
}

/// Statistics buffer for computing min/max/mean of the last telemetry period.
#[derive(Copy, Clone, Debug)]
pub struct Buffer {
    min: f64,
    max: f64,
    m1: f64,
    m2: f64,
    first: f64,
    counter: u32,
}

impl Buffer {
    /// Add a new temperature sample to the buffer. This will add it to the accumulator,
    /// update min/max and increment the counter.
    pub fn update(&mut self, temp: f64) {
        self.max = self.max.max(temp);
        self.min = self.min.min(temp);
        if self.counter == 0 {
            self.first = temp;
        };
        let t = temp - self.first;
        self.m1 += t;
        self.m2 += t * t;
        self.counter += 1;
    }
}

impl Default for Buffer {
    fn default() -> Self {
        Self {
            counter: 0,
            m1: 0.,
            m2: 0.,
            first: 0.,
            max: f64::NEG_INFINITY,
            min: f64::INFINITY,
        }
    }
}
