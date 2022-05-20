//! # Thermostat_EEM temperature telemetry processing

use serde::Serialize;

/// Statistics telemetry struct.
#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Statistics {
    mean: f32,
    min: f32,
    max: f32,
}

/// Statistics buffer for computing min/max/mean of the last telemetry period.
#[derive(Default, Copy, Clone, Debug)]
pub struct StatisticsBuffer {
    min: f32,
    max: f32,
    accumulator: f32,
    counter: u32,
}

impl StatisticsBuffer {
    /// Add a new temperature sample to the buffer. This will add it to the accumulator,
    /// update min/max and increment the counter.
    pub fn add(&mut self, temp: f32) {
        self.max = self.max.max(temp);
        self.min = self.min.min(temp);
        self.accumulator += temp;
        self.counter += 1;
    }

    /// Process temperature buffer. Calculates mean, returns the finalized Statistics type
    /// and resets the buffer.
    pub fn process(&mut self) -> Statistics {
        let mean = self.accumulator / self.counter as f32;
        let temp = Statistics {
            mean,
            max: self.max,
            min: self.min,
        };
        self.counter = 0;
        self.accumulator = 0.;
        self.max = f32::NEG_INFINITY;
        self.min = f32::INFINITY;
        temp
    }
}
