//! # Thermostat_EEM temperature telemetry processing

use serde::Serialize;

/// Temperature telemetry struct.
#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Temperature {
    average: f32,
    min: f32,
    max: f32,
}

/// Temperature buffer for computing min/max/average of the last telemetry period.
#[derive(Default, Copy, Clone, Debug)]
pub struct TemperatureBuffer {
    min: f32,
    max: f32,
    accumulator: f32,
    counter: u32,
}

impl TemperatureBuffer {
    /// Add a new temperature sample to the buffer. This will add it to the accumulator,
    /// update min/max and increment the counter.
    pub fn add(&mut self, temp: f32) {
        if self.counter == 0 {
            self.min = temp;
            self.max = temp;
        } else {
            self.max = self.max.max(temp);
            self.min = self.min.min(temp);
        }
        self.accumulator += temp;
        self.counter += 1;
    }

    /// Process temperature buffer. Calculates average, returns the finalized Temperature type
    /// and resets the buffer.
    pub fn process(&mut self) -> Temperature {
        let average = self.accumulator / self.counter as f32;
        let temp = Temperature {
            average,
            max: self.max,
            min: self.min,
        };
        self.counter = 0;
        self.accumulator = 0.;
        self.max = 0.;
        self.min = 0.;
        temp
    }
}
