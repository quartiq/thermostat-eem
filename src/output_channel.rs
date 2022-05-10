//! # Thermostat_EEM IIR wrapper.
//!

use idsp::iir;
use miniconf::MiniconfAtomic;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, Deserialize, Serialize, MiniconfAtomic)]
pub struct OutputChannel {
    /// IIR filter parameters.
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    iir: iir::IIR<f64>,

    /// Thermostat input channel weights. Each input temperature is multiplied by its weight
    /// and the accumulated output is fed into the IIR.
    ///
    /// # Value
    /// [f32; 8]
    weights: [f32; 8],
}

impl OutputChannel {
    /// idsp https://docs.rs/idsp/latest/idsp/ f64 implementation with input
    /// weights to route and weigh 8 input channels into one IIR.
    pub fn new(gain: f64, y_min: f64, y_max: f64, weights: [f32; 8]) -> Self {
        OutputChannel {
            iir: iir::IIR::new(gain, y_min, y_max),
            weights,
        }
    }

    /// compute weigthed iir input, iir state and return the new output
    pub fn update(
        &mut self,
        channel_temperatures: &[f64; 8],
        iir_state: &mut iir::Vec5<f64>,
        hold: bool,
    ) -> f32 {
        let weighted_temperature = channel_temperatures
            .iter()
            .zip(self.weights.iter())
            .map(|(temperature, weight)| *temperature * *weight as f64)
            .sum();
        self.iir.update(iir_state, weighted_temperature, hold) as f32
    }
}
