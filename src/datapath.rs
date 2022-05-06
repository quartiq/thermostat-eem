//! # Thermostat_EEM IIR wrapper.
//!

use idsp::iir;

#[derive(Copy, Clone, Debug)]
pub struct Datapath {
    iir: iir::IIR<f64>,
    weights: [f64; 8],
}

impl Datapath {
    /// idsp https://docs.rs/idsp/latest/idsp/ f64 implementation with input
    /// weights to route and weigh 8 input channels into one IIR.
    pub fn new(gain: f64, y_min: f64, y_max: f64, weights: [f64; 8]) -> Self {
        Datapath {
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
    ) -> f64 {
        let weighted_input = channel_temperatures
            .iter()
            .zip(self.weights.iter())
            .fold(0.0, |acc, (temperature, weight)| {
                acc + *temperature * *weight
            });
        self.iir.update(iir_state, weighted_input, hold)
    }
}
