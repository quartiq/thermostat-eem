//! # Thermostat_EEM IIR wrapper.
//!

use crate::hardware::pwm::Pwm;
use idsp::iir;
use miniconf::Miniconf;
use num_traits::Signed;

#[derive(Copy, Clone, Debug, Miniconf)]
pub struct OutputChannel {
    /// En-/Disables the TEC driver. This implies "hold".
    ///
    /// # Value
    /// true to shut the driver down, false to enable the driver.
    pub shutdown: bool,

    /// Hold the output.
    ///
    /// # Value
    /// true to hold, false to continue with default iir.
    pub hold: bool,

    /// Maximum absolute (positive and negative) TEC voltage in volt.
    /// These will be clamped to the maximum of 4.3 V.
    ///
    /// # Value
    /// 0.0 to 4.3
    pub voltage_limit: f32,

    /// IIR filter parameters.
    /// The y limits will be clamped to the maximum output current of +-3 A.
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    pub iir: iir::IIR<f64>,

    /// Thermostat input channel weights. Each input temperature is multiplied by its weight
    /// and the accumulated output is fed into the IIR.
    /// The weights will be internally normalized to one (sum of the absolute values).
    #[miniconf(defer)]
    pub weights: miniconf::Array<miniconf::Array<Option<f32>, 4>, 4>,
}

impl OutputChannel {
    /// idsp https://docs.rs/idsp/latest/idsp/ f64 implementation with input
    /// weights to route and weigh 8 input channels into one IIR.
    pub fn new(gain: f64, y_min: f64, y_max: f64) -> Self {
        OutputChannel {
            shutdown: true,
            hold: false,
            voltage_limit: 1.0,
            iir: iir::IIR::new(gain, y_min, y_max),
            weights: Default::default(),
        }
    }

    /// compute weighted iir input, iir state and return the new output
    pub fn update(
        &mut self,
        channel_temperatures: &[[Option<f64>; 4]; 4],
        iir_state: &mut iir::Vec5<f64>,
        hold: bool,
    ) -> f32 {
        // Global "hold" IIR to apply to a channel iir state [x0,x1,x2,y0,y1] when the output should hold.
        const IIR_HOLD: iir::IIR<f64> = iir::IIR {
            ba: [0., 0., 0., 1., 0.],
            y_offset: 0.,
            y_min: f64::MIN,
            y_max: f64::MAX,
        };

        let weighted_temperature = channel_temperatures
            .iter()
            .flatten()
            .zip(self.weights.iter().flatten())
            // zero default for weight and temp is OK here since they should always be 'None' together and then we want to add zero
            .map(|(t, w)| t.unwrap_or(0.) * w.unwrap_or(0.) as f64)
            .sum();
        if self.shutdown || self.hold {
            IIR_HOLD.update(iir_state, weighted_temperature, hold) as f32
        } else {
            self.iir.update(iir_state, weighted_temperature, hold) as f32
        }
    }

    /// Performs finalization of the output_channel miniconf settings:
    /// - Clamping of the limits
    /// - Normalization of the weights
    /// Returns the current limits.
    pub fn finalize_settings(&mut self) -> [f32; 2] {
        self.iir.y_max = self
            .iir
            .y_max
            .clamp(-Pwm::MAX_CURRENT_LIMIT, Pwm::MAX_CURRENT_LIMIT);
        self.iir.y_min = self
            .iir
            .y_min
            .clamp(-Pwm::MAX_CURRENT_LIMIT, Pwm::MAX_CURRENT_LIMIT);
        self.voltage_limit = self.voltage_limit.clamp(0.0, Pwm::MAX_VOLTAGE_LIMIT);
        let divisor: f32 = self
            .weights
            .iter()
            .map(|w| w.iter().map(|w| w.unwrap_or(0.).abs()).sum::<f32>())
            .sum();
        // maybe todo: ensure that the weights actually impact an enabled channel
        if divisor != 0.0 {
            self.weights.iter_mut().flatten().for_each(|w| {
                if let Some(w) = w {
                    *w /= divisor
                }
            });
        }
        [
            // [Pwm::MAX_CURRENT_LIMIT] + 5% is still below 100% duty cycle for the PWM limits and therefore OK.
            // Might not be OK for a different shunt resistor or different PWM setup.
            (self.iir.y_max + 0.05 * Pwm::MAX_CURRENT_LIMIT).max(0.) as f32,
            (self.iir.y_min - 0.05 * Pwm::MAX_CURRENT_LIMIT).min(0.) as f32,
        ]
    }
}
