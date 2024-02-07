//! # Thermostat_EEM IIR wrapper.
//!

use crate::hardware::pwm::Pwm;
use idsp::iir;
use miniconf::Tree;
use num_traits::Signed;

#[derive(Copy, Clone, Debug, Tree)]
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
    /// See [iir::Biquad]
    pub iir: iir::Biquad<f64>,

    /// Thermostat input channel weights. Each input temperature of an enabled channel
    /// is multiplied by its weight and the accumulated output is fed into the IIR.
    /// The weights will be internally normalized to one (sum of the absolute values).
    ///
    /// # Path
    /// `weights/<adc>/<channel>`
    /// * `<adc> := [0, 1, 2, 3]` specifies which adc to configure.
    /// * `<channel>` specifies which channel of an ADC to configure. Only the enabled channels for the specific ADC are available.
    ///
    /// # Value
    /// f32
    pub weights: [[f32; 4]; 4],
}

impl Default for OutputChannel {
    fn default() -> Self {
        Self {
            shutdown: true,
            hold: false,
            voltage_limit: 0.0,
            iir: Default::default(),
            weights: [[0.0; 4]; 4],
        }
    }
}

impl OutputChannel {
    /// compute weighted iir input, iir state and return the new output
    pub fn update(
        &mut self,
        channel_temperatures: &[[f64; 4]; 4],
        iir_state: &mut [f64; 4],
    ) -> f32 {
        let weighted_temperature = channel_temperatures
            .iter()
            .flatten()
            .zip(self.weights.iter().flatten())
            // weight is `None` if temperature is invalid (phy cfg absent)
            .map(|(t, w)| t * *w as f64)
            .sum();
        if self.shutdown || self.hold {
            iir::Biquad::HOLD.update(iir_state, weighted_temperature) as f32
        } else {
            self.iir.update(iir_state, weighted_temperature) as f32
        }
    }

    /// Performs finalization of the output_channel miniconf settings:
    /// - Clamping of the limits
    /// - Normalization of the weights
    /// Returns the current limits.
    pub fn finalize_settings(&mut self) -> [f32; 2] {
        self.iir.set_max(
            self.iir
                .max()
                .clamp(-Pwm::MAX_CURRENT_LIMIT, Pwm::MAX_CURRENT_LIMIT),
        );
        self.iir.set_min(
            self.iir
                .min()
                .clamp(-Pwm::MAX_CURRENT_LIMIT, Pwm::MAX_CURRENT_LIMIT),
        );
        self.voltage_limit = self.voltage_limit.clamp(0.0, Pwm::MAX_VOLTAGE_LIMIT);
        let divisor: f32 = self
            .weights
            .iter()
            .map(|w| w.iter().map(|w| w.abs()).sum::<f32>())
            .sum();
        // Note: The weights which are not 'None' should always affect an enabled channel and therefore count for normalization.
        if divisor != 0.0 {
            for w in self.weights.iter_mut().flatten() {
                *w /= divisor;
            }
        }
        [
            // [Pwm::MAX_CURRENT_LIMIT] + 5% is still below 100% duty cycle for the PWM limits and therefore OK.
            // Might not be OK for a different shunt resistor or different PWM setup.
            (self.iir.max() + 0.05 * Pwm::MAX_CURRENT_LIMIT).max(0.) as f32,
            (self.iir.min() - 0.05 * Pwm::MAX_CURRENT_LIMIT).min(0.) as f32,
        ]
    }
}
