//! # Thermostat_EEM IIR wrapper.
//!

use crate::{hardware::pwm::Pwm, DacCode};
use idsp::iir;
use miniconf::{Leaf, Tree};
use num_traits::Float;

#[derive(
    Copy, Clone, Default, Debug, PartialEq, PartialOrd, serde::Serialize, serde::Deserialize,
)]
pub enum State {
    /// Active TEC driver and Biquad/PID.
    On,
    /// Hold the output.
    Hold,
    /// Disables the TEC driver. This implies "hold".
    #[default]
    Off,
}

#[derive(Clone, Debug, Tree)]
pub struct OutputChannel {
    pub state: Leaf<State>,

    /// Maximum absolute (positive and negative) TEC voltage in volt.
    /// These will be clamped to the maximum of 4.3 V.
    ///
    /// # Value
    /// 0.0 to 4.3
    #[tree(validate=self.validate_voltage_limit)]
    pub voltage_limit: Leaf<f32>,

    /// PID/Biquad/IIR filter parameters
    ///
    /// The y limits will be clamped to the maximum output current of +-3 A.
    #[tree(validate=self.validate_pid)]
    pub pid: iir::BiquadRepr<f32, f64>,

    #[tree(skip)]
    pub iir: iir::Biquad<f64>,

    /// Thermostat input channel weights. Each input of an enabled input channel
    /// is multiplied by its weight and the accumulated output is fed into the IIR.
    /// The weights will be internally normalized to one (sum of the absolute values)
    /// if they are not all zero.
    #[tree(validate=self.validate_weights)]
    pub weights: Leaf<[[f32; 4]; 4]>,
}

impl Default for OutputChannel {
    fn default() -> Self {
        let mut s = Self {
            state: State::Off.into(),
            voltage_limit: Pwm::MAX_VOLTAGE_LIMIT.into(),
            pid: iir::BiquadRepr::Pid(iir::Pid {
                max: Leaf(0.01),
                min: Leaf(-0.01),
                setpoint: Leaf(25.0),
                ..Default::default()
            }),
            iir: Default::default(),
            weights: Default::default(),
        };
        s.validate_pid(0).unwrap();
        s.validate_voltage_limit(0).unwrap();
        s.validate_weights(0).unwrap();
        s
    }
}

impl OutputChannel {
    /// compute weighted iir input, iir state and return the new output
    pub fn update(&mut self, temperatures: &[[f64; 4]; 4], iir_state: &mut [f64; 4]) -> f64 {
        let temperature = temperatures
            .as_flattened()
            .iter()
            .zip(self.weights.as_flattened().iter())
            .map(|(t, w)| t * *w as f64)
            .sum();
        let iir = if *self.state == State::On {
            &self.iir
        } else {
            &iir::Biquad::HOLD
        };
        iir.update(iir_state, temperature)
    }

    fn validate_pid(&mut self, depth: usize) -> Result<usize, &'static str> {
        self.iir = self.pid.build::<f64>(1007.0.recip(), 1.0);
        let range = DacCode::MAX_CURRENT.min(Pwm::MAX_CURRENT_LIMIT);
        self.iir
            .set_max(self.iir.max().clamp(-range as _, range as _));
        self.iir
            .set_min(self.iir.min().clamp(-range as _, range as _));
        Ok(depth)
    }

    fn validate_voltage_limit(&mut self, depth: usize) -> Result<usize, &'static str> {
        *self.voltage_limit = (*self.voltage_limit).clamp(0.0, Pwm::MAX_VOLTAGE_LIMIT);
        Ok(depth)
    }

    fn validate_weights(&mut self, depth: usize) -> Result<usize, &'static str> {
        let divisor: f32 = self.weights.as_flattened().iter().map(|w| w.abs()).sum();
        // Note: The weights which are not 'None' should always affect an enabled channel and therefore count for normalization.
        if divisor != 0.0 {
            let n = divisor.recip();
            for w in self.weights.as_flattened_mut().iter_mut() {
                *w *= n;
            }
        }
        Ok(depth)
    }

    pub fn current_limits(&self) -> [f32; 2] {
        [
            // give 5% extra headroom for PWM current limits
            // [Pwm::MAX_CURRENT_LIMIT] + 5% is still below 100% duty cycle for the PWM limits and therefore OK.
            // Might not be OK for a different shunt resistor or different PWM setup.
            (self.iir.max() as f32 + 0.05 * Pwm::MAX_CURRENT_LIMIT).max(0.),
            (self.iir.min() as f32 - 0.05 * Pwm::MAX_CURRENT_LIMIT).min(0.),
        ]
    }
}
