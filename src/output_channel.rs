//! # Thermostat_EEM IIR wrapper.
//!

use crate::{hardware::pwm::Pwm, DacCode};
use idsp::iir;
use miniconf::Tree;
use num_traits::Float;

#[derive(Copy, Clone, Debug, Tree)]
pub struct Pid {
    pub ki: f32,
    pub kp: f32,
    pub kd: f32,
    pub li: Option<f32>,
    pub ld: Option<f32>,
    pub setpoint: f32,
    pub min: Option<f32>,
    pub max: Option<f32>,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            ki: 0.,
            kp: 0., // positive, sign reference for all gains and limits
            kd: 0.,
            li: None,
            ld: None,
            setpoint: 25.,
            min: None,
            max: None,
        }
    }
}

impl TryFrom<Pid> for iir::Biquad<f64> {
    type Error = iir::PidError;
    fn try_from(value: Pid) -> Result<Self, Self::Error> {
        let mut biquad: iir::Biquad<f64> = iir::Pid::<f64>::default()
            .period(1.0 / 1007.0) // ADC sample rate (ODR) including zero-oder-holds
            .gain(iir::Action::Ki, value.ki.copysign(value.kp) as _)
            .gain(iir::Action::Kp, value.kp as _)
            .gain(iir::Action::Kd, value.kd.copysign(value.kp) as _)
            .limit(
                iir::Action::Ki,
                value.li.unwrap_or(f32::INFINITY).copysign(value.kp) as _,
            )
            .limit(
                iir::Action::Kd,
                value.ld.unwrap_or(f32::INFINITY).copysign(value.kp) as _,
            )
            .build()?
            .into();
        biquad.set_input_offset(-value.setpoint as _);
        biquad.set_min(value.min.unwrap_or(f32::NEG_INFINITY) as _);
        biquad.set_max(value.max.unwrap_or(f32::INFINITY) as _);
        Ok(biquad)
    }
}

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

#[derive(Copy, Clone, Debug, Tree)]
pub struct OutputChannel {
    pub state: State,

    /// Maximum absolute (positive and negative) TEC voltage in volt.
    /// These will be clamped to the maximum of 4.3 V.
    ///
    /// # Value
    /// 0.0 to 4.3
    pub voltage_limit: f32,

    #[tree]
    pub pid: Pid,

    /// IIR filter parameters.
    /// The y limits will be clamped to the maximum output current of +-3 A.
    ///
    /// # Value
    /// See [iir::Biquad]
    #[tree(skip)]
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
            state: State::Off,
            voltage_limit: Pwm::MAX_VOLTAGE_LIMIT,
            pid: Default::default(),
            iir: Default::default(),
            weights: Default::default(),
        }
    }
}

impl OutputChannel {
    /// compute weighted iir input, iir state and return the new output
    pub fn update(
        &mut self,
        channel_temperatures: &[[f64; 4]; 4],
        iir_state: &mut [f64; 4],
    ) -> f64 {
        let weighted_temperature = channel_temperatures
            .iter()
            .flatten()
            .zip(self.weights.iter().flatten())
            .map(|(t, w)| t * *w as f64)
            .sum();
        let iir = if self.state == State::On {
            &self.iir
        } else {
            &iir::Biquad::HOLD
        };
        iir.update(iir_state, weighted_temperature)
    }

    /// Performs finalization of the output_channel miniconf settings:
    /// - Clamping of the limits
    /// - Normalization of the weights
    /// Returns the current limits.
    pub fn finalize_settings(&mut self) {
        if let Ok(iir) = self.pid.try_into() {
            self.iir = iir;
        } else {
            log::info!("Pid build failure, update not applied.");
        }
        let range = DacCode::MAX_CURRENT.min(Pwm::MAX_CURRENT_LIMIT);
        self.iir
            .set_max(self.iir.max().clamp(-range as _, range as _));
        self.iir
            .set_min(self.iir.min().clamp(-range as _, range as _));
        self.voltage_limit = self.voltage_limit.clamp(0.0, Pwm::MAX_VOLTAGE_LIMIT);
        let divisor: f32 = self.weights.iter().flatten().map(|w| w.abs()).sum::<f32>();
        // Note: The weights which are not 'None' should always affect an enabled channel and therefore count for normalization.
        if divisor != 0.0 {
            let n = divisor.recip();
            for w in self.weights.iter_mut().flatten() {
                *w *= n;
            }
        }
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
