//! # Thermostat_EEM IIR wrapper.
//!

use core::iter::Take;

use crate::{hardware::pwm::Pwm, DacCode};
use idsp::{
    iir, {AccuOsc, Sweep},
};
use miniconf::{Leaf, Tree};
use num_traits::Float;

#[derive(Clone, Debug, Tree)]
pub struct Pid {
    /// Integral gain
    ///
    /// Units: output/input per second
    pub ki: Leaf<f32>,
    /// Proportional gain
    ///
    /// Note that this is the sign reference for all gains and limits
    ///
    /// Units: output/input
    pub kp: Leaf<f32>,
    /// Derivative gain
    ///
    /// Units: output/input*second
    pub kd: Leaf<f32>,
    /// Integral gain limit
    ///
    /// Units: output/input
    pub li: Leaf<f32>,
    /// Derivative gain limit
    ///
    /// Units: output/input
    pub ld: Leaf<f32>,
    /// Setpoint
    ///
    /// Units: input
    pub setpoint: Leaf<f32>,
    /// Output lower limit
    ///
    /// Units: output
    pub min: Leaf<f32>,
    /// Output upper limit
    ///
    /// Units: output
    pub max: Leaf<f32>,
    /// Update/sample period
    ///
    /// Units: seconds
    #[tree(skip)]
    pub period: f32,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            ki: 0.0.into(),
            kp: 0.0.into(), // positive default
            kd: 0.0.into(),
            li: f32::INFINITY.into(),
            ld: f32::INFINITY.into(),
            setpoint: 0.0.into(),
            min: f32::NEG_INFINITY.into(),
            max: f32::INFINITY.into(),
            period: 1.0,
        }
    }
}

impl Pid {
    fn biquad(&self) -> Result<iir::Biquad<f64>, iir::PidError> {
        let mut biquad: iir::Biquad<f64> = iir::Pid::<f64>::default()
            .period(self.period as _) // ADC sample rate (ODR) including zero-oder-holds
            .gain(iir::Action::Ki, self.ki.copysign(*self.kp) as _)
            .gain(iir::Action::Kp, *self.kp as _)
            .gain(iir::Action::Kd, self.kd.copysign(*self.kp) as _)
            .limit(
                iir::Action::Ki,
                if self.li.is_finite() {
                    *self.li
                } else {
                    f32::INFINITY
                }
                .copysign(*self.kp) as _,
            )
            .limit(
                iir::Action::Kd,
                if self.ld.is_finite() {
                    *self.ld
                } else {
                    f32::INFINITY
                }
                .copysign(*self.kp) as _,
            )
            .build()?
            .into();
        biquad.set_input_offset(-*self.setpoint as _);
        biquad.set_min(if self.min.is_finite() {
            *self.min
        } else {
            f32::NEG_INFINITY
        } as _);
        biquad.set_max(if self.max.is_finite() {
            *self.max
        } else {
            f32::INFINITY
        } as _);
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

#[derive(Clone, Debug, Tree)]
pub struct OutputChannel {
    /// Thermostat input channel weights. Each input of an enabled input channel
    /// is multiplied by its weight and the accumulated output is fed into the IIR.
    /// The weights will be internally normalized to one (sum of the absolute values)
    /// if they are not all zero.
    #[tree(validate=self.validate_weights)]
    pub weights: Leaf<[[f32; 4]; 4]>,

    /// PID/Biquad/IIR filter parameters
    ///
    /// The y limits will be clamped to the maximum output current of +-3 A.
    #[tree(validate=self.validate_pid)]
    pub pid: Pid,

    #[tree(skip)]
    pub iir: iir::Biquad<f64>,

    #[tree(skip)]
    iir_state: [f64; 4],

    pub state: Leaf<State>,

    pub sweep: SineSweep,

    /// Maximum absolute (positive and negative) TEC voltage in volt.
    /// These will be clamped to the maximum of 4.3 V.
    ///
    /// # Value
    /// 0.0 to 4.3
    #[tree(validate=self.validate_voltage_limit)]
    pub voltage_limit: Leaf<f32>,
}

#[derive(Clone, Debug, Tree)]
pub struct SineSweep {
    #[tree(skip)]
    sweep: Take<AccuOsc<Sweep>>,

    rate: Leaf<i32>,
    cycles: Leaf<i32>,
    length: Leaf<usize>,
    amp: Leaf<f32>,
    #[tree(validate=self.trigger)]
    trigger: Leaf<bool>,
}

impl Default for SineSweep {
    fn default() -> Self {
        Self {
            sweep: AccuOsc::new(Sweep::new(0, 0)).take(0),
            rate: Leaf(0),
            cycles: Leaf(0),
            length: Leaf(0),
            amp: Leaf(0.0),
            trigger: Leaf(false),
        }
    }
}

impl Iterator for SineSweep {
    type Item = f32;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        const SCALE: f32 = 1.0 / (1i64 << 31) as f32;
        self.sweep.next().map(|c| (c.im as f32 * SCALE * *self.amp))
    }
}

impl SineSweep {
    fn trigger(&mut self, depth: usize) -> Result<usize, &'static str> {
        self.sweep = AccuOsc::new(Sweep::new(
            *self.rate,
            ((*self.rate * *self.cycles) as i64) << 32,
        ))
        .take(*self.length);
        self.trigger = Leaf(false);
        Ok(depth)
    }
}

impl Default for OutputChannel {
    fn default() -> Self {
        let mut s = Self {
            state: State::Off.into(),
            voltage_limit: Pwm::MAX_VOLTAGE_LIMIT.into(),
            pid: Pid {
                period: 1.0 / 1007.0,
                max: 0.01.into(),
                min: (-0.01).into(),
                setpoint: 25.0.into(),
                ..Default::default()
            },
            iir: Default::default(),
            iir_state: Default::default(),
            weights: Default::default(),
            sweep: Default::default(),
        };
        s.validate_pid(0).unwrap();
        s.validate_voltage_limit(0).unwrap();
        s.validate_weights(0).unwrap();
        s
    }
}

impl OutputChannel {
    /// compute weighted iir input, iir state and return the new output
    pub fn update(&mut self, temperatures: &[[f64; 4]; 4]) -> f32 {
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
        let y = iir.update(&mut self.iir_state, temperature);
        let s = self.sweep.next().unwrap_or_default();
        y as f32 + s
    }

    fn validate_pid(&mut self, depth: usize) -> Result<usize, &'static str> {
        if let Ok(iir) = self.pid.biquad() {
            self.iir = iir;
        } else {
            return Err("Pid build failure, update not applied.");
        }
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
