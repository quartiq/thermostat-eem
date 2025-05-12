//! # Thermostat_EEM IIR wrapper.
//!

use core::iter::Take;

use crate::{hardware::pwm::Pwm, DacCode};
use idsp::{
    iir, {AccuOsc, Sweep},
};
use miniconf::{Error, Keys, Leaf, Tree, TreeDeserialize};
use num_traits::Float;
use serde::Deserializer;

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
    /// is multiplied by the corresponding weight and the accumulated output is
    /// fed into the IIR.
    pub weights: Leaf<[[f32; 4]; 4]>,

    /// Biquad parameters
    #[tree(typ="Leaf<iir::BiquadReprDiscriminants>", rename="typ",
        with(serialize=self.biquad.tag_serialize, deserialize=self.biquad.tag_deserialize),
        deny(ref_any="deny", mut_any="deny"))]
    _tag: (),

    /// PID/Biquad/IIR filter parameters
    ///
    /// The y limits will be clamped to the maximum output current of +-3 A.
    #[tree(with(deserialize=self.validate_biquad))]
    pub biquad: iir::BiquadRepr<f32, f64>,

    /// Built biquad
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
    #[tree(with(deserialize=self.validate_voltage_limit))]
    pub voltage_limit: Leaf<f32>,
}

#[derive(Clone, Debug, Tree)]
pub struct SineSweep {
    rate: Leaf<i32>,
    state: Leaf<i64>,
    length: Leaf<usize>,
    amp: Leaf<f32>,
    /// Trigger both signal sources
    #[tree(with(deserialize=self.validate_trigger))]
    trigger: Leaf<()>,
    #[tree(skip)]
    sweep: Take<AccuOsc<Sweep>>,
}

impl Default for SineSweep {
    fn default() -> Self {
        Self {
            sweep: AccuOsc::new(Sweep::new(0, 0)).take(0),
            rate: Leaf(0),
            state: Leaf(0),
            length: Leaf(0),
            amp: Leaf(0.0),
            trigger: Leaf(()),
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
    fn validate_trigger<'de, D: Deserializer<'de>, K: Keys>(
        &mut self,
        keys: K,
        de: D,
    ) -> Result<(), Error<D::Error>> {
        self.trigger.deserialize_by_key(keys, de)?;
        self.sweep = AccuOsc::new(Sweep::new(*self.rate, *self.state)).take(*self.length);
        Ok(())
    }
}

impl Default for OutputChannel {
    fn default() -> Self {
        let b = iir::BiquadRepr::Pid(iir::Pid {
            max: Leaf(0.01),
            min: Leaf(-0.01),
            setpoint: Leaf(25.0),
            ..Default::default()
        });
        Self {
            state: Default::default(),
            voltage_limit: Leaf(Pwm::MAX_VOLTAGE_LIMIT),
            _tag: (),
            iir: b.build::<f64>(1007.0.recip(), 1.0, 1.0),
            biquad: b,
            iir_state: Default::default(),
            weights: Default::default(),
            sweep: Default::default(),
        }
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

    fn validate_biquad<'de, D: Deserializer<'de>, K: Keys>(
        &mut self,
        keys: K,
        de: D,
    ) -> Result<(), Error<D::Error>> {
        self.biquad.deserialize_by_key(keys, de)?;
        self.iir = self.biquad.build::<f64>(1007.0.recip(), 1.0, 1.0);
        let range = DacCode::MAX_CURRENT.min(Pwm::MAX_CURRENT_LIMIT);
        self.iir
            .set_max(self.iir.max().clamp(-range as _, range as _));
        self.iir
            .set_min(self.iir.min().clamp(-range as _, range as _));
        Ok(())
    }

    fn validate_voltage_limit<'de, D: Deserializer<'de>, K: Keys>(
        &mut self,
        keys: K,
        de: D,
    ) -> Result<(), Error<D::Error>> {
        self.voltage_limit.deserialize_by_key(keys, de)?;
        *self.voltage_limit = (*self.voltage_limit).clamp(0.0, Pwm::MAX_VOLTAGE_LIMIT);
        Ok(())
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
