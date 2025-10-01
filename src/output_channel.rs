//! # Thermostat_EEM IIR wrapper.
//!

use core::{iter::Take, ops::Range};

use crate::convert::{DacCode, MAX_CURRENT_LIMIT, MAX_VOLTAGE_LIMIT};
use heapless::String;
use idsp::{
    iir, {AccuOsc, Sweep},
};
use miniconf::{Leaf, Tree};
use num_traits::Float;

#[derive(
    Copy,
    Clone,
    Default,
    Debug,
    PartialEq,
    PartialOrd,
    serde::Serialize,
    serde::Deserialize,
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
    #[tree(with=miniconf::leaf)]
    pub weights: [[f32; 4]; 4],

    /// Biquad parameters
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.biquad)]
    _typ: (),

    /// PID/Biquad/IIR filter parameters
    ///
    /// The y limits will be clamped to the maximum output current of +-3 A.
    #[tree(with=validate_biquad, defer=*self)]
    pub biquad: iir::BiquadRepr<f32, f64>,

    /// Built biquad
    #[tree(skip)]
    pub iir: iir::Biquad<f64>,

    #[tree(skip)]
    iir_state: [f64; 4],

    #[tree(with=miniconf::leaf)]
    pub state: State,

    pub sweep: SineSweep,

    /// Maximum absolute (positive and negative) TEC voltage in volt.
    /// These will be clamped to the maximum of 4.3 V.
    ///
    /// # Value
    /// 0.0 to 4.3
    #[tree(with=validate_voltage_limit)]
    pub voltage_limit: f32,
}

#[derive(Clone, Debug, Tree)]
pub struct SineSweep {
    rate: i32,
    state: i64,
    length: usize,
    amp: f32,
    /// Trigger both signal sources
    #[tree(with=validate_trigger, defer=*self)]
    trigger: (),
    #[tree(skip)]
    sweep: Take<AccuOsc<Sweep>>,
}

impl Default for SineSweep {
    fn default() -> Self {
        Self {
            sweep: AccuOsc::new(Sweep::new(0, 0)).take(0),
            rate: 0,
            state: 0,
            length: 0,
            amp: 0.0,
            trigger: (),
        }
    }
}

impl Iterator for SineSweep {
    type Item = f32;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        const SCALE: f32 = 1.0 / (1i64 << 31) as f32;
        self.sweep.next().map(|c| c.im as f32 * SCALE * self.amp)
    }
}

mod validate_trigger {
    use super::SineSweep;
    use idsp::{AccuOsc, Sweep};
    use miniconf::{
        Deserializer, Keys, SerdeError, Serializer, TreeDeserialize,
    };

    pub use miniconf::{
        deny::{mut_any_by_key, ref_any_by_key},
        leaf::SCHEMA,
    };

    pub fn serialize_by_key<S: Serializer>(
        value: &SineSweep,
        keys: impl Keys,
        ser: S,
    ) -> Result<S::Ok, SerdeError<S::Error>> {
        miniconf::leaf::serialize_by_key(&value.trigger, keys, ser)
    }

    pub fn deserialize_by_key<'de, D: Deserializer<'de>>(
        value: &mut SineSweep,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        value.trigger.deserialize_by_key(keys, de)?;
        value.sweep = AccuOsc::new(Sweep::new(value.rate, value.state))
            .take(value.length);
        Ok(())
    }

    pub fn probe_by_key<'de, T: ?Sized, D: Deserializer<'de>>(
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        <()>::probe_by_key(keys, de)
    }
}

impl Default for OutputChannel {
    fn default() -> Self {
        let b = iir::BiquadRepr::Pid(iir::Pid {
            max: 0.01,
            min: -0.01,
            setpoint: 25.0,
            ..Default::default()
        });
        Self {
            state: Default::default(),
            voltage_limit: MAX_VOLTAGE_LIMIT,
            _typ: (),
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
        let iir = if self.state == State::On {
            &self.iir
        } else {
            &iir::Biquad::HOLD
        };
        let y = iir.update(&mut self.iir_state, temperature);
        let s = self.sweep.next().unwrap_or_default();
        y as f32 + s
    }

    pub fn current_limits(&self) -> [f32; 2] {
        [
            // give 5% extra headroom for PWM current limits
            // [Pwm::MAX_CURRENT_LIMIT] + 5% is still below 100% duty cycle for the PWM limits and therefore OK.
            // Might not be OK for a different shunt resistor or different PWM setup.
            (self.iir.max() as f32 + 0.05 * MAX_CURRENT_LIMIT).max(0.),
            (self.iir.min() as f32 - 0.05 * MAX_CURRENT_LIMIT).min(0.),
        ]
    }

    pub fn error(&self) -> f32 {
        (self.iir_state[0] as f32 + self.iir_state[1] as f32) * 0.5
            + self.iir.input_offset() as f32
    }
}

mod validate_biquad {
    use super::{DacCode, MAX_CURRENT_LIMIT, OutputChannel, iir};
    use miniconf::{
        Deserializer, Keys, Schema, SerdeError, Serializer, TreeDeserialize,
        TreeSchema, TreeSerialize,
    };

    pub use miniconf::deny::{mut_any_by_key, ref_any_by_key};
    pub const SCHEMA: &'static Schema = iir::BiquadRepr::<f32, f64>::SCHEMA;

    pub fn serialize_by_key<S: Serializer>(
        value: &OutputChannel,
        keys: impl Keys,
        ser: S,
    ) -> Result<S::Ok, SerdeError<S::Error>> {
        value.biquad.serialize_by_key(keys, ser)
    }

    pub fn deserialize_by_key<'de, D: Deserializer<'de>>(
        value: &mut OutputChannel,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        value.biquad.deserialize_by_key(keys, de)?;
        value.iir = value.biquad.build::<f64>(1007.0f32.recip(), 1.0, 1.0);
        let range = DacCode::MAX_CURRENT.min(MAX_CURRENT_LIMIT);
        value
            .iir
            .set_max(value.iir.max().clamp(-range as _, range as _));
        value
            .iir
            .set_min(value.iir.min().clamp(-range as _, range as _));
        Ok(())
    }

    pub fn probe_by_key<'de, T: ?Sized, D: Deserializer<'de>>(
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        iir::BiquadRepr::<f32, f64>::probe_by_key(keys, de)
    }
}

mod validate_voltage_limit {
    use crate::convert::MAX_VOLTAGE_LIMIT;
    use miniconf::{Deserializer, Keys, SerdeError, TreeDeserialize};

    pub use miniconf::{
        deny::{mut_any_by_key, ref_any_by_key},
        leaf::{SCHEMA, probe_by_key, serialize_by_key},
    };

    pub fn deserialize_by_key<'de, D: Deserializer<'de>>(
        value: &mut f32,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        let mut new = *value;
        new.deserialize_by_key(keys, de)?;
        *value = new.clamp(0.0, MAX_VOLTAGE_LIMIT);
        Ok(())
    }
}

/// Miniconf settings for the MQTT alarm.
/// The alarm simply publishes "false" onto its `target` as long as all the channels are
/// within their `temperature_limits`` (aka logical OR of all channels).
/// Otherwise it publishes "true" (aka true, there is an alarm).
///
/// The publishing interval is given by `period_ms`.
///
/// The alarm is non-latching. If alarm was "true" for a while and the temperatures come within
/// limits again, alarm will be "false" again.
#[derive(Clone, Debug, Tree)]
pub struct Alarm {
    /// Alarm target.
    /// The alarm will publish its state (true or false) onto this mqtt path.
    /// Full path to the desired target. No wildcards. Or `None` to disable.
    #[tree(with=miniconf::leaf)]
    pub target: Option<String<128>>,

    /// Alarm period in seconds.
    /// The alarm will publish its state with this period.
    pub period: f32,

    /// Temperature limits for the alarm.
    ///
    /// Array of lower and upper limits for the valid temperature range of the alarm.
    /// The alarm will be asserted if any of the enabled input channels goes below its minimum or above its maximum temperature.
    /// The alarm is non latching and clears itself once all channels are in their respective limits.
    ///
    /// `temperature_limits/<adc>/<channel>`
    pub temperature_limits: [[Leaf<Option<Range<f32>>>; 4]; 4],
}

impl Default for Alarm {
    fn default() -> Self {
        Self {
            target: Default::default(),
            period: 1.0,
            temperature_limits: Default::default(),
        }
    }
}
