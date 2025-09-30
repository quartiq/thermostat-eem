use arbitrary_int::u2;
use bitbybit::bitenum;
use miniconf::Tree;
use num_traits::float::Float;
use strum::{AsRefStr, EnumString};

/// DAC value out of bounds error.
#[derive(Debug)]
pub enum Error {
    Bounds,
}

pub const MAX_CURRENT_LIMIT: f32 = 3.0; // As per MAX1968 datasheet. (Pwm::V_PWM * 0.15) / (VREF_TEC * R_SENSE) for 100% duty cycle equivalent.
pub const MAX_VOLTAGE_LIMIT: f32 = 4.3; // As per MAX1968 datasheet

// DAC and PWM shared constants
pub const R_SENSE: f32 = 0.05; // TEC current sense resistor
pub const VREF_TEC: f32 = 1.5; // TEC driver reference voltage

/// A type representing a DAC sample.
#[derive(Copy, Clone, Debug)]
pub struct DacCode(u32);
impl DacCode {
    // DAC constants
    const MAX_DAC_WORD: i32 = 1 << 20; // maximum DAC dataword (exclusive) plus 2 bit due to interface alignment
    const VREF_DAC: f32 = 3.0; // DAC reference voltage
    pub const MAX_CURRENT: f32 = ((DacCode::MAX_DAC_WORD - 1) as f32
        / DacCode::MAX_DAC_WORD as f32
        * DacCode::VREF_DAC
        - VREF_TEC)
        / (10.0 * R_SENSE);
}

impl TryFrom<f32> for DacCode {
    type Error = Error;
    /// Convert an f32 representing a current int the corresponding DAC output code.
    fn try_from(current: f32) -> Result<DacCode, Error> {
        // Current to DAC word conversion
        let ctli_voltage = current * (10.0 * R_SENSE) + VREF_TEC;
        let dac_code = (ctli_voltage
            * (DacCode::MAX_DAC_WORD as f32 / DacCode::VREF_DAC))
            as i32;

        if !(0..DacCode::MAX_DAC_WORD).contains(&dac_code) {
            return Err(Error::Bounds);
        };

        Ok(Self(dac_code as u32))
    }
}

impl From<DacCode> for u32 {
    fn from(code: DacCode) -> u32 {
        code.0
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
#[bitenum(u2, exhaustive = true)]
pub enum AdcPhy {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

impl AdcPhy {
    /// ADC phy readout schedule.
    /// There are 4 physical ADCs present on Thermostat-EEM. Each of them has up to
    /// four individual input channels. The ADCs all share the same clock and sample rate
    /// and read out one after the other in a round-robin fashion. Therefore the sample rate
    /// of a channel depends on how many channels are enabled on an ADC.
    pub fn next(&self) -> Self {
        // Round-robin
        Self::new_with_raw_value(self.raw_value().wrapping_add(u2::from_u8(1)))
    }

    pub const ALL: [Self; 4] = [Self::Zero, Self::One, Self::Two, Self::Three];
}

/// A type representing an ADC sample.
/// Might be extended to support different input types (other NTCs, ref resistors etc.) in the future.
#[derive(Copy, Clone, Debug)]
pub struct AdcCode(u32);

impl From<u32> for AdcCode {
    /// Construct an ADC code from a provided binary (ADC-formatted) code.
    fn from(value: u32) -> Self {
        Self(value)
    }
}

impl From<AdcCode> for u32 {
    fn from(code: AdcCode) -> u32 {
        code.0
    }
}

impl From<AdcCode> for f32 {
    fn from(value: AdcCode) -> Self {
        // Unchanged ADC GAIN and OFFSET registers (default reset values)
        const GAIN: f32 = 0x555555 as _; // Default ADC gain from datasheet.
        // ADC relative full scale per LSB
        // Inverted equation from datasheet p. 40 with V_Ref normalized to 1
        const FS_PER_LSB: f32 =
            0x400000 as f32 / (2.0 * (1 << 23) as f32 * GAIN * 0.75);
        value.0 as Self * FS_PER_LSB
    }
}

pub trait Convert {
    fn convert(&self, code: AdcCode) -> f64;
}

/// Relative_voltage * gain + offset
///
/// Use also for RTD
#[derive(Clone, Copy, Debug, Tree)]
pub struct Linear {
    /// Units: output
    offset: f32,
    /// Units: output/input
    gain: f32,
}

impl Convert for Linear {
    fn convert(&self, code: AdcCode) -> f64 {
        (f32::from(code) * self.gain) as f64 + self.offset as f64
    }
}

impl Default for Linear {
    fn default() -> Self {
        Self {
            offset: 0.0,
            gain: 1.0,
        }
    }
}

/// Beta equation (Steinhart-Hart with c=0)
#[derive(Clone, Copy, Debug, Tree)]
pub struct Ntc {
    t0_inv: f32,   // inverse reference temperature (1/K)
    r_rel: f32,    // reference resistor over NTC resistance at t0,
    beta_inv: f32, // inverse beta
}

impl Ntc {
    pub fn new(t0: f32, r0: f32, r_ref: f32, beta: f32) -> Self {
        Self {
            t0_inv: (1.0 / (t0 + ZERO_C)),
            r_rel: (r_ref / r0),
            beta_inv: (1.0 / beta),
        }
    }
}

impl Convert for Ntc {
    fn convert(&self, code: AdcCode) -> f64 {
        // A f32 output dataformat leads to an output quantization of about 31 uK at T0.
        // Additionally there is some error (in addition to the re-quantization) introduced during the
        // various computation steps. If the input data has less than about 5 bit RMS noise, f32 should be
        // avoided. Input values must not close to minimum/maximum (~1000 codes difference)
        // https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
        let relative_voltage = f32::from(code) as f64;
        let relative_resistance =
            relative_voltage / (1.0 - relative_voltage) * self.r_rel as f64;
        (self.t0_inv as f64 + self.beta_inv as f64 * relative_resistance.ln())
            .recip()
            - ZERO_C as f64
    }
}

impl Default for Ntc {
    fn default() -> Self {
        Self::new(25., 10.0e3, 10.0e3, 3988.)
    }
}

/// DT-670 Silicon diode
#[derive(Clone, Copy, Debug, Tree)]
pub struct Dt670 {
    v_ref: f32, // effective reference voltage (V)
}

impl Default for Dt670 {
    fn default() -> Self {
        Self { v_ref: 2.5 }
    }
}

impl Convert for Dt670 {
    fn convert(&self, code: AdcCode) -> f64 {
        let voltage = f32::from(code) * self.v_ref;
        const CURVE: &[(f32, f32, f32)] = &super::dt670::CURVE;
        // This is clearly simplistic.
        // It is discontinuous at LUT jumps due to dvdt precision.
        // Should use proper interpolation, there are some crates.
        let idx = CURVE.partition_point(|&(_, v, _)| v < voltage);
        let (t, v, dvdt) = CURVE.get(idx).or(CURVE.last()).unwrap();
        (t + (voltage - v) * 1.0e3 / dvdt) as f64
    }
}

/// ADC configuration structure.
#[derive(Clone, Copy, Debug, Tree, EnumString, AsRefStr)]
pub enum Sensor {
    Linear(Linear),
    Ntc(Ntc),
    Dt670(Dt670),
}

const ZERO_C: f32 = 273.15; // 0°C in °K

impl Default for Sensor {
    fn default() -> Self {
        Self::Linear(Linear::default())
    }
}

impl Sensor {
    pub fn convert(&self, code: AdcCode) -> f64 {
        match self {
            Self::Linear(linear) => linear.convert(code),
            Self::Ntc(ntc) => ntc.convert(code),
            Self::Dt670(dt670) => dt670.convert(code),
        }
    }
}

#[derive(Copy, Clone, Debug, serde::Serialize)]
pub enum PoePower {
    /// No Power over Ethernet detected
    Absent,
    /// 802.3af (12.95 W) Power over Ethernet present
    Low,
    /// 802.3at (25.5 W) Power over Ethernet present
    High,
}

impl Default for PoePower {
    fn default() -> Self {
        Self::Absent
    }
}

/// TEC driver PWM frequency setting
#[derive(Copy, Clone, Debug)]
pub enum TecFrequency {
    /// Low frequency (~500 kHz)
    Low,
    /// High frequency (~1 MHz)
    High,
}
