// Thermostat unit and data description conversion
// TODO: Refactor to use the rust type system like stabilizer
use core::f32;
use num_traits::float::Float;

// ADC constants
const GAIN: u32 = 0x555555; // default ADC gain from datasheet
const R_INNER: f32 = 2.0 * 5100.0; // ratiometric resistor setup. 5.1k high and low side.

// Steinhart-Hart Parameters
const ZEROK: f32 = 273.15; // 0°C in °K
const B: f32 = 3988.0; // NTC beta value
const T_N_INV: f32 = 1.0 / (25.0 + ZEROK); // T_n = 25°C
const R_N: f32 = 10000.0; // TEC resistance at 25°C

// PWM constants
const V_PWM: f32 = 3.3; // MCU PWM pin output high voltage

// DAC constants
const R_SENSE: f32 = 0.05; // TEC current sense resistor
pub const VREF_TEC: f32 = 1.5; // TEC driver reference voltage
const MAXCODE: f32 = (1 << 18) as _; // maximum DAC dataword
const VREF_OS: f32 = 0.0; // Device specific offset voltage for zero current at half dac scale
pub const VREF_DAC: f32 = 3.0 + VREF_OS; // DAC reference voltage target plus offset

// IIR constants
const SCALE: f32 = (1 << 23) as _; // half the ADC maximum dataword

/// Convert raw adc code to temperature in °C.
pub fn adc_to_temp(adc: u32) -> f32 {
    // raw to R
    let data = (adc as f32) * (0.5 * 0x400000 as f32 / GAIN as f32);
    let vin = data as f32 / (0.75 * SCALE);
    let r = (R_INNER as f32) / ((1.0 / vin) - 1.0);

    // R to T (°C)
    let t_inv = T_N_INV + (1.0 / B) * (r / R_N).ln();
    (1.0 / t_inv) - ZEROK
}

/// Convert TEC drive current to dac code.
pub fn i_to_dac(i: f32) -> u32 {
    let v = (i * 10.0 * R_SENSE) + VREF_TEC;
    ((v * MAXCODE) / VREF_DAC) as u32
}

/// Convert dac code to TEC drive current.
pub fn dac_to_i(val: u32) -> f32 {
    let v = VREF_DAC * (val as f32 / MAXCODE);
    (v - VREF_TEC) / (10.0 * R_SENSE)
}

/// Convert maximum current to relative pulsewidth for the (analog voltage)
/// max output current inputs of the TEC driver.
pub fn i_to_pwm(i: f32) -> f32 {
    i * ((10.0 * R_SENSE) / V_PWM)
}

/// Convert maximum voltage to relative pulsewidth for the (analog voltage)
/// max output voltage of the TEC driver.
pub fn v_to_pwm(v: f32) -> f32 {
    v / (4.0 * V_PWM)
}

/// Convert a temperature in °C to an effective adc code. This can be used to
/// compute an effective input iir offset.
pub fn temp_to_iiroffset(temp: f32) -> f32 {
    // T (°C) to R
    let t_inv = 1.0 / (temp + ZEROK);
    let r = R_N * (B * (t_inv - T_N_INV)).exp();

    // R to raw
    let v = r / (R_INNER + r);
    let data = 0.75 * SCALE * v;
    (-data * GAIN as f32) / (0.5 * 0x400000 as f32)
}
