#![allow(non_camel_case_types)]
use crate::units::{Unit, flow, temp};

/// The constants that different sensors from this family may have
pub trait Slf3sVariant {
    const NAME: &'static str;

    /// I2C address of the sensor
    const ADDRESS: u8;

    /// Conversion of the liquid flow rate sensor signals to a physical value is done with the scale factor.
    /// The physical value can be calculated as follows: physical_value = raw_value / LIQUID_FLOW_RATE_SCALE_FACTOR
    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32;
    /// The (final) flow unit of the sensor. E.g. ml/min or μl/min
    type FlowUnit: Unit;

    /// Conversion of the  temperature sensor signals to a physical value is done with the scale factor.
    /// The physical value can be calculated as follows: physical_value = raw_value / TEMPERATURE_SCALE_FACTOR
    const TEMPERATURE_SCALE_FACTOR: f32;
    /// The (final) temperature unit of the sensor. E.g. °C
    type TempUnit: Unit;
}

// === Concrete sensor models ===

/// https://sensirion.com/products/catalog/SLF3S-0600F
pub struct SLF3S_0600F;

impl Slf3sVariant for SLF3S_0600F {
    const NAME: &'static str = "SLF3S-0600F";
    const ADDRESS: u8 = 0x8;
    /// SLF3S-0600F : 10 (μl/min)-1
    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32 = 10.0;
    ///  SLF3S-0600F : 200 °C-1
    const TEMPERATURE_SCALE_FACTOR: f32 = 200.0;

    type FlowUnit = flow::UlPerMin;
    type TempUnit = temp::Celsius;
}

/// https://sensirion.com/products/catalog/SLF3S-1300F
pub struct SLF3S_1300F;

impl Slf3sVariant for SLF3S_1300F {
    const NAME: &'static str = "SLF3S-1300F";
    const ADDRESS: u8 = 0x8;

    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32 = 500.0;

    const TEMPERATURE_SCALE_FACTOR: f32 = 200.0;

    type FlowUnit = flow::MlPerMin;
    type TempUnit = temp::Celsius;
}
