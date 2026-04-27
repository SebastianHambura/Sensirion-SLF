#![allow(non_camel_case_types)]
use super::Slf3sVariant;

/// TODO: Add link to datasheet
pub struct SLF3S_0600F;

impl Slf3sVariant for SLF3S_0600F {
    const NAME: &'static str = "SLF3S-0600F";    
    const ADDRESS: u8 = 0x8;
    /// SLF3S-0600F : 10 (μl/min)-1
    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32 = 10.0;
    ///  SLF3S-0600F : 200 °C-1
    const TEMPERATURE_SCALE_FACTOR: f32 = 200.0;

    type FlowUnit = super::units::flow::UlPerMin;
    type TempUnit = super::units::temp::Celsius;
}

/// TODO: Add link to datasheet
pub struct SLF3S_1300F;

impl Slf3sVariant for SLF3S_1300F {
    const NAME: &'static str = "SLF3S-1300F";
    const ADDRESS: u8 = 0x8;

    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32 = 500.0;

    const TEMPERATURE_SCALE_FACTOR: f32 = 200.0;

    type FlowUnit = super::units::flow::MlPerMin;
    type TempUnit = super::units::temp::Celsius;
}
