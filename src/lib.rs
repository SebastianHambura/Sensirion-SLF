#![no_std]
use anyhow::{Result, anyhow};
use bitbybit::bitfield;

use crate::units::sensor_raw_data;

pub mod models;
pub mod slf3_driver;
pub mod units;

#[cfg(feature = "fake_sensor")]
pub mod fake_sensor;

/// The constants that different sensors from this family may have
pub trait Slf3sVariant {
    const NAME: &'static str;

    /// I2C address of the sensor
    const ADDRESS: u8;

    /// Conversion of the liquid flow rate sensor signals to a physical value is done with the scale factor.
    /// The physical value can be calculated as follows: physical_value = raw_value / LIQUID_FLOW_RATE_SCALE_FACTOR
    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32;
    /// The (final) flow unit of the sensor. E.g. ml/min or μl/min
    type FlowUnit;

    /// Conversion of the  temperature sensor signals to a physical value is done with the scale factor.
    /// The physical value can be calculated as follows: physical_value = raw_value / TEMPERATURE_SCALE_FACTOR
    const TEMPERATURE_SCALE_FACTOR: f32;
    /// The (final) temperature unit of the sensor. E.g. °C
    type TempUnit;
}
pub trait SensorCommunication {
    fn read_product_id(&mut self) -> Result<(ProductIdentifier, sensor_raw_data::SerialNumber)>;
    fn start_continuous_measurement_water(&mut self) -> Result<()>;
    fn start_continuous_measurement_alcohol(&mut self) -> Result<()>;
    fn read_measurement(
        &mut self,
    ) -> Result<(
        sensor_raw_data::FlowrateData,
        sensor_raw_data::TemperatureData,
        SignalFlags,
    )>;
    fn stop_measurement(&mut self) -> Result<()>;
    fn soft_reset(&mut self) -> Result<()>;
}

pub trait Slf3sSensor: SensorCommunication + Slf3sVariant {}
impl<T> Slf3sSensor for T where T: SensorCommunication + Slf3sVariant {}

/// According to https://sensirion.com/media/documents/C4F8D965/66F56F53/LQ_DS_SLF3S-0600F_Datasheet.pdf
///
/// Table 9: Bit assignment of 16-bit signaling flags
#[bitfield(u16, debug, default = 0)]
pub struct SignalFlags {
    /// Air-in-Line flag
    #[bit(0, rw)]
    pub air_in_line: bool,
    /// High Flow flag
    #[bit(1, rw)]
    pub high_flow: bool,
    /// Exponential smoothing active
    #[bit(5, rw)]
    pub exponential_smoothing: bool,
}

/// According to https://sensirion.com/media/documents/C4F8D965/66F56F53/LQ_DS_SLF3S-0600F_Datasheet.pdf
///
/// Table 14:  Interpretation of product identifier
#[bitfield(u32, debug)]
pub struct ProductIdentifier {
    /// Liquid flow sensor
    #[bits(24..=31, rw)]
    pub liquid_flow_sensor: u8,
    /// Product family (e.g. SLF3x)
    #[bits(16..=23, rw)]
    pub product_family: u8,
    /// Subtype (e.g. SLF3S-0600F)
    #[bits(8..=15, rw)]
    pub subtype: u8,
    /// Revision number (changes with minor firmware or hardware revisions)
    #[bits(0..=7, rw)]
    pub revision_number: u8,
}
