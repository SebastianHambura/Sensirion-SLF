#![no_std]
use anyhow::{Result, anyhow};
use bitbybit::bitfield;

pub mod slf3;

#[cfg(feature = "fake_sensor")]
pub mod fake_sensor;

pub trait Sensor {
    const ADDRESS: u8;
    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32;
    const TEMPERATURE_SCALE_FACTOR: f32;
    fn read_product_id(&mut self) -> Result<(ProductIdentifier, u64)>;
    fn start_continuous_measurement_water(&mut self) -> Result<()>;
    fn start_continuous_measurement_alcohol(&mut self) -> Result<()>;
    fn read_measurement(&mut self) -> Result<(u16, u16, SignalFlags)>;
    fn stop_measurement(&mut self) -> Result<()>;
    fn soft_reset(&mut self) -> Result<()>;
}

/// According to https://sensirion.com/media/documents/C4F8D965/66F56F53/LQ_DS_SLF3S-0600F_Datasheet.pdf
///
/// Table 9: Bit assignment of 16-bit signaling flags
#[bitfield(u16, debug, default=0)]
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
    /// Revision number (changes with minor firmware or hardware revisions
    #[bits(0..=7, rw)]
    pub revision_number: u8,
}