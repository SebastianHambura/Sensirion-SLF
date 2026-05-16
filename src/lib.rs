#![no_std]
use anyhow::Result;

use crate::units::sensor_raw_data::{self};

pub mod models;
pub mod slf3_driver;
pub mod types;
pub mod units;

pub use types::{Command, ProductIdentifier, SignalFlags};
#[cfg(feature = "fake_sensor")]
pub mod fake_sensor;

#[allow(non_camel_case_types)] // To keep the same naming as the datasheet
pub enum SensorDriver<Port: embedded_hal::i2c::I2c> {
    SLF3S_0600F(slf3_driver::Slf3sDriver<Port, models::SLF3S_0600F>),
    SLF3S_1300F(slf3_driver::Slf3sDriver<Port, models::SLF3S_1300F>),
}

/// The different commands that can be sent to the sensor. For more details see the datasheet.
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

/// The different information that can be retrieved from the sensor
pub trait SensorInformation {
    fn name(&self) -> &'static str;
    fn address(&self) -> u8;
    fn flow_unit(&self) -> &'static str;
    fn flow_factor(&self) -> f32;
    fn temp_unit(&self) -> &'static str;
    fn temp_factor(&self) -> f32;
}

impl<Port: embedded_hal::i2c::I2c> SensorDriver<Port> {
    pub fn new(port: Port, id: ProductIdentifier) -> Self {
        match id.subtype() {
            0x02 => Self::SLF3S_1300F(slf3_driver::Slf3sDriver::new(port)),
            0x03 => Self::SLF3S_0600F(slf3_driver::Slf3sDriver::new(port)),
            0x04 => panic!("SLF3C-1300F is not supported yet"),
            0x05 => panic!("SLF3S-4000B is not supported yet"),
            _ => panic!(
                "Unknown subtype: {}. Cannot determine sensor variant",
                id.subtype()
            ),
        }
    }
}

/// Forwards the command to the correct driver implementation depending on the sensor variant
macro_rules! dispatch {
    ($self:expr, $var:ident => $expr:expr) => {
        match $self {
            Self::SLF3S_0600F($var) => $expr,
            Self::SLF3S_1300F($var) => $expr,
        }
    };
}

impl<Port: embedded_hal::i2c::I2c> SensorCommunication for SensorDriver<Port> {
    fn read_product_id(&mut self) -> Result<(ProductIdentifier, sensor_raw_data::SerialNumber)> {
        dispatch!(self, d => d.read_product_id())
    }

    fn start_continuous_measurement_water(&mut self) -> Result<()> {
        dispatch!(self, d => d.start_continuous_measurement_water())
    }

    fn start_continuous_measurement_alcohol(&mut self) -> Result<()> {
        dispatch!(self, d => d.start_continuous_measurement_alcohol())
    }

    fn read_measurement(
        &mut self,
    ) -> Result<(
        sensor_raw_data::FlowrateData,
        sensor_raw_data::TemperatureData,
        SignalFlags,
    )> {
        dispatch!(self, d => d.read_measurement())
    }

    fn stop_measurement(&mut self) -> Result<()> {
        dispatch!(self, d => d.stop_measurement())
    }

    fn soft_reset(&mut self) -> Result<()> {
        dispatch!(self, d => d.soft_reset())
    }
}

impl<Port: embedded_hal::i2c::I2c> SensorInformation for SensorDriver<Port> {
    fn name(&self) -> &'static str {
        dispatch!(self, d => d.name())
    }

    fn address(&self) -> u8 {
        dispatch!(self, d => d.address())
    }

    fn flow_unit(&self) -> &'static str {
        dispatch!(self, d => d.flow_unit())
    }

    fn temp_unit(&self) -> &'static str {
        dispatch!(self, d => d.temp_unit())
    }

    fn flow_factor(&self) -> f32 {
        dispatch!(self, d => d.flow_factor())
    }

    fn temp_factor(&self) -> f32 {
        dispatch!(self, d => d.temp_factor())
    }
}
