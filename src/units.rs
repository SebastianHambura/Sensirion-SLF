pub trait Unit: From<f32> {
    const DISPLAY_NAME: &'static str;
}
pub mod flow {
    use super::*;
    pub struct UlPerMin(f32);

    impl Unit for UlPerMin {
        const DISPLAY_NAME: &'static str = "μl/min";
    }

    impl From<f32> for UlPerMin {
        fn from(value: f32) -> Self {
            UlPerMin(value)
        }
    }

    pub struct MlPerMin(f32);

    impl Unit for MlPerMin {
        const DISPLAY_NAME: &'static str = "ml/min";
    }

    impl From<f32> for MlPerMin {
        fn from(value: f32) -> Self {
            MlPerMin(value)
        }
    }
}

pub mod temp {
    use super::*;
    pub struct Celsius(f32);

    impl Unit for Celsius {
        const DISPLAY_NAME: &'static str = "°C";
    }

    impl From<f32> for Celsius {
        fn from(value: f32) -> Self {
            Celsius(value)
        }
    }
}

pub mod sensor_raw_data {
    use super::*;
    pub type FlowrateData = u16;
    pub type TemperatureData = u16;
    pub type SerialNumber = u64;
}
