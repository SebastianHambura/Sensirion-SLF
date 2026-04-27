pub trait Unit {
    const DISPLAY_NAME: &'static str;

    fn display_name(&self) -> &'static str {
        Self::DISPLAY_NAME
    }

    fn from_value(value: f32) -> Self;
    fn value(&self) -> f32;
}
pub mod flow {
    use super::*;
    pub struct UlPerMin(f32);

    impl Unit for UlPerMin {
        const DISPLAY_NAME: &'static str = "μl/min";

        fn from_value(value: f32) -> Self {
            Self(value)
        }

        fn value(&self) -> f32 {
            self.0
        }
    }

    pub struct MlPerMin(f32);

    impl Unit for MlPerMin {
        const DISPLAY_NAME: &'static str = "ml/min";

        fn from_value(value: f32) -> Self {
            Self(value)
        }

        fn value(&self) -> f32 {
            self.0
        }
    }
}

pub mod temp {
    use super::*;
    pub struct Celsius(f32);

    impl Unit for Celsius {
        const DISPLAY_NAME: &'static str = "°C";

        fn from_value(value: f32) -> Self {
            Self(value)
        }

        fn value(&self) -> f32 {
            self.0
        }
    }
}

pub mod sensor_raw_data {
    use super::*;
    pub type FlowrateData = u16;
    pub type TemperatureData = u16;
    pub type SerialNumber = u64;
}
