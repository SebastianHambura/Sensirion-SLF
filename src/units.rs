pub trait Unit: From<f32> + Into<f32> {
    const DISPLAY_NAME: &'static str;

    fn display_name(&self) -> &'static str {
        Self::DISPLAY_NAME
    }
}

pub mod flow {
    use super::*;
    pub struct UlPerMin(f32);

    impl Unit for UlPerMin {
        const DISPLAY_NAME: &'static str = "ul/min"; // μl/min might not be well supported in all environments, so using ul/min instead
    }

    impl From<f32> for UlPerMin {
        fn from(value: f32) -> Self {
            Self(value)
        }
    }

    impl From<UlPerMin> for f32 {
        fn from(val: UlPerMin) -> Self {
            val.0
        }
    }

    pub struct MlPerMin(f32);

    impl Unit for MlPerMin {
        const DISPLAY_NAME: &'static str = "ml/min";
    }

    impl From<f32> for MlPerMin {
        fn from(value: f32) -> Self {
            Self(value)
        }
    }

    impl From<MlPerMin> for f32 {
        fn from(val: MlPerMin) -> Self {
            val.0
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
            Self(value)
        }
    }

    impl From<Celsius> for f32 {
        fn from(value: Celsius) -> Self {
            value.0
        }
    }
}

pub mod sensor_raw_data {
    pub type FlowrateData = i16;
    pub type TemperatureData = i16;
    pub type SerialNumber = u64;
}
