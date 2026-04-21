pub mod flow {
pub type UlPerMin = f32;
pub type MlPerMin = f32;
}

pub mod temp {
    pub type Celsius = f32;
}

pub mod sensor_raw_data {
    pub type FlowrateData = u16; 
    pub type TemperatureData = u16;
    pub type SerialNumber = u64;
}
