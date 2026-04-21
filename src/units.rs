pub mod flow {
pub struct UlPerMin;
pub struct MlPerMin;
}

pub mod temp {
    pub struct Celsius;
}

pub mod sensor_raw_data {
    pub type FlowrateData = u16; 
    pub type TemperatureData = u16;
    pub type SerialNumber = u64;
}
