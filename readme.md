A small library to communicate with the Sensirion SLF sensors. 

It gives a common interface to this sensor family. Currently supports the following sensors: 
- [SLF3S_0600F](https://sensirion.com/products/catalog/SLF3S-0600F)
- [SLF3S_1300F](https://sensirion.com/products/catalog/SLF3S-1300F)

The communication to the sensor is done with the [sensirion-i2c](https://crates.io/crates/sensirion-i2c) crate.

Here is how you could use it: 
```rust
let i2c = ... // get the embedded_hal::i2c::I2c port where the sensor is connected
let mut slf_sensor: Slf3sDriver<_, SLF3S_0600F> = sensirion_SLF::slf3_driver::Slf3sDriver::new(i2c);

slf_sensor.soft_reset()?; // don't forget to wait for a bit before sending the next command to the sensor!
let (id, serial_number) = slf_sensor.read_product_id()? ; // you could use the id to make sure you have the correct sensor model
let (raw_flow, raw_temp, flags) = slf_sensor.read_measurement()?;

// Convert from raw i16 into physical units
let flow: f32 = raw_flow.into();
let real_flow = flow / slf_sensor.flow_factor();

let temp: f32 = temp.into() ; 
let real_temp = temp / slf_sensor.temp_factor();

// retrieve access to the i2c port
let i2c = slf_sensor.into_inner()
```
