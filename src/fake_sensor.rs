use crate::{Sensor, SignalFlags};

pub struct FakeSLF3 {
    min_value: u16,
    max_value: u16,
    period: u32,
    current_point: u32,
}

impl FakeSLF3 {
    pub fn new(min_value: u16, max_value: u16, period: u32) -> Self {
        Self {
            min_value,
            max_value,
            period,
            current_point: 0,
        }
    }
}

impl Sensor for FakeSLF3 {
    const ADDRESS: u8 = 0x8;

    const LIQUID_FLOW_RATE_SCALE_FACTOR: f32 = 11.0;

    const TEMPERATURE_SCALE_FACTOR: f32 = 201.0;

    fn read_product_id(&mut self) -> anyhow::Result<(crate::ProductIdentifier, u64)> {
        Ok((
            crate::ProductIdentifier::builder()
                .with_liquid_flow_sensor(42)
                .with_product_family(42)
                .with_subtype(42)
                .with_revision_number(42)
                .build(),
            0xDEADBEEFDEADBEEF,
        ))
    }

    fn start_continuous_measurement_water(&mut self) -> anyhow::Result<()> {
        Ok(())
    }

    fn start_continuous_measurement_alcohol(&mut self) -> anyhow::Result<()> {
        Ok(())
    }

    fn read_measurement(&mut self) -> anyhow::Result<(u16, u16, crate::SignalFlags)> {
        let value = self.min_value as f32
            + ((self.max_value - self.min_value) as f32 * self.current_point as f32
                / self.period as f32);
        let value = value as u16;

        let mut signal_flag = SignalFlags::new_with_raw_value(0);
        signal_flag.set_air_in_line(false);
        signal_flag.set_exponential_smoothing(true);
        signal_flag.set_high_flow(false);

        self.current_point = (self.current_point + 1) % self.period;
        Ok((value, 25000, signal_flag))
    }

    fn stop_measurement(&mut self) -> anyhow::Result<()> {
        Ok(())
    }

    fn soft_reset(&mut self) -> anyhow::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
pub mod tests {
    use crate::Sensor;
    extern crate std;

    #[test]
    fn test_fake_sensor() {
        let mut sensor = super::FakeSLF3::new(1000, 2000, 10);
        for i in 0..20 {
            let (flow, temp, flags) = sensor.read_measurement().unwrap();
            std::println!(
                "Measurement {}: flow = {}, temp = {}, flags = {:?}",
                i,
                flow,
                temp,
                flags
            );
            let expected_flow = 1000 + ((i % 10) * 100);
            assert_eq!(flow, expected_flow);
            assert_eq!(temp, 25000);
            assert_eq!(flags.air_in_line(), false);
            assert_eq!(flags.exponential_smoothing(), true);
            assert_eq!(flags.high_flow(), false);
        }
    }
}
