use bitbybit::bitfield;

/// According to https://sensirion.com/media/documents/C4F8D965/66F56F53/LQ_DS_SLF3S-0600F_Datasheet.pdf
///
/// 4.2 I2C Sequences
/// The commands are 16-bit.
#[repr(u16)]
#[derive(Clone, Copy)]
pub enum Command {
    /// This command starts the continuous measurement
    /// mode for H2O. Outputs are the liquid flow rate, the
    /// sensor’s temperature and the signaling flags.
    ContinuousMeasurementWater = 0x3608,
    /// This command starts the continuous measurement
    /// mode for IPA. Outputs are the liquid flow rate, the
    /// sensor’s temperature and the signaling flags.
    ContinuousMeasurementIsopropylAlcohol = 0x3615,

    /// This command stops the continuous measurement and
    /// puts the sensor in idle mode. After it receives the stop
    /// command, the sensor needs up to 0.5 ms to power
    /// down the heater, enter idle mode and be receptive for a
    /// new command.
    StopContinuousMeasurment = 0x3FF9,

    /// This sequence resets the sensor with a separate reset
    /// block, which is as much as possible detached from the
    /// rest of the system on chip.
    /// **Note that the I2C address is 0x00, which is the general call
    /// address, and that the command is 8-bit**, i.e., the soft reset
    /// command must not be preceded by an I2C write header.
    /// The reset is implemented according to the I2C
    /// specification
    GeneralCallReset = 0x0006,

    ReadProductIdentifier1 = 0x367C,
    ReadProductIdentifier2 = 0xE102,
}

impl Command {
    /// Returns a big endian byte representation of the command.
    pub fn to_be_bytes(&self) -> [u8; 2] {
        (*self as u16).to_be_bytes()
    }
}

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