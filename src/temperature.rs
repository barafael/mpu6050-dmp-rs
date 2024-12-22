/// Temperature reading from the MPU-6050's internal temperature sensor.
///
/// Note: This measures the temperature of the MPU-6050 chip itself, not the ambient
/// room temperature. The sensor readings will typically be a few degrees higher than
/// the ambient temperature due to self-heating of the device during operation.
///
/// The raw temperature value from the sensor needs to be converted using a formula
/// from the datasheet to get the actual temperature in degrees Celsius.
/// While the raw value is available via [`raw()`], you should typically use [`celsius()`]
/// to get the temperature in a standard unit.
///
/// # Example
/// ```
/// # use mpu6050_dmp::temperature::Temperature;
/// let temp = Temperature::new(3990);
///
/// // Get temperature in Celsius (recommended)
/// let celsius = temp.celsius(); // Returns ~48.26°C
///
/// // Get raw value (typically not needed)
/// let raw = temp.raw(); // Returns 3990
/// ```
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Temperature {
    pub(crate) raw: i16,
}

impl Temperature {
    pub fn new(raw: i16) -> Self {
        Self { raw }
    }

    pub fn from_bytes(data: [u8; 2]) -> Self {
        Self {
            raw: i16::from_be_bytes(data),
        }
    }

    pub fn to_bytes(&self) -> [u8; 2] {
        self.raw.to_be_bytes()
    }

    /// Returns the raw temperature value from the sensor.
    ///
    /// Note: In most cases, you should use [`celsius()`] instead to get the temperature
    /// in degrees Celsius. The raw value is primarily useful for debugging or
    /// custom temperature conversion implementations.
    pub fn raw(&self) -> i16 {
        self.raw
    }

    /// Convert raw temperature to degrees Celsius
    /// Formula from datasheet: Temperature = (TEMP_OUT)/340 + 36.53
    pub fn celsius(&self) -> f32 {
        (self.raw as f32) / 340.0 + 36.53
    }
}
