/// Raw temperature reading
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

    pub fn raw(&self) -> i16 {
        self.raw
    }

    /// Convert raw temperature to degrees Celsius
    /// Formula from datasheet: Temperature = (TEMP_OUT)/340 + 36.53
    pub fn celsius(&self) -> f32 {
        (self.raw as f32) / 340.0 + 36.53
    }
}
