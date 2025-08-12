//! Accelerometer Data Processing
//!
//! The MPU6050's accelerometer measures linear acceleration along three axes:
//! - X: Forward/backward motion
//! - Y: Left/right motion
//! - Z: Up/down motion (including gravity)
//!
//! Raw readings are in ADC units and must be scaled according to the
//! configured full-scale range to get actual g-force values.

/// Raw acceleration readings from the sensor's ADC.
///
/// This structure represents:
/// - Raw sensor readings (when reading acceleration)
/// - Calibration offsets (when used for calibration)
/// - Values are in ADC units (-32768 to +32767)
/// - Must be scaled by full-scale range for g-force values
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub struct Accel {
    pub(crate) x: i16,
    pub(crate) y: i16,
    pub(crate) z: i16,
}

impl Accel {
    pub const fn new(x: i16, y: i16, z: i16) -> Self {
        Self { x, y, z }
    }

    /// Converts raw sensor bytes into acceleration values.
    ///
    /// The MPU6050 provides 16-bit acceleration values as:
    /// - 2 bytes per axis (high byte, low byte)
    /// - Big-endian byte order
    /// - Signed integers (-32768 to +32767)
    pub const fn from_bytes(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_be_bytes(x),
            y: i16::from_be_bytes(y),
            z: i16::from_be_bytes(z),
        }
    }

    pub const fn to_bytes(&self) -> [u8; 6] {
        let x = self.x.to_be_bytes();
        let y = self.y.to_be_bytes();
        let z = self.z.to_be_bytes();
        [x[0], x[1], y[0], y[1], z[0], z[1]]
    }

    pub const fn x(&self) -> i16 {
        self.x
    }

    pub const fn y(&self) -> i16 {
        self.y
    }

    pub const fn z(&self) -> i16 {
        self.z
    }

    /// Converts raw ADC values to g-force units.
    ///
    /// The conversion process:
    /// 1. Takes raw ADC values (-32768 to +32767)
    /// 2. Divides by scale factor based on full-scale range
    /// 3. Results in g-force values (e.g., ±2g, ±4g, etc.)
    pub const fn scaled(&self, scale: AccelFullScale) -> AccelF32 {
        AccelF32 {
            x: scale.scale_value(self.x),
            y: scale.scale_value(self.y),
            z: scale.scale_value(self.z),
        }
    }
}

/// Full-scale range settings for the accelerometer.
///
/// Each setting provides different sensitivity:
/// - G2: ±2g range (most sensitive, best for small movements)
/// - G4: ±4g range
/// - G8: ±8g range
/// - G16: ±16g range (least sensitive, best for high-impact movements)
///
/// The range affects:
/// - Sensitivity (LSB/g) - higher range = lower sensitivity
/// - Resolution - lower range = better resolution
/// - Maximum measurable acceleration
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub enum AccelFullScale {
    /// ±2g range (16384 LSB/g)
    G2 = 0,
    /// ±4g range (8192 LSB/g)
    G4 = 1,
    /// ±8g range (4096 LSB/g)
    G8 = 2,
    /// ±16g range (2048 LSB/g)
    G16 = 3,
}

impl AccelFullScale {
    pub const fn scale(self) -> f32 {
        match self {
            Self::G2 => 16384.0,
            Self::G4 => 8192.0,
            Self::G8 => 4096.0,
            Self::G16 => 2048.0,
        }
    }

    pub const fn scale_value(self, value: i16) -> f32 {
        (value as f32) / self.scale()
    }
}

/// Acceleration values in g-force units.
///
/// This structure holds accelerometer data after conversion from
/// raw ADC values to actual g-forces. Values typically range:
/// - ±2g in G2 mode
/// - ±4g in G4 mode
/// - ±8g in G8 mode
/// - ±16g in G16 mode
///
/// Note: The Z axis will read ~1g when stationary due to gravity
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub struct AccelF32 {
    /// X-axis acceleration in g-force
    x: f32,
    /// Y-axis acceleration in g-force  
    y: f32,
    /// Z-axis acceleration in g-force
    z: f32,
}

impl AccelF32 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const fn x(&self) -> f32 {
        self.x
    }

    pub const fn y(&self) -> f32 {
        self.y
    }

    pub const fn z(&self) -> f32 {
        self.z
    }
}
