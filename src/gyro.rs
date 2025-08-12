//! Gyroscope Data Processing
//!
//! The MPU6050's gyroscope measures angular velocity (rotation speed)
//! around three axes:
//! - X: Roll rate (side-to-side rotation)
//! - Y: Pitch rate (forward/backward rotation)
//! - Z: Yaw rate (horizontal rotation)

/// Raw gyroscope readings from the sensor.
/// Values represent rotation rate in ADC units.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub struct Gyro {
    pub(crate) x: i16,
    pub(crate) y: i16,
    pub(crate) z: i16,
}

impl Gyro {
    pub const fn new(x: i16, y: i16, z: i16) -> Self {
        Self { x, y, z }
    }

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

    pub const fn scaled(&self, scale: GyroFullScale) -> GyroF32 {
        GyroF32 {
            x: scale.scale_value(self.x),
            y: scale.scale_value(self.y),
            z: scale.scale_value(self.z),
        }
    }
}

impl From<Gyro> for [i16; 3] {
    fn from(value: Gyro) -> Self {
        [value.x, value.y, value.z]
    }
}

/// Full-scale range settings for the gyroscope.
///
/// Each setting defines the maximum measurable rotation rate:
/// - Deg250: ±250 degrees/second
/// - Deg500: ±500 degrees/second
/// - Deg1000: ±1000 degrees/second
/// - Deg2000: ±2000 degrees/second
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub enum GyroFullScale {
    /// ±250°/s range (131 LSB/°/s)
    Deg250 = 0,
    /// ±500°/s range (65.5 LSB/°/s)
    Deg500 = 1,
    /// ±1000°/s range (32.8 LSB/°/s)
    Deg1000 = 2,
    /// ±2000°/s range (16.4 LSB/°/s)
    Deg2000 = 3,
}

impl GyroFullScale {
    pub const fn scale(self) -> f32 {
        match self {
            Self::Deg250 => 131.0,
            Self::Deg500 => 65.5,
            Self::Deg1000 => 32.8,
            Self::Deg2000 => 16.4,
        }
    }

    pub const fn scale_value(self, value: i16) -> f32 {
        (value as f32) / self.scale()
    }
}

/// Gyroscope readings in degrees per second.
///
/// After scaling, values represent actual rotation rates:
/// - Positive: Clockwise rotation
/// - Negative: Counter-clockwise rotation
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub struct GyroF32 {
    /// Roll rate (°/s)
    x: f32,
    /// Pitch rate (°/s)
    y: f32,
    /// Yaw rate (°/s)
    z: f32,
}

impl GyroF32 {
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

impl From<GyroF32> for [f32; 3] {
    fn from(value: GyroF32) -> Self {
        [value.x, value.y, value.z]
    }
}
