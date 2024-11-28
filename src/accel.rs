/// Raw acceleration readings vector.
/// Also used to represent acceleration calibration offsets.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Accel {
    pub(crate) x: i16,
    pub(crate) y: i16,
    pub(crate) z: i16,
}

impl Accel {
    pub fn new(x: i16, y: i16, z: i16) -> Self {
        Self { x, y, z }
    }

    pub fn from_bytes(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_be_bytes(x),
            y: i16::from_be_bytes(y),
            z: i16::from_be_bytes(z),
        }
    }

    pub fn to_bytes(&self) -> [u8; 6] {
        let x = self.x.to_be_bytes();
        let y = self.y.to_be_bytes();
        let z = self.z.to_be_bytes();
        [x[0], x[1], y[0], y[1], z[0], z[1]]
    }

    pub fn x(&self) -> i16 {
        self.x
    }

    pub fn y(&self) -> i16 {
        self.y
    }

    pub fn z(&self) -> i16 {
        self.z
    }

    pub fn scaled(&self, scale: AccelFullScale) -> AccelF32 {
        AccelF32 {
            x: scale.scale_value(self.x),
            y: scale.scale_value(self.y),
            z: scale.scale_value(self.z),
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum AccelFullScale {
    G2 = 0,
    G4 = 1,
    G8 = 2,
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

    pub fn scale_value(self, value: i16) -> f32 {
        (value as f32) / self.scale()
    }
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[derive(Debug, Clone, Copy)]
pub struct AccelF32 {
    x: f32,
    y: f32,
    z: f32,
}

impl AccelF32 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn x(&self) -> f32 {
        self.x
    }

    pub fn y(&self) -> f32 {
        self.y
    }

    pub fn z(&self) -> f32 {
        self.z
    }
}
