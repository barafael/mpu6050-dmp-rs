#[derive(Copy, Clone, Debug)]
pub struct Accel {
    x: i16,
    y: i16,
    z: i16,
}

impl Accel {
    pub(crate) fn new(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_be_bytes(x),
            y: i16::from_be_bytes(y),
            z: i16::from_be_bytes(z),
        }
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
pub enum AccelFullScale {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelFullScale {
    pub const fn scale(self) -> f32 {
        match self {
            AccelFullScale::G2 => 16384.0,
            AccelFullScale::G4 => 8192.0,
            AccelFullScale::G8 => 4096.0,
            AccelFullScale::G16 => 2048.0,
        }
    }

    pub fn scale_value(self, value: i16) -> f32 {
        (value as f32) / self.scale()
    }
}

pub struct AccelF32 {
    x: f32,
    y: f32,
    z: f32,
}

impl AccelF32 {
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
