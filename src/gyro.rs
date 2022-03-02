#[derive(Copy, Clone, Debug)]
pub struct Gyro {
    x: i16,
    y: i16,
    z: i16,
}

impl Gyro {
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
}

struct GyroBiases {
    x: i16,
    y: i16,
    z: i16,
}

#[derive(Copy, Clone, Debug)]
pub enum GyroFullScale {
    Deg250 = 0,
    Deg500 = 1,
    Deg1000 = 2,
    Deg2000 = 3,
}
