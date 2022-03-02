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
}

#[derive(Copy, Clone, Debug)]
pub enum AccelFullScale {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}
