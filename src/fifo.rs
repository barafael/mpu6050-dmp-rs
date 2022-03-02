#[derive(Debug, Default, Copy, Clone)]
pub struct Fifo {
    pub temp: bool,
    pub xg: bool,
    pub yg: bool,
    pub zg: bool,
    pub accel: bool,
    pub slv2: bool,
    pub slv1: bool,
    pub slv0: bool,
}

impl Fifo {
    pub fn all_disabled() -> Self {
        Self::default()
    }

    pub(crate) fn from_byte(byte: u8) -> Self {
        Self {
            temp: (byte & 0b1000_0000) != 0,
            xg: (byte & 0b0100_0000) != 0,
            yg: (byte & 0b0010_0000) != 0,
            zg: (byte & 0b0001_0000) != 0,
            accel: (byte & 0b0000_1000) != 0,
            slv2: (byte & 0b0000_0100) != 0,
            slv1: (byte & 0b0000_0010) != 0,
            slv0: (byte & 0b0000_0001) != 0,
        }
    }

    pub(crate) fn to_byte(&self) -> u8 {
        let mut byte = 0;
        if self.temp {
            byte |= 1 << 7;
        }
        if self.xg {
            byte |= 1 << 6;
        }
        if self.yg {
            byte |= 1 << 5;
        }
        if self.zg {
            byte |= 1 << 4;
        }
        if self.accel {
            byte |= 1 << 3;
        }
        if self.slv2 {
            byte |= 1 << 2;
        }
        if self.slv1 {
            byte |= 1 << 1;
        }
        if self.slv0 {
            byte |= 1 << 0;
        }

        byte
    }
}
