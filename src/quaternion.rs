#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ()> {
        if bytes.len() != 16 {
            return Err(());
        }

        let w = i32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);

        let x = i32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);

        let y = i32::from_be_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);

        let z = i32::from_be_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]); // as f32 / 16384.0;

        //log::info!("---> {} {} {} {}", w, x, y, z);

        Ok(Self {
            w: w as f32 / 16384.0,
            x: x as f32 / 16384.0,
            y: y as f32 / 16384.0,
            z: z as f32 / 16384.0,
        })
    }

    pub fn magnitude(&self) -> f32 {
        return libm::sqrt(
            (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z) as f64,
        ) as f32;
    }

    pub fn normalize(&self) -> Self {
        let m = self.magnitude();
        Self {
            w: self.w / m,
            x: self.x / m,
            y: self.y / m,
            z: self.z / m,
        }
    }
}
