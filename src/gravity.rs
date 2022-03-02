use crate::quaternion::Quaternion;

#[derive(Debug, Copy, Clone)]
pub struct Gravity {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Gravity {}

impl From<Quaternion> for Gravity {
    fn from(q: Quaternion) -> Self {
        Self {
            x: 2.0 * (q.x * q.z - q.w * q.y),
            y: 2.0 * (q.w * q.x + q.y * q.z),
            z: q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
        }
    }
}
