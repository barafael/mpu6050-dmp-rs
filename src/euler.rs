use crate::quaternion::Quaternion;

#[derive(Debug, Clone, Copy)]
pub struct Euler {
    pub psi: f64,
    pub theta: f64,
    pub phi: f64,
}

impl Euler {}

impl From<Quaternion> for Euler {
    fn from(q: Quaternion) -> Self {
        Self {
            psi: libm::atan2(
                (2.0 * q.x * q.y - 2.0 * q.w * q.z) as f64,
                (2.0 * q.w * q.w + 2.0 * q.x * q.x - 1.0) as f64,
            ),
            theta: -libm::asin((2.0 * q.x * q.z + 2.0 * q.w * q.y) as f64),
            phi: libm::atan2(
                (2.0 * q.y * q.z - 2.0 * q.w * q.x) as f64,
                (2.0 * q.w * q.w + 2.0 * q.z * q.z - 1.0) as f64,
            ),
        }
    }
}
