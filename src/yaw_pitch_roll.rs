use crate::gravity::Gravity;
use crate::quaternion::Quaternion;

const PI: f64 = core::f64::consts::PI;

#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

impl YawPitchRoll {}

impl From<Quaternion> for YawPitchRoll {
    fn from(q: Quaternion) -> Self {
        let gravity = Gravity::from(q);
        // yaw: (about Z axis)
        let yaw = libm::atan2(
            (2.0 * q.x * q.y - 2.0 * q.w * q.z) as f64,
            (2.0 * q.w * q.w + 2.0 * q.x * q.x - 1.0) as f64,
        );
        // pitch: (nose up/down, about Y axis)
        let mut pitch = libm::atan2(
            gravity.x as f64,
            libm::sqrt((gravity.y * gravity.y + gravity.z * gravity.z) as f64),
        );
        // roll: (tilt left/right, about X axis)
        let roll = libm::atan2(gravity.y as f64, gravity.z as f64);

        pitch = PI - pitch;

        Self {
            yaw: yaw as f32,
            pitch: pitch as f32,
            roll: roll as f32,
        }
    }
}
