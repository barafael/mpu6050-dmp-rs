//! Conversion between Quaternions and Euler angles (Yaw, Pitch, Roll)
//!
//! While quaternions are used internally for calculations, Euler angles are:
//! - More intuitive for humans to understand
//! - Directly correspond to physical rotations
//! - Commonly used in navigation and robotics
//!
//! The conversion process:
//! 1. Extracts gravity vector from quaternion
//! 2. Calculates angles using arctangent (atan2) functions
//! 3. Returns angles in radians (-π to π range)

use crate::gravity::Gravity;
use crate::quaternion::Quaternion;

/// Represents orientation as three angles in radians.
///
/// The angles describe rotations around three axes:
/// - Yaw: Rotation around Z (vertical) axis, like shaking head "no"
/// - Pitch: Rotation around Y axis, like nodding head "yes"
/// - Roll: Rotation around X axis, like tilting head toward shoulder
///
/// Note: These angles can experience gimbal lock when pitch approaches ±90°
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct YawPitchRoll {
    /// Rotation around vertical (Z) axis, -π to π radians
    pub yaw: f32,
    /// Forward/backward tilt (Y axis), -π/2 to π/2 radians
    pub pitch: f32,
    /// Side-to-side tilt (X axis), -π to π radians
    pub roll: f32,
}

impl From<Quaternion> for YawPitchRoll {
    /// Converts a quaternion to Euler angles (Yaw, Pitch, Roll).
    ///
    /// The conversion process:
    /// 1. Calculate gravity vector from quaternion
    /// 2. Use arctangent (atan2) to find angles:
    ///    - Yaw: rotation in X-Y plane
    ///    - Pitch: angle from horizontal plane
    ///    - Roll: rotation around forward axis
    ///
    /// Note: The resulting angles are in radians:
    /// - Yaw: -π to π (180 degrees)
    /// - Pitch: -π/2 to π/2 (90 degrees)
    /// - Roll: -π to π (180 degrees)
    fn from(q: Quaternion) -> Self {
        let gravity = Gravity::from(q);
        // Calculate yaw (ψ) around Z axis using quaternion components
        // Formula: ψ = atan2(2(qx*qy - qw*qz), 2(qw^2 + qx^2) - 1)
        let yaw = libm::atan2(
            (2.0 * q.x * q.y - 2.0 * q.w * q.z) as f64,
            (2.0 * q.w * q.w + 2.0 * q.x * q.x - 1.0) as f64,
        );

        // Calculate pitch (θ) using gravity vector
        // Formula: θ = atan2(gx, sqrt(gy^2 + gz^2))
        let pitch = libm::atan2(
            gravity.x as f64,
            libm::sqrt((gravity.y * gravity.y + gravity.z * gravity.z) as f64),
        );

        // Calculate roll (φ) using gravity vector
        // Formula: φ = atan2(gy, gz)
        let roll = libm::atan2(gravity.y as f64, gravity.z as f64);

        //pitch = PI - pitch;

        Self {
            yaw: yaw as f32,
            pitch: pitch as f32,
            roll: roll as f32,
        }
    }
}
