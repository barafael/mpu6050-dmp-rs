/// A quaternion representing 3D rotation/orientation.
///
/// Quaternions provide a way to represent 3D rotations that avoids the
/// gimbal lock problems associated with Euler angles (yaw, pitch, roll).
/// They consist of:
/// - A scalar component (w): represents the amount of rotation
/// - Three vector components (x,y,z): represent the axis of rotation
///
/// Properties:
/// - When representing orientation, w will be close to 1 when level
/// - x,y,z components indicate rotation around respective axes:
///   * x: Roll (side-to-side tilt)
///   * y: Pitch (forward-backward tilt)
///   * z: Yaw (rotation around vertical axis)
/// - Magnitude should be 1 for a pure rotation (normalized)
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub struct Quaternion {
    /// Scalar (real) component
    pub w: f32,
    /// X (i) component - roll axis
    pub x: f32,
    /// Y (j) component - pitch axis
    pub y: f32,
    /// Z (k) component - yaw axis
    pub z: f32,
}

impl Quaternion {
    /// Converts raw DMP output bytes into a quaternion.
    ///
    /// The DMP provides quaternion data as four signed 32-bit integers
    /// scaled by 2^14 (16384). This function:
    /// 1. Reads four 4-byte values (w,x,y,z)
    /// 2. Converts from big-endian byte order
    /// 3. Scales values to floating point (-1 to 1 range)
    pub const fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 16 {
            return None;
        }

        let w = i32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);

        let x = i32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);

        let y = i32::from_be_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);

        let z = i32::from_be_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]); // as f32 / 16384.0;

        //log::info!("---> {} {} {} {}", w, x, y, z);

        Some(Self {
            w: w as f32 / 16384.0,
            x: x as f32 / 16384.0,
            y: y as f32 / 16384.0,
            z: z as f32 / 16384.0,
        })
    }

    /// Calculates the magnitude (length) of the quaternion.
    ///
    /// The magnitude is the square root of the sum of squares of all components.
    /// For a pure rotation quaternion (no scaling), the magnitude should be 1.
    /// Values other than 1 indicate either:
    /// - Scaling has been applied
    /// - Numerical errors have accumulated
    /// - The quaternion needs normalization
    pub fn magnitude(&self) -> f32 {
        libm::sqrt((self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z) as f64)
            as f32
    }

    /// Normalizes the quaternion to have magnitude 1.
    ///
    /// Normalization is important because:
    /// 1. Only unit quaternions (magnitude = 1) represent pure rotations
    /// 2. Prevents scaling effects from accumulating during calculations
    /// 3. Maintains numerical stability in orientation tracking
    ///
    /// The process divides each component by the quaternion's magnitude.
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
