//! MPU6050 Sensor Calibration
//!
//! Calibration process:
//! - Takes multiple readings while sensor is stationary
//! - Calculates offset values to zero out errors
//! - Compensates for gravity's effect on accelerometer
//! - Stores calibration in sensor registers

use crate::{
    accel::{Accel, AccelFullScale},
    gyro::{Gyro, GyroFullScale},
};
use core::fmt::Debug;

/// Number of warmup iterations (when values are discarded)
pub(crate) const WARMUP_ITERATIONS: usize = 30;
/// Number of iterations for average error computation
pub(crate) const ITERATIONS: usize = 200;
/// Delay between measurements
pub(crate) const DELAY_MS: u32 = 2;

/// Maximum allowed deviation from zero after calibration
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CalibrationThreshold {
    value: i16,
}

impl CalibrationThreshold {
    // Reasonable acceleration threshold value at a given scale.
    pub const fn from_accel_scale(scale: AccelFullScale) -> Self {
        Self {
            value: match scale {
                AccelFullScale::G2 => 8,
                AccelFullScale::G4 => 4,
                AccelFullScale::G8 => 2,
                AccelFullScale::G16 => 1,
            },
        }
    }

    // Reasonable gyro threshold value at a given scale.
    pub const fn from_gyro_scale(scale: GyroFullScale) -> Self {
        Self {
            value: match scale {
                _ => 1,
            },
        }
    }

    /// Get the threshold value
    pub fn value(&self) -> i16 {
        self.value
    }

    /// Check if the given value is within the threshold
    pub(crate) fn is_value_within(self, value: i16) -> bool {
        value.abs() <= self.value
    }

    /// Check if the given acceleration vector is within the threshold
    pub fn is_accel_within(self, accel: &Accel) -> bool {
        self.is_value_within(accel.x())
            && self.is_value_within(accel.y())
            && self.is_value_within(accel.z())
    }

    /// Check if the given gyro vector is within the threshold
    pub fn is_gyro_within(self, gyro: &Gyro) -> bool {
        self.is_value_within(gyro.x())
            && self.is_value_within(gyro.y())
            && self.is_value_within(gyro.z())
    }

    /// If the current computed mean value is not acceptable, compute the next likely
    /// calibration offset.
    ///
    /// This is technically the single step of a PID controller where we are using only
    /// the `I` part (`D` is not needed because calibration is not time-dependent,
    /// and `P` because noise is mitigated by working on averages).
    pub fn next_offset(self, current_mean: i16, current_offset: i16) -> i16 {
        // In this PID controller the "error" is the observed average (when the calibration
        // is correct the average is expected to be zero, or anyway within the given threshold).
        if self.is_value_within(current_mean) {
            // If we are withing the expected threshold do not change the offset (there's no need!).
            current_offset
        } else {
            // Otherwise adjust the offset.
            //
            // The current measured mean value is the PID error, and the Ki PID factor is -0.1
            // (we are dividing `current_mean` by 10).
            //
            // The `signum` factor is there because we work in the integer domain and if the error
            // is small `current_mean / 10` is zero and the algorithm does not make progress.
            // Adding the `signum` is negligible during normal operation but ensures that the offset
            // keeps changing during the final tuning runs (when the error is already very small).
            //
            // Use saturating arithmetic to prevent overflow when subtracting from current_offset
            current_offset.saturating_sub((current_mean / 10) + current_mean.signum())
        }
    }
}

/// Reference gravity direction for calibration.
///
/// During calibration, one axis should point straight down:
/// - XN/XP: X-axis pointing down/up
/// - YN/YP: Y-axis pointing down/up
/// - ZN/ZP: Z-axis pointing down/up
/// - Zero: No gravity compensation
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ReferenceGravity {
    Zero,
    XN,
    XP,
    YN,
    YP,
    ZN,
    ZP,
}

impl ReferenceGravity {
    /// Actual `g` value at a given scale
    fn gravity_value(scale: AccelFullScale) -> i16 {
        match scale {
            AccelFullScale::G2 => 16384,
            AccelFullScale::G4 => 8192,
            AccelFullScale::G8 => 4096,
            AccelFullScale::G16 => 2048,
        }
    }

    /// Acceleration vector representing gravity compensation in the given direction
    pub fn gravity_compensation(self, scale: AccelFullScale) -> Accel {
        match self {
            Self::Zero => Accel::new(0, 0, 0),
            Self::XN => Accel::new(-Self::gravity_value(scale), 0, 0),
            Self::XP => Accel::new(Self::gravity_value(scale), 0, 0),
            Self::YN => Accel::new(0, -Self::gravity_value(scale), 0),
            Self::YP => Accel::new(0, Self::gravity_value(scale), 0),
            Self::ZN => Accel::new(0, 0, -Self::gravity_value(scale)),
            Self::ZP => Accel::new(0, 0, Self::gravity_value(scale)),
        }
    }
}

/// Tracks which sensor axes still need calibration.
///
/// Each bit represents one axis:
/// - Bits 0-2: Accelerometer (X,Y,Z)
/// - Bits 3-5: Gyroscope (X,Y,Z)
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CalibrationActions {
    flags: u8,
}

impl CalibrationActions {
    const ACCEL_X: u8 = 1 << 0;
    const ACCEL_Y: u8 = 1 << 1;
    const ACCEL_Z: u8 = 1 << 2;
    const GYRO_X: u8 = 1 << 3;
    const GYRO_Y: u8 = 1 << 4;
    const GYRO_Z: u8 = 1 << 5;

    /// Build an empty bit set
    pub fn empty() -> Self {
        Self { flags: 0 }
    }

    /// Build a full bit set
    pub fn all() -> Self {
        Self { flags: 0x3f }
    }

    /// Check if we have nothing more to calibrate
    pub fn is_empty(self) -> bool {
        self.flags == 0
    }

    /// Check if acceleration x axis calibration is required
    pub fn accel_x(self) -> bool {
        self.flags & Self::ACCEL_X != 0
    }
    /// Check if acceleration y axis calibration is required
    pub fn accel_y(self) -> bool {
        self.flags & Self::ACCEL_Y != 0
    }
    /// Check if acceleration z axis calibration is required
    pub fn accel_z(self) -> bool {
        self.flags & Self::ACCEL_Z != 0
    }
    /// Check if gyro x axis calibration is required
    pub fn gyro_x(self) -> bool {
        self.flags & Self::GYRO_X != 0
    }
    /// Check if gyro y axis calibration is required
    pub fn gyro_y(self) -> bool {
        self.flags & Self::GYRO_Y != 0
    }
    /// Check if gyro z axis calibration is required
    pub fn gyro_z(self) -> bool {
        self.flags & Self::GYRO_Z != 0
    }

    /// Set the given flag
    fn with_flag(self, value: bool, flag: u8) -> Self {
        Self {
            flags: if value {
                self.flags | flag
            } else {
                self.flags & !flag
            },
        }
    }

    /// Set acceleration x flag
    pub fn with_accel_x(self, value: bool) -> Self {
        self.with_flag(value, Self::ACCEL_X)
    }
    /// Set acceleration y flag
    pub fn with_accel_y(self, value: bool) -> Self {
        self.with_flag(value, Self::ACCEL_Y)
    }
    /// Set acceleration z flag
    pub fn with_accel_z(self, value: bool) -> Self {
        self.with_flag(value, Self::ACCEL_Z)
    }
    /// Set gyro x flag
    pub fn with_gyro_x(self, value: bool) -> Self {
        self.with_flag(value, Self::GYRO_X)
    }
    /// Set gyro y flag
    pub fn with_gyro_y(self, value: bool) -> Self {
        self.with_flag(value, Self::GYRO_Y)
    }
    /// Set gyro z flag
    pub fn with_gyro_z(self, value: bool) -> Self {
        self.with_flag(value, Self::GYRO_Z)
    }
}

/// Configuration for the calibration process.
///
/// Parameters control:
/// - Sensor ranges (AccelFullScale, GyroFullScale)
/// - Acceptable error thresholds
/// - Number of samples to take
/// - Gravity compensation direction
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CalibrationParameters {
    /// Acceleration scale
    pub accel_scale: AccelFullScale,
    /// Acceleration threshold value
    pub accel_threshold: CalibrationThreshold,
    /// Gyro scale
    pub gyro_scale: GyroFullScale,
    /// Gyro threshold value
    pub gyro_threshold: CalibrationThreshold,
    /// Number of warmup iterations when computing mean values
    pub warmup_iterations: usize,
    /// Number of warmup iterations when computing values
    pub iterations: usize,
    /// Reference gravity (will be subtracted from acceleration readings)
    pub gravity: ReferenceGravity,
}

impl CalibrationParameters {
    /// Create calibration parameters given accel and gyro scale and a reference gravity
    /// (sensible defaults are used for all other parameters)
    pub fn new(
        accel_scale: AccelFullScale,
        gyro_scale: GyroFullScale,
        gravity: ReferenceGravity,
    ) -> Self {
        Self {
            accel_scale,
            accel_threshold: CalibrationThreshold::from_accel_scale(accel_scale),
            gyro_scale,
            gyro_threshold: CalibrationThreshold::from_gyro_scale(gyro_scale),
            warmup_iterations: WARMUP_ITERATIONS,
            iterations: ITERATIONS,
            gravity,
        }
    }

    /// Change acceleration threshold
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_accel_threshold(self, threshold: i16) -> Self {
        Self {
            accel_threshold: CalibrationThreshold { value: threshold },
            ..self
        }
    }

    /// Change gyro threshold
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_gyro_threshold(self, threshold: i16) -> Self {
        Self {
            gyro_threshold: CalibrationThreshold { value: threshold },
            ..self
        }
    }

    /// Change warmup iterations count
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_warmup_iterations(self, warmup_iterations: usize) -> Self {
        Self {
            warmup_iterations,
            ..self
        }
    }

    /// Change iterations count
    /// (consumes and returns `Self` to be callable in a "builder-like" pattern)
    pub fn with_iterations(self, iterations: usize) -> Self {
        Self { iterations, ..self }
    }
}

/// Accumulates sensor readings during calibration.
///
/// Stores:
/// - Running sum of accelerometer readings
/// - Running sum of gyroscope readings
/// - Gravity compensation vector
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MeanAccumulator {
    pub ax: i32,
    pub ay: i32,
    pub az: i32,
    pub gx: i32,
    pub gy: i32,
    pub gz: i32,
    pub gravity_compensation: Accel,
}

impl MeanAccumulator {
    /// Initializes the means with zero values
    /// (and also fixes the reference gravity compensation)
    pub fn new(accel_scale: AccelFullScale, gravity: ReferenceGravity) -> Self {
        Self {
            ax: 0,
            ay: 0,
            az: 0,
            gx: 0,
            gy: 0,
            gz: 0,
            gravity_compensation: gravity.gravity_compensation(accel_scale),
        }
    }

    /// Adds a new sample (subtracting the reference gravity)
    pub fn add(&mut self, accel: &Accel, gyro: &Gyro) {
        self.ax += (accel.x() as i32) - (self.gravity_compensation.x() as i32);
        self.ay += (accel.y() as i32) - (self.gravity_compensation.y() as i32);
        self.az += (accel.z() as i32) - (self.gravity_compensation.z() as i32);
        self.gx += gyro.x() as i32;
        self.gy += gyro.y() as i32;
        self.gz += gyro.z() as i32;
    }

    /// Compute average values (consumes `self` because the computation is done)
    pub fn means(mut self) -> (Accel, Gyro) {
        self.ax /= ITERATIONS as i32;
        self.ay /= ITERATIONS as i32;
        self.az /= ITERATIONS as i32;
        self.gx /= ITERATIONS as i32;
        self.gy /= ITERATIONS as i32;
        self.gz /= ITERATIONS as i32;
        (
            Accel::new(self.ax as i16, self.ay as i16, self.az as i16),
            Gyro::new(self.gx as i16, self.gy as i16, self.gz as i16),
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        accel::{Accel, AccelFullScale},
        gyro::Gyro,
    };

    use super::{CalibrationThreshold, MeanAccumulator, ReferenceGravity};

    #[test]
    fn test_mean_accumulator_with_compensation() {
        // Case #1: attempt to subtract with overflow
        {
            let accel = Accel {
                x: -1180,
                y: -32768,
                z: 32767,
            };
            let gyro = Gyro {
                x: -3,
                y: -7,
                z: -10,
            };

            let mut mean_acc = MeanAccumulator::new(AccelFullScale::G2, ReferenceGravity::ZN);
            mean_acc.add(&accel, &gyro);

            // This test verifies that extreme accelerometer values (32767) don't overflow
            // when combined with gravity compensation (-16384). The calculation must be
            // done in i32 to get the correct result of 49151.
            assert_eq!(mean_acc.az, 49151);
        }
    }

    #[test]
    fn test_mean_accumulator_with_compensation_negate_panic() {
        // Case #2: attempt to subtract with overflow
        {
            let mut mean_acc = MeanAccumulator {
                ax: -700924,
                ay: -6520832,
                az: 3260217,
                gx: -3345,
                gy: 770,
                gz: -7648,
                gravity_compensation: Accel {
                    x: 0,
                    y: 0,
                    z: -16384,
                },
            };
            let accel = Accel {
                x: -3536,
                y: -32768,
                z: 32767,
            };
            let gyro = Gyro {
                x: -105,
                y: 100,
                z: -36,
            };

            mean_acc.add(&accel, &gyro);
        }
    }

    #[test]
    fn test_next_offset_overflow_protection() {
        // Test case for the overflow issue reported by users:
        // "[ERROR] panicked at attempt to subtract with overflow"
        let threshold = CalibrationThreshold { value: 8 };

        // Case 1: Large negative offset with positive mean (would overflow without saturating arithmetic)
        let result = threshold.next_offset(30000, -32000);
        // Should not panic and should be clamped to valid i16 range
        assert!(result >= i16::MIN && result <= i16::MAX);
        assert_eq!(result, i16::MIN); // Should saturate to minimum value

        // Case 2: Large positive offset with negative mean (would overflow in positive direction)
        let result = threshold.next_offset(-30000, 32000);
        assert!(result >= i16::MIN && result <= i16::MAX);
        // Result should be 32000 - ((-30000 / 10) + (-1)) = 32000 - (-3000 - 1) = 32000 + 3001 = 35001
        // But saturated to i16::MAX
        assert_eq!(result, i16::MAX);

        // Case 3: Normal operation should work as before (mean > threshold so adjustment happens)
        let result = threshold.next_offset(100, 1000);
        assert_eq!(result, 1000 - (10 + 1)); // 1000 - 11 = 989
        assert_eq!(result, 989);

        // Case 4: Small values within threshold should return unchanged offset
        let result = threshold.next_offset(5, 100);
        assert_eq!(result, 100); // No change because 5 <= 8 (within threshold)

        // Case 5: Edge case at threshold boundary
        let result = threshold.next_offset(8, 100);
        assert_eq!(result, 100); // No change because 8 == 8 (within threshold)

        // Case 6: Just outside threshold should apply adjustment
        let result = threshold.next_offset(9, 100);
        assert_eq!(result, 100 - (0 + 1)); // 100 - 1 = 99 (9/10 = 0 in integer division)
        assert_eq!(result, 99);
    }
}
