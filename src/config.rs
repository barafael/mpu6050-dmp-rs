//! MPU6050 Digital Low-Pass Filter Configuration
//!
//! The digital low-pass filter (DLPF) reduces noise in sensor readings by:
//! - Filtering out high-frequency vibrations
//! - Smoothing rapid changes in sensor data
//! - Reducing aliasing effects
//!
//! Filter selection involves balancing:
//! - Noise reduction (higher filtering)
//! - Response time (lower filtering)
//! - Power consumption
//! - Output data rate

/// Digital low-pass filter configurations.
///
/// Each filter setting provides different:
/// - Bandwidth (Hz): How much high-frequency data passes through
/// - Delay (ms): How long it takes for changes to be reflected
/// - Noise reduction: How much sensor noise is filtered out
///
/// Based on values from the official register map [RM-MPU-6000A](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DigitalLowPassFilter {
    /// Minimal filtering
    /// - Fastest response
    /// - Highest bandwidth
    /// - Least noise reduction
    /// - Only option that allows for 8kHz gyro sampling
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 260Hz
    /// - Accel Delay: ~0ms
    /// - Gyro Bandwidth: 256Hz
    /// - Gyro Delay: 0.98ms
    Filter0 = 0,

    /// Light filtering
    /// - Good balance for most uses
    /// - Quick response
    /// - Moderate noise reduction
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 184Hz
    /// - Accel Delay: 2ms
    /// - Gyro Bandwidth: 188Hz
    /// - Gyro Delay: 1.9ms
    Filter1 = 1,

    /// Moderate filtering
    /// - Balanced response/filtering
    /// - Good for general motion
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 94Hz
    /// - Accel Delay: 3.0ms
    /// - Gyro Bandwidth: 98Hz
    /// - Gyro Delay: 2.8ms
    Filter2 = 2,

    /// Strong filtering
    /// - Better noise reduction
    /// - Slower response
    /// - Good for stable readings
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 44Hz
    /// - Accel Delay: 4.9ms
    /// - Gyro Bandwidth: 42Hz
    /// - Gyro Delay: 4.8ms
    Filter3 = 3,

    /// Very strong filtering
    /// - High noise reduction
    /// - Significant delay
    /// - Good for slow movements
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 21Hz
    /// - Accel Delay: 8.5ms
    /// - Gyro Bandwidth: 20Hz
    /// - Gyro Delay: 8.3ms
    Filter4 = 4,

    /// Heavy filtering
    /// - Very high noise reduction
    /// - Long delay
    /// - Best for very slow changes
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 10Hz
    /// - Accel Delay: 13.8ms
    /// - Gyro Bandwidth: 10Hz
    /// - Gyro Delay: 13.4ms
    Filter5 = 5,

    /// Maximum filtering
    /// - Maximum noise reduction
    /// - Longest delay
    /// - Use only for nearly static measurements
    /// # LPF Specs
    /// ***
    /// - Accel Bandwidth: 5Hz
    /// - Accel Delay: 19.0ms
    /// - Gyro Bandwidth: 5Hz
    /// - Gyro Delay: 18.6ms
    Filter6 = 6,
}
