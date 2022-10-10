use crate::{
    accel::{Accel, AccelFullScale},
    error::Error,
    gyro::{Gyro, GyroFullScale},
    sensor::Mpu6050,
};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Write, WriteRead};

// The threshold value that average readings must not cross after calibration.
#[derive(Copy, Clone, Debug)]
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
    fn is_value_within(self, value: i16) -> bool {
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
        // is correct the average is espected to be zero, or anyway within the given threshold).
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
            current_offset - ((current_mean / 10) + current_mean.signum())
        }
    }
}

/// Symbolic representation of a gravity vector aligned to one of the axes
/// (gravity must be subtracted to acceleration readings during calibration)
#[derive(Copy, Clone, Debug)]
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
            ReferenceGravity::Zero => Accel::new(0, 0, 0),
            ReferenceGravity::XN => Accel::new(-Self::gravity_value(scale), 0, 0),
            ReferenceGravity::XP => Accel::new(Self::gravity_value(scale), 0, 0),
            ReferenceGravity::YN => Accel::new(0, -Self::gravity_value(scale), 0),
            ReferenceGravity::YP => Accel::new(0, Self::gravity_value(scale), 0),
            ReferenceGravity::ZN => Accel::new(0, 0, -Self::gravity_value(scale)),
            ReferenceGravity::ZP => Accel::new(0, 0, Self::gravity_value(scale)),
        }
    }
}

/// Bit flags stating which axes must still be calibrated
/// (six bits are used: acceleration x, y, z and gyro x, y, z)
#[derive(Copy, Clone, Debug)]
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

/// Calibration parameters.
/// (all the values that influence calibration and do not change between calibration loop runs)
#[derive(Copy, Clone, Debug)]
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

/// Holds temporary vaues during sample mean computation
/// (includes the reference gravity compensation for simplicity)
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
        self.ax += (accel.x() - self.gravity_compensation.x()) as i32;
        self.ay += (accel.y() - self.gravity_compensation.y()) as i32;
        self.az += (accel.z() - self.gravity_compensation.z()) as i32;
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

/// Number of warmup iterations (when values are discardet)
const WARMUP_ITERATIONS: usize = 30;
/// Number of iterations for average error computation
const ITERATIONS: usize = 200;
/// Delay between measurements
const DELAY_MS: u32 = 2;

/// Perform a single loop computing the means of the readings
pub fn collect_mean_values<I2c>(
    mpu: &mut Mpu6050<I2c>,
    delay: &mut impl DelayMs<u32>,
    accel_scale: AccelFullScale,
    gravity: ReferenceGravity,
) -> Result<(Accel, Gyro), Error<I2c>>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    let mut accumulator = MeanAccumulator::new(accel_scale, gravity);

    for _ in 0..WARMUP_ITERATIONS {
        _ = mpu.accel()?;
        _ = mpu.gyro()?;
        delay.delay_ms(DELAY_MS);
    }

    for _ in 0..ITERATIONS {
        let accel = mpu.accel()?;
        let gyro = mpu.gyro()?;
        accumulator.add(&accel, &gyro);
        delay.delay_ms(DELAY_MS);
    }

    Ok(accumulator.means())
}

/// A single, full-fledged calibration loop (it also alters the device offest)
pub fn calibration_loop<I2c>(
    mpu: &mut Mpu6050<I2c>,
    delay: &mut impl DelayMs<u32>,
    parameters: &CalibrationParameters,
    actions: CalibrationActions,
) -> Result<(CalibrationActions, Accel, Gyro), Error<I2c>>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    let mut actions = actions;

    mpu.set_accel_full_scale(parameters.accel_scale)?;
    mpu.set_gyro_full_scale(parameters.gyro_scale)?;
    let mut accel_offset = mpu.get_accel_calibration()?;
    let mut gyro_offset = mpu.get_gyro_calibration()?;
    let (accel_mean, gyro_mean) =
        collect_mean_values(mpu, delay, parameters.accel_scale, parameters.gravity)?;

    if parameters.accel_threshold.is_value_within(accel_mean.x()) {
        actions = actions.with_accel_x(false);
    }
    if parameters.accel_threshold.is_value_within(accel_mean.y()) {
        actions = actions.with_accel_y(false);
    }
    if parameters.accel_threshold.is_value_within(accel_mean.z()) {
        actions = actions.with_accel_z(false);
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.x()) {
        actions = actions.with_gyro_x(false);
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.y()) {
        actions = actions.with_gyro_y(false);
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.z()) {
        actions = actions.with_gyro_z(false);
    }

    if actions.accel_x() {
        accel_offset.x = parameters
            .accel_threshold
            .next_offset(accel_mean.x(), accel_offset.x());
    }
    if actions.accel_y() {
        accel_offset.y = parameters
            .accel_threshold
            .next_offset(accel_mean.y(), accel_offset.y());
    }
    if actions.accel_z() {
        accel_offset.z = parameters
            .accel_threshold
            .next_offset(accel_mean.z(), accel_offset.z());
    }
    if actions.gyro_x() {
        gyro_offset.x = parameters
            .gyro_threshold
            .next_offset(gyro_mean.x(), gyro_offset.x());
    }
    if actions.gyro_y() {
        gyro_offset.y = parameters
            .gyro_threshold
            .next_offset(gyro_mean.y(), gyro_offset.y());
    }
    if actions.gyro_z() {
        gyro_offset.z = parameters
            .gyro_threshold
            .next_offset(gyro_mean.z(), gyro_offset.z());
    }

    if !actions.is_empty() {
        mpu.set_accel_calibration(&accel_offset)?;
        mpu.set_gyro_calibration(&gyro_offset)?;
    }

    Ok((actions, accel_mean, gyro_mean))
}

/// Repeatedly perform calibration loops until the errors are within the given thresholds
pub fn calibrate<I2c>(
    mpu: &mut Mpu6050<I2c>,
    delay: &mut impl DelayMs<u32>,
    parameters: &CalibrationParameters,
) -> Result<(Accel, Gyro), Error<I2c>>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    let mut actions = CalibrationActions::all();
    while !actions.is_empty() {
        (actions, _, _) = calibration_loop(mpu, delay, parameters, actions)?;
    }
    Ok((mpu.get_accel_calibration()?, mpu.get_gyro_calibration()?))
}
