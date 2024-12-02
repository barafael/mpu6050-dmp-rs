use crate::{
    accel::{Accel, AccelFullScale},
    address::Address,
    calibration::{CalibrationActions, CalibrationParameters, ReferenceGravity},
    calibration_blocking::{calibrate, calibration_loop, collect_mean_values},
    clock_source::ClockSource,
    config::DigitalLowPassFilter,
    error::{Error, InitError},
    fifo::Fifo,
    gyro::{Gyro, GyroFullScale},
    registers::Register,
};
use embedded_hal::{delay, i2c::I2c};

/// InvenSense MPU-6050 Driver
pub struct Mpu6050<I>
where
    I: I2c,
{
    i2c: I,
    address: u8,
}

impl<I> Mpu6050<I>
where
    I: I2c,
{
    /// Construct a new i2c driver for the MPU-6050
    pub fn new(i2c: I, address: Address) -> Result<Self, InitError<I>> {
        let mut sensor = Self {
            i2c,
            address: address.into(),
        };

        if let Err(error) = sensor.disable_sleep() {
            Err(InitError {
                error,
                i2c: sensor.i2c,
            })
        } else {
            Ok(sensor)
        }
    }

    /// Returns the underlying I2C peripheral, consuming this driver.
    pub fn release(self) -> I {
        self.i2c
    }

    /// Load DMP firmware and perform all appropriate initialization.
    pub fn initialize_dmp(&mut self, delay: &mut impl delay::DelayNs) -> Result<(), Error<I>> {
        self.reset(delay)?;
        self.disable_sleep()?;
        self.reset_signal_path(delay)?;
        self.disable_dmp()?;
        self.set_clock_source(ClockSource::Xgyro)?;
        self.disable_interrupts()?;
        self.set_fifo_enabled(Fifo::all_disabled())?;
        self.set_accel_full_scale(AccelFullScale::G2)?;
        self.set_sample_rate_divider(4)?;
        self.set_digital_lowpass_filter(DigitalLowPassFilter::Filter1)?;
        self.load_firmware()?;
        self.boot_firmware()?;
        self.set_gyro_full_scale(GyroFullScale::Deg2000)?;
        self.enable_fifo()?;
        self.reset_fifo()?;
        self.disable_dmp()?;
        self.enable_dmp()?;
        Ok(())
    }

    pub(crate) fn read(&mut self, bytes: &[u8], response: &mut [u8]) -> Result<(), Error<I>> {
        self.i2c
            .write_read(self.address, bytes, response)
            .map_err(|e| Error::WriteReadError(e))
    }

    pub(crate) fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I>> {
        self.i2c
            .write(self.address, bytes)
            .map_err(|e| Error::WriteError(e))
    }

    pub(crate) fn read_register(&mut self, reg: Register) -> Result<u8, Error<I>> {
        let mut buf = [0; 1];
        self.read(&[reg as u8], &mut buf)?;
        Ok(buf[0])
    }

    pub(crate) fn read_registers<'a>(
        &mut self,
        reg: Register,
        buf: &'a mut [u8],
    ) -> Result<&'a [u8], Error<I>> {
        self.read(&[reg as u8], buf)?;
        Ok(buf)
    }

    pub(crate) fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<I>> {
        self.write(&[reg as u8, value])
    }

    /// Perform power reset of the MPU
    pub fn reset(&mut self, clock: &mut impl delay::DelayNs) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value |= 1 << 7;
        self.write_register(Register::PwrMgmt1, value)?;
        clock.delay_ms(200);
        Ok(())
    }

    /// Perform reset of the signal path
    pub fn reset_signal_path(&mut self, clock: &mut impl delay::DelayNs) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 0;
        self.write_register(Register::UserCtrl, value)?;
        clock.delay_ms(200);
        Ok(())
    }

    /// Pick the clock-source
    pub fn set_clock_source(&mut self, clock_source: ClockSource) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value |= clock_source as u8;
        self.write_register(Register::PwrMgmt1, value)?;
        Ok(())
    }

    pub fn disable_interrupts(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0x00)
    }

    /// Enables a FIFO buffer overflow to generate an interrupt
    pub fn interrupt_fifo_oflow_en(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0b0001_0000)
    }

    /// Enables any of the i2c master interrupt sources to generate an interrupt
    pub fn interrupt_i2c_mst_int_en(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0b0000_1000)
    }

    /// Enables the Data Ready interrupt, which occurs each time a write
    /// operation to all of the sensor registers has been completed
    pub fn interrupt_data_ready_en(&mut self) -> Result<(), Error<I>> {
        self.write_register(Register::IntEnable, 0b1000_0000)
    }

    /// Read the interrupt status register and clear it.
    pub fn interrupt_read_clear(&mut self) -> Result<u8, Error<I>> {
        self.read_register(Register::IntStatus)
    }

    /// Super simple averaging calibration of the accelerometers.
    /// Probably should be called before initializing the DMP.
    /// Deprecated because the new calibration framework (`calibrate`) works way better.
    // TODO that's obviously not optimal situation, fix this with typestates or something.
    // TODO: consider this: https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
    #[deprecated]
    pub fn calibrate_accel(
        &mut self,
        loops: u8,
        delay: &mut impl delay::DelayNs,
    ) -> Result<(), Error<I>> {
        self.set_accel_calibration(&Accel::new(0, 0, 0))?;

        delay.delay_ms(10);
        let mut accumulator = [0i16; 3];
        for _ in 0..loops {
            let sample = self.accel().unwrap();
            accumulator[0] += ((sample.x() as f32) / loops as f32) as i16;
            accumulator[1] += ((sample.y() as f32) / loops as f32) as i16;
            accumulator[2] += ((sample.z() as f32) / loops as f32) as i16;
            delay.delay_ms(1);
        }

        let values = Accel::new(accumulator[0], accumulator[1], accumulator[2]);
        self.set_accel_calibration(&values)?;

        Ok(())
    }

    /// Super simple averaging calibration of the gyroscopes.
    /// Probably should be called before initializing the DMP.
    /// Deprecated because the new calibration framework (`calibrate`) works way better.
    // TODO that's obviously not optimal situation, fix this with typestates or something.
    // TODO: consider this: https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
    #[deprecated]
    pub fn calibrate_gyro(
        &mut self,
        loops: u8,
        delay: &mut impl delay::DelayNs,
    ) -> Result<(), Error<I>> {
        self.set_gyro_calibration(&Gyro::new(0, 0, 0))?;

        delay.delay_ms(10);
        let mut accumulator = [0i16; 3];
        for _ in 0..loops {
            let gyros = self.gyro()?;
            accumulator[0] += (gyros.x() as f32 / loops as f32) as i16;
            accumulator[1] += (gyros.y() as f32 / loops as f32) as i16;
            accumulator[2] += (gyros.z() as f32 / loops as f32) as i16;
            delay.delay_ms(1);
        }

        let values = Gyro::new(accumulator[0], accumulator[1], accumulator[2]);
        self.set_gyro_calibration(&values)?;

        Ok(())
    }

    /// Through calibration, taken from <https://wired.chillibasket.com/2015/01/calibrating-mpu6050/>
    /// This is supposed to be run once, printing the results, and reusing them by setting the
    /// calibration using `set_accel_calibration` and `set_gyro_calibration` with offsets hardcoded
    /// as constants in the application code (or storing and retrieving them to-from persistent
    /// storage, if available).
    /// If calibration parameters are not "reasonable" this function might never return.
    pub fn calibrate(
        &mut self,
        delay: &mut impl delay::DelayNs,
        parameters: &CalibrationParameters,
    ) -> Result<(Accel, Gyro), Error<I>> {
        calibrate(self, delay, parameters)
    }

    /// A building block for performing calibration: collect several samples return their average
    pub fn collect_mean_values(
        &mut self,
        delay: &mut impl delay::DelayNs,
        accel_scale: AccelFullScale,
        gravity: ReferenceGravity,
    ) -> Result<(Accel, Gyro), Error<I>> {
        collect_mean_values(self, delay, accel_scale, gravity)
    }

    /// Perform a full device calibration (note that this function might never terminate).
    ///
    /// A higher level building block for performing calibration,
    /// this function should be called repeatedly, until `is_empty()` returns `true` on the
    /// returned `CalibrationActions`.
    ///
    /// At each iteration the calibration gets "better", and it is considered completed when
    /// there are no more actions to perform.
    ///
    /// This is an example of how a calibration loop can be written:
    ///
    /// ```ignore
    ///     let calibration_parameters =
    ///         CalibrationParameters::new(ACCEL_SCALE, GYRO_SCALE, ReferenceGravity::ZN);
    ///     let mut actions = CalibrationActions::all();
    ///     let mut accel_error: Accel;
    ///     let mut gyro_error: Gyro;
    ///     while !actions.is_empty() {
    ///         (actions, accel_error, gyro_error) = mpu
    ///             .calibration_loop(&mut delay, &calibration_parameters, actions)
    ///             .unwrap();
    ///         uprintln!(
    ///             serial,
    ///             "errors: [a {} {} {}] [g {} {} {}] [done: {}]",
    ///             accel_error.x(),
    ///             accel_error.y(),
    ///             accel_error.z(),
    ///             gyro_error.x(),
    ///             gyro_error.y(),
    ///             gyro_error.z(),
    ///             actions.is_empty(),
    ///         );
    ///     }
    ///     let accel_offset = mpu.get_accel_calibration().unwrap();
    ///     let gyro_offset = mpu.get_gyro_calibration().unwrap();
    ///     uprintln!(
    ///         serial,
    ///         "offsets: [a {} {} {}] [g {} {} {}]",
    ///         accel_offset.x(),
    ///         accel_offset.y(),
    ///         accel_offset.z(),
    ///         gyro_offset.x(),
    ///         gyro_offset.y(),
    ///         gyro_offset.z(),
    ///     );
    /// ```
    ///
    /// When the loop is done the offsets can be collected and hardcoded in the initialization
    /// of the application that will use this device with exactly these settings.
    pub fn calibration_loop(
        &mut self,
        delay: &mut impl delay::DelayNs,
        parameters: &CalibrationParameters,
        actions: CalibrationActions,
    ) -> Result<(CalibrationActions, Accel, Gyro), Error<I>> {
        calibration_loop(self, delay, parameters, actions)
    }

    pub fn get_accel_calibration(&mut self) -> Result<Accel, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::AccelOffsetX_H, &mut data)?;
        Ok(Accel::from_bytes(data))
    }

    pub fn get_gyro_calibration(&mut self) -> Result<Gyro, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::GyroOffsetX_H, &mut data)?;
        Ok(Gyro::from_bytes(data))
    }

    pub fn set_accel_calibration(&mut self, values: &Accel) -> Result<(), Error<I>> {
        let data = values.to_bytes();
        let bytes = [
            Register::AccelOffsetX_H as u8,
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
        ];
        self.write(&bytes)
    }

    pub fn set_gyro_calibration(&mut self, values: &Gyro) -> Result<(), Error<I>> {
        let data = values.to_bytes();
        let bytes = [
            Register::GyroOffsetX_H as u8,
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
        ];
        self.write(&bytes)
    }

    pub fn set_accel_full_scale(&mut self, scale: AccelFullScale) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::AccelConfig)?;
        value |= (scale as u8) << 3;
        self.write_register(Register::AccelConfig, value)
    }

    pub fn set_gyro_full_scale(&mut self, scale: GyroFullScale) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::GyroConfig)?;
        value |= (scale as u8) << 3;
        self.write_register(Register::GyroConfig, value)
    }

    pub fn set_sample_rate_divider(&mut self, div: u8) -> Result<(), Error<I>> {
        self.write_register(Register::SmpRtDiv, div)
    }

    pub fn set_digital_lowpass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::Config)?;
        value |= filter as u8;
        self.write_register(Register::Config, value)
    }

    pub fn reset_fifo(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 2;
        self.write_register(Register::UserCtrl, value)
    }

    pub fn enable_fifo(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 6;
        self.write_register(Register::UserCtrl, value)
    }

    /// Set the DMP bit.
    /// To perform full DMP initialization, see `initialize_dmp()`
    pub fn enable_dmp(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 7;
        self.write_register(Register::UserCtrl, value)
    }

    // Unset the DMP bit.
    pub fn disable_dmp(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value &= !(1 << 7);
        self.write_register(Register::UserCtrl, value)
    }

    /// Reset the DMP processor
    pub fn reset_dmp(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 3;
        self.write_register(Register::UserCtrl, value)
    }

    /// Read the FIFO
    pub fn read_fifo<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a [u8], Error<I>> {
        let mut len = self.get_fifo_count()?;

        if buf.len() < len {
            len = buf.len();
        }

        if len == 0 {
            Ok(&buf[0..0])
        } else {
            self.read_registers(Register::FifoRw, &mut buf[0..len])
        }
    }

    pub fn get_fifo_enabled(&mut self) -> Result<Fifo, Error<I>> {
        let value = self.read_register(Register::FifoEn)?;
        Ok(Fifo::from_byte(value))
    }

    pub fn set_fifo_enabled(&mut self, fifo: Fifo) -> Result<(), Error<I>> {
        self.write_register(Register::FifoEn, fifo.to_byte())
    }

    pub fn get_fifo_count(&mut self) -> Result<usize, Error<I>> {
        let mut buf = [0; 2];
        let _value = self.read_registers(Register::FifoCount_H, &mut buf)?;
        Ok(u16::from_be_bytes(buf) as usize)
    }

    pub fn disable_sleep(&mut self) -> Result<(), Error<I>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value &= !(1 << 6);
        self.write_register(Register::PwrMgmt1, value)
    }

    pub fn accel(&mut self) -> Result<Accel, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::AccelX_H, &mut data)?;
        Ok(Accel::from_bytes(data))
    }

    pub fn gyro(&mut self) -> Result<Gyro, Error<I>> {
        let mut data = [0; 6];
        self.read_registers(Register::GyroX_H, &mut data)?;
        Ok(Gyro::from_bytes(data))
    }

    /// Gets the 6 degrees of freedom at once - Acceleration and Gyroscope.
    pub fn motion6(&mut self) -> Result<(Accel, Gyro), Error<I>> {
        let mut data = [0; 14];
        self.read_registers(Register::AccelX_H, &mut data)?;

        let accel = Accel::from_bytes([data[0], data[1], data[2], data[3], data[4], data[5]]);
        let gyro = Gyro::from_bytes([data[8], data[9], data[10], data[11], data[12], data[13]]);
        Ok((accel, gyro))
    }
}
