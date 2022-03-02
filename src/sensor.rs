use crate::accel::{Accel, AccelFullScale};
use crate::address::Address;
use crate::clock_source::ClockSource;
use crate::config::DigitalLowPassFilter;
use crate::error::Error;
use crate::fifo::Fifo;
use crate::gyro::{Gyro, GyroFullScale};
use crate::registers::Register;
use drogue_embedded_timer::Delay;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_time::duration::Milliseconds;

/// InvenSense MPU-6050 Driver
pub struct Mpu6050<'clock, I2c, Clock>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
    Clock: embedded_time::Clock,
{
    i2c: I2c,
    address: u8,
    clock: &'clock Clock,
}

impl<'clock, I2c, Clock> Mpu6050<'clock, I2c, Clock>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
    Clock: embedded_time::Clock,
{
    /// Construct a new i2c driver for the MPU-6050
    pub fn new(i2c: I2c, address: Address, clock: &'clock Clock) -> Result<Self, Error<I2c>> {
        let mut sensor = Self {
            i2c,
            address: address.into(),
            clock,
        };

        sensor.disable_sleep()?;

        Ok(sensor)
    }

    /// Load DMP firmware and perform all appropriate initialization.
    pub fn initialize_dmp(&mut self) -> Result<(), Error<I2c>> {
        self.reset()?;
        self.disable_sleep()?;
        self.reset_signal_path()?;
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

    pub(crate) fn read(&mut self, bytes: &[u8], response: &mut [u8]) -> Result<(), Error<I2c>> {
        self.i2c
            .write_read(self.address, bytes, response)
            .map_err(|e| Error::WriteReadError(e))
    }

    pub(crate) fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2c>> {
        self.i2c
            .write(self.address, bytes)
            .map_err(|e| Error::WriteError(e))
    }

    pub(crate) fn read_register(&mut self, reg: Register) -> Result<u8, Error<I2c>> {
        let mut buf = [0; 1];
        self.read(&[reg as u8], &mut buf)?;
        Ok(buf[0])
    }

    pub(crate) fn read_registers<'a>(
        &mut self,
        reg: Register,
        buf: &'a mut [u8],
    ) -> Result<&'a [u8], Error<I2c>> {
        self.read(&[reg as u8], buf)?;
        Ok(buf)
    }

    pub(crate) fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<I2c>> {
        self.write(&[reg as u8, value])
    }

    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------

    /// Perform power reset of the MPU
    pub fn reset(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value |= 1 << 7;
        self.write_register(Register::PwrMgmt1, value)?;
        Delay::new(self.clock).delay(Milliseconds(200));
        Ok(())
    }

    /// Perform reset of the signal path
    pub fn reset_signal_path(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 0;
        self.write_register(Register::UserCtrl, value)?;
        Delay::new(self.clock).delay(Milliseconds(200));
        Ok(())
    }

    /// Pick the clock-source
    pub fn set_clock_source(&mut self, clock_source: ClockSource) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value |= clock_source as u8;
        self.write_register(Register::PwrMgmt1, value)?;
        Ok(())
    }

    pub fn disable_interrupts(&mut self) -> Result<(), Error<I2c>> {
        self.write_register(Register::IntEnable, 0x00)
    }

    pub fn calibrate_accel(&mut self, loops: u8) -> Result<(), Error<I2c>> {
        Ok(())
    }

    pub fn set_accel_full_scale(&mut self, scale: AccelFullScale) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::AccelConfig)?;
        value |= (scale as u8) << 3;
        self.write_register(Register::AccelConfig, value)
    }

    pub fn set_gyro_full_scale(&mut self, scale: GyroFullScale) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::GyroConfig)?;
        value |= (scale as u8) << 3;
        self.write_register(Register::GyroConfig, value)
    }

    pub fn set_sample_rate_divider(&mut self, div: u8) -> Result<(), Error<I2c>> {
        self.write_register(Register::SmpRtDiv, div)
    }

    pub fn set_digital_lowpass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::Config)?;
        value |= filter as u8;
        self.write_register(Register::Config, value)
    }

    pub fn reset_fifo(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 2;
        self.write_register(Register::UserCtrl, value)
    }

    pub fn enable_fifo(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 6;
        self.write_register(Register::UserCtrl, value)
    }

    /// Set the DMP bit.
    /// To perform full DMP initialization, see `initialize_dmp()`
    pub fn enable_dmp(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 7;
        self.write_register(Register::UserCtrl, value)
    }

    // Unset the DMP bit.
    pub fn disable_dmp(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value &= !(1 << 7);
        self.write_register(Register::UserCtrl, value)
    }

    /// Reset the DMP processor
    pub fn reset_dmp(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::UserCtrl)?;
        value |= 1 << 3;
        self.write_register(Register::UserCtrl, value)
    }

    /// Read the FIFO
    pub fn read_fifo<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a [u8], Error<I2c>> {
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

    pub fn get_fifo_enabled(&mut self) -> Result<Fifo, Error<I2c>> {
        let value = self.read_register(Register::FifoEn)?;
        Ok(Fifo::from_byte(value))
    }

    pub fn set_fifo_enabled(&mut self, fifo: Fifo) -> Result<(), Error<I2c>> {
        self.write_register(Register::FifoEn, fifo.to_byte())
    }

    pub fn get_fifo_count(&mut self) -> Result<usize, Error<I2c>> {
        let mut buf = [0; 2];
        let _value = self.read_registers(Register::FifoCount_H, &mut buf)?;
        Ok(u16::from_be_bytes(buf) as usize)
    }

    pub fn disable_sleep(&mut self) -> Result<(), Error<I2c>> {
        let mut value = self.read_register(Register::PwrMgmt1)?;
        value &= !(1 << 6);
        self.write_register(Register::PwrMgmt1, value)
    }

    pub fn accel(&mut self) -> Result<Accel, Error<I2c>> {
        let mut data = [0; 6];
        self.read_registers(Register::AccelX_H, &mut data)?;
        Ok(Accel::new(data))
    }

    pub fn gyro(&mut self) -> Result<Gyro, Error<I2c>> {
        let mut data = [0; 6];
        self.read_registers(Register::GyroX_H, &mut data)?;
        Ok(Gyro::new(data))
    }
}
