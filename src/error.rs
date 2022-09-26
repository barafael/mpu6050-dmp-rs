use core::fmt::Formatter;
use embedded_hal::blocking::i2c::{Write, WriteRead};

/// Error during initialization of sensor. Wraps [`Error`].
pub struct InitError<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    pub i2c: I2c,
    pub error: Error<I2c>,
}

impl<I2c> core::fmt::Debug for InitError<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

/// Error for sensor operations.
pub enum Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    WriteError(<I2c as Write>::Error),
    WriteReadError(<I2c as WriteRead>::Error),
    WrongDevice,
}

impl<I2c> core::fmt::Debug for Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            Error::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Error::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            Error::WrongDevice => f.write_str("WrongDevice"),
        }
    }
}
