#![no_std]

pub mod accel;
pub mod address;
pub mod clock_source;
pub mod config;
mod dmp_firmware;
pub mod error;
pub mod euler;
pub mod fifo;
mod firmware_loader;
pub mod gravity;
pub mod gyro;
pub mod quaternion;
pub mod registers;
pub mod sensor;
pub mod yaw_pitch_roll;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
