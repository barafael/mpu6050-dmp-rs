#[derive(Debug, Copy, Clone)]
pub enum ClockSource {
    Internal = 0,
    Xgyro = 1,
    Ygyro = 2,
    Zgyro = 3,
    External32768 = 4,
    External19200 = 5,
    Stop = 7,
}
