#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum Register {
    Config = 0x1A,
    PwrMgmt1 = 0x6B,
    SmpRtDiv = 0x19,

    AccelX_H = 0x3B,
    AccelX_L = 0x3C,
    AccelY_H = 0x3D,
    AccelY_L = 0x3E,
    AccelZ_H = 0x3F,
    AccelZ_L = 0x40,

    AccelConfig = 0x1C,

    GyroX_H = 0x43,
    GyroX_L = 0x44,
    GyroY_H = 0x45,
    GyroY_L = 0x46,
    GyroZ_H = 0x47,
    GyroZ_L = 0x48,

    GyroConfig = 0x1B,

    UserCtrl = 0x6A,
    IntEnable = 0x38,

    FifoEn = 0x23,
    FifoCount_H = 0x72,
    FifoCount_L = 0x73,
    FifoRw = 0x74,

    // ---
    BankSel = 0x6D,
    MemStartAddr = 0x6E,
    MemRw = 0x6F,
    PrgmStart = 0x70,
    DmpConfig = 0x71,
}
