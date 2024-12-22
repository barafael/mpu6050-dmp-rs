#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Register {
    Config = 0x1A,
    PwrMgmt1 = 0x6B,
    SmpRtDiv = 0x19,

    AccelOffsetX_H = 0x06,
    AccelOffsetX_L = 0x07,
    AccelOffsetY_H = 0x08,
    AccelOffsetY_L = 0x09,
    AccelOffsetZ_H = 0x0A,
    AccelOffsetZ_L = 0x0B,

    GyroOffsetX_H = 0x13,
    GyroOffsetX_L = 0x14,
    GyroOffsetY_H = 0x15,
    GyroOffsetY_L = 0x16,
    GyroOffsetZ_H = 0x17,
    GyroOffsetZ_L = 0x18,

    AccelX_H = 0x3B,
    AccelX_L = 0x3C,
    AccelY_H = 0x3D,
    AccelY_L = 0x3E,
    AccelZ_H = 0x3F,
    AccelZ_L = 0x40,

    TempOut_H = 0x41,
    TempOut_L = 0x42,

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
    IntStatus = 0x3A,

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
