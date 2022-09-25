#[derive(Copy, Clone, Debug)]
pub struct Address(pub u8);

impl Default for Address {
    fn default() -> Self {
        Self(0x68)
    }
}

impl From<Address> for u8 {
    fn from(addr: Address) -> Self {
        addr.0
    }
}

impl From<u8> for Address {
    fn from(addr: u8) -> Self {
        Self(addr)
    }
}
