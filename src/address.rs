#[derive(Copy, Clone, Debug)]
pub struct Address(u8);

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
