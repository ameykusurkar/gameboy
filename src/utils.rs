pub fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}

// Writes `x` to the nth bit in `byte`
pub fn set_bit(byte: u8, n: u8, x: bool) -> u8 {
    (byte & !(1 << n)) | ((x as u8) << n)
}
