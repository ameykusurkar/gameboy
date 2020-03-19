use RegisterIndex::*;

pub const ZERO_FLAG: u8       = 7;
pub const SUBTRACT_FLAG: u8   = 6;
pub const HALF_CARRY_FLAG: u8 = 5;
pub const CARRY_FLAG: u8      = 4;

#[derive(Default, Debug)]
pub struct Registers {
    // TODO: Make this an array for faster lookup and cleaner code
    a: u8, f: u8,
    b: u8, c: u8,
    d: u8, e: u8,
    h: u8, l: u8,
}

#[derive(Copy, Clone, Debug)]
pub enum RegisterIndex {
    A, F,
    B, C,
    D, E,
    H, L,
}

#[derive(Copy, Clone)]
pub enum TwoRegisterIndex {
    AF,
    BC,
    DE,
    HL,
}

#[derive(Default, Copy, Clone)]
pub struct Flags {
    pub zero: bool,
    pub subtract: bool,
    pub half_carry: bool,
    pub carry: bool,
}

impl TwoRegisterIndex {
    fn split_index(&self) -> (RegisterIndex, RegisterIndex) {
        match self {
            TwoRegisterIndex::AF => (A, F),
            TwoRegisterIndex::BC => (B, C),
            TwoRegisterIndex::DE => (D, E),
            TwoRegisterIndex::HL => (H, L),
        }
    }
}

impl Registers {
    pub fn write(&mut self, index: TwoRegisterIndex, val: u16) {
        let (high, low) = index.split_index();
        self[high] = ((val & 0xFF00) >> 8) as u8;
        self[low] = (val & 0x00FF) as u8;
    }

    pub fn read(&self, index: TwoRegisterIndex) -> u16 {
        let (high, low) = index.split_index();
        ((self[high] as u16) << 8) | self[low] as u16
    }

    pub fn write_flags(&mut self, flags: Flags) {
        self[F] = u8::from(flags);
    }
}

impl std::ops::Index<RegisterIndex> for Registers {
    type Output = u8;

    fn index(&self, index: RegisterIndex) -> &Self::Output {
        match index {
            RegisterIndex::A => &self.a,
            RegisterIndex::F => &self.f,

            RegisterIndex::B => &self.b,
            RegisterIndex::C => &self.c,

            RegisterIndex::D => &self.d,
            RegisterIndex::E => &self.e,

            RegisterIndex::H => &self.h,
            RegisterIndex::L => &self.l,
        }
    }
}

impl std::ops::IndexMut<RegisterIndex> for Registers {
    fn index_mut(&mut self, index: RegisterIndex) -> &mut Self::Output {
        match index {
            RegisterIndex::A => &mut self.a,
            RegisterIndex::F => &mut self.f,

            RegisterIndex::B => &mut self.b,
            RegisterIndex::C => &mut self.c,

            RegisterIndex::D => &mut self.d,
            RegisterIndex::E => &mut self.e,

            RegisterIndex::H => &mut self.h,
            RegisterIndex::L => &mut self.l,
        }
    }
}

impl std::convert::From<Flags> for u8 {
    fn from(flags: Flags) -> Self {
        (flags.zero as u8) << ZERO_FLAG
            | (flags.subtract as u8) << SUBTRACT_FLAG
            | (flags.half_carry as u8) << HALF_CARRY_FLAG
            | (flags.carry as u8) << CARRY_FLAG
    }
}
