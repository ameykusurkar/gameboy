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

    sp_high: u8, sp_low: u8,
    pc_high: u8, pc_low: u8,
    temp_high: u8, temp_low: u8,
}

#[derive(PartialEq, Copy, Clone, Debug)]
pub enum RegisterIndex {
    A, F,
    B, C,
    D, E,
    H, L,
    SPHigh, SPLow,
    PCHigh, PCLow,
    TempHigh, TempLow,
}

#[derive(Copy, Clone, Debug)]
pub enum TwoRegisterIndex {
    AF,
    BC,
    DE,
    HL,
    SP,
    PC,
    Temp16,
}

#[derive(Default, Copy, Clone)]
pub struct Flags {
    pub zero: bool,
    pub subtract: bool,
    pub half_carry: bool,
    pub carry: bool,
}

impl TwoRegisterIndex {
    pub fn split_index(&self) -> (RegisterIndex, RegisterIndex) {
        match self {
            TwoRegisterIndex::AF => (A, F),
            TwoRegisterIndex::BC => (B, C),
            TwoRegisterIndex::DE => (D, E),
            TwoRegisterIndex::HL => (H, L),
            TwoRegisterIndex::SP => (SPHigh, SPLow),
            TwoRegisterIndex::PC => (PCHigh, PCLow),
            TwoRegisterIndex::Temp16 => (TempHigh, TempLow),
        }
    }
}

impl Registers {
    pub fn write16(&mut self, index: TwoRegisterIndex, val: u16) {
        let (high, low) = index.split_index();
        self[high] = ((val & 0xFF00) >> 8) as u8;
        match low {
            // Bits 0-3 of the flags register are unused and always stay zeroed
            F => self[low] = (val & 0x00F0) as u8,
            _ => self[low] = (val & 0x00FF) as u8,
        };
    }

    pub fn read16(&self, index: TwoRegisterIndex) -> u16 {
        let (high, low) = index.split_index();
        ((self[high] as u16) << 8) | self[low] as u16
    }

    pub fn write_flags(&mut self, flags: Flags) {
        self[F] = u8::from(flags);
    }

    pub fn read_flags(&self) -> Flags {
        Flags::from(self[F])
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

            RegisterIndex::SPHigh => &self.sp_high,
            RegisterIndex::SPLow => &self.sp_low,

            RegisterIndex::PCHigh => &self.pc_high,
            RegisterIndex::PCLow => &self.pc_low,

            RegisterIndex::TempHigh => &self.temp_high,
            RegisterIndex::TempLow => &self.temp_low,
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

            RegisterIndex::SPHigh => &mut self.sp_high,
            RegisterIndex::SPLow => &mut self.sp_low,

            RegisterIndex::PCHigh => &mut self.pc_high,
            RegisterIndex::PCLow => &mut self.pc_low,

            RegisterIndex::TempHigh => &mut self.temp_high,
            RegisterIndex::TempLow => &mut self.temp_low,
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

impl std::convert::From<u8> for Flags {
    fn from(flags: u8) -> Self {
        Flags {
            zero: flags & (1 << ZERO_FLAG) > 0,
            subtract: flags & (1 << SUBTRACT_FLAG) > 0,
            half_carry: flags & (1 << HALF_CARRY_FLAG) > 0,
            carry: flags & (1 << CARRY_FLAG) > 0,
        }
    }
}
