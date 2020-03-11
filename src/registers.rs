use RegisterIndex::*;

#[derive(Default, Debug)]
pub struct Registers {
    // TODO: Make this an array for faster lookup and cleaner code
    a: u8, f: u8,
    b: u8, c: u8,
    d: u8, e: u8,
    h: u8, l: u8,
}

#[derive(Copy, Clone)]
pub enum RegisterIndex {
    A, F,
    B, C,
    D, E,
    H, L,
}

impl Registers {
    pub fn write_bc(&mut self, val: u16) {
        self.write_u16(B, C, val);
    }

    pub fn write_de(&mut self, val: u16) {
        self.write_u16(D, E, val);
    }

    pub fn write_hl(&mut self, val: u16) {
        self.write_u16(H, L, val);
    }

    pub fn read_bc(&self) -> u16 {
        self.read_u16(B, C)
    }

    pub fn read_de(&self) -> u16 {
        self.read_u16(D, E)
    }

    pub fn read_hl(&self) -> u16 {
        self.read_u16(H, L)
    }

    fn write_u16(&mut self, high: RegisterIndex, low: RegisterIndex, val: u16) {
        self[high] = ((val & 0xFF00) >> 8) as u8;
        self[low] = (val & 0x00FF) as u8;
    }

    fn read_u16(&self, high: RegisterIndex, low: RegisterIndex) -> u16 {
        ((self[high] as u16) << 8) | self[low] as u16
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
