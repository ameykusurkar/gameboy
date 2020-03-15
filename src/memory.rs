pub struct Memory {
    memory: [u8; 1 << 16],
    bootrom: [u8; 256],
}

impl Memory {
    pub fn new() -> Memory {
        Memory {
            memory: [0; 1 << 16],
            bootrom: [0; 256],
        }
    }

    pub fn load_bootrom(&mut self, buffer: &[u8]) {
        self.bootrom.copy_from_slice(buffer);
    }

    pub fn load_rom(&mut self, buffer: &[u8]) {
        let rom_segment = &mut self.memory[0..buffer.len()];
        rom_segment.copy_from_slice(buffer);
    }
}

impl std::ops::Index<u16> for Memory {
    type Output = u8;

    fn index(&self, addr: u16) -> &Self::Output {
        &self.memory[addr as usize]
    }
}

impl std::ops::IndexMut<u16> for Memory {
    fn index_mut(&mut self, addr: u16) -> &mut Self::Output {
        &mut self.memory[addr as usize]
    }
}
