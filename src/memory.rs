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

    pub fn get_vram(&self) -> &[u8] {
        &self.memory[0x8000..0xA000]
    }

    fn is_bootrom_active(&self) -> bool {
        (self.memory[0xFF50] & 0x01) == 0
    }
}

impl std::ops::Index<u16> for Memory {
    type Output = u8;

    fn index(&self, addr: u16) -> &Self::Output {
        if addr < 0x100 && self.is_bootrom_active() {
            &self.bootrom[addr as usize]
        } else {
            &self.memory[addr as usize]
        }
    }
}

impl std::ops::IndexMut<u16> for Memory {
    fn index_mut(&mut self, addr: u16) -> &mut Self::Output {
        &mut self.memory[addr as usize]
    }
}
