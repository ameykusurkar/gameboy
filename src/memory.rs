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

    pub fn get_memory(&self) -> &[u8] {
        &self.memory
    }

    fn is_bootrom_active(&self) -> bool {
        (self.memory[0xFF50] & 0x01) == 0
    }

    pub fn master_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    pub fn master_write(&mut self, addr: u16, val: u8) {
        if addr == 0xFF02 && self.memory[addr as usize] == 0x81 {
            println!("SERIAL: {}", self.memory[0xFF01 as usize] as char);
        }

        self.memory[addr as usize] = val;
    }

    pub fn cpu_read(&self, addr: u16) -> u8 {
        if addr < 0x100 && self.is_bootrom_active() {
            self.bootrom[addr as usize]
        } else {
            self.memory[addr as usize]
        }
    }

    pub fn cpu_write(&mut self, addr: u16, val: u8) {
        if addr == 0xFF02 && self.memory[addr as usize] == 0x81 {
            println!("SERIAL: {}", self.memory[0xFF01 as usize] as char);
        }

        self.memory[addr as usize] = val;
    }
}
