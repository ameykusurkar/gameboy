use crate::memory::MemoryAccess;

pub struct Cartridge {
    rom: Vec<u8>,
}

impl Cartridge {
    pub fn new(rom: Vec<u8>) -> Self {
        Cartridge {
            rom,
        }
    }
}

impl MemoryAccess for Cartridge {
    fn read(&self, addr: u16) -> u8 {
        *self.rom.get(addr as usize).unwrap_or(&0xFF)
    }

    fn write(&mut self, _addr: u16, _byte: u8) {
        // TODO: Change this when implementing MBC1
        // Writes to cartridge happen when it has a memory bank controller
        // allowing us to access a larger ROM address space. We ignore
        // these writes for now
    }
}
