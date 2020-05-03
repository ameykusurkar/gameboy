use crate::cartridge::Mbc;

pub struct Mbc3 {
    rom_bank: u8,
    ram_bank: u8,
    // Also enables the Real Time Clock (RTC) register
    ram_enabled: bool,
}

impl Mbc3 {
    pub fn new() -> Self {
        Mbc3 {
            rom_bank: 1,
            ram_bank: 0,
            ram_enabled: false,
        }
    }

    fn get_real_ram_addr(&self, ram: &[u8], addr: u16) -> Option<usize> {
        if ram.is_empty() { return None; }

        let real_addr = ((self.ram_bank as usize) << 13 | (addr as usize) & 0x1FFF) % ram.len();
        Some(real_addr)
    }

    fn build_rom_addr(&self, rom: &[u8], rom_bank: usize, addr: u16) -> usize {
        let addr = addr as usize;
        (rom_bank << 14 | addr & 0x3FFF) % rom.len()
    }
}

impl Mbc for Mbc3 {
    fn read_rom(&self, rom: &[u8], addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => {
                let rom_addr = self.build_rom_addr(rom, 0, addr);
                rom[rom_addr]
            },
            0x4000..=0x7FFF => {
                let rom_addr = self.build_rom_addr(rom, self.rom_bank as usize, addr);
                rom[rom_addr]
            },
            _ => unreachable!("Invalid rom read address: {:04x}", addr),
        }
    }

    fn write_rom(&mut self, _rom: &mut [u8], addr: u16, byte: u8) {
        match addr {
            0x0000..=0x1FFF => {
                self.ram_enabled = (byte & 0x0F) == 0x0A;
            },
            0x2000..=0x3FFF => {
                let byte = byte & 0b0111_1111;
                self.rom_bank = if byte == 0 { 1 } else { byte };
            },
            0x4000..=0x5FFF => {
                match byte {
                    0x00..=0x03 => self.ram_bank = byte,
                    0x08..=0x0C => unimplemented!("Yet to implement RTC registers!"),
                    _ => unreachable!("Invalid RAM bank/RTC select: {:02x}", byte),
                }
            },
            0x6000..=0x7FFF => {
                // TODO: Latches clock data, RTC registers yet to be implemented
            },
            _ => unreachable!("Invalid rom write address: {:04x}", addr),
        }
    }

    fn read_ram(&self, ram: &[u8], addr: u16) -> u8 {
        let real_addr = self.get_real_ram_addr(ram, addr);

        if self.ram_enabled && real_addr.is_some() {
            ram[real_addr.unwrap()]
        } else {
            0xFF
        }
    }

    fn write_ram(&mut self, ram: &mut [u8], addr: u16, byte: u8) {
        let real_addr = self.get_real_ram_addr(ram, addr);

        if self.ram_enabled && real_addr.is_some() {
            ram[real_addr.unwrap()] = byte;
        }
    }
}
