use crate::memory::MemoryAccess;

pub struct Cartridge {
    rom: Vec<u8>,
    ram: Vec<u8>,
    rom_bank_lower_bits: u8,
    upper_bits: u8,
    ram_enabled: bool,
    banking_mode: BankingMode,
}

enum BankingMode {
    ROM,
    RAM,
}

impl Cartridge {
    pub fn new(rom: Vec<u8>) -> Self {
        let ram_size = match rom[0x0149] {
            0x00 => 0,
            0x01 => 1 << (10 + 1), // 2KB
            0x02 => 1 << (10 + 3), // 8KB
            _    => 1 << (10 + 5), // 32KB
        };

        println!("CARTRIDGE TYPE: {}", rom[0x0147]);
        Cartridge {
            rom,
            ram: vec![0; ram_size],
            rom_bank_lower_bits: 1,
            upper_bits: 0,
            ram_enabled: false,
            banking_mode: BankingMode::ROM,
        }
    }

    fn get_real_ram_addr(&self, addr: u16) -> Option<usize> {
        if self.ram.is_empty() {
            return None;
        }

        let ram_bank = match self.banking_mode {
            BankingMode::ROM => 0,
            BankingMode::RAM => self.upper_bits as usize,
        };

        let real_addr = (ram_bank << 13 | (addr as usize) & 0x1FFF) % self.ram.len();
        Some(real_addr)
    }

    fn get_bank0_rom_addr(&self, addr: u16) -> usize {
        let upper_bits = match self.banking_mode {
            BankingMode::ROM => 0,
            BankingMode::RAM => self.upper_bits as usize,
        };

        let rom_bank = upper_bits << 5;
        self.build_rom_addr(rom_bank, addr)
    }

    fn get_bank1_rom_addr(&self, addr: u16) -> usize {
        let upper_bits = match self.banking_mode {
            BankingMode::ROM => self.upper_bits as usize,
            BankingMode::RAM => 0,
        };

        let rom_bank = upper_bits << 5 | (self.rom_bank_lower_bits as usize);
        self.build_rom_addr(rom_bank, addr)
    }

    fn build_rom_addr(&self, rom_bank: usize, addr: u16) -> usize {
        let addr = addr as usize;
        (rom_bank << 14 | addr & 0x3FFF) % self.rom.len()
    }
}

impl MemoryAccess for Cartridge {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => {
                self.rom[self.get_bank0_rom_addr(addr)]
            },
            0x4000..=0x7FFF => {
                self.rom[self.get_bank1_rom_addr(addr)]
            },
            0xA000..=0xBFFF => {
                let real_addr = self.get_real_ram_addr(addr);

                if self.ram_enabled && real_addr.is_some() {
                    self.ram[real_addr.unwrap()]
                } else {
                    0xFF
                }
            },
            _ => unreachable!("Invalid cartridge read address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0x0000..=0x1FFF => {
                self.ram_enabled = (byte & 0x0F) == 0x0A;
            },
            0x2000..=0x3FFF => {
                let byte = byte & 0b0001_1111;
                self.rom_bank_lower_bits = if byte == 0 { 1 } else { byte };
            },
            0x4000..=0x5FFF => {
                self.upper_bits = byte & 0b000_0011;
            },
            0x6000..=0x7FFF => {
                self.banking_mode = match byte & 0b0000_0001 {
                    0 => BankingMode::ROM,
                    1 => BankingMode::RAM,
                    _ => unreachable!("Invalid cartridge banking mode: {:04x}", byte),
                }
            },
            0xA000..=0xBFFF => {
                let real_addr = self.get_real_ram_addr(addr);

                if self.ram_enabled && real_addr.is_some() {
                    self.ram[real_addr.unwrap()] = byte;
                }
            },
            _ => unreachable!("Invalid cartridge write address: {:04x}", addr),
        }
    }
}
