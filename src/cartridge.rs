use crate::memory::MemoryAccess;

pub struct Cartridge {
    rom: Vec<u8>,
    ram: Vec<u8>,
    mbc: Box<dyn Mbc>,
}

trait Mbc {
    fn read_rom(&self, rom: &[u8], addr: u16) -> u8;
    fn write_rom(&mut self, rom: &mut [u8], addr: u16, byte: u8);

    fn read_ram(&self, ram: &[u8], addr: u16) -> u8;
    fn write_ram(&mut self, ram: &mut [u8], addr: u16, byte: u8);
}

impl Cartridge {
    pub fn new(rom: Vec<u8>) -> Self {
        let ram_size = match rom[0x0149] {
            0x00 => 0,
            0x01 => 1 << (10 + 1), // 2KB
            0x02 => 1 << (10 + 3), // 8KB
            _    => 1 << (10 + 5), // 32KB
        };

        println!("CARTRIDGE TYPE: {:02x}h", rom[0x0147]);
        let mbc: Box<dyn Mbc> = match rom[0x147] {
            0x00 => Box::new(NoMbc::default()),
            0x01..=0x03 => Box::new(Mbc1::new()),
            0x0F | 0x10 => unimplemented!("MBC Timer not implemented!"),
            0x11..=0x13 => Box::new(Mbc3::new()),
            _ => unimplemented!("Unimplemented cartridge type: {:02x}h", rom[0x147]),
        };

        Cartridge {
            rom,
            ram: vec![0; ram_size],
            mbc,
        }
    }
}

impl MemoryAccess for Cartridge {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x7FFF => {
                self.mbc.read_rom(&self.rom, addr)
            },
            0xA000..=0xBFFF => {
                self.mbc.read_ram(&self.ram, addr)
            },
            _ => unreachable!("Invalid cartridge read address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0x0000..=0x7FFF => {
                self.mbc.write_rom(&mut self.rom, addr, byte)
            },
            0xA000..=0xBFFF => {
                self.mbc.write_ram(&mut self.ram, addr, byte)
            },
            _ => unreachable!("Invalid cartridge write address: {:04x}", addr),
        }
    }
}

#[derive(Default)]
struct NoMbc;

impl Mbc for NoMbc {
    fn read_rom(&self, rom: &[u8], addr: u16) -> u8 {
        rom[addr as usize]
    }

    fn write_rom(&mut self, _rom: &mut [u8], _addr: u16, _byte: u8) {}
    fn read_ram(&self, _ram: &[u8], _addr: u16) -> u8 { 0xFF }
    fn write_ram(&mut self, _ram: &mut [u8], _addr: u16, _byte: u8) {}
}

enum BankingMode {
    ROM,
    RAM,
}

struct Mbc1 {
    rom_bank_lower_bits: u8,
    upper_bits: u8,
    ram_enabled: bool,
    banking_mode: BankingMode,
}

impl Mbc1 {
    fn new() -> Mbc1 {
        Mbc1 {
            rom_bank_lower_bits: 1,
            upper_bits: 0,
            ram_enabled: false,
            banking_mode: BankingMode::ROM,
        }
    }

    fn get_real_ram_addr(&self, ram: &[u8], addr: u16) -> Option<usize> {
        if ram.is_empty() {
            return None;
        }

        let ram_bank = match self.banking_mode {
            BankingMode::ROM => 0,
            BankingMode::RAM => self.upper_bits as usize,
        };

        let real_addr = (ram_bank << 13 | (addr as usize) & 0x1FFF) % ram.len();
        Some(real_addr)
    }

    fn get_bank0_rom_addr(&self, rom: &[u8], addr: u16) -> usize {
        let upper_bits = match self.banking_mode {
            BankingMode::ROM => 0,
            BankingMode::RAM => self.upper_bits as usize,
        };

        let rom_bank = upper_bits << 5;
        self.build_rom_addr(rom, rom_bank, addr)
    }

    fn get_bank1_rom_addr(&self, rom: &[u8], addr: u16) -> usize {
        let upper_bits = match self.banking_mode {
            BankingMode::ROM => self.upper_bits as usize,
            BankingMode::RAM => 0,
        };

        let rom_bank = upper_bits << 5 | (self.rom_bank_lower_bits as usize);
        self.build_rom_addr(rom, rom_bank, addr)
    }

    fn build_rom_addr(&self, rom: &[u8], rom_bank: usize, addr: u16) -> usize {
        let addr = addr as usize;
        (rom_bank << 14 | addr & 0x3FFF) % rom.len()
    }
}

impl Mbc for Mbc1 {
    fn read_rom(&self, rom: &[u8], addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => {
                rom[self.get_bank0_rom_addr(rom, addr)]
            },
            0x4000..=0x7FFF => {
                rom[self.get_bank1_rom_addr(rom, addr)]
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

struct Mbc3 {
    rom_bank: u8,
    ram_bank: u8,
    // Also enables the Real Time Clock (RTC) register
    ram_enabled: bool,
}

impl Mbc3 {
    fn new() -> Self {
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
