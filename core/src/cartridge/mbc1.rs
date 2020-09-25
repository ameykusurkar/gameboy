use crate::cartridge::Mbc;

enum BankingMode {
    ROM,
    RAM,
}

pub struct Mbc1 {
    rom_bank_lower_bits: u8,
    upper_bits: u8,
    ram_enabled: bool,
    banking_mode: BankingMode,
    has_battery: bool,
    write_since_last_save: bool,
}

impl Mbc1 {
    pub fn new(has_battery: bool) -> Self {
        Mbc1 {
            rom_bank_lower_bits: 1,
            upper_bits: 0,
            ram_enabled: false,
            banking_mode: BankingMode::ROM,
            has_battery,
            write_since_last_save: false,
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
            0x0000..=0x3FFF => rom[self.get_bank0_rom_addr(rom, addr)],
            0x4000..=0x7FFF => rom[self.get_bank1_rom_addr(rom, addr)],
            _ => unreachable!("Invalid rom read address: {:04x}", addr),
        }
    }

    fn write_rom(&mut self, _rom: &mut [u8], addr: u16, byte: u8) {
        match addr {
            0x0000..=0x1FFF => {
                self.ram_enabled = (byte & 0x0F) == 0x0A;
            }
            0x2000..=0x3FFF => {
                let byte = byte & 0b0001_1111;
                self.rom_bank_lower_bits = if byte == 0 { 1 } else { byte };
            }
            0x4000..=0x5FFF => {
                self.upper_bits = byte & 0b000_0011;
            }
            0x6000..=0x7FFF => {
                self.banking_mode = match byte & 0b0000_0001 {
                    0 => BankingMode::ROM,
                    1 => BankingMode::RAM,
                    _ => unreachable!("Invalid cartridge banking mode: {:04x}", byte),
                }
            }
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

        self.write_since_last_save = true;
    }

    fn has_battery(&self) -> bool {
        self.has_battery
    }

    fn should_save_ram(&self) -> bool {
        self.has_battery && self.write_since_last_save
    }

    fn mark_ram_as_saved(&mut self) {
        self.write_since_last_save = false;
    }
}
