use crate::cartridge::Mbc;
use chrono::{Timelike, Datelike};

pub struct Mbc3 {
    rom_bank: u8,
    ram_select: RamSelect,
    // Also enables the Real Time Clock (RTC) register
    ram_enabled: bool,
    has_battery: bool,
    write_since_last_save: bool,
    rtc_registers: RtcRegisters,
    latch_waiting: bool,
}

enum RamSelect {
    BankNumber(u8),
    RTCRegister(u8),
}

#[derive(Default)]
struct RtcRegisters {
    registers: [u8; 5],
}

impl RtcRegisters {
    fn latch(&mut self) {
        let now = chrono::Utc::now();
        self.registers[0] = now.second() as u8;
        self.registers[1] = now.minute() as u8;
        self.registers[2] = now.hour() as u8;
        self.registers[3] = now.ordinal0() as u8;
    }
}

impl Mbc3 {
    pub fn new(has_battery: bool) -> Self {
        Mbc3 {
            rom_bank: 1,
            ram_select: RamSelect::BankNumber(0),
            ram_enabled: false,
            has_battery,
            write_since_last_save: false,
            rtc_registers: RtcRegisters::default(),
            latch_waiting: false,
        }
    }

    fn get_real_ram_addr(ram: &[u8], ram_bank: u8, addr: u16) -> Option<usize> {
        if ram.is_empty() { return None; }

        let real_addr = ((ram_bank as usize) << 13 | (addr as usize) & 0x1FFF) % ram.len();
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
                let byte = byte & 0x0F;
                match byte {
                    0x00..=0x03 => self.ram_select = RamSelect::BankNumber(byte),
                    0x08..=0x0C => self.ram_select = RamSelect::RTCRegister(byte),
                    _ => (),
                }
            },
            0x6000..=0x7FFF => {
                match byte {
                    0x00 => {
                        self.latch_waiting = true;
                    },
                    0x01 => {
                        if self.latch_waiting {
                            self.rtc_registers.latch();
                        }

                        self.latch_waiting = false;
                    },
                    _ => (),
                }
            },
            _ => unreachable!("Invalid rom write address: {:04x}", addr),
        }
    }

    fn read_ram(&self, ram: &[u8], addr: u16) -> u8 {
        if !self.ram_enabled {
            return 0xFF;
        }

        match self.ram_select {
            RamSelect::BankNumber(ram_bank) => {
                Self::get_real_ram_addr(ram, ram_bank, addr)
                    .map_or(0xFF, |real_addr| ram[real_addr])
            },
            RamSelect::RTCRegister(reg_no) => {
                self.rtc_registers.registers[reg_no as usize - 0x8]
            },
        }
    }

    fn write_ram(&mut self, ram: &mut [u8], addr: u16, byte: u8) {
        if !self.ram_enabled { return; }

        match self.ram_select {
            RamSelect::BankNumber(ram_bank) => {
                Self::get_real_ram_addr(ram, ram_bank, addr)
                    .map(|real_addr| ram[real_addr] = byte);
            },
            RamSelect::RTCRegister(reg_no) => {
                self.rtc_registers.registers[reg_no as usize - 0x8] = byte;
            },
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
