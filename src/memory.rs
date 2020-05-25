use crate::ppu::LcdMode;
use crate::joypad::Joypad;
use crate::cartridge::Cartridge;
use crate::sound_controller::SoundController;

use crate::utils::read_bit;

pub struct Memory {
    cartridge: Cartridge,
    memory: [u8; 1 << 16],
    bootrom: [u8; 256],
    pub joypad: Joypad,
    pub sound_controller: SoundController,
}

pub trait MemoryAccess {
    fn read(&self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, byte: u8);
}

const VRAM_RANGE: std::ops::Range<usize> = 0x8000..0xA000;
const OAM_RANGE: std::ops::Range<usize>  = 0xFE00..0xFEA0;

impl Memory {
    pub fn new(rom: Vec<u8>, external_ram: Option<Vec<u8>>) -> Memory {
        Memory {
            cartridge: Cartridge::new(rom, external_ram),
            memory: [0; 1 << 16],
            bootrom: [0; 256],
            joypad: Joypad::default(),
            sound_controller: SoundController::new(),
        }
    }

    pub fn load_bootrom(&mut self, buffer: &[u8]) {
        self.bootrom.copy_from_slice(buffer);
    }

    fn is_bootrom_active(&self) -> bool {
        (self.memory[0xFF50] & 0x01) == 0
    }

    pub fn cpu_read(&self, addr: u16) -> u8 {
        let addr = addr as usize;

        // TODO: Refactor how memory access is delegated
        if addr < 0x100 && self.is_bootrom_active() {
            self.bootrom[addr]
        } else if (0x0000..0x8000).contains(&addr) {
            self.cartridge.read(addr as u16)
        } else if (0xA000..0xC000).contains(&addr) {
            self.cartridge.read(addr as u16)
        } else if VRAM_RANGE.contains(&addr) && self.vram_blocked() {
            0xFF
        } else if OAM_RANGE.contains(&addr) && self.oam_blocked() {
            0xFF
        } else if addr == 0xFF00 {
            self.joypad.read(addr as u16)
        } else if Self::is_sound_addr(addr as u16) {
            self.sound_controller.read(addr as u16)
        } else {
            self.memory[addr]
        }
    }

    pub fn cpu_write(&mut self, addr: u16, val: u8) {
        // Hard-coded because test roms write results to serial port
        // if addr == 0xFF02 && self.memory[addr as usize] == 0x81 {
        //     println!("SERIAL: {}", self.memory[0xFF01 as usize] as char);
        // }

        match addr {
            // Cartridge
            0x0000..=0x7FFF => {
                self.cartridge.write(addr, val);
            },
            // VRAM
            0x8000..=0x9FFF => {
                if !self.vram_blocked() {
                    self.memory[addr as usize] = val;
                }
            },
            // External RAM, also handled by cartridge
            0xA000..=0xBFFF => {
                self.cartridge.write(addr, val);
            },
            // OAM
            0xFE00..=0xFE9F => {
                if !self.oam_blocked() {
                    self.memory[addr as usize] = val;
                }
            },
            // Joypad
            0xFF00 => {
                self.joypad.write(addr, val);
            },
            _ if Self::is_sound_addr(addr) => {
                self.sound_controller.write(addr, val);
            },
            // DMA Transfer
            0xFF46 => {
                // A write to this address indicates a request to copy memory to
                // the OAM. Technically this takes 40 machine cycles, but for now
                // we will do it instantly.
                let start = (val as usize) * 0x100;
                self.memory.copy_within(start..start+0xA0, 0xFE00);
            },
            _ => self.memory[addr as usize] = val,
        }
    }

    fn is_sound_addr(addr: u16) -> bool {
        (0xFF10..=0xFF14).contains(&addr)
            || (0xFF16..=0xFF19).contains(&addr)
            || addr == 0xFF24 || addr == 0xFF25 || addr == 0xFF26
    }

    fn vram_blocked(&self) -> bool {
        self.lcd_enabled() && self.get_lcd_mode() == LcdMode::PixelTransfer
    }

    fn oam_blocked(&self) -> bool {
        let lcd_mode = self.get_lcd_mode();
        self.lcd_enabled() && (lcd_mode == LcdMode::PixelTransfer && lcd_mode == LcdMode::OAMSearch)
    }

    pub fn lcd_enabled(&self) -> bool {
        read_bit(self.memory[0xFF40], 7)
    }

    pub fn ppu_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    pub fn ppu_read_range(&self, addr_range: std::ops::Range<usize>) -> &[u8] {
        &self.memory[addr_range]
    }

    pub fn ppu_write(&mut self, addr: u16, val: u8) {
        self.memory[addr as usize] = val;
    }

    pub fn get_lcd_mode(&self) -> LcdMode {
        match self.memory[0xFF41] & 0b11 {
            0 => LcdMode::HBlank,
            1 => LcdMode::VBlank,
            2 => LcdMode::OAMSearch,
            3 => LcdMode::PixelTransfer,
            _ => panic!("Not a valid LCD mode!"),
        }
    }

    pub fn get_external_ram(&self) -> Option<&[u8]> {
        if self.cartridge.mbc.should_save_ram() {
            Some(&self.cartridge.ram)
        } else {
            None
        }
    }

    pub fn mark_external_ram_as_saved(&mut self) {
        self.cartridge.mbc.mark_ram_as_saved()
    }
}
