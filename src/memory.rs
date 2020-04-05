use crate::ppu::LcdMode;

pub struct Memory {
    memory: [u8; 1 << 16],
    bootrom: [u8; 256],
}

const VRAM_RANGE: std::ops::Range<usize> = 0x8000..0xA000;
const OAM_RANGE: std::ops::Range<usize>  = 0xFE00..0xFEA0;

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

    fn is_bootrom_active(&self) -> bool {
        (self.memory[0xFF50] & 0x01) == 0
    }

    pub fn cpu_read(&self, addr: u16) -> u8 {
        let addr = addr as usize;

        if addr < 0x100 && self.is_bootrom_active() {
            self.bootrom[addr]
        } else if VRAM_RANGE.contains(&addr) && self.vram_blocked() {
            0xFF
        } else if OAM_RANGE.contains(&addr) && self.oam_blocked() {
            0xFF
        } else if addr == 0xFF00 {
            // Hardcode joypad register to have all buttons unpressed for now
            0x0F
        } else {
            self.memory[addr]
        }
    }

    pub fn cpu_write(&mut self, addr: u16, val: u8) {
        // Hard-coded because test roms write results to serial port
        if addr == 0xFF02 && self.memory[addr as usize] == 0x81 {
            println!("SERIAL: {}", self.memory[0xFF01 as usize] as char);
        }

        match addr {
            // Cartridge
            0x0000..=0x7FFF => {
                // TODO: Change this when implementing MBC1
                // Writes to cartridge happen when it has a memory bank controller
                // allowing us to access a larger ROM address space. We ignore
                // these writes for now
            },
            // VRAM
            0x8000..=0x9FFF => {
                if !self.vram_blocked() {
                    self.memory[addr as usize] = val;
                }
            },
            // OAM
            0xFE00..=0xFE9F => {
                if !self.oam_blocked() {
                    self.memory[addr as usize] = val;
                }
            },
            // DMA Transfer
            0xFF46 => {
                // A write to this address indicates a request to copy memory to
                // the OAM. Technically this takes 40 machine cycles, but for now
                // we will do it instantly.
                let start = (val as usize) * 0x100;
                for i in 0..0xA0 {
                    self.memory[0xFE00 + i] = self.memory[start + i];
                }
            },
            _ => self.memory[addr as usize] = val,
        }
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
}

fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}
