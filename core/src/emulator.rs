use crate::cpu::Cpu;
use crate::ppu::PixelColor;
use crate::memory::Memory;
use crate::bootrom::BOOTROM;

pub const DEBUG: bool = false;

pub struct Emulator {
    pub cpu: Cpu,
    pub memory: Memory,
}

impl Emulator {
    pub fn new(rom: Vec<u8>, external_ram: Option<Vec<u8>>) -> Self {
        let mut memory = Memory::new(rom, external_ram);
        memory.load_bootrom(&BOOTROM);

        Emulator {
            cpu: Cpu::new(),
            memory,
        }
    }

    pub fn clock(&mut self) {
        self.cpu.step(&mut self.memory);
        self.memory.clock();
    }

    pub fn get_screen_buffer(&self) -> Option<&[PixelColor]> {
        if self.memory.ppu.lcd_enabled() {
            Some(&self.memory.ppu.get_screen_buffer())
        } else {
            None
        }
    }
}
