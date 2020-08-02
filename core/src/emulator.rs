use crate::cpu::Cpu;
use crate::ppu::{Ppu, PixelColor};
use crate::memory::Memory;
use crate::bootrom::BOOTROM;

pub const DEBUG: bool = false;

pub struct Emulator {
    pub cpu: Cpu,
    pub ppu: Ppu,
    pub memory: Memory,
}

impl Emulator {
    pub fn new(rom: Vec<u8>, external_ram: Option<Vec<u8>>) -> Self {
        let mut memory = Memory::new(rom, external_ram);
        memory.load_bootrom(&BOOTROM);

        Emulator {
            cpu: Cpu::new(),
            ppu: Ppu::new(),
            memory,
        }
    }

    pub fn clock(&mut self) {
        self.cpu.step(&mut self.memory);
        self.ppu.clock(&mut self.memory);
        self.memory.sound_controller.clock();
        self.memory.clock();
    }

    pub fn get_screen_buffer(&self) -> Option<&[PixelColor]> {
        if self.memory.lcd_enabled() {
            Some(&self.ppu.get_screen_buffer())
        } else {
            None
        }
    }
}
