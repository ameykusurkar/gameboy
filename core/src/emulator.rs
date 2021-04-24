use crate::bootrom::{BOOTROM, CGB_BOOTROM};
use crate::cpu::Cpu;
use crate::memory::Memory;
use crate::ppu::PixelColor;

pub const DEBUG: bool = false;

pub struct Emulator {
    pub cpu: Cpu,
    pub memory: Memory,
}

const FORCE_DMG: bool = false;

impl Emulator {
    pub fn new(rom: Vec<u8>, external_ram: Option<Vec<u8>>) -> Self {
        let mut memory = Memory::new(rom, external_ram);

        let is_cgb = !FORCE_DMG && memory.is_cgb();

        let bootrom = get_bootrom(is_cgb);
        memory.load_bootrom(bootrom);

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

fn get_bootrom(cgb: bool) -> Vec<u8> {
    if cgb {
        CGB_BOOTROM.to_vec()
    } else {
        BOOTROM.to_vec()
    }
}
