use std::path::PathBuf;
use std::fs::File;
use std::io::Write;

use crate::cpu::Cpu;
use crate::ppu::Ppu;
use crate::memory::Memory;

pub const DEBUG: bool = false;

pub struct Emulator {
    pub cpu: Cpu,
    pub ppu: Ppu,
    pub memory: Memory,
    save_path: PathBuf,
}

impl Emulator {
    pub fn new(bootrom: &[u8], rom: Vec<u8>,
               external_ram: Option<Vec<u8>>,
               save_path: PathBuf) -> Self {
        let mut memory = Memory::new(rom, external_ram);
        memory.load_bootrom(bootrom);

        Emulator {
            cpu: Cpu::new(),
            ppu: Ppu::new(),
            memory,
            save_path,
        }
    }

    pub fn clock(&mut self) {
        self.cpu.step(&mut self.memory);
        self.ppu.clock(&mut self.memory);
        self.memory.sound_controller.clock();
    }

    pub fn save_external_ram(&mut self) {
        self.memory.get_external_ram()
            .and_then(|ram| {
                File::create(&self.save_path)
                    .and_then(|mut f| f.write_all(ram))
                    .ok()
            }).map(|_| self.memory.mark_external_ram_as_saved());
    }

    pub fn get_screen_buffer(&self) -> Option<&[u8]> {
        if self.memory.lcd_enabled() {
            Some(&self.ppu.get_screen_buffer())
        } else {
            None
        }
    }
}
