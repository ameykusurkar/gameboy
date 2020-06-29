// use core::emulator::Emulator;
use core::cpu::Cpu;
use core::ppu::Ppu;
use core::memory::Memory;
use core::bootrom::BOOTROM;
use core::utils::read_bit;

use std::fs::File;
use std::io::Read;

struct TestEmulator {
    cpu: Cpu,
    ppu: Ppu,
    memory: Memory,
    previous_serial_transfer_start: bool,
}

impl TestEmulator {
    fn new(rom: Vec<u8>) -> Self {
        let mut memory = Memory::new(rom, None);
        memory.load_bootrom(&BOOTROM);

        Self {
            cpu: Cpu::new(),
            ppu: Ppu::new(),
            memory,
            previous_serial_transfer_start: false,
        }
    }

    fn clock(&mut self) {
        self.previous_serial_transfer_start = self.serial_transfer_start();
        self.cpu.step(&mut self.memory);
        self.ppu.clock(&mut self.memory);
        self.memory.sound_controller.clock();
    }

    fn serial_transfer_start(&self) -> bool {
        // read_bit(self.memory.cpu_read(0xFF02), 7)
        self.memory.cpu_read(0xFF02) == 0x81
    }

    fn read_serial(&self) -> Option<char> {
        if self.previous_serial_transfer_start && self.memory.serial_transfer_write {
            Some(self.memory.cpu_read(0xFF01) as char)
        } else {
            None
        }
    }
}

fn get_rom(rom_path: &str) -> Vec<u8> {
    let mut f = File::open(&rom_path).expect("Could not open rom file");
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).expect("Could not read rom");

    buffer
}

fn without_trailing_whitespace(s: &String) -> String {
    let s_reverse = s.chars().rev();
    let without_whitespace = s_reverse.skip_while(|c| c.is_whitespace()).collect::<String>();
    without_whitespace.chars().rev().collect()
}

#[test]
fn it_runs_the_cpu() {
    let buffer = get_rom("tests/roms/cpu_instrs/06-ld r,r.gb");

    let mut emu = TestEmulator::new(buffer);

    let mut result = String::new();

    for _ in 0..6500000 {
        emu.clock();
        emu.read_serial().map(|c| result.push(c));
    }

    assert_eq!(
        "00666---lllddd   rrr,,,rrr\n\n\n\n\n\n\n\n\nPPPaaasssssseeeddd",
        without_trailing_whitespace(&result)
    );
}
