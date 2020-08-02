use core::cpu::Cpu;
use core::memory::Memory;
use core::bootrom::BOOTROM;

pub struct TestEmulator {
    cpu: Cpu,
    memory: Memory,
}

impl TestEmulator {
    pub fn new(rom: Vec<u8>) -> Self {
        let mut memory = Memory::new(rom, None);
        memory.load_bootrom(&BOOTROM);

        Self {
            cpu: Cpu::new(),
            memory,
        }
    }

    pub fn clock(&mut self) {
        self.cpu.step(&mut self.memory);
        self.memory.clock();
    }

    fn raw_serial_buffer(&self) -> &[u8] {
        self.memory.serial_transfer.buffer()
    }

    pub fn serial_buffer(&self) -> &str {
        std::str::from_utf8(self.raw_serial_buffer()).unwrap()
    }
}

