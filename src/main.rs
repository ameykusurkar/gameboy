use std::fs::File;
use std::io::Read;

mod cpu;
mod registers;
mod memory;
use cpu::Cpu;

fn main() -> std::io::Result<()> {
    let mut cpu = Cpu::new();

    let path = "bootrom.bin";
    let mut f = File::open(&path)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;
    cpu.load_bootrom(&buffer);

    let path = "cpu_instrs/individual/01-special.gb";
    let mut f = File::open(&path)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;
    cpu.load_rom(&buffer);

    // TODO: Implement proper bootrom loading
    cpu.test_setup();

    loop {
        cpu.step();
    }

    // Ok(())
}
