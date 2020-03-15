use std::fs::File;
use std::io::Read;

mod cpu;
mod registers;
use cpu::Cpu;

fn main() -> std::io::Result<()> {
    let path = "cpu_instrs/individual/01-special.gb";
    let mut f = File::open(&path)?;

    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;

    let mut cpu = Cpu::new();

    // TODO: Rename this method
    cpu.load_bootrom(&buffer);

    // TODO: Implement proper bootrom loading
    cpu.test_setup();

    loop {
        cpu.step();
    }

    // Ok(())
}
