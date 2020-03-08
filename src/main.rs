use std::fs::File;
use std::io::Read;

mod cpu;
use cpu::Cpu;

fn main() -> std::io::Result<()> {
    let path = "bootrom.bin";
    let mut f = File::open(&path)?;

    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;

    // for byte in &buffer {
    //     println!("{:0x}", byte);
    // }

    let mut cpu = Cpu::new();
    cpu.load_bootrom(&buffer);

    loop {
        cpu.step();
    }

    // Ok(())
}
