use std::fs::File;
use std::io::Read;

use test_emulator::TestEmulator;

mod test_emulator;

fn read_rom(rom_path: &str) -> Vec<u8> {
    let mut f = File::open(&rom_path).expect("Could not open rom file");
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer).expect("Could not read rom");

    buffer
}

#[test]
fn it_runs_the_cpu() {
    let buffer = read_rom("tests/roms/cpu_instrs/06-ld r,r.gb");
    let mut emu = TestEmulator::new(buffer);

    for _ in 0..6500000 {
        emu.clock();
    }

    assert_eq!("06-ld r,r\n\n\nPassed\n", emu.serial_buffer());
}
