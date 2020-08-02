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

fn run_test_rom(rom: Vec<u8>, num_cycles: u32) -> String {
    let mut emu = TestEmulator::new(rom);

    for _ in 0..num_cycles {
        emu.clock();
    }

    emu.serial_buffer().to_string()
}

///// Blarg's Tests /////

#[test]
fn it_passes_cpu_test_01() {
    let rom = read_rom("tests/roms/cpu_instrs/01-special.gb");
    let output = run_test_rom(rom, 8_400_000);

    assert_eq!("01-special\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_02() {
    let rom = read_rom("tests/roms/cpu_instrs/02-interrupts.gb");
    let output = run_test_rom(rom, 6_400_000);

    assert_eq!("02-interrupts\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_03() {
    let rom = read_rom("tests/roms/cpu_instrs/03-op sp,hl.gb");
    let output = run_test_rom(rom, 8_400_000);

    assert_eq!("03-op sp,hl\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_04() {
    let rom = read_rom("tests/roms/cpu_instrs/04-op r,imm.gb");
    let output = run_test_rom(rom, 9_000_000);

    assert_eq!("04-op r,imm\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_05() {
    let rom = read_rom("tests/roms/cpu_instrs/05-op rp.gb");
    let output = run_test_rom(rom, 10_000_000);

    assert_eq!("05-op rp\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_06() {
    let rom = read_rom("tests/roms/cpu_instrs/06-ld r,r.gb");
    let output = run_test_rom(rom, 6_500_000);

    assert_eq!("06-ld r,r\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_07() {
    let rom = read_rom("tests/roms/cpu_instrs/07-jr,jp,call,ret,rst.gb");
    let output = run_test_rom(rom, 6_700_000);

    assert_eq!("07-jr,jp,call,ret,rst\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_08() {
    let rom = read_rom("tests/roms/cpu_instrs/08-misc instrs.gb");
    let output = run_test_rom(rom, 6_500_000);

    assert_eq!("08-misc instrs\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_09() {
    let rom = read_rom("tests/roms/cpu_instrs/09-op r,r.gb");
    let output = run_test_rom(rom, 16_000_000);

    assert_eq!("09-op r,r\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_10() {
    let rom = read_rom("tests/roms/cpu_instrs/10-bit ops.gb");
    let output = run_test_rom(rom, 20_500_000);

    assert_eq!("10-bit ops\n\n\nPassed\n", output);
}

#[test]
fn it_passes_cpu_test_11() {
    let rom = read_rom("tests/roms/cpu_instrs/11-op a,(hl).gb");
    let output = run_test_rom(rom, 25_000_000);

    assert_eq!("11-op a,(hl)\n\n\nPassed\n", output);
}

#[test]
fn it_passes_instr_timing_test() {
    let rom = read_rom("tests/roms/instr_timing.gb");
    let output = run_test_rom(rom, 7_000_000);

    assert_eq!("instr_timing\n\n\nPassed\n", output);
}
