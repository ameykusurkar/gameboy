use std::fs::File;
use std::io::Read;

mod cpu;
mod ppu;
mod registers;
mod memory;
mod instruction;
mod emulator;

use emulator::Emulator;
use crate::ppu::NUM_PIXELS_IN_LINE;

use pge;

fn main() -> std::io::Result<()> {
    if std::env::args().len() < 2 {
        println!("Please provide a rom file");
        return Ok(())
    }

    let mut emulator = Emulator::new();

    let path = "bootrom.bin";
    let mut f = File::open(&path)?;
    let mut bootrom_buffer = Vec::new();
    f.read_to_end(&mut bootrom_buffer)?;

    let path = std::env::args().nth(1).unwrap();
    let mut f = File::open(&path)?;
    let mut rom_buffer = Vec::new();
    f.read_to_end(&mut rom_buffer)?;

    emulator.load_setup(&bootrom_buffer, &rom_buffer);

    let width = 32 * NUM_PIXELS_IN_LINE;
    let height = 32 * NUM_PIXELS_IN_LINE;
    let scale = 2;

    let mut pge = pge::PGE::construct("Gameboy", width, height, scale, scale);
    pge.start(&mut emulator);

    Ok(())
}
