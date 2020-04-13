use std::fs::File;
use std::io::Read;

mod cpu;
mod ppu;
mod registers;
mod memory;
mod instruction;
mod emulator;
mod joypad;
mod utils;
mod cartridge;

use emulator::Emulator;
use crate::ppu::{LCD_WIDTH, LCD_HEIGHT};

use pge;

fn main() -> std::io::Result<()> {
    if std::env::args().len() < 2 {
        println!("Please provide a rom file");
        return Ok(())
    }

    let path = "bootrom.bin";
    let mut f = File::open(&path)?;
    let mut bootrom_buffer = Vec::new();
    f.read_to_end(&mut bootrom_buffer)?;

    let path = std::env::args().nth(1).unwrap();
    let mut f = File::open(&path)?;
    let mut rom_buffer = Vec::new();
    f.read_to_end(&mut rom_buffer)?;

    let mut emulator = Emulator::new(&bootrom_buffer, rom_buffer);

    let width = LCD_WIDTH * 6;
    let height = LCD_HEIGHT * 6;
    let scale = 1;

    let mut pge = pge::PGE::construct("Gameboy", width as usize, height as usize, scale, scale);
    pge.start(&mut emulator);

    Ok(())
}
