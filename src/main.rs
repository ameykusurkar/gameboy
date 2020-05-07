use std::fs::File;
use std::io::Read;
use std::path::Path;

mod cpu;
mod ppu;
mod registers;
mod memory;
mod instruction;
mod joypad;
mod utils;
mod cartridge;
mod emulator;
mod frontend_sdl;
mod frontend_pge;

use emulator::Emulator;
use frontend_pge::FrontendPge;
use frontend_sdl::FrontendSdl;

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

    let save_path = Path::new(&path).with_extension("sav");

    let save_buffer = if save_path.exists() {
        let mut f = File::open(&save_path)?;
        let mut buffer = Vec::new();
        f.read_to_end(&mut buffer)?;
        Some(buffer)
    } else {
        None
    };

    let emulator = Emulator::new(&bootrom_buffer, rom_buffer, save_buffer, save_path);

    // let mut frontend = FrontendSdl::new(emulator);
    let mut frontend = FrontendPge::new(emulator);

    frontend.start().expect("Error while running gameboy");

    Ok(())
}
