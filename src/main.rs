use std::fs::File;
use std::io::Read;
use std::path::Path;

use clap::{App, Arg};

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
    let matches = App::new("Gameboy")
        .arg(Arg::with_name("frontend")
             .short("f")
             .long("frontend")
             .value_name("TYPE")
             .help("Selects the frontend for the emulator")
             .takes_value(true))
        .arg(Arg::with_name("ROM")
             .required(true)
             .help("The game to run")
             .index(1))
        .get_matches();

    let frontend_arg = matches.value_of("frontend").unwrap_or("frontend-sdl");
    let rom_path = matches.value_of("ROM").unwrap();

    if std::env::args().len() < 2 {
        println!("Please provide a rom file");
        return Ok(())
    }

    let path = "bootrom.bin";
    let mut f = File::open(&path)?;
    let mut bootrom_buffer = Vec::new();
    f.read_to_end(&mut bootrom_buffer)?;

    let mut f = File::open(&rom_path)?;
    let mut rom_buffer = Vec::new();
    f.read_to_end(&mut rom_buffer)?;

    let save_path = Path::new(&rom_path).with_extension("sav");

    let save_buffer = if save_path.exists() {
        let mut f = File::open(&save_path)?;
        let mut buffer = Vec::new();
        f.read_to_end(&mut buffer)?;
        Some(buffer)
    } else {
        None
    };

    let emulator = Emulator::new(&bootrom_buffer, rom_buffer, save_buffer, save_path);

    match frontend_arg {
        "frontend-sdl" => {
            let mut frontend = FrontendSdl::new(emulator);
            frontend.start().expect("Error while running gameboy");
        },
        "frontend-pge" => {
            let mut frontend = FrontendPge::new(emulator);
            frontend.start().expect("Error while running gameboy");
        },
        _ => panic!("Invalid frontend type: {}", frontend_arg),
    };

    Ok(())
}
