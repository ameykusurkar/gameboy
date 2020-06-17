use std::fs::File;
use std::io::Read;
use std::path::Path;

use clap::{App, Arg};

mod frontend_sdl;
mod frontend_pge;

use core::emulator::Emulator;

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
        .arg(Arg::with_name("save_file")
             .short("s")
             .long("save_file")
             .value_name("PATH")
             .help("Specify path to save file")
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

    let mut f = File::open(&rom_path)?;
    let mut rom_buffer = Vec::new();
    f.read_to_end(&mut rom_buffer)?;

    let save_path = match matches.value_of("save_file") {
        Some(path) => Path::new(&path).to_owned(),
        None       => Path::new(&rom_path).with_extension("sav"),
    };

    let save_buffer = if save_path.exists() {
        let mut f = File::open(&save_path)?;
        let mut buffer = Vec::new();
        f.read_to_end(&mut buffer)?;
        Some(buffer)
    } else {
        None
    };

    let emulator = Emulator::new(rom_buffer, save_buffer);

    match frontend_arg {
        "frontend-sdl" => {
            FrontendSdl::start(emulator, save_path)
                .expect("Error while running gameboy");
        },
        "frontend-pge" => {
            let mut frontend = FrontendPge::new(emulator, save_path);
            frontend.start().expect("Error while running gameboy");
        },
        _ => panic!("Invalid frontend type: {}", frontend_arg),
    };

    Ok(())
}
