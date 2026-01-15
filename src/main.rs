use std::fs::File;
use std::io::Read;
use std::path::Path;

use clap::{Parser, ValueEnum};

mod frontend_pge;
mod frontend_sdl;

use core::emulator::Emulator;

use frontend_pge::FrontendPge;
use frontend_sdl::FrontendSdl;

#[derive(Debug, Clone, ValueEnum)]
enum Frontend {
    #[value(name = "sdl")]
    Sdl,
    #[value(name = "pge")]
    Pge,
}

#[derive(Parser)]
#[command(name = "Gameboy")]
struct Args {
    /// Selects the frontend for the emulator
    #[arg(short, long, value_enum, default_value_t = Frontend::Sdl)]
    frontend: Frontend,

    /// Specify path to save file
    #[arg(short, long)]
    save_file: Option<String>,

    /// The game to run
    rom: String,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let rom_path = &args.rom;

    let mut f = File::open(&rom_path)?;
    let mut rom_buffer = Vec::new();
    f.read_to_end(&mut rom_buffer)?;

    let save_path = match &args.save_file {
        Some(path) => Path::new(path).to_owned(),
        None => Path::new(rom_path).with_extension("sav"),
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

    match args.frontend {
        Frontend::Sdl => {
            FrontendSdl::start(emulator, save_path).expect("Error while running gameboy");
        }
        Frontend::Pge => {
            let mut frontend = FrontendPge::new(emulator, save_path);
            frontend.start().expect("Error while running gameboy");
        }
    };

    Ok(())
}
