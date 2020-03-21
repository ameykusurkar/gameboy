use std::fs::File;
use std::io::Read;

mod cpu;
mod ppu;
mod registers;
mod memory;
mod window;

use cpu::Cpu;
use window::Window;
use ppu::NUM_PIXELS_IN_LINE;

fn main() -> std::io::Result<()> {
    if std::env::args().len() < 2 {
        println!("Please provide a rom file");
        return Ok(())
    }

    let mut cpu = Cpu::new();

    let path = "bootrom.bin";
    let mut f = File::open(&path)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;
    cpu.load_bootrom(&buffer);

    let path = std::env::args().nth(1).unwrap();
    let mut f = File::open(&path)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;
    cpu.load_rom(&buffer);

    // cpu.skip_bootrom();

    let width = 32 * NUM_PIXELS_IN_LINE;
    let height = 32 * NUM_PIXELS_IN_LINE;

    let mut window = Window::new(width, height);
    window.set_title("Gameboy");

    while window.is_open() {
        for _ in 0..20000 {
            cpu.step();
        }
        let memory = cpu.get_memory();
        let pixel_buffer = ppu::get_pixel_buffer(memory, width, height);
        window.update(&pixel_buffer);
        // std::thread::sleep(std::time::Duration::from_millis(1));
    }

    Ok(())
}
