use std::fs::File;
use std::io::Read;

mod cpu;
mod ppu;
mod registers;
mod memory;
mod window;
use cpu::Cpu;
use window::Window;

fn main() -> std::io::Result<()> {
    let mut cpu = Cpu::new();

    let path = "bootrom.bin";
    let mut f = File::open(&path)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;
    cpu.load_bootrom(&buffer);

    let path = "cpu_instrs/individual/01-special.gb";
    let mut f = File::open(&path)?;
    let mut buffer = Vec::new();
    f.read_to_end(&mut buffer)?;
    cpu.load_rom(&buffer);

    // cpu.skip_bootrom();

    let mut window = Window::new(16 * 8, 24 * 8);

    while window.is_open() {
        for _ in 0..20_0 {
            cpu.step();
        }
        let memory = cpu.get_memory();
        let pixel_buffer = ppu::get_pixel_buffer(memory, 16 * 8, 24 * 8);
        window.update(&pixel_buffer);
        // std::thread::sleep(std::time::Duration::from_millis(1));
    }

    Ok(())
}
