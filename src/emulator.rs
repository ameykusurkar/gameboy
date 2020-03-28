use crate::cpu::Cpu;
use crate::ppu;

use crate::ppu::NUM_PIXELS_IN_LINE;

pub struct Emulator {
    cpu: Cpu,
    cycles: u32,
}

impl Emulator {
    pub fn new() -> Emulator {
        Emulator {
            cpu: Cpu::new(),
            cycles: 0,
        }
    }

    pub fn load_setup(&mut self, bootrom: &[u8], rom: &[u8]) {
        self.cpu.load_bootrom(bootrom);
        self.cpu.load_rom(rom);
    }

    fn clock(&mut self) {
        self.cpu.step();
        self.cycles += 1;
    }
    
    fn color(pixel: u8) -> pge::Pixel {
        match pixel {
            0x0 => pge::Pixel::rgb(0xFF, 0xFF, 0xFF),
            0x1 => pge::Pixel::rgb(0x80, 0x80, 0x80),
            0x2 => pge::Pixel::rgb(0x40, 0x40, 0x40),
            0x3 => pge::Pixel::rgb(0x00, 0x00, 0x00),
            _ => panic!("Unknown pixel value {:02x}", pixel),
        }
    }
}

impl pge::State for Emulator {
    fn on_user_create(&mut self) -> bool {
        true
    }

    fn on_user_update(&mut self, pge: &mut pge::PGE, _elapsed_time: f32) -> bool {
        let now = std::time::Instant::now();

        for _ in 0..17_556 {
            self.clock();
        }

        println!("CYCLE: {}", now.elapsed().as_nanos());

        let now = std::time::Instant::now();

        pge.clear(&pge::BLACK);

        let width = 32 * NUM_PIXELS_IN_LINE;
        let height = 32 * NUM_PIXELS_IN_LINE;
        let pixel_buffer = ppu::get_pixel_buffer(self.cpu.get_memory(), width, height);

        let mut sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(0, 0, &sprite, 1);

        println!("DRAW: {}", now.elapsed().as_nanos());

        true
    }
}
