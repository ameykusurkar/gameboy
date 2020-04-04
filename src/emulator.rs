use crate::cpu::Cpu;
use crate::ppu::Ppu;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex::HL;

use crate::ppu::{LCD_WIDTH, LCD_HEIGHT, MAP_WIDTH, MAP_HEIGHT};

pub const DEBUG: bool = false;
const NORMAL_SPEED: bool = true;

pub struct Emulator {
    cpu: Cpu,
    ppu: Ppu,
    cycles: u32,
    single_step_mode: bool,
}

impl Emulator {
    pub fn new(bootrom: &[u8], rom: &[u8]) -> Emulator {
        let mut cpu = Cpu::new();
        cpu.load_bootrom(bootrom);
        cpu.load_rom(rom);

        Emulator {
            cpu,
            ppu: Ppu::new(),
            cycles: 0,
            single_step_mode: false,
        }
    }

    fn clock(&mut self) {
        self.cpu.step();
        self.ppu.clock(self.cpu.get_memory_mut());
        self.cycles += 1;
    }

    fn color(pixel: u8) -> pge::Pixel {
        match pixel {
            0x0 => pge::Pixel::rgb(0xFF, 0xFF, 0xFF),
            0x1 => pge::Pixel::rgb(0xC0, 0xC0, 0xC0),
            0x2 => pge::Pixel::rgb(0x60, 0x60, 0x60),
            0x3 => pge::Pixel::rgb(0x00, 0x00, 0x00),
            _ => panic!("Unknown pixel value {:02x}", pixel),
        }
    }

    fn draw_maps(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let width = MAP_WIDTH;
        let height = MAP_HEIGHT;
        let pixel_buffer = Ppu::get_background_map(&self.cpu.memory);

        let mut background_map_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            background_map_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        let pixel_buffer = Ppu::get_window_map(&self.cpu.memory);
        let mut window_map_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            window_map_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        let gap_x = 100;
        pge.draw_sprite(x, y, &background_map_sprite, scale);
        pge.draw_sprite(x + width as i32 + gap_x, y, &window_map_sprite, scale);

        (x + (scale * 256 * 2) as i32 + gap_x, y + (scale * 256) as i32)
    }

    fn draw_tileset(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let (width, height) = (16 * 8, 24 * 8);
        let pixel_buffer = Ppu::get_tileset(&self.cpu.memory);

        let mut tileset_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            tileset_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(x, y, &tileset_sprite, scale);
        (x + (width * scale) as i32, y + (height * scale) as i32)
    }

    fn draw_cpu_state(&self, pge: &mut pge::PGE, x: i32, y: i32) {
        let scale = 2;
        let (cx, cy) = (scale * 8 + 2, scale * 8 + 2);
        pge.draw_string(x, y, "REGISTERS", &pge::WHITE, scale);

        pge.draw_string(x, y + cy, &format!("A: {:02X}", self.cpu.regs[A]), &pge::WHITE, scale);
        pge.draw_string(x + 6 * cx, y + cy, &format!("(HL): {:02X}", self.cpu.memory.cpu_read(self.cpu.regs.read(HL))), &pge::WHITE, scale);

        pge.draw_string(x, y + 2 * cy, &format!("B: {:02X}", self.cpu.regs[B]), &pge::WHITE, scale);
        pge.draw_string(x + 6 * cx, y + 2 * cy, &format!("C:    {:02X}", self.cpu.regs[C]), &pge::WHITE, scale);

        pge.draw_string(x, y + 3 * cy, &format!("D: {:02X}", self.cpu.regs[D]), &pge::WHITE, scale);
        pge.draw_string(x + 6 * cx, y + 3 * cy, &format!("E:    {:02X}", self.cpu.regs[E]), &pge::WHITE, scale);

        pge.draw_string(x, y + 4 * cy, &format!("H: {:02X}", self.cpu.regs[H]), &pge::WHITE, scale);
        pge.draw_string(x + 6 * cx, y + 4 * cy, &format!("L:    {:02X}", self.cpu.regs[L]), &pge::WHITE, scale);

        pge.draw_string(x, y + 6 * cy, "FLAGS", &pge::WHITE, scale);

        let flags = self.cpu.regs.read_flags();
        let z_color = if flags.zero { &pge::WHITE } else { &pge::DARK_GREY };
        let n_color = if flags.subtract { &pge::WHITE } else { &pge::DARK_GREY };
        let h_color = if flags.half_carry { &pge::WHITE } else { &pge::DARK_GREY };
        let c_color = if flags.carry { &pge::WHITE } else { &pge::DARK_GREY };
        pge.draw_string(x, y + 7 * cy, "Z", z_color, scale);
        pge.draw_string(x + 2 * cx, y + 7 * cy, "N", n_color, scale);
        pge.draw_string(x + 4 * cx, y + 7 * cy, "H", h_color, scale);
        pge.draw_string(x + 6 * cx, y + 7 * cy, "C", c_color, scale);

        pge.draw_string(x, y + 9 * cy, &format!("PC: {:04X}", self.cpu.pc), &pge::WHITE, scale);
        pge.draw_string(x, y + 10 * cy, &format!("SP: {:04X}", self.cpu.sp), &pge::WHITE, scale);

        pge.draw_string(x, y + 12 * cy, &format!("CYCLES: {}", self.cycles), &pge::WHITE, scale);

        let mode = if self.single_step_mode { "STEP" } else { "NORMAL" };
        pge.draw_string(x, y + 14 * cy, &format!("MODE: {}", mode), &pge::WHITE, scale);

        pge.draw_string(x, y + 16 * cy, &format!("TIMA: {}", self.cpu.memory.cpu_read(0xFF05)), &pge::WHITE, scale);

        pge.draw_string(x, y + 18 * cy, &format!("IE: {:08b}", self.cpu.memory.cpu_read(0xFFFF)), &pge::WHITE, scale);
        pge.draw_string(x, y + 19 * cy, &format!("IF: {:08b}", self.cpu.memory.cpu_read(0xFF0F)), &pge::WHITE, scale);

        pge.draw_string(x, y + 21 * cy, &format!("LCDC: {:08b}", self.cpu.memory.cpu_read(0xFF40)), &pge::WHITE, scale);
        pge.draw_string(x, y + 22 * cy, &format!("STAT: {:08b}", self.cpu.memory.cpu_read(0xFF41)), &pge::WHITE, scale);
        pge.draw_string(x, y + 23 * cy, &format!("LY: {}", self.cpu.memory.cpu_read(0xFF44)), &pge::WHITE, scale);

        let start_y = y + 25 * cy;
        let instructions = self.cpu.disassemble(self.cpu.pc, self.cpu.pc + 10);
        for (i, (addr, repr)) in instructions.iter().enumerate() {
            let formatted = format!("{:#04x}: {}", addr, repr);
            let color = if *addr == self.cpu.pc { &pge::WHITE } else { &pge::DARK_GREY };
            pge.draw_string(x, y + start_y + (i as i32) * cy, &formatted, color, scale);
        }
    }

    fn draw_screen(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let (width, height) = (LCD_WIDTH as usize, LCD_HEIGHT as usize);

        if self.cpu.memory.lcd_enabled() {
            let mut screen_sprite = pge::Sprite::new(width, height);
            for (i, pixel) in self.ppu.screen.iter().enumerate() {
                let (x, y) = (i % width, i / width);
                screen_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
            }
            pge.draw_sprite(x, y, &screen_sprite, scale);
        } else {
            pge.fill_rect(x, y, (width * scale) as i32, (height * scale) as i32, &pge::WHITE);
        }

        (x + (width * scale) as i32, y + (height * scale) as i32)
    }
}

impl pge::State for Emulator {
    fn on_user_create(&mut self) -> bool {
        true
    }

    fn on_user_update(&mut self, pge: &mut pge::PGE, _elapsed_time: f32) -> bool {
        let now = std::time::Instant::now();

        let step_pressed = pge.get_key(minifb::Key::S).pressed;
        if step_pressed {
            self.single_step_mode = true;
        }

        if pge.get_key(minifb::Key::N).pressed {
            self.single_step_mode = false;
        }

        let num_steps = if self.single_step_mode { 1 } else { 17_556 };

        if !self.single_step_mode || step_pressed {
            for _ in 0..num_steps {
                self.clock();
            }
        }

        if !self.single_step_mode && NORMAL_SPEED {
            // TODO: Make this accurate based on elapsed time
            std::thread::sleep(std::time::Duration::from_millis(9));
        }

        if DEBUG {
            println!("CYCLE: {}", now.elapsed().as_nanos());
        }

        let now = std::time::Instant::now();

        pge.clear(&pge::BLACK);

        let screen_scale = 3;

        let (screen_end_x, screen_end_y) = self.draw_screen(pge, 10, 10, screen_scale);
        let (maps_end_x, _) = self.draw_maps(pge, 10, screen_end_y + 150, 1);
        self.draw_cpu_state(pge, screen_end_x + 10, 10);
        self.draw_tileset(pge, maps_end_x + 100, screen_end_y + 150, 1);

        if DEBUG {
            println!("DRAW: {}", now.elapsed().as_nanos());
        }

        true
    }
}
