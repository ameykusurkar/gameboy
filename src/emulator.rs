use crate::cpu::Cpu;
use crate::ppu::Ppu;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex::HL;

use crate::ppu::{LCD_WIDTH, LCD_HEIGHT, MAP_WIDTH, MAP_HEIGHT};

pub const DEBUG: bool = false;
const NORMAL_SPEED: bool = false;

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
            0x1 => pge::Pixel::rgb(0x80, 0x80, 0x80),
            0x2 => pge::Pixel::rgb(0x40, 0x40, 0x40),
            0x3 => pge::Pixel::rgb(0x00, 0x00, 0x00),
            _ => panic!("Unknown pixel value {:02x}", pixel),
        }
    }

    fn draw_maps(&mut self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) {
        let width = MAP_WIDTH;
        let height = MAP_HEIGHT;
        let pixel_buffer = Ppu::get_background_map(self.cpu.get_memory());

        let mut background_map_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            background_map_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        let pixel_buffer = Ppu::get_window_map(self.cpu.get_memory());
        let mut window_map_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            window_map_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(x, y, &background_map_sprite, scale);
        pge.draw_sprite(x + width as i32 + 10, y, &window_map_sprite, scale);
    }

    fn draw_tileset(&mut self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) {
        let (width, height) = (16 * 8, 24 * 8);
        let pixel_buffer = Ppu::get_tilset(self.cpu.get_memory());

        let mut tileset_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            tileset_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(x, y, &tileset_sprite, scale);
    }

    fn draw_cpu_state(&mut self, pge: &mut pge::PGE, x: i32, y: i32) {
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
        pge.draw_string(x, y + 20 * cy, &format!("IF: {:08b}", self.cpu.memory.cpu_read(0xFF0F)), &pge::WHITE, scale);

        let instructions = self.cpu.disassemble(self.cpu.pc, self.cpu.pc + 10);
        let start_y = y + 22 * cy;
        for (i, (addr, repr)) in instructions.iter().enumerate() {
            let formatted = format!("{:#04x}: {}", addr, repr);
            let color = if *addr == self.cpu.pc { &pge::WHITE } else { &pge::DARK_GREY };
            pge.draw_string(x, y + start_y + (i as i32) * cy, &formatted, color, scale);
        }
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
            std::thread::sleep(std::time::Duration::from_millis(16));
        }

        if DEBUG {
            println!("CYCLE: {}", now.elapsed().as_nanos());
        }

        let now = std::time::Instant::now();

        pge.clear(&pge::BLACK);

        let screen_scale = 3;

        self.draw_tileset(pge, 0, 0, 2);
        self.draw_maps(pge, 0, (LCD_HEIGHT * screen_scale) as i32, 1);
        self.draw_cpu_state(pge, (LCD_WIDTH * (screen_scale + 1)) as i32, 0);

        if DEBUG {
            println!("DRAW: {}", now.elapsed().as_nanos());
        }

        true
    }
}
