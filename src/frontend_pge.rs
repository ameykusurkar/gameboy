use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use core::registers::RegisterIndex::*;
use core::registers::TwoRegisterIndex::{HL, PC, SP};

use core::disassembly;
use core::emulator::{DEBUG, Emulator};

use core::ppu::{FRAME_CYCLES, LCD_HEIGHT, LCD_WIDTH, MAP_HEIGHT, MAP_WIDTH};

const NORMAL_SPEED: bool = true;

pub const MACHINE_CYCLES_PER_SECOND: u32 = 1_048_576;
const FRAME_INTERVAL: std::time::Duration = std::time::Duration::from_nanos(
    (FRAME_CYCLES as f32 / MACHINE_CYCLES_PER_SECOND as f32 * 1e9) as u64,
);

pub struct FrontendPge {
    pge_state: PgeState,
}

impl FrontendPge {
    pub fn new(emulator: Emulator, save_path: PathBuf) -> Self {
        FrontendPge {
            pge_state: PgeState::new(emulator, save_path),
        }
    }

    pub fn start(&mut self) -> Result<(), String> {
        let width = LCD_WIDTH * 6;
        let height = LCD_HEIGHT * 6;
        let scale = 1;

        let mut pge = pge::PGE::construct("Gameboy", width as usize, height as usize, scale, scale);
        pge.start(&mut self.pge_state);

        Ok(())
    }
}

pub struct PgeState {
    emulator: Emulator,
    cycles: u32,
    single_step_mode: bool,
    last_render_at: std::time::Instant,
    save_path: PathBuf,
}

impl PgeState {
    pub fn new(emulator: Emulator, save_path: PathBuf) -> Self {
        PgeState {
            emulator,
            cycles: 0,
            single_step_mode: false,
            last_render_at: std::time::Instant::now(),
            save_path,
        }
    }

    fn clock(&mut self) {
        self.emulator.clock();
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

    fn save_external_ram(&mut self) {
        self.emulator
            .memory
            .get_external_ram()
            .and_then(|ram| {
                File::create(&self.save_path)
                    .and_then(|mut f| f.write_all(ram))
                    .ok()
            })
            .map(|_| self.emulator.memory.mark_external_ram_as_saved());
    }

    fn draw_maps(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let width = MAP_WIDTH;
        let height = MAP_HEIGHT;
        let pixel_buffer = self.emulator.memory.ppu.get_background_map();

        let mut background_map_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            background_map_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        let pixel_buffer = self.emulator.memory.ppu.get_window_map();
        let mut window_map_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            window_map_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        let gap_x = 100;
        pge.draw_sprite(x, y, &background_map_sprite, scale);
        pge.draw_sprite(x + width as i32 + gap_x, y, &window_map_sprite, scale);

        (
            x + (scale * 256 * 2) as i32 + gap_x,
            y + (scale * 256) as i32,
        )
    }

    fn draw_tileset(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let (width, height) = (16 * 8, 24 * 8);
        let pixel_buffer = self.emulator.memory.ppu.get_tileset();

        let mut tileset_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            tileset_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(x, y, &tileset_sprite, scale);
        (x + (width * scale) as i32, y + (height * scale) as i32)
    }

    fn draw_sprites_map(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let (width, height) = (10 * 8, 4 * 8);
        let pixel_buffer = self.emulator.memory.ppu.get_sprites();

        let mut sprites_sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            sprites_sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(x, y, &sprites_sprite, scale);
        (x + (width * scale) as i32, y + (height * scale) as i32)
    }

    fn draw_cpu_state(&self, pge: &mut pge::PGE, x: i32, y: i32) {
        let scale = 2;
        let (cx, cy) = (scale * 8 + 2, scale * 8 + 2);
        pge.draw_string(x, y, "REGISTERS", &pge::WHITE, scale);

        pge.draw_string(
            x,
            y + cy,
            &format!("A: {:02X}", self.emulator.cpu.regs.read(A)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x + 6 * cx,
            y + cy,
            &format!(
                "(HL): {:02X}",
                self.emulator
                    .memory
                    .cpu_read(self.emulator.cpu.regs.read16(HL))
            ),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 2 * cy,
            &format!("B: {:02X}", self.emulator.cpu.regs.read(B)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x + 6 * cx,
            y + 2 * cy,
            &format!("C:    {:02X}", self.emulator.cpu.regs.read(C)),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 3 * cy,
            &format!("D: {:02X}", self.emulator.cpu.regs.read(D)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x + 6 * cx,
            y + 3 * cy,
            &format!("E:    {:02X}", self.emulator.cpu.regs.read(E)),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 4 * cy,
            &format!("H: {:02X}", self.emulator.cpu.regs.read(H)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x + 6 * cx,
            y + 4 * cy,
            &format!("L:    {:02X}", self.emulator.cpu.regs.read(L)),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(x, y + 6 * cy, "FLAGS", &pge::WHITE, scale);

        let flags = self.emulator.cpu.regs.read_flags();
        let z_color = if flags.zero {
            &pge::WHITE
        } else {
            &pge::DARK_GREY
        };
        let n_color = if flags.subtract {
            &pge::WHITE
        } else {
            &pge::DARK_GREY
        };
        let h_color = if flags.half_carry {
            &pge::WHITE
        } else {
            &pge::DARK_GREY
        };
        let c_color = if flags.carry {
            &pge::WHITE
        } else {
            &pge::DARK_GREY
        };
        pge.draw_string(x, y + 7 * cy, "Z", z_color, scale);
        pge.draw_string(x + 2 * cx, y + 7 * cy, "N", n_color, scale);
        pge.draw_string(x + 4 * cx, y + 7 * cy, "H", h_color, scale);
        pge.draw_string(x + 6 * cx, y + 7 * cy, "C", c_color, scale);

        pge.draw_string(
            x,
            y + 9 * cy,
            &format!("PC: {:04X}", self.emulator.cpu.regs.read16(PC)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x,
            y + 10 * cy,
            &format!("SP: {:04X}", self.emulator.cpu.regs.read16(SP)),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 12 * cy,
            &format!("CYCLES: {}", self.cycles),
            &pge::WHITE,
            scale,
        );

        let mode = if self.single_step_mode {
            "STEP"
        } else {
            "NORMAL"
        };
        pge.draw_string(
            x,
            y + 14 * cy,
            &format!("MODE: {}", mode),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 16 * cy,
            &format!("TIMA: {}", self.emulator.memory.cpu_read(0xFF05)),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 18 * cy,
            &format!("IE: {:08b}", self.emulator.memory.cpu_read(0xFFFF)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x,
            y + 19 * cy,
            &format!("IF: {:08b}", self.emulator.memory.cpu_read(0xFF0F)),
            &pge::WHITE,
            scale,
        );

        pge.draw_string(
            x,
            y + 21 * cy,
            &format!("LCDC: {:08b}", self.emulator.memory.cpu_read(0xFF40)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x,
            y + 22 * cy,
            &format!("STAT: {:08b}", self.emulator.memory.cpu_read(0xFF41)),
            &pge::WHITE,
            scale,
        );
        pge.draw_string(
            x,
            y + 23 * cy,
            &format!("LY: {}", self.emulator.memory.cpu_read(0xFF44)),
            &pge::WHITE,
            scale,
        );

        let start_y = y + 25 * cy;
        let mut instructions = disassembly::disassemble(
            &self.emulator.memory,
            self.emulator.cpu.regs.read16(PC),
            self.emulator.cpu.regs.read16(PC) + 10,
        );
        let num_instructions = std::cmp::min(instructions.len(), 5);
        let instructions = instructions.drain(..num_instructions);
        for (i, (addr, repr)) in instructions.enumerate() {
            let formatted = format!("{:#04x}: {}", addr, repr);
            let color = if addr == self.emulator.cpu.regs.read16(PC) {
                &pge::WHITE
            } else {
                &pge::DARK_GREY
            };
            pge.draw_string(x, y + start_y + (i as i32) * cy, &formatted, color, scale);
        }
    }

    fn draw_screen(&self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) -> (i32, i32) {
        let (width, height) = (LCD_WIDTH as usize, LCD_HEIGHT as usize);

        if let Some(screen_buffer) = self.emulator.get_screen_buffer() {
            let mut screen_sprite = pge::Sprite::new(width, height);
            for (i, pixel) in screen_buffer.iter().enumerate() {
                let (x, y) = (i % width, i / width);
                screen_sprite.set_pixel(x as i32, y as i32, &Self::color(pixel.raw()));
            }
            pge.draw_sprite(x, y, &screen_sprite, scale);
        } else {
            pge.fill_rect(
                x,
                y,
                (width * scale) as i32,
                (height * scale) as i32,
                &pge::WHITE,
            );
        }

        (x + (width * scale) as i32, y + (height * scale) as i32)
    }
}

impl pge::State for PgeState {
    fn on_user_create(&mut self) -> bool {
        true
    }

    fn on_user_update(&mut self, pge: &mut pge::PGE, _elapsed_time: f32) -> bool {
        let now = std::time::Instant::now();

        let step_pressed = pge.get_key(minifb::Key::T).pressed;
        if step_pressed {
            self.single_step_mode = true;
        }

        if pge.get_key(minifb::Key::Y).pressed {
            self.single_step_mode = false;
        }

        // TODO: Trigger joypad interrupt
        self.emulator.memory.joypad.down = pge.get_key(minifb::Key::J).held;
        self.emulator.memory.joypad.up = pge.get_key(minifb::Key::K).held;
        self.emulator.memory.joypad.left = pge.get_key(minifb::Key::H).held;
        self.emulator.memory.joypad.right = pge.get_key(minifb::Key::L).held;
        self.emulator.memory.joypad.select = pge.get_key(minifb::Key::V).held;
        self.emulator.memory.joypad.start = pge.get_key(minifb::Key::N).held;
        self.emulator.memory.joypad.b = pge.get_key(minifb::Key::D).held;
        self.emulator.memory.joypad.a = pge.get_key(minifb::Key::F).held;

        if !self.single_step_mode && NORMAL_SPEED {
            let now = std::time::Instant::now();
            let next_render_at = self.last_render_at + FRAME_INTERVAL;

            if now < next_render_at {
                std::thread::sleep(next_render_at - now);
            }

            self.last_render_at = next_render_at;
        }

        if !self.single_step_mode {
            loop {
                self.clock();
                if self.emulator.memory.ppu.frame_complete {
                    break;
                };
            }

            self.emulator.memory.ppu.frame_complete = false;
        } else if step_pressed {
            self.clock();
        }

        self.emulator.memory.joypad.clear();

        self.save_external_ram();

        if DEBUG {
            println!("CYCLE: {}", now.elapsed().as_nanos());
        }

        let now = std::time::Instant::now();

        pge.clear(&pge::BLACK);

        let screen_scale = 3;

        let (screen_end_x, screen_end_y) = self.draw_screen(pge, 10, 10, screen_scale);
        let (maps_end_x, _) = self.draw_maps(pge, 10, screen_end_y + 150, 1);
        self.draw_cpu_state(pge, screen_end_x + 10, 10);
        let (_, tiles_end_y) = self.draw_tileset(pge, maps_end_x + 100, screen_end_y + 150, 1);
        self.draw_sprites_map(pge, maps_end_x + 100, tiles_end_y + 10, 1);

        if DEBUG {
            println!("DRAW: {}", now.elapsed().as_nanos());
        }

        true
    }
}
