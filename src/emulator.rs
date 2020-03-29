use crate::cpu::Cpu;
use crate::ppu;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex::HL;

use crate::ppu::NUM_PIXELS_IN_LINE;

pub const DEBUG: bool = false;

pub struct Emulator {
    cpu: Cpu,
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
            cycles: 0,
            single_step_mode: false,
        }
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

    fn draw_background_map(&mut self, pge: &mut pge::PGE, x: i32, y: i32, scale: usize) {
        let width = 32 * NUM_PIXELS_IN_LINE;
        let height = 32 * NUM_PIXELS_IN_LINE;
        let pixel_buffer = ppu::get_pixel_buffer(self.cpu.get_memory(), width, height);

        let mut sprite = pge::Sprite::new(width, height);
        for (i, pixel) in pixel_buffer.iter().enumerate() {
            let x = i % width;
            let y = i / width;
            sprite.set_pixel(x as i32, y as i32, &Self::color(*pixel));
        }

        pge.draw_sprite(x, y, &sprite, scale);
    }

    fn draw_cpu_state(&mut self, pge: &mut pge::PGE, x: i32, y: i32) {
        let scale = 2;
        let (cx, cy) = (scale * 8 + 2, scale * 8 + 2);
        pge.draw_string(x, y, "REGISTERS", &pge::WHITE, scale);

        pge.draw_string(x, y + cy, &format!("A: {:02X}", self.cpu.regs[A]), &pge::WHITE, scale);
        pge.draw_string(x + 6 * cx, y + cy, &format!("(HL): {:02X}", self.cpu.memory[self.cpu.regs.read(HL)]), &pge::WHITE, scale);

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

        pge.draw_string(x, y + 16 * cy, &format!("TIMA: {}", self.cpu.memory[0xFF05]), &pge::WHITE, scale);

        let instructions = self.cpu.disassemble(self.cpu.pc, self.cpu.pc + 10);
        let start_y = y + 18 * cy;
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

        if DEBUG {
            println!("CYCLE: {}", now.elapsed().as_nanos());
        }

        let now = std::time::Instant::now();

        pge.clear(&pge::BLACK);

        self.draw_background_map(pge, 0, 0, 2);
        self.draw_cpu_state(pge, (32 * NUM_PIXELS_IN_LINE * 2) as i32, 0);

        if DEBUG {
            println!("DRAW: {}", now.elapsed().as_nanos());
        }

        true
    }
}
