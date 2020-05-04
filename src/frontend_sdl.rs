use sdl2;

use sdl2::pixels::Color;
use sdl2::pixels::PixelFormatEnum;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::rect::Rect;

use crate::emulator::Emulator;
use crate::ppu::{LCD_HEIGHT, LCD_WIDTH};

const SCALE: u32 = 3;

pub struct FrontendSdl {
    emulator: Emulator,
}

impl FrontendSdl {
    pub fn new(emulator: Emulator) -> Self {
        FrontendSdl {
            emulator,
        }
    }

    pub fn start(&mut self) -> Result<(), String> {
        let sdl_context = sdl2::init()?;
        let video_subsystem = sdl_context.video()?;

        let window = video_subsystem.window("Gameboy", LCD_WIDTH * SCALE, LCD_HEIGHT * SCALE)
            .position_centered()
            .opengl()
            .build()
            .map_err(|e| e.to_string())?;

        let mut canvas = window.into_canvas().build().map_err(|e| e.to_string())?;
        let texture_creator = canvas.texture_creator();

        let mut texture = texture_creator
            .create_texture_streaming(PixelFormatEnum::RGB24, LCD_WIDTH, LCD_HEIGHT)
            .map_err(|e| e.to_string())?;

        canvas.set_draw_color(Color::BLACK);
        canvas.clear();
        canvas.present();

        let mut event_pump = sdl_context.event_pump()?;

        canvas.set_scale(SCALE as f32, SCALE as f32)?;

        'running: loop {
            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit {..} | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                        break 'running
                    },
                    Event::KeyDown {..} => {},
                    _ => {}
                }
            }

            self.clock_frame();

            if let Some(screen_buffer) = self.emulator.get_screen_buffer() {
                let width = LCD_WIDTH as usize;

                texture.with_lock(None, |buffer: &mut [u8], pitch: usize| {
                    for (i, pixel) in screen_buffer.iter().enumerate() {
                        let (x, y) = (i % width, i / width);
                        let (r, g, b) = Self::color(*pixel);

                        let offset = y * pitch + x * 3;
                        buffer[offset] = r;
                        buffer[offset + 1] = g;
                        buffer[offset + 2] = b;
                    }
                })?;
            }

            canvas.copy(&texture, None, Some(Rect::new(0, 0, LCD_WIDTH, LCD_HEIGHT)))?;
            canvas.present();
        }

        Ok(())
    }

    fn clock_frame(&mut self) {
        loop {
            self.emulator.clock();
            if self.emulator.ppu.frame_complete { break };
        }

        self.emulator.ppu.frame_complete = false;
    }

    fn color(pixel: u8) -> (u8, u8, u8) {
        match pixel {
            0x0 => (0xFF, 0xFF, 0xFF),
            0x1 => (0xC0, 0xC0, 0xC0),
            0x2 => (0x60, 0x60, 0x60),
            0x3 => (0x00, 0x00, 0x00),
            _ => panic!("Unknown pixel value {:02x}", pixel),
        }
    }
}
