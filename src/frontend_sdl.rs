use sdl2;

use sdl2::pixels::Color;
use sdl2::pixels::PixelFormatEnum;
use sdl2::event::Event;
use sdl2::rect::Rect;
use sdl2::keyboard::Scancode;
use sdl2::gfx::framerate::FPSManager;

use sdl2::audio::{AudioCallback, AudioSpecDesired};

use crate::emulator::Emulator;
use crate::ppu::{LCD_HEIGHT, LCD_WIDTH};

use crate::frontend_pge::MACHINE_CYCLES_PER_SECOND;

const SCALE: u32 = 3;

pub struct FrontendSdl;

impl AudioCallback for Emulator {
    type Channel = f32;

    fn callback(&mut self, out: &mut [f32]) {
        let num_samples = out.len();
        let time_taken = num_samples as f32 / 44_100.0;
        let num_cycles = (MACHINE_CYCLES_PER_SECOND as f32 * time_taken) as u32;

        for _ in 0..num_cycles {
            self.clock();
        }

        for x in out.iter_mut() {
            *x = 0.0;
        }
    }
}

impl FrontendSdl {
    pub fn start(emulator: Emulator) -> Result<(), String> {
        let sdl_context = sdl2::init()?;
        let video_subsystem = sdl_context.video()?;
        let audio_subsystem = sdl_context.audio()?;

        let desired_spec = AudioSpecDesired {
            freq: Some(44_100),
            channels: Some(1),  // mono
            samples: None       // default sample size
        };

        let mut audio_device = audio_subsystem.open_playback(None, &desired_spec, |spec| {
            println!("{:?}", spec);

            emulator
        })?;
        println!("AUDIO DEVICE CREATED");

        audio_device.resume();

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

        let mut fps_manager = FPSManager::new();
        fps_manager.set_framerate(60)?;

        'running: loop {
            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit {..} => { break 'running },
                    _ => {}
                }
            }

            let keyboard_state = event_pump.keyboard_state();

            {
                let mut emulator = audio_device.lock();
                // println!("LOCK");

                // TODO: Trigger joypad interrupt
                emulator.memory.joypad.down   = keyboard_state.is_scancode_pressed(Scancode::J);
                emulator.memory.joypad.up     = keyboard_state.is_scancode_pressed(Scancode::K);
                emulator.memory.joypad.left   = keyboard_state.is_scancode_pressed(Scancode::H);
                emulator.memory.joypad.right  = keyboard_state.is_scancode_pressed(Scancode::L);
                emulator.memory.joypad.select = keyboard_state.is_scancode_pressed(Scancode::V);
                emulator.memory.joypad.start  = keyboard_state.is_scancode_pressed(Scancode::N);
                emulator.memory.joypad.b      = keyboard_state.is_scancode_pressed(Scancode::D);
                emulator.memory.joypad.a      = keyboard_state.is_scancode_pressed(Scancode::F);

                // self.clock_frame();

                // emulator.memory.joypad.clear();
                emulator.save_external_ram();

                if emulator.ppu.frame_complete {
                    if let Some(screen_buffer) = emulator.get_screen_buffer() {
                        let width = LCD_WIDTH as usize;

                        texture.with_lock(None, |buffer: &mut [u8], pitch: usize| {
                            for (i, pixel) in screen_buffer.iter().enumerate() {
                                let (x, y) = (i % width, i / width);
                                let (r, g, b) = Self::color(*pixel);

                                let offset = y * pitch + x * 3;
                                buffer[offset]     = r;
                                buffer[offset + 1] = g;
                                buffer[offset + 2] = b;
                            }
                        })?;
                    }
                    emulator.ppu.frame_complete = false;
                }
            }

            canvas.copy(&texture, None, Some(Rect::new(0, 0, LCD_WIDTH, LCD_HEIGHT)))?;
            canvas.present();
        }

        Ok(())
    }

    fn color(pixel: u8) -> (u8, u8, u8) {
        match pixel {
            // 0x0 => (0xFF, 0xFF, 0xFF),
            // 0x1 => (0xC0, 0xC0, 0xC0),
            // 0x2 => (0x60, 0x60, 0x60),
            // 0x3 => (0x00, 0x00, 0x00),
            0x0 => (255, 190, 118),
            0x1 => (235, 77, 75),
            0x2 => (48, 51, 107),
            0x3 => (19, 15, 64),
            _ => panic!("Unknown pixel value {:02x}", pixel),
        }
    }
}
