use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use sdl2;

use sdl2::event::Event;
use sdl2::gfx::framerate::FPSManager;
use sdl2::keyboard::Scancode;
use sdl2::pixels::Color;
use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;

use sdl2::audio::{AudioCallback, AudioSpecDesired};

use core::emulator::Emulator;
use core::memory::Memory;
use core::ppu::{PixelColor, LCD_HEIGHT, LCD_WIDTH};

use crate::frontend_pge::MACHINE_CYCLES_PER_SECOND;

const SCALE: u32 = 3;

const TIME_PER_CLOCK: f32 = 1.0 / MACHINE_CYCLES_PER_SECOND as f32;
const TIME_PER_SAMPLE: f32 = 1.0 / 44100.0;

const MONOCHROME_PALETTE: [(u8, u8, u8); 4] = [
    (255, 228, 204),
    (240, 100, 120),
    (48, 51, 107),
    (19, 15, 64),
];

pub struct FrontendSdl;

struct SdlState {
    emulator: Emulator,
    num_channels: usize,
    sound_on: bool,
}

impl AudioCallback for SdlState {
    type Channel = f32;

    fn callback(&mut self, out: &mut [f32]) {
        let mut sample_elapsed = TIME_PER_SAMPLE;
        let mut clock_elapsed = 0.0;

        for xs in out.chunks_mut(self.num_channels) {
            while clock_elapsed < sample_elapsed {
                self.emulator.clock();
                clock_elapsed += TIME_PER_CLOCK;
            }

            let (left_sample, right_sample) = if self.sound_on {
                self.emulator.memory.sound_controller.get_current_samples()
            } else {
                (0.0, 0.0)
            };

            match xs.len() {
                2 => {
                    xs[0] = left_sample;
                    xs[1] = right_sample;
                }
                _ => xs[0] = (left_sample + right_sample) / 2.0,
            }

            sample_elapsed += TIME_PER_SAMPLE;
        }
    }
}

impl FrontendSdl {
    pub fn start(emulator: Emulator, save_path: PathBuf) -> Result<(), String> {
        let sdl_context = sdl2::init()?;
        let video_subsystem = sdl_context.video()?;
        let audio_subsystem = sdl_context.audio()?;

        let desired_spec = AudioSpecDesired {
            freq: Some(44_100),
            channels: Some(2),  // request stereo
            samples: Some(512), // default sample size
        };

        let mut audio_device = audio_subsystem.open_playback(None, &desired_spec, |spec| {
            println!("{:?}", spec);

            SdlState {
                emulator,
                sound_on: true,
                num_channels: spec.channels as usize,
            }
        })?;
        println!("AUDIO DEVICE CREATED");

        audio_device.resume();

        let window = video_subsystem
            .window("Gameboy", LCD_WIDTH * SCALE, LCD_HEIGHT * SCALE)
            .position_centered()
            .opengl()
            .build()
            .map_err(|e| e.to_string())?;

        let mut canvas = window
            .into_canvas()
            .present_vsync()
            .build()
            .map_err(|e| e.to_string())?;
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
                    Event::Quit { .. } => break 'running,
                    Event::KeyDown {
                        scancode: Some(Scancode::M),
                        ..
                    } => {
                        let mut device = audio_device.lock();
                        device.sound_on ^= true;
                    }
                    _ => {}
                }
            }

            let keyboard_state = event_pump.keyboard_state();

            {
                let mut device = audio_device.lock();

                if device.emulator.memory.ppu.frame_complete {
                    // TODO: Trigger joypad interrupt
                    device.emulator.memory.joypad.down =
                        keyboard_state.is_scancode_pressed(Scancode::J);
                    device.emulator.memory.joypad.up =
                        keyboard_state.is_scancode_pressed(Scancode::K);
                    device.emulator.memory.joypad.left =
                        keyboard_state.is_scancode_pressed(Scancode::H);
                    device.emulator.memory.joypad.right =
                        keyboard_state.is_scancode_pressed(Scancode::L);
                    device.emulator.memory.joypad.select =
                        keyboard_state.is_scancode_pressed(Scancode::V);
                    device.emulator.memory.joypad.start =
                        keyboard_state.is_scancode_pressed(Scancode::N);
                    device.emulator.memory.joypad.b =
                        keyboard_state.is_scancode_pressed(Scancode::D);
                    device.emulator.memory.joypad.a =
                        keyboard_state.is_scancode_pressed(Scancode::F);

                    if let Some(screen_buffer) = device.emulator.get_screen_buffer() {
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

                        canvas.copy(
                            &texture,
                            None,
                            Some(Rect::new(0, 0, LCD_WIDTH, LCD_HEIGHT)),
                        )?;
                        canvas.present();
                    }

                    device.emulator.memory.ppu.frame_complete = false;
                    Self::save_external_ram(&mut device.emulator.memory, &save_path);
                }
            }

            fps_manager.delay();
        }

        Ok(())
    }

    fn save_external_ram(memory: &mut Memory, save_path: &PathBuf) {
        memory
            .get_external_ram()
            .and_then(|ram| {
                File::create(save_path)
                    .and_then(|mut f| f.write_all(ram))
                    .ok()
            })
            .map(|_| memory.mark_external_ram_as_saved());
    }

    fn color(pixel: PixelColor) -> (u8, u8, u8) {
        match pixel {
            PixelColor::Monochrome(p) => MONOCHROME_PALETTE[p as usize],
            PixelColor::Color(r, g, b) => (r << 3, g << 3, b << 3),
        }
    }
}
