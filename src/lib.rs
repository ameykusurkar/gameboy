use wasm_bindgen::prelude::*;
use crate::emulator::Emulator;
use std::path::Path;

extern crate console_error_panic_hook;

mod cpu;
mod ppu;
mod registers;
mod memory;
mod instruction;
mod joypad;
mod utils;
mod cartridge;
mod emulator;
mod sound_controller;
mod square_wave_channel;
mod wave_channel;
mod noise_channel;
mod audio_components;

mod bootrom;
use crate::bootrom::BOOTROM;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen]
extern {
    fn alert(s: &str);
}

#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
struct EmulatorState {
    emulator: Emulator,
}

const EMPTY: [u8; 0] = [];

#[wasm_bindgen]
impl EmulatorState {
    pub fn new(rom_buffer: &[u8]) -> EmulatorState {
        let emulator = Emulator::new(&BOOTROM, rom_buffer.to_vec(), None);
        EmulatorState { emulator }
    }

    #[wasm_bindgen(js_name = clockFrame)]
    pub fn clockFrame(&mut self) {
        while !self.emulator.ppu.frame_complete {
            self.emulator.clock();
        }
        
        self.emulator.ppu.frame_complete = false;
    }

    pub fn pixels(&self) -> *const u8 {
        self.emulator.get_screen_buffer().unwrap_or(&EMPTY).as_ptr()
    }
}

#[wasm_bindgen]
pub fn greet(bytes: &[u8]) {
    alert(&format!("{:x}{:x}{:x}", bytes[0], bytes[1], bytes[2]));
}
