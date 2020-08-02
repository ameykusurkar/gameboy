use wasm_bindgen::prelude::*;

extern crate console_error_panic_hook;

use core::emulator::Emulator;
use core::ppu::PixelColor;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen(js_name = initPanicHook)]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub struct EmulatorState {
    emulator: Emulator,
    pixel_buffer: Vec<u8>,
}

#[wasm_bindgen]
impl EmulatorState {
    pub fn new(rom_buffer: &[u8]) -> EmulatorState {
        let emulator = Emulator::new(rom_buffer.to_vec(), None);
        EmulatorState { emulator, pixel_buffer: vec![0; 160 * 144] }
    }

    #[wasm_bindgen(js_name = updateJoypad)]
    pub fn update_joypad(&mut self, states: &[u8]) {
        self.emulator.memory.joypad.down   = states[0] > 0;
        self.emulator.memory.joypad.up     = states[1] > 0;
        self.emulator.memory.joypad.left   = states[2] > 0;
        self.emulator.memory.joypad.right  = states[3] > 0;
        self.emulator.memory.joypad.select = states[4] > 0;
        self.emulator.memory.joypad.start  = states[5] > 0;
        self.emulator.memory.joypad.b      = states[6] > 0;
        self.emulator.memory.joypad.a      = states[7] > 0;
    }

    pub fn pixels(&mut self) -> *const u8 {
        match self.emulator.get_screen_buffer() {
            Some(buffer) => {
                let processed_buffer: Vec<u8> = buffer.iter().map(PixelColor::raw).collect();
                self.pixel_buffer.copy_from_slice(&processed_buffer);
            },
            None => (),
        }

        self.pixel_buffer.as_ptr()
    }

    // TODO: Implement stereo sound
    #[wasm_bindgen(js_name = currentAudioSample)]
    pub fn current_audio_sample(&self) -> f32 {
        (
            self.emulator.memory.sound_controller.get_current_samples().0 +
            self.emulator.memory.sound_controller.get_current_samples().1
        ) / 2.0
    }

    #[wasm_bindgen(js_name = isFrameComplete)]
    pub fn is_frame_complete(&self) -> bool {
        self.emulator.memory.ppu.frame_complete
    }

    #[wasm_bindgen(js_name = markFrameAsComplete)]
    pub fn mark_frame_as_complete(&mut self) {
        self.emulator.memory.ppu.frame_complete = false;
    }

    pub fn clock(&mut self) {
        self.emulator.clock();
    }
}
