use wasm_bindgen::prelude::*;

extern crate console_error_panic_hook;

use core::emulator::Emulator;

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
struct EmulatorState {
    emulator: Emulator,
}

const EMPTY: [u8; 1] = [4];

#[wasm_bindgen]
impl EmulatorState {
    pub fn new(rom_buffer: &[u8]) -> EmulatorState {
        let emulator = Emulator::new(rom_buffer.to_vec(), None);
        EmulatorState { emulator }
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

    pub fn pixels(&self) -> *const u8 {
        self.emulator.get_screen_buffer().unwrap_or(&EMPTY).as_ptr()
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
        self.emulator.ppu.frame_complete
    }

    #[wasm_bindgen(js_name = markFrameAsComplete)]
    pub fn mark_frame_as_complete(&mut self) {
        self.emulator.ppu.frame_complete = false;
    }

    pub fn clock(&mut self) {
        self.emulator.clock();
    }
}
