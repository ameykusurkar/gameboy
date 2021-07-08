use core::joypad::JoypadInput;
use core::ppu::PixelColor;

use std::collections::VecDeque;

pub trait Player {
    fn play_frame(&mut self, screen_buffer: Option<&[PixelColor]>) -> JoypadInput;
}

#[derive(Default)]
pub struct AutoplayController {
    pub instructions: VecDeque<AutoplayInstruction>,
}

impl Player for AutoplayController {
    fn play_frame(&mut self, _screen_buffer: Option<&[PixelColor]>) -> JoypadInput {
        let mut input = JoypadInput::default();

        if self.instructions.is_empty() {
            println!("No more instructions");
            return input;
        }

        match self.instructions[0] {
            AutoplayInstruction::Wait(1) => {
                self.instructions.pop_front();
            }
            AutoplayInstruction::Wait(frames) => {
                self.instructions[0] = AutoplayInstruction::Wait(frames - 1);
            }
            AutoplayInstruction::Right => {
                input.right = true;
                self.instructions.pop_front();
            }
            AutoplayInstruction::Start => {
                input.start = true;
                self.instructions.pop_front();
            }
            AutoplayInstruction::A => {
                input.a = true;
                self.instructions.pop_front();
            }
        }

        input
    }
}

#[derive(Copy, Clone)]
pub enum AutoplayInstruction {
    Wait(u32),
    Right,
    Start,
    A,
}
