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
        if self.instructions.is_empty() {
            self.process_screen()
        } else {
            self.play_starting_instruction()
        }
    }
}

impl AutoplayController {
    fn play_starting_instruction(&mut self) -> JoypadInput {
        match self.instructions[0] {
            AutoplayInstruction::Repeat(button, count) => {
                let input = process_button(button);

                match count {
                    1 => {
                        self.instructions.pop_front();
                    }
                    _ => self.instructions[0] = AutoplayInstruction::Repeat(button, count - 1),
                }

                input
            }
            AutoplayInstruction::Alternate(btn1, btn2, count) => {
                let input = if count % 2 == 0 {
                    process_button(btn1)
                } else {
                    process_button(btn2)
                };

                match count {
                    1 => {
                        self.instructions.pop_front();
                    }
                    _ => {
                        self.instructions[0] = AutoplayInstruction::Alternate(btn1, btn2, count - 1)
                    }
                }

                input
            }
        }
    }

    fn process_screen(&mut self) -> JoypadInput {
        println!("No more instructions");
        return JoypadInput::default();
    }
}

fn process_button(button: Button) -> JoypadInput {
    let mut input = JoypadInput::default();

    match button {
        Button::Wait => (),
        Button::Down => input.down = true,
        Button::Start => input.start = true,
        Button::A => input.a = true,
    }

    input
}

#[derive(Copy, Clone)]
pub enum AutoplayInstruction {
    Repeat(Button, u32),
    Alternate(Button, Button, u32),
}

#[derive(Copy, Clone)]
pub enum Button {
    Wait,
    Down,
    Start,
    A,
}
