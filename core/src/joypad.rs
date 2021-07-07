use crate::memory::MemoryAccess;
use crate::utils::read_bit;

#[derive(Default)]
pub struct Joypad {
    pub button_selected: bool,
    pub direction_selected: bool,
    pub down: bool,
    pub up: bool,
    pub left: bool,
    pub right: bool,
    pub start: bool,
    pub select: bool,
    pub b: bool,
    pub a: bool,
}

pub struct JoypadInput {
    pub down: bool,
    pub up: bool,
    pub left: bool,
    pub right: bool,
    pub start: bool,
    pub select: bool,
    pub b: bool,
    pub a: bool,
}

impl Joypad {
    pub fn clear(&mut self) {
        self.down = false;
        self.up = false;
        self.left = false;
        self.right = false;
        self.start = false;
        self.select = false;
        self.b = false;
        self.a = false;
    }

    pub fn set_input(&mut self, input: &JoypadInput) {
        self.down = input.down;
        self.up = input.up;
        self.left = input.left;
        self.right = input.right;
        self.start = input.start;
        self.select = input.select;
        self.b = input.b;
        self.a = input.a;
    }

    fn read_buttons(&self) -> u8 {
        (!self.start as u8) << 3
            | (!self.select as u8) << 2
            | (!self.b as u8) << 1
            | (!self.a as u8) << 0
    }

    fn read_directions(&self) -> u8 {
        (!self.down as u8) << 3
            | (!self.up as u8) << 2
            | (!self.left as u8) << 1
            | (!self.right as u8) << 0
    }
}

impl MemoryAccess for Joypad {
    fn read(&self, _addr: u16) -> u8 {
        let keys = if self.button_selected {
            self.read_buttons()
        } else if self.direction_selected {
            self.read_directions()
        } else {
            0x0F
        };

        0xC0 | (!self.button_selected as u8) << 5 | (!self.direction_selected as u8) << 4 | keys
    }

    fn write(&mut self, _addr: u16, byte: u8) {
        self.button_selected = !read_bit(byte, 5);
        self.direction_selected = !read_bit(byte, 4);
    }
}
