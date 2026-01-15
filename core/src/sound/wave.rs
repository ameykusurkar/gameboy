use super::components::Timer;
use crate::memory::MemoryAccess;
use crate::utils::read_bit;

pub struct WaveChannel {
    channel_on: bool,
    frequency_timer: Timer,
    wave_pattern: [u8; 16],
    wave_position: usize,
    sample_buffer: u32,

    frame_timer: Timer,
    frame_counter: u32,

    pub enabled: bool,

    pub length_counter: u32,
    pub length_counter_select: bool,

    pub volume_shift: u32,
}

impl WaveChannel {
    pub fn new() -> Self {
        Self {
            channel_on: false,
            frequency_timer: Timer::new(0),
            wave_position: 0,
            wave_pattern: [0; 16],
            sample_buffer: 0,

            frame_timer: Timer::new(2048),
            frame_counter: 0,

            enabled: false,

            length_counter: 0,
            length_counter_select: false,

            volume_shift: 0,
        }
    }

    pub fn clock(&mut self) {
        if self.frame_timer.clock() {
            self.frame_counter = (self.frame_counter + 1) % 8;

            if self.frame_counter % 2 == 0 && self.enabled && self.length_counter > 0 {
                self.length_counter -= 1;
            }

            if self.length_counter == 0 && self.length_counter_select {
                self.enabled = false;
            }
        }

        if self.frequency_timer.clock() {
            self.wave_position = (self.wave_position + 1) % 32;
            self.sample_buffer = self.get_wave_sample(self.wave_position);
        }
    }

    pub fn sample(&self) -> f32 {
        if !self.enabled || !self.channel_on || self.volume_shift == 0 {
            return 0.0;
        }

        let wave_sample = self.sample_buffer >> (self.volume_shift - 1);

        (wave_sample as f32 / 7.5) - 1.0
    }

    fn get_frequency(&self) -> u32 {
        2048 - (self.frequency_timer.period * 2)
    }

    fn set_frequency(&mut self, frequency: u32) {
        self.frequency_timer.period = (2048 - frequency) / 2;
    }

    fn set_frequency_low_bits(&mut self, byte: u8) {
        let old = self.get_frequency();
        self.set_frequency((old & 0b111_0000_0000) | byte as u32);
    }

    fn set_frequency_high_bits(&mut self, byte: u8) {
        let old = self.get_frequency();
        self.set_frequency((old & 0b000_1111_1111) | (byte as u32) << 8);
    }

    fn get_wave_sample(&self, index: usize) -> u32 {
        let (pattern_index, byte_index) = (index / 2, index % 2);
        let wave_pattern_byte = self.wave_pattern[pattern_index] as u32;
        if byte_index == 0 {
            (wave_pattern_byte & 0xF0) >> 4
        } else {
            wave_pattern_byte & 0x0F
        }
    }

    fn reset(&mut self) {
        self.frequency_timer.reload();
        self.enabled = self.channel_on;

        self.wave_position = 0;

        if self.length_counter == 0 {
            self.length_counter = 256;
        }
    }
}

impl MemoryAccess for WaveChannel {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF1A => (self.channel_on as u8) << 7,
            0xFF1B => (256 - self.length_counter) as u8,
            0xFF1C => (self.volume_shift << 5) as u8,
            0xFF1D => 0xFF,
            0xFF1E => (self.length_counter_select as u8) << 6,
            0xFF30..=0xFF3F => self.wave_pattern[addr as usize - 0xFF30],
            _ => unreachable!("Invalid noise channel address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF1A => {
                self.channel_on = read_bit(byte, 7);

                if !self.channel_on {
                    self.enabled = false;
                }
            }
            0xFF1B => {
                self.length_counter = 256 - byte as u32;
            }
            0xFF1C => {
                self.volume_shift = ((byte & 0b0110_0000) >> 5) as u32;
            }
            0xFF1D => {
                self.set_frequency_low_bits(byte);
            }
            0xFF1E => {
                self.set_frequency_high_bits(byte & 0b0000_0111);

                let should_restart = read_bit(byte, 7);

                if should_restart {
                    self.reset();
                }

                self.length_counter_select = read_bit(byte, 6);
            }
            0xFF30..=0xFF3F => {
                self.wave_pattern[addr as usize - 0xFF30] = byte;
            }
            _ => unreachable!("Invalid noise channel address: {:04x}", addr),
        }
    }
}
