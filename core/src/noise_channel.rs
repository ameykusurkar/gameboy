use crate::audio_components::{Timer, VolumeEnvelope};
use crate::memory::MemoryAccess;
use crate::utils::read_bit;

pub struct NoiseChannel {
    frequency_timer: Timer,
    half_size_mode: bool,
    counter_data: u8,
    linear_feedback: u16,

    frame_timer: Timer,
    frame_counter: u32,

    pub enabled: bool,

    pub length_counter: u32,
    pub length_counter_select: bool,

    pub volume_envelope: VolumeEnvelope,
}

impl NoiseChannel {
    pub fn new() -> Self {
        Self {
            frequency_timer: Timer::new(0),
            half_size_mode: false,
            counter_data: 0,
            linear_feedback: 0xFF,

            frame_timer: Timer::new(2048),
            frame_counter: 0,

            enabled: false,

            length_counter: 0,
            length_counter_select: false,

            volume_envelope: VolumeEnvelope::new(),
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

            if self.frame_counter == 7 {
                self.volume_envelope.clock();
            }
        }

        if self.frequency_timer.clock() {
            let last_bit = self.linear_feedback & 0b1;
            let penultimate_bit = (self.linear_feedback & 0b10) >> 1;
            let new_bit = last_bit ^ penultimate_bit;

            self.linear_feedback >>= 1;

            self.linear_feedback &= !(1 << 14); // Clear bit 14
            self.linear_feedback |= new_bit << 14; // Set bit 14

            if self.half_size_mode {
                self.linear_feedback &= !(1 << 6); // Clear bit 14
                self.linear_feedback |= new_bit << 6; // Set bit 6
            }
        }
    }

    pub fn sample(&self) -> f32 {
        if !self.enabled {
            return 0.0;
        }

        let amplitude = self.volume_envelope.get_current() as f32 / 16.0;

        if (self.linear_feedback & 0b1) == 0 {
            amplitude
        } else {
            -amplitude
        }
    }

    fn reset(&mut self) {
        self.frequency_timer.reload();
        self.enabled = true;

        self.linear_feedback = 0xFF;

        if self.length_counter == 0 {
            self.length_counter = 64;
        }

        self.volume_envelope.reset();
    }
}

impl MemoryAccess for NoiseChannel {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF20 => (64 - self.length_counter) as u8,
            0xFF21 => {
                (self.volume_envelope.initial << 4) as u8
                    | (self.volume_envelope.inc_mode as u8) << 3
                    | self.volume_envelope.timer.period as u8
            }
            0xFF22 => self.counter_data,
            0xFF23 => (self.length_counter_select as u8) << 6,
            _ => unreachable!("Invalid noise channel address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF20 => {
                let sound_length_data = byte & 0b0011_1111;
                self.length_counter = 64 - sound_length_data as u32;
            }
            0xFF21 => {
                self.volume_envelope.initial = ((byte & 0b1111_0000) >> 4) as u32;
                self.volume_envelope.inc_mode = read_bit(byte, 3);
                self.volume_envelope.timer.period = (byte & 0b111) as u32;

                if byte == 0x08 {
                    self.volume_envelope.increment();
                }
            }
            0xFF22 => {
                self.counter_data = byte;

                self.half_size_mode = read_bit(byte, 3);

                let base_shift = ((byte & 0b1111_0000) >> 4) as u32;
                let base_period = (byte & 0b0000_0111) as u32;

                self.frequency_timer.period = if base_period == 0 {
                    1 << (base_shift + 1)
                } else {
                    2 * base_period * (1 << (base_shift + 1))
                };
            }
            0xFF23 => {
                let should_restart = read_bit(byte, 7);

                if should_restart {
                    self.reset();
                }

                self.length_counter_select = read_bit(byte, 6);
            }
            _ => unreachable!("Invalid noise channel address: {:04x}", addr),
        }
    }
}
