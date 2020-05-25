use crate::memory::MemoryAccess;
use crate::utils::{read_bit, set_bit};

use crate::square_wave_channel::SquareWaveChannel;

pub struct SoundController {
    square_wave_1: SquareWaveChannel,
    master_sound_on: bool,
}

impl SoundController {
    pub fn new() -> Self {
        SoundController {
            square_wave_1: SquareWaveChannel::new(),
            master_sound_on: false,
        }
    }

    fn set_frequency_low_bits(&mut self, byte: u8) {
        let old = self.square_wave_1.get_frequency();
        self.square_wave_1.set_frequency((old & 0b111_0000_0000) | byte as u32);
    }

    fn set_frequency_high_bits(&mut self, byte: u8) {
        let old = self.square_wave_1.get_frequency();
        self.square_wave_1.set_frequency((old & 0b000_1111_1111) | (byte as u32) << 8);
    }

    pub fn clock(&mut self) {
        self.square_wave_1.clock();
    }

    pub fn get_current_sample(&self) -> f32 {
        if !self.is_sound_on() {
            return 0.0;
        }

        self.square_wave_1.sample()
    }

    fn is_sound_on(&self) -> bool {
        // self.master_sound_switch && self.sound1_switch
        self.master_sound_on
    }
}

impl MemoryAccess for SoundController {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF10 => {
                (self.square_wave_1.sweep_timer.period as u8) << 4
                    | (self.square_wave_1.frequency_subtract_mode as u8) << 3
                    | self.square_wave_1.sweep_shift as u8
            },
            0xFF11 => {
                let duty_cycle = match self.square_wave_1.waveform {
                    0b0000_0001 => 0,
                    0b1000_0001 => 1,
                    0b1000_0111 => 2,
                    0b0111_1110 => 3,
                    _ => unreachable!("Invalid waveform: {:02x}", self.square_wave_1.waveform),
                };

                (duty_cycle << 6 | (64 - self.square_wave_1.length_counter)) as u8
            },
            0xFF12 => {
                (self.square_wave_1.initial_volume << 4) as u8
                    | (self.square_wave_1.volume_up as u8) << 3
                    | self.square_wave_1.volume_timer.period as u8
            },
            0xFF13 => {
                0xFF
            },
            0xFF14 => {
                (self.square_wave_1.length_counter_select as u8) << 6
            },
            0xFF26 => {
                let data = set_bit(0b1111_1111, 7, self.master_sound_on);
                set_bit(data, 7, self.square_wave_1.enabled)
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF10 => {
                self.square_wave_1.sweep_timer.period = ((byte & 0b0111_0000) >> 4) as u32;
                self.square_wave_1.frequency_subtract_mode = read_bit(byte, 3);
                self.square_wave_1.sweep_shift = (byte & 0b0000_0111) as u32;
            },
            0xFF11 => {
                let sound_length_data = byte & 0b0011_1111;
                self.square_wave_1.length_counter = 64 - sound_length_data as u32;

                self.square_wave_1.waveform = match (byte & 0b1100_0000) >> 6 {
                    0 => 0b0000_0001, // 12.5% Duty cycle
                    1 => 0b1000_0001, // 25%   Duty cycle
                    2 => 0b1000_0111, // 50%   Duty cycle
                    3 => 0b0111_1110, // 75%   Duty cycle
                    _ => unreachable!("Invalid duty cycle byte: {:02x}", byte),
                };
            },
            0xFF12 => {
                self.square_wave_1.initial_volume = ((byte & 0b1111_0000) >> 4) as u32;
                self.square_wave_1.volume_up = read_bit(byte, 3);
                self.square_wave_1.volume_timer.period = (byte & 0b111) as u32;

                if byte == 0x08 {
                    self.square_wave_1.increment_volume();
                }
            },
            0xFF13 => {
                self.set_frequency_low_bits(byte);
            },
            0xFF14 => {
                self.set_frequency_high_bits(byte & 0b0000_0111);

                let should_restart = read_bit(byte, 7);

                if should_restart {
                    self.square_wave_1.reset();
                }

                self.square_wave_1.length_counter_select = read_bit(byte, 6);
            },
            0xFF26 => {
                self.master_sound_on = read_bit(byte, 7);
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }
}
