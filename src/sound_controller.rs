use crate::memory::MemoryAccess;
use crate::utils::{read_bit, set_bit};

use crate::square_wave_channel::SquareWaveChannel;

pub struct SoundController {
    square_wave_1: SquareWaveChannel,
    square_wave_2: SquareWaveChannel,

    master_sound_on: bool,
    sound_output_select: u8,
    channel_control: u8,

    output_1_volume: u8,
    output_2_volume: u8,
}

impl SoundController {
    pub fn new() -> Self {
        SoundController {
            square_wave_1: SquareWaveChannel::new(),
            square_wave_2: SquareWaveChannel::new(),

            master_sound_on: false,
            sound_output_select: 0,
            channel_control: 0,

            output_1_volume: 0,
            output_2_volume: 0,
        }
    }

    fn set_frequency_low_bits(square_wave: &mut SquareWaveChannel, byte: u8) {
        let old = square_wave.get_frequency();
        square_wave.set_frequency((old & 0b111_0000_0000) | byte as u32);
    }

    fn set_frequency_high_bits(square_wave: &mut SquareWaveChannel, byte: u8) {
        let old = square_wave.get_frequency();
        square_wave.set_frequency((old & 0b000_1111_1111) | (byte as u32) << 8);
    }

    pub fn clock(&mut self) {
        self.square_wave_1.clock();
        self.square_wave_2.clock();
    }

    pub fn get_current_sample_left(&self) -> f32 {
        if !self.is_sound_on() {
            return 0.0;
        }

        let mut sample = 0.0;

        if read_bit(self.sound_output_select, 4) {
            sample += self.square_wave_1.sample();
        }

        if read_bit(self.sound_output_select, 5) {
            sample += self.square_wave_2.sample();
        }

        0.2 * (self.output_2_volume as f32 / 7.0) * sample
    }

    pub fn get_current_sample_right(&self) -> f32 {
        if !self.is_sound_on() {
            return 0.0;
        }

        let mut sample = 0.0;

        if read_bit(self.sound_output_select, 0) {
            sample += self.square_wave_1.sample();
        }

        if read_bit(self.sound_output_select, 1) {
            sample += self.square_wave_2.sample();
        }

        0.2 * (self.output_1_volume as f32 / 7.0) * sample
    }

    fn is_sound_on(&self) -> bool {
        self.master_sound_on
    }

    fn square_wave_for_addr(&self, addr: u16) -> &SquareWaveChannel {
        if (0xFF10..=0xFF14).contains(&addr) {
            &self.square_wave_1
        } else {
            &self.square_wave_2
        }
    }

    fn square_wave_for_addr_mut(&mut self, addr: u16) -> &mut SquareWaveChannel {
        if (0xFF10..=0xFF14).contains(&addr) {
            &mut self.square_wave_1
        } else {
            &mut self.square_wave_2
        }
    }
}

impl MemoryAccess for SoundController {
    fn read(&self, addr: u16) -> u8 {
        let square_wave = self.square_wave_for_addr(addr);

        match addr {
            0xFF10 => {
                (square_wave.sweep_timer.period as u8) << 4
                    | (square_wave.frequency_subtract_mode as u8) << 3
                    | square_wave.sweep_shift as u8
            },
            0xFF11 | 0xFF16 => {
                let duty_cycle = match square_wave.waveform {
                    0b0000_0001 => 0,
                    0b1000_0001 => 1,
                    0b1000_0111 => 2,
                    0b0111_1110 => 3,
                    _ => unreachable!("Invalid waveform: {:02x}", square_wave.waveform),
                };

                (duty_cycle << 6 | (64 - square_wave.length_counter)) as u8
            },
            0xFF12 | 0xFF17 => {
                (square_wave.initial_volume << 4) as u8
                    | (square_wave.volume_up as u8) << 3
                    | square_wave.volume_timer.period as u8
            },
            0xFF13 | 0xFF18 => {
                0xFF
            },
            0xFF14 | 0xFF19 => {
                (square_wave.length_counter_select as u8) << 6
            },
            0xFF24 => {
                self.channel_control
            },
            0xFF25 => {
                self.sound_output_select
            },
            0xFF26 => {
                let mut data = set_bit(0b1111_1111, 7, self.master_sound_on);
                data = set_bit(data, 0, self.square_wave_1.enabled);
                data = set_bit(data, 1, self.square_wave_2.enabled);
                data
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        let mut square_wave = self.square_wave_for_addr_mut(addr);

        match addr {
            0xFF10 => {
                square_wave.sweep_timer.period = ((byte & 0b0111_0000) >> 4) as u32;
                square_wave.frequency_subtract_mode = read_bit(byte, 3);
                square_wave.sweep_shift = (byte & 0b0000_0111) as u32;
            },
            0xFF11 | 0xFF16 => {
                let sound_length_data = byte & 0b0011_1111;
                square_wave.length_counter = 64 - sound_length_data as u32;

                square_wave.waveform = match (byte & 0b1100_0000) >> 6 {
                    0 => 0b0000_0001, // 12.5% Duty cycle
                    1 => 0b1000_0001, // 25%   Duty cycle
                    2 => 0b1000_0111, // 50%   Duty cycle
                    3 => 0b0111_1110, // 75%   Duty cycle
                    _ => unreachable!("Invalid duty cycle byte: {:02x}", byte),
                };
            },
            0xFF12 | 0xFF17 => {
                square_wave.initial_volume = ((byte & 0b1111_0000) >> 4) as u32;
                square_wave.volume_up = read_bit(byte, 3);
                square_wave.volume_timer.period = (byte & 0b111) as u32;

                if byte == 0x08 {
                    square_wave.increment_volume();
                }
            },
            0xFF13 | 0xFF18 => {
                Self::set_frequency_low_bits(&mut square_wave, byte);
            },
            0xFF14 | 0xFF19 => {
                Self::set_frequency_high_bits(&mut square_wave, byte & 0b0000_0111);

                let should_restart = read_bit(byte, 7);

                if should_restart {
                    square_wave.reset();
                }

                square_wave.length_counter_select = read_bit(byte, 6);
            },
            0xFF24 => {
                self.channel_control = byte;

                self.output_1_volume = byte & 0b0000_0111;
                self.output_2_volume = (byte & 0b0111_0000) >> 4;
            },
            0xFF25 => {
                self.sound_output_select = byte;
            },
            0xFF26 => {
                self.master_sound_on = read_bit(byte, 7);
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }
}
