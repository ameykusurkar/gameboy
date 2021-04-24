use crate::memory::MemoryAccess;
use crate::utils::{read_bit, set_bit};

use crate::noise_channel::NoiseChannel;
use crate::square_wave_channel::SquareWaveChannel;
use crate::wave_channel::WaveChannel;

pub struct SoundController {
    square_wave_1: SquareWaveChannel,
    square_wave_2: SquareWaveChannel,
    wave_channel: WaveChannel,
    noise_channel: NoiseChannel,

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
            wave_channel: WaveChannel::new(),
            noise_channel: NoiseChannel::new(),

            master_sound_on: false,
            sound_output_select: 0,
            channel_control: 0,

            output_1_volume: 0,
            output_2_volume: 0,
        }
    }

    pub fn clock(&mut self) {
        self.square_wave_1.clock();
        self.square_wave_2.clock();
        self.wave_channel.clock();
        self.noise_channel.clock();
    }

    pub fn get_current_samples(&self) -> (f32, f32) {
        if !self.is_sound_on() {
            return (0.0, 0.0);
        }

        let left_sample = self.get_current_sample_output_2();
        let right_sample = self.get_current_sample_output_1();
        (0.2 * left_sample, 0.2 * right_sample)
    }

    fn get_current_sample_output_2(&self) -> f32 {
        let mut sample = 0.0;

        if read_bit(self.sound_output_select, 4) {
            sample += self.square_wave_1.sample();
        }

        if read_bit(self.sound_output_select, 5) {
            sample += self.square_wave_2.sample();
        }

        if read_bit(self.sound_output_select, 6) {
            sample += self.wave_channel.sample();
        }

        if read_bit(self.sound_output_select, 7) {
            sample += self.noise_channel.sample();
        }

        (self.output_2_volume as f32 / 7.0) * sample
    }

    fn get_current_sample_output_1(&self) -> f32 {
        let mut sample = 0.0;

        if read_bit(self.sound_output_select, 0) {
            sample += self.square_wave_1.sample();
        }

        if read_bit(self.sound_output_select, 1) {
            sample += self.square_wave_2.sample();
        }

        if read_bit(self.sound_output_select, 2) {
            sample += self.wave_channel.sample();
        }

        if read_bit(self.sound_output_select, 3) {
            sample += self.noise_channel.sample();
        }

        (self.output_1_volume as f32 / 7.0) * sample
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
        match addr {
            0xFF10..=0xFF14 | 0xFF16..=0xFF19 => {
                let square_wave = self.square_wave_for_addr(addr);
                square_wave.read(addr)
            }
            0xFF1A..=0xFF1E => self.wave_channel.read(addr),
            0xFF20..=0xFF23 => self.noise_channel.read(addr),
            0xFF24 => self.channel_control,
            0xFF25 => self.sound_output_select,
            0xFF26 => {
                let mut data = set_bit(0b1111_1111, 7, self.master_sound_on);
                data = set_bit(data, 0, self.square_wave_1.enabled);
                data = set_bit(data, 1, self.square_wave_2.enabled);
                data = set_bit(data, 2, self.wave_channel.enabled);
                data = set_bit(data, 3, self.noise_channel.enabled);
                data
            }
            0xFF30..=0xFF3F => self.wave_channel.read(addr),
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF10..=0xFF14 | 0xFF16..=0xFF19 => {
                let square_wave = self.square_wave_for_addr_mut(addr);
                square_wave.write(addr, byte);
            }
            0xFF1A..=0xFF1E => self.wave_channel.write(addr, byte),
            0xFF20..=0xFF23 => self.noise_channel.write(addr, byte),
            0xFF24 => {
                self.channel_control = byte;

                self.output_1_volume = byte & 0b0000_0111;
                self.output_2_volume = (byte & 0b0111_0000) >> 4;
            }
            0xFF25 => {
                self.sound_output_select = byte;
            }
            0xFF26 => {
                self.master_sound_on = read_bit(byte, 7);
            }
            0xFF30..=0xFF3F => self.wave_channel.write(addr, byte),
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }
}
