use super::components::{Timer, VolumeEnvelope};
use crate::memory::MemoryAccess;
use crate::utils::read_bit;

pub struct SquareWaveChannel {
    frame_timer: Timer,
    frame_counter: u32,

    waveform: u8,
    pub enabled: bool,
    frequency_timer: Timer,
    waveform_bit: u8,

    length_counter: u32,
    length_counter_select: bool,

    volume_envelope: VolumeEnvelope,

    sweep_timer: Timer,
    sweep_shift: u32,
    frequency_subtract_mode: bool,
    shadow_frequency: u32,
    sweep_enabled: bool,
}

impl SquareWaveChannel {
    pub fn new() -> Self {
        Self {
            frame_timer: Timer::new(2048),
            frame_counter: 0,

            waveform: 0b0000_1111,
            enabled: false,
            frequency_timer: Timer::new(0),
            waveform_bit: 0,

            length_counter: 0,
            length_counter_select: false,

            volume_envelope: VolumeEnvelope::new(),

            sweep_timer: Timer::new(0),
            sweep_shift: 0,
            frequency_subtract_mode: false,
            shadow_frequency: 0,
            sweep_enabled: true,
        }
    }

    fn get_frequency(&self) -> u32 {
        2048 - self.frequency_timer.period
    }

    fn set_frequency(&mut self, frequency: u32) {
        self.frequency_timer.period = 2048 - frequency;
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

            if (self.frame_counter == 2 || self.frame_counter == 6) && self.sweep_timer.clock() {
                if self.sweep_enabled && self.sweep_timer.period > 0 {
                    match self.new_frequency() {
                        None => self.enabled = false,
                        Some(new_freq) => {
                            if self.sweep_shift > 0 {
                                self.shadow_frequency = new_freq;
                                self.set_frequency(new_freq);

                                if self.new_frequency().is_none() {
                                    self.enabled = false;
                                }
                            }
                        }
                    }
                }
            }
        }

        if self.frequency_timer.clock() {
            self.waveform_bit = (self.waveform_bit + 1) % 8;
        }
    }

    pub fn sample(&self) -> f32 {
        if !self.enabled {
            return 0.0;
        }

        let amplitude = self.volume_envelope.get_current() as f32 / 16.0;

        if read_bit(self.waveform, self.waveform_bit) {
            amplitude
        } else {
            -amplitude
        }
    }

    fn new_frequency(&self) -> Option<u32> {
        let delta = self.shadow_frequency >> self.sweep_shift;

        let result = if self.frequency_subtract_mode {
            self.shadow_frequency - delta
        } else {
            self.shadow_frequency + delta
        };

        if result < 2048 {
            Some(result)
        } else {
            None
        }
    }

    fn set_frequency_low_bits(&mut self, byte: u8) {
        let old = self.get_frequency();
        self.set_frequency((old & 0b111_0000_0000) | byte as u32);
    }

    fn set_frequency_high_bits(&mut self, byte: u8) {
        let old = self.get_frequency();
        self.set_frequency((old & 0b000_1111_1111) | (byte as u32) << 8);
    }

    fn reset(&mut self) {
        self.frequency_timer.reload();
        self.enabled = true;

        if self.length_counter == 0 {
            self.length_counter = 64;
        }

        self.volume_envelope.reset();

        self.shadow_frequency = self.get_frequency();
        self.sweep_timer.reload();
        self.sweep_enabled = (self.sweep_timer.period > 0) || (self.sweep_shift > 0);

        if self.sweep_shift > 0 && self.new_frequency().is_none() {
            self.enabled = false;
        }
    }
}

impl MemoryAccess for SquareWaveChannel {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF10 => {
                (self.sweep_timer.period as u8) << 4
                    | (self.frequency_subtract_mode as u8) << 3
                    | self.sweep_shift as u8
            }
            0xFF11 | 0xFF16 => {
                let duty_cycle = match self.waveform {
                    0b0000_0001 => 0,
                    0b1000_0001 => 1,
                    0b1000_0111 => 2,
                    0b0111_1110 => 3,
                    _ => unreachable!("Invalid waveform: {:02x}", self.waveform),
                };

                (duty_cycle << 6 | (64 - self.length_counter)) as u8
            }
            0xFF12 | 0xFF17 => {
                (self.volume_envelope.initial << 4) as u8
                    | (self.volume_envelope.inc_mode as u8) << 3
                    | self.volume_envelope.timer.period as u8
            }
            0xFF13 | 0xFF18 => 0xFF,
            0xFF14 | 0xFF19 => (self.length_counter_select as u8) << 6,
            _ => unreachable!("Invalid square wave address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF10 => {
                self.sweep_timer.period = ((byte & 0b0111_0000) >> 4) as u32;
                self.frequency_subtract_mode = read_bit(byte, 3);
                self.sweep_shift = (byte & 0b0000_0111) as u32;
            }
            0xFF11 | 0xFF16 => {
                let sound_length_data = byte & 0b0011_1111;
                self.length_counter = 64 - sound_length_data as u32;

                self.waveform = match (byte & 0b1100_0000) >> 6 {
                    0 => 0b0000_0001, // 12.5% Duty cycle
                    1 => 0b1000_0001, // 25%   Duty cycle
                    2 => 0b1000_0111, // 50%   Duty cycle
                    3 => 0b0111_1110, // 75%   Duty cycle
                    _ => unreachable!("Invalid duty cycle byte: {:02x}", byte),
                };
            }
            0xFF12 | 0xFF17 => {
                self.volume_envelope.initial = ((byte & 0b1111_0000) >> 4) as u32;
                self.volume_envelope.inc_mode = read_bit(byte, 3);
                self.volume_envelope.timer.period = (byte & 0b111) as u32;

                if byte == 0x08 {
                    self.volume_envelope.increment();
                }
            }
            0xFF13 | 0xFF18 => {
                self.set_frequency_low_bits(byte);
            }
            0xFF14 | 0xFF19 => {
                self.set_frequency_high_bits(byte & 0b0000_0111);

                let should_restart = read_bit(byte, 7);

                if should_restart {
                    self.reset();
                }

                self.length_counter_select = read_bit(byte, 6);
            }
            _ => unreachable!("Invalid square wave address: {:04x}", addr),
        }
    }
}
