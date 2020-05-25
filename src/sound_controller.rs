use crate::memory::MemoryAccess;
use crate::utils::{read_bit, set_bit};

pub struct SoundController {
    sequencer: Sequencer,
    master_sound_on: bool,
}

impl SoundController {
    pub fn new() -> Self {
        SoundController {
            sequencer: Sequencer::new(),
            master_sound_on: false,
        }
    }

    fn set_frequency_low_bits(&mut self, byte: u8) {
        self.sequencer.frequency = (self.sequencer.frequency & 0b111_0000_0000) | byte as u32;
    }

    fn set_frequency_high_bits(&mut self, byte: u8) {
        self.sequencer.frequency = (self.sequencer.frequency & 0b000_1111_1111) | (byte as u32) << 8;
    }

    pub fn clock(&mut self) {
        self.sequencer.clock();
    }

    pub fn get_current_sample(&self) -> f32 {
        if !self.is_sound_on() {
            return 0.0;
        }

        self.sequencer.sample()
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
                (self.sequencer.sweep_period as u8) << 4
                    | (self.sequencer.frequency_subtract_mode as u8) << 3
                    | self.sequencer.sweep_shift as u8
            },
            0xFF11 => {
                let duty_cycle = match self.sequencer.waveform {
                    0b0000_0001 => 0,
                    0b1000_0001 => 1,
                    0b1000_0111 => 2,
                    0b0111_1110 => 3,
                    _ => unreachable!("Invalid waveform: {:02x}", self.sequencer.waveform),
                };

                (duty_cycle << 6 | (64 - self.sequencer.length_counter)) as u8
            },
            0xFF12 => {
                (self.sequencer.initial_volume << 4) as u8
                    | (self.sequencer.volume_up as u8) << 3
                    | self.sequencer.volume_period as u8
            },
            0xFF13 => {
                0xFF
            },
            0xFF14 => {
                (self.sequencer.length_counter_select as u8) << 6
            },
            0xFF26 => {
                let data = set_bit(0b1111_1111, 7, self.master_sound_on);
                set_bit(data, 7, self.sequencer.enabled)
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF10 => {
                self.sequencer.sweep_period = ((byte & 0b0111_0000) >> 4) as u32;
                self.sequencer.frequency_subtract_mode = read_bit(byte, 3);
                self.sequencer.sweep_shift = (byte & 0b0000_0111) as u32;
            },
            0xFF11 => {
                let sound_length_data = byte & 0b0011_1111;
                self.sequencer.length_counter = 64 - sound_length_data as u32;

                self.sequencer.waveform = match (byte & 0b1100_0000) >> 6 {
                    0 => 0b0000_0001, // 12.5% Duty cycle
                    1 => 0b1000_0001, // 25%   Duty cycle
                    2 => 0b1000_0111, // 50%   Duty cycle
                    3 => 0b0111_1110, // 75%   Duty cycle
                    _ => unreachable!("Invalid duty cycle byte: {:02x}", byte),
                };
            },
            0xFF12 => {
                self.sequencer.initial_volume = ((byte & 0b1111_0000) >> 4) as u32;
                self.sequencer.volume_up = read_bit(byte, 3);
                self.sequencer.volume_period = (byte & 0b111) as u32;

                if byte == 0x08 {
                    self.sequencer.current_volume = (self.sequencer.current_volume + 1) % 16;
                }
            },
            0xFF13 => {
                self.set_frequency_low_bits(byte);
            },
            0xFF14 => {
                self.set_frequency_high_bits(byte & 0b0000_0111);

                let should_restart = read_bit(byte, 7);

                if should_restart {
                    self.sequencer.reset();
                }

                self.sequencer.length_counter_select = read_bit(byte, 6);
            },
            0xFF26 => {
                self.master_sound_on = read_bit(byte, 7);
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }
}

struct Sequencer {
    counter: u32,
    frame_counter: u32,

    timer: u32,
    waveform: u8,
    waveform_bit: u8,
    enabled: bool,

    length_counter: u32,
    length_counter_select: bool,

    initial_volume: u32,
    current_volume: i32,
    volume_up: bool,
    volume_period: u32,
    volume_timer: u32,

    frequency: u32,
    shadow_frequency: u32,
    sweep_period: u32,
    sweep_timer: u32,
    sweep_shift: u32,
    frequency_subtract_mode: bool,
    sweep_enabled: bool,
}

impl Sequencer {
    fn new() -> Self {
        Sequencer {
            counter: 0,
            frame_counter: 0,

            timer: 0,
            waveform: 0b0000_1111,
            waveform_bit: 0,
            enabled: false,

            length_counter: 0,
            length_counter_select: false,

            initial_volume: 0,
            current_volume: 0,
            volume_up: true,
            volume_period: 0,
            volume_timer: 0,

            frequency: 0,
            shadow_frequency: 0,
            sweep_period: 0,
            sweep_timer: 0,
            sweep_shift: 0,
            frequency_subtract_mode: false,
            sweep_enabled: true,
        }
    }

    fn period(&self) -> u32 {
        2048 - self.frequency
    }

    fn clock(&mut self) {
        self.counter += 1;

        if self.counter % 2048 == 0 {
            self.frame_counter = (self.frame_counter + 1) % 8;

            if self.frame_counter % 2 == 0 && self.enabled && self.length_counter > 0 {
                self.length_counter -= 1;
            }

            if self.length_counter == 0 {
                self.enabled = false;
            }

            if self.frame_counter == 7 && self.volume_timer > 0 {
                self.volume_timer -= 1;
            }

            if self.volume_timer == 0 {
                self.volume_timer = self.volume_period;

                if self.volume_period > 0 {
                    if self.volume_up {
                        self.current_volume = (self.current_volume + 1).min(0xF);
                    } else {
                        self.current_volume = (self.current_volume - 1).max(0);
                    }
                }
            }

            if (self.frame_counter == 2 || self.frame_counter == 6) && self.sweep_timer > 0 {
                self.sweep_timer -= 1;
            }

            if self.sweep_timer == 0 {
                self.sweep_timer = self.sweep_period;

                if self.sweep_enabled && self.sweep_period > 0 {
                    match self.new_frequency() {
                        None => self.enabled = false,
                        Some(new_freq) => {
                            if self.sweep_shift > 0 {
                                self.shadow_frequency = new_freq;
                                self.frequency = new_freq;

                                if self.new_frequency().is_none() {
                                    self.enabled = false;
                                }
                            }
                        }
                    }
                }
            }
        }

        self.timer -= 1;

        if self.timer == 0 {
            self.timer = self.period();
            self.waveform_bit = (self.waveform_bit + 1) % 8;
        }

    }

    fn sample(&self) -> f32 {
        if !self.enabled && self.length_counter_select {
            return 0.0;
        }

        let amplitude = 0.3 * self.current_volume as f32 / 16.0;

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

    fn reset(&mut self) {
        self.timer = self.period();
        self.enabled = true;

        if self.length_counter == 0 {
            self.length_counter = 64;
        }

        self.current_volume = self.initial_volume as i32;
        self.volume_timer = self.volume_period;

        self.shadow_frequency = self.frequency;
        self.sweep_timer = self.sweep_period;
        self.sweep_enabled = (self.sweep_period > 0) || (self.sweep_shift > 0);

        if self.sweep_shift > 0 && self.new_frequency().is_none() {
            self.enabled = false;
        }
    }
}
