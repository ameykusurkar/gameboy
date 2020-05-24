use crate::memory::MemoryAccess;
use crate::utils::read_bit;

pub struct SoundController {
    frequency: u32,
    sequencer: Sequencer,
    master_sound_switch: bool,
}

impl SoundController {
    pub fn new() -> Self {
        SoundController {
            frequency: 0,
            sequencer: Sequencer::new(),
            master_sound_switch: false,
        }
    }

    fn set_frequency_low_bits(&mut self, byte: u8) {
        self.frequency = (self.frequency & 0b111_0000_0000) | byte as u32;
        self.sequencer.period = 2048 - self.frequency;
    }

    fn set_frequency_high_bits(&mut self, byte: u8) {
        self.frequency = (self.frequency & 0b000_1111_1111) | (byte as u32) << 8;
        self.sequencer.period = 2048 - self.frequency;
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
        self.master_sound_switch
    }
}

impl MemoryAccess for SoundController {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF10 => {
                0xFF
            },
            0xFF11 => {
                0xFF
            },
            0xFF12 => {
                0xFF
            },
            0xFF13 => {
                0xFF
            },
            0xFF14 => {
                0xFF
            },
            0xFF26 => {
                0xFF
                // let data = set_bit(0b1111_1111, 7, self.master_sound_switch);
                // set_bit(data, 7, self.sound1_switch)
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0xFF10 => {
                // let sweep_time_data = (byte & 0b0111_0000) >> 4;
                // self.sweep_time = sweep_time_data as f32 / 128.0;

                // self.frequency_subtract = read_bit(byte, 3);
                // self.sweep_shift = byte & 0b0000_0111;
            },
            0xFF11 => {
                let sound_length_data = byte & 0b0011_1111;
                self.sequencer.length_counter = 64 - sound_length_data as u32;
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
                    println!("RESET");
                    self.sequencer.reset();
                }

                self.sequencer.length_counter_select = read_bit(byte, 6);
            },
            0xFF26 => {
                self.master_sound_switch = read_bit(byte, 7);
            },
            _ => unreachable!("Invalid sound controller address: {:04x}", addr),
        }
    }
}

struct Sequencer {
    counter: u32,
    frame_counter: u32,

    period: u32,
    timer: u32,
    waveform: u8,
    enabled: bool,

    length_counter: u32,
    length_counter_select: bool,

    initial_volume: u32,
    current_volume: i32,
    volume_up: bool,
    volume_period: u32,
    volume_timer: u32,
    auto_volume: bool,
}

impl Sequencer {
    fn new() -> Self {
        Sequencer {
            counter: 0,
            frame_counter: 0,

            period: 0,
            timer: 0,
            waveform: 0b0000_0011,
            enabled: false,

            length_counter: 0,
            length_counter_select: false,

            initial_volume: 0,
            current_volume: 0,
            volume_up: true,
            volume_period: 0,
            volume_timer: 0,
            auto_volume: true,
        }
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

            if self.volume_timer == 0 {
                self.volume_timer = self.volume_period;

                if self.volume_period > 0  && self.auto_volume {
                    let old = self.current_volume;

                    if self.volume_up {
                        self.current_volume = (self.current_volume + 1).min(0xF);
                    } else {
                        self.current_volume = (self.current_volume - 1).max(0);
                    }

                    if old == self.current_volume {
                        self.auto_volume = false;
                    }
                }
            }
            if self.frame_counter == 7 && self.volume_timer > 0 {
                self.volume_timer -= 1;
            }

        }

        self.timer -= 1;

        if self.timer == 0 {
            self.timer = self.period;
            self.waveform = self.waveform.rotate_right(1);
        }

    }

    fn sample(&self) -> f32 {
        if !self.enabled && self.length_counter_select {
            return 0.0;
        }

        let current_bit = self.waveform &0b1;

        let amplitude = 0.3 * self.current_volume as f32 / 16.0;

        if current_bit == 1 {
            amplitude
        } else {
            -amplitude
        }
    }

    fn reset(&mut self) {
        self.timer = self.period;
        self.enabled = true;

        if self.length_counter == 0 {
            self.length_counter = 64;
        }

        self.current_volume = self.initial_volume as i32;
        self.volume_timer = self.volume_period;
        self.auto_volume = true;
    }
}
