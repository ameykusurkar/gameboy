use crate::utils::read_bit;

pub struct SquareWaveChannel {
    frame_timer: Timer,
    frame_counter: u32,

    pub waveform: u8,
    pub enabled: bool,
    frequency_timer: Timer,
    waveform_bit: u8,

    pub length_counter: u32,
    pub length_counter_select: bool,

    pub volume_envelope: VolumeEnvelope,

    pub sweep_timer: Timer,
    pub sweep_shift: u32,
    pub frequency_subtract_mode: bool,
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

    pub fn get_frequency(&self) -> u32 {
        2048 - self.frequency_timer.period
    }

    pub fn set_frequency(&mut self, frequency: u32) {
        self.frequency_timer.period = 2048 - frequency;
    }

    pub fn increment_volume(&mut self) {
        self.volume_envelope.increment();
    }

    pub fn clock(&mut self) {
        if self.frame_timer.clock() {
            self.frame_counter = (self.frame_counter + 1) % 8;

            if self.frame_counter % 2 == 0 && self.enabled && self.length_counter > 0 {
                self.length_counter -= 1;
            }

            if self.length_counter == 0 {
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
        if !self.enabled && self.length_counter_select {
            return 0.0;
        }

        let amplitude = self.volume_envelope.current as f32 / 16.0;

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

    pub fn reset(&mut self) {
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

pub struct VolumeEnvelope {
    pub initial: u32,
    pub inc_mode: bool,
    pub timer: Timer,
    current: i32,
}

impl VolumeEnvelope {
    fn new() -> Self {
        Self {
            initial: 0,
            inc_mode: true,
            timer: Timer::new(0),
            current: 0,
        }
    }

    fn clock(&mut self) {
        if self.timer.clock() && self.timer.period > 0 {
            if self.inc_mode {
                self.current = (self.current + 1).min(0xF);
            } else {
                self.current = (self.current - 1).max(0);
            }
        }
    }

    fn reset(&mut self) {
        self.current = self.initial as i32;
        self.timer.reload();
    }

    fn increment(&mut self) {
        self.current = (self.current + 1) % 16;
    }
}

pub struct Timer {
    pub period: u32,
    countdown: u32,
}

impl Timer {
    fn new(period: u32) -> Self {
        Timer {
            period,
            countdown: period,
        }
    }

    fn reload(&mut self) {
        self.countdown = self.period;
    }

    fn clock(&mut self) -> bool {
        if self.countdown > 0 {
            self.countdown -= 1;

            if self.countdown == 0 {
                self.reload();
                true
            } else {
                false
            }
        } else {
            false
        }
    }
}
