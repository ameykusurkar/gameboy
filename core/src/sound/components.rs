pub struct VolumeEnvelope {
    pub initial: u32,
    pub inc_mode: bool,
    pub timer: Timer,
    current: i32,
}

impl VolumeEnvelope {
    pub fn new() -> Self {
        Self {
            initial: 0,
            inc_mode: true,
            timer: Timer::new(0),
            current: 0,
        }
    }

    pub fn clock(&mut self) {
        if self.timer.clock() && self.timer.period > 0 {
            if self.inc_mode {
                self.current = (self.current + 1).min(0xF);
            } else {
                self.current = (self.current - 1).max(0);
            }
        }
    }

    pub fn reset(&mut self) {
        self.current = self.initial as i32;
        self.timer.reload();
    }

    pub fn increment(&mut self) {
        self.current = (self.current + 1) % 16;
    }

    pub fn get_current(&self) -> i32 {
        self.current
    }
}

pub struct Timer {
    pub period: u32,
    countdown: u32,
}

impl Timer {
    pub fn new(period: u32) -> Self {
        Timer {
            period,
            countdown: period,
        }
    }

    pub fn reload(&mut self) {
        self.countdown = self.period;
    }

    pub fn clock(&mut self) -> bool {
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
