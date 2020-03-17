use minifb;

pub struct Window {
    window: minifb::Window,
    width: usize,
    height: usize,
}

impl Window {
    pub fn new(width: usize, height: usize) -> Window {
        let mut options = minifb::WindowOptions::default();
        options.scale = minifb::Scale::X2;

        let window = minifb::Window::new(
            "",
            width,
            height,
            options,
        ).unwrap();

        Window { window, width, height }
    }

    // pub fn set_title(&mut self, title: &str) {
    //     self.window.set_title(title);
    // }

    pub fn update(&mut self, pixels: &[u8]) {
        let buffer: Vec<u32> = pixels.iter().map(|p| Self::color(*p)).collect();

        self.window
            .update_with_buffer(&buffer, self.width, self.height)
            .unwrap();
    }

    pub fn is_open(&self) -> bool {
        self.window.is_open()
    }

    fn color(pixel: u8) -> u32 {
        match pixel {
            0x0 => 0x00FFFFFF,
            0x1 => 0x00808080,
            0x2 => 0x00404040,
            0x3 => 0x00000000,
            _ => panic!("Unknown pixel value {:02x}", pixel),
        }
    }
}
