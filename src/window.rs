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

    pub fn update(&mut self, buffer: &[u8]) {
        let pixels = self.buffer_into_pixels(buffer);

        self.window
            .update_with_buffer(&pixels, self.width, self.height)
            .unwrap();
    }

    pub fn is_open(&self) -> bool {
        self.window.is_open()
    }

    fn buffer_into_pixels(&self, buffer: &[u8]) -> Vec<u32> {
        let mut pixels: Vec<u32> = vec![0; self.width * self.height];

        for (tile_no, tile) in buffer.chunks(16).enumerate() {
            for (line_no, line) in tile.chunks(2).enumerate() {
                for i in 0..8 {
                    let upper_bit = read_bit(line[0], 8-1-i) as u8;
                    let lower_bit = read_bit(line[1], 8-1-i) as u8;
                    let pixel_bits = upper_bit << 1 | lower_bit;
                    let pixel = Self::color(pixel_bits);
                    let pixel_x = (tile_no % (self.width / 8)) * 8 + i as usize;
                    let pixel_y = tile_no / (self.width / 8) * 8 + line_no;
                    let pixel_index = pixel_y * self.width + pixel_x;
                    pixels[pixel_index] = pixel;
                }
            }
        }

        pixels
    }

    fn color(bits: u8) -> u32 {
        match bits {
            0x0 => 0x00FFFFFF,
            0x1 => 0x00000000,
            0x2 => 0x00000000,
            0x3 => 0x00000000,
            _ => panic!("Unknown bits {:02x}", bits),
        }
    }
}

fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}
