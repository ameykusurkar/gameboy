pub fn get_pixel_buffer(memory: &[u8], width: usize, height: usize) -> Vec<u8> {
    let tileset = &memory[0x8000..0x9800];
    let mut pixels: Vec<u8> = vec![0; width * height];

    let palette = memory[0xFF47];
    for (tile_no, tile) in tileset.chunks(16).enumerate() {
        for (line_no, line) in tile.chunks(2).enumerate() {
            for i in 0..8 {
                let upper_bit = read_bit(line[0], 8-1-i) as u8;
                let lower_bit = read_bit(line[1], 8-1-i) as u8;
                let pixel = pixel_map(upper_bit << 1 | lower_bit, palette);
                let pixel_x = (tile_no % (width / 8)) * 8 + i as usize;
                let pixel_y = tile_no / (width / 8) * 8 + line_no;
                let pixel_index = pixel_y * width + pixel_x;
                pixels[pixel_index] = pixel;
            }
        }
    }

    pixels
}

fn pixel_map(color_number: u8, palette: u8) -> u8 {
    let high_bit = read_bit(palette, color_number * 2 + 1) as u8;
    let low_bit = read_bit(palette, color_number * 2) as u8;
    high_bit << 1 | low_bit
}

fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}
