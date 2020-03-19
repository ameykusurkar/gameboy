pub const NUM_PIXELS_IN_LINE: usize = 8;
const NUM_LINES_IN_TILE: usize = 8;
const LINES_NUM_BYTES: usize = 2;
const TILE_NUM_BYTES: usize = LINES_NUM_BYTES * NUM_LINES_IN_TILE;

pub fn get_pixel_buffer(memory: &[u8], width: usize, height: usize) -> Vec<u8> {
    let background_map = get_background_map(memory);
    let background_palette = memory[0xFF47];

    let mut pixels = vec![0; width * height];

    for (tile_no, tile_index) in background_map.iter().enumerate() {
        let tile = get_tile(memory, *tile_index);
        for (line_no, line) in tile.chunks(LINES_NUM_BYTES).enumerate() {
            for i in 0..NUM_PIXELS_IN_LINE {
                let upper_bit = read_bit(line[0], (NUM_PIXELS_IN_LINE-1-i) as u8) as u8;
                let lower_bit = read_bit(line[1], (NUM_PIXELS_IN_LINE-1-i) as u8) as u8;
                let pixel = pixel_map(upper_bit << 1 | lower_bit, background_palette);
                let (x, y) = coords_for_pixel(tile_no, line_no, i as usize, width);
                let pixel_index = y * width + x;
                pixels[pixel_index] = pixel;
            }
        }
    }

    pixels
}

fn coords_for_pixel(tile_no: usize,
                    line_no: usize,
                    pixel_no: usize,
                    width: usize) -> (usize, usize) {
    let tiles_per_row = width / NUM_PIXELS_IN_LINE;
    let x = (tile_no % (tiles_per_row)) * NUM_PIXELS_IN_LINE + pixel_no;
    let y = tile_no / (tiles_per_row) * NUM_LINES_IN_TILE + line_no;
    (x, y)
}

fn get_tile(memory: &[u8], tile_index: u8) -> &[u8] {
    let start_index = if read_bit(memory[0xFF40], 4) {
        0x8000 + (tile_index as usize * TILE_NUM_BYTES)
    } else {
        ((0x9000 as i32) + (tile_index as i32)) as usize
    };

    &memory[start_index..start_index+TILE_NUM_BYTES]
}

fn get_background_map(memory: &[u8]) -> &[u8] {
    if read_bit(memory[0xFF40], 3) {
        &memory[0x9C00..0x9FFF]
    } else {
        &memory[0x9800..0x9BFF]
    }
}

fn pixel_map(color_number: u8, palette: u8) -> u8 {
    let high_bit = read_bit(palette, color_number * 2 + 1) as u8;
    let low_bit = read_bit(palette, color_number * 2) as u8;
    high_bit << 1 | low_bit
}

fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}
