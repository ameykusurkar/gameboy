use crate::memory::Memory;

use crate::cpu::IF_ADDR;

const NUM_PIXELS_IN_LINE: usize = 8;
const NUM_LINES_IN_TILE: usize = 8;
const LINES_NUM_BYTES: usize = 2;
const TILE_NUM_BYTES: usize = LINES_NUM_BYTES * NUM_LINES_IN_TILE;
pub const MAP_WIDTH: usize = 32 * NUM_PIXELS_IN_LINE;
pub const MAP_HEIGHT: usize = 32 * NUM_LINES_IN_TILE;

pub const LCD_WIDTH: u32 = 160;
pub const LCD_HEIGHT: u32 = 144;

const OAM_SEARCH_CYCLES: u32 = 20;
const PIXEL_TRANSFER_CYCLES: u32 = 43;
const H_BLANK_CYCLES: u32 = 51;

const SCANLINE_CYCLES: u32 = OAM_SEARCH_CYCLES + PIXEL_TRANSFER_CYCLES + H_BLANK_CYCLES;
const V_BLANK_LINES: u32 = 10;
const LINES_PER_FRAME: u32 = LCD_HEIGHT + V_BLANK_LINES;
const FRAME_CYCLES: u32 = SCANLINE_CYCLES * LINES_PER_FRAME;

// Video I/O Register addresses
const LCDC_ADDR: u16 = 0xFF40;
const STAT_ADDR: u16 = 0xFF41;
const SCY_ADDR: u16  = 0xFF42;
const SCX_ADDR: u16  = 0xFF43;
const LY_ADDR: u16   = 0xFF44;
const LYC_ADDR: u16  = 0xFF45;
const BGP_ADDR: u16  = 0xFF47;
const OBP1_ADDR: u16 = 0xFF48;
const OBP2_ADDR: u16 = 0xFF49;
const WX_ADDR: u16   = 0xFF4A;
const WY_ADDR: u16   = 0xFF4B;

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LcdMode {
    HBlank,
    VBlank,
    OAMSearch,
    PixelTransfer,
}

pub struct Ppu {
    screen: [u8; (LCD_WIDTH * LCD_HEIGHT) as usize],
    cycles: u32,
    scanline: u32,
}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {
            screen: [0; (LCD_WIDTH * LCD_HEIGHT) as usize],
            cycles: 0,
            scanline: 0,
        }
    }

    pub fn clock(&mut self, memory: &mut Memory) {
        // TODO: Update screen buffer

        let old_mode = Self::get_lcd_mode(self.cycles, self.scanline);

        self.cycles = (self.cycles + 1) % SCANLINE_CYCLES;
        if self.cycles == 0 {
            self.scanline = (self.scanline + 1) % LINES_PER_FRAME;

            memory.ppu_write(LY_ADDR, self.scanline as u8);

            // Request interrupt if scanline matches the requested one
            if memory.ppu_read(LY_ADDR) == memory.ppu_read(LYC_ADDR) {
                let status = memory.ppu_read(STAT_ADDR);
                memory.ppu_write(STAT_ADDR, set_bit(status, 2, true));
                if read_bit(status, 6) {
                    Self::set_status_interrupt(memory);
                }
            }
        }

        let mode = Self::get_lcd_mode(self.cycles, self.scanline);

        if mode != old_mode {
            Self::set_lcd_mode(memory, mode);

            if mode == LcdMode::VBlank {
                Self::set_vblank_interrupt(memory);
            }

            if Self::status_interrupt_enabled(memory, mode) {
                Self::set_status_interrupt(memory);
            }
        }
    }

    fn get_lcd_mode(cycles: u32, scanline: u32) -> LcdMode {
        if scanline >= LCD_HEIGHT {
            LcdMode::VBlank
        } else if cycles < OAM_SEARCH_CYCLES {
            LcdMode::OAMSearch
        } else if cycles < OAM_SEARCH_CYCLES + PIXEL_TRANSFER_CYCLES {
            LcdMode::PixelTransfer
        } else {
            LcdMode::HBlank
        }
    }

    fn status_interrupt_enabled(memory: &Memory, mode: LcdMode) -> bool {
        let status = memory.ppu_read(STAT_ADDR);

        match mode {
            LcdMode::HBlank => read_bit(status, 3),
            LcdMode::VBlank => read_bit(status, 4),
            LcdMode::OAMSearch => read_bit(status, 5),
            LcdMode::PixelTransfer => false,
        }
    }

    fn set_status_interrupt(memory: &mut Memory) {
        let interrupt_flags = memory.ppu_read(IF_ADDR);
        memory.ppu_write(IF_ADDR, interrupt_flags | 2);
    }

    fn set_vblank_interrupt(memory: &mut Memory) {
        let interrupt_flags = memory.ppu_read(IF_ADDR);
        memory.ppu_write(IF_ADDR, interrupt_flags | 1);
    }

    fn set_lcd_mode(memory: &mut Memory, mode: LcdMode) {
        let mode_bits = match mode {
            LcdMode::HBlank => 0,
            LcdMode::VBlank => 1,
            LcdMode::OAMSearch => 2,
            LcdMode::PixelTransfer => 3,
        };

        let mut status = memory.ppu_read(STAT_ADDR);
        status = set_bit(status, 0, mode_bits & 0x1 > 0);
        status = set_bit(status, 1, mode_bits & 0x2 > 0);
        memory.ppu_write(STAT_ADDR, status);
    }

    pub fn get_background_map(memory: &[u8]) -> Vec<u8> {
        let background_map_memory = get_background_map_memory(memory);
        Self::get_map_data(memory, background_map_memory)
    }

    pub fn get_window_map(memory: &[u8]) -> Vec<u8> {
        let window_map_memory = get_window_map_memory(memory);
        Self::get_map_data(memory, window_map_memory)
    }

    pub fn get_map_data(memory: &[u8], map_memory: &[u8]) -> Vec<u8> {
        let background_palette = memory[BGP_ADDR as usize];

        let mut pixels = vec![0; MAP_WIDTH * MAP_HEIGHT];

        for (tile_no, tile_index) in map_memory.iter().enumerate() {
            let tile = get_tile(memory, *tile_index);
            for (line_no, line) in tile.chunks(LINES_NUM_BYTES).enumerate() {
                for i in 0..NUM_PIXELS_IN_LINE {
                    let upper_bit = read_bit(line[0], (NUM_PIXELS_IN_LINE-1-i) as u8) as u8;
                    let lower_bit = read_bit(line[1], (NUM_PIXELS_IN_LINE-1-i) as u8) as u8;
                    let pixel = pixel_map(upper_bit << 1 | lower_bit, background_palette);
                    let (x, y) = coords_for_pixel(tile_no, line_no, i as usize, MAP_WIDTH);
                    let pixel_index = y * MAP_WIDTH + x;
                    pixels[pixel_index] = pixel;
                }
            }
        }

        pixels
    }

    pub fn get_tilset(memory: &[u8]) -> Vec<u8> {
        let (width, height) = (16 * 8, 24 * 8);
        let background_palette = memory[BGP_ADDR as usize];

        let mut pixels = vec![0; width * height];

        for (tile_no, tile) in get_tileset_memory(memory).chunks(TILE_NUM_BYTES).enumerate() {
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
    let start_index = if read_bit(memory[LCDC_ADDR as usize], 4) {
        0x8000 + (tile_index as usize * TILE_NUM_BYTES)
    } else {
        ((0x9000 as i32) + ((tile_index as i8) as i32 * TILE_NUM_BYTES as i32)) as usize
    };

    &memory[start_index..start_index+TILE_NUM_BYTES]
}

fn get_background_map_memory(memory: &[u8]) -> &[u8] {
    if read_bit(memory[LCDC_ADDR as usize], 3) {
        &memory[0x9C00..0xA000]
    } else {
        &memory[0x9800..0x9C00]
    }
}

fn get_window_map_memory(memory: &[u8]) -> &[u8] {
    if read_bit(memory[LCDC_ADDR as usize], 6) {
        &memory[0x9C00..0xA000]
    } else {
        &memory[0x9800..0x9C00]
    }
}

fn get_tileset_memory(memory: &[u8]) -> &[u8] {
    &memory[0x8000..0x9800]
}

fn pixel_map(color_number: u8, palette: u8) -> u8 {
    let high_bit = read_bit(palette, color_number * 2 + 1) as u8;
    let low_bit = read_bit(palette, color_number * 2) as u8;
    high_bit << 1 | low_bit
}

fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}

fn set_bit(byte: u8, n: u8, x: bool) -> u8 {
    (byte & !(1 << n)) | ((x as u8) << n)
}
