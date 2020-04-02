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
    pub screen: [u8; (LCD_WIDTH * LCD_HEIGHT) as usize],
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
        let old_mode = Self::get_lcd_mode(self.cycles, self.scanline);

        if old_mode == LcdMode::PixelTransfer && (self.cycles - OAM_SEARCH_CYCLES) < 40 {
            // We are drawing 4 pixels per cycle
            let start_x = (self.cycles - OAM_SEARCH_CYCLES) * 4;

            let scroll_x = memory.ppu_read(SCX_ADDR);
            let scroll_y = memory.ppu_read(SCY_ADDR);

            let map = Self::get_background_map_memory(memory);

            for x in start_x..(start_x + 4) {
                let pixel = Self::get_pixel(
                    // Since we are using u8, x and y should automatically wrap around 256
                    memory, (x as u8) + scroll_x, (self.scanline as u8) + scroll_y, map,
                );
                let index = self.scanline * LCD_WIDTH + x;
                self.screen[index as usize] = pixel;
            }
        }

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

    fn get_pixel(memory: &Memory, x: u8, y: u8, map: &[u8]) -> u8 {
        let (tile_x, tile_y) = (x as usize / NUM_PIXELS_IN_LINE, y as usize / NUM_PIXELS_IN_LINE);
        let tileset_index = map[tile_y * 32 + tile_x];
        let (line_x, line_y) = (x % NUM_PIXELS_IN_LINE as u8, y % NUM_PIXELS_IN_LINE as u8);

        let tile = Self::get_tile(memory, tileset_index);
        Self::get_tile_pixel(memory, tile, line_x, line_y)
    }

    fn get_tile_pixel(memory: &Memory, tile: &[u8], line_x: u8, line_y: u8) -> u8 {
        let upper_bit = read_bit(tile[(line_y * 2) as usize], 7 - line_x) as u8;
        let lower_bit = read_bit(tile[(line_y * 2 + 1) as usize], 7 - line_x) as u8;

        let background_palette = memory.ppu_read(BGP_ADDR);
        pixel_map(upper_bit << 1 | lower_bit, background_palette)
    }

    fn get_tile(memory: &Memory, tile_index: u8) -> &[u8] {
        let start_index = if read_bit(memory.ppu_read(LCDC_ADDR), 4) {
            0x8000 + (tile_index as usize * TILE_NUM_BYTES)
        } else {
            ((0x9000 as i32) + ((tile_index as i8) as i32 * TILE_NUM_BYTES as i32)) as usize
        };

        memory.ppu_read_range(start_index..start_index+TILE_NUM_BYTES)
    }

    fn get_background_map_memory(memory: &Memory) -> &[u8] {
        if read_bit(memory.ppu_read(LCDC_ADDR), 3) {
            memory.ppu_read_range(0x9C00..0xA000)
        } else {
            memory.ppu_read_range(0x9800..0x9C00)
        }
    }

    fn get_window_map_memory(memory: &Memory) -> &[u8] {
        if read_bit(memory.ppu_read(LCDC_ADDR), 6) {
            memory.ppu_read_range(0x9C00..0xA000)
        } else {
            memory.ppu_read_range(0x9800..0x9C00)
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

    pub fn get_background_map(memory: &Memory) -> Vec<u8> {
        let background_map_memory = Self::get_background_map_memory(memory);
        Self::get_map_data(memory, background_map_memory)
    }

    pub fn get_window_map(memory: &Memory) -> Vec<u8> {
        let window_map_memory = Self::get_window_map_memory(memory);
        Self::get_map_data(memory, window_map_memory)
    }

    pub fn get_map_data(memory: &Memory, map: &[u8]) -> Vec<u8> {
        (0..MAP_WIDTH * MAP_HEIGHT).map(|i| {
            let (x, y) = (i % MAP_WIDTH, i / MAP_WIDTH);
            // Since we are using u8, x and y should automatically wrap around 256
            Self::get_pixel(memory, x as u8, y as u8, map)
        }).collect()
    }

    pub fn get_tileset(memory: &Memory) -> Vec<u8> {
        let num_tiles = 16 * 24;
        let num_pixels_in_tile = NUM_LINES_IN_TILE * NUM_PIXELS_IN_LINE;
        let mut pixels = vec![0; num_tiles * num_pixels_in_tile];

        for (tile_index, tile) in memory.ppu_read_range(0x8000..0x9800).chunks(TILE_NUM_BYTES).enumerate() {
            for p in 0..num_pixels_in_tile {
                let (line_x, line_y) = (p % NUM_PIXELS_IN_LINE, p / NUM_PIXELS_IN_LINE);
                let x = (tile_index % 16) * NUM_PIXELS_IN_LINE + line_x;
                let y = (tile_index / 16) * NUM_PIXELS_IN_LINE + line_y;
                pixels[y * NUM_PIXELS_IN_LINE * 16 + x] =
                    Self::get_tile_pixel(memory, tile, line_x as u8, line_y as u8);
            }
        }

        pixels
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

fn set_bit(byte: u8, n: u8, x: bool) -> u8 {
    (byte & !(1 << n)) | ((x as u8) << n)
}
