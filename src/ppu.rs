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

const SPRITES_PER_LINE: usize = 10;

// Video I/O Register addresses
const LCDC_ADDR: u16 = 0xFF40;
const STAT_ADDR: u16 = 0xFF41;
const SCY_ADDR: u16  = 0xFF42;
const SCX_ADDR: u16  = 0xFF43;
const LY_ADDR: u16   = 0xFF44;
const LYC_ADDR: u16  = 0xFF45;
const BGP_ADDR: u16  = 0xFF47;
const OBP0_ADDR: u16 = 0xFF48;
const OBP1_ADDR: u16 = 0xFF49;
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
    visible_sprites: Vec<Sprite>,
}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {
            screen: [0; (LCD_WIDTH * LCD_HEIGHT) as usize],
            cycles: 0,
            scanline: 0,
            visible_sprites: Vec::new(),
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
                // TODO: Only show background/sprites when enabled
                // TODO: Implement windows
                let mut pixel = Self::get_background_pixel(
                    // Since we are using u8, x and y should automatically wrap around 256
                    memory, (x as u8) + scroll_x, (self.scanline as u8) + scroll_y, map,
                );

                // TODO: Account for sprite priority
                self.get_sprite(x as u8)
                    .and_then(|sprite| Self::get_sprite_pixel(sprite, memory, x as u8, self.scanline as u8))
                    .map(|sprite_pixel| pixel = sprite_pixel);

                let index = self.scanline * LCD_WIDTH + x;
                self.screen[index as usize] = pixel;
            }
        }

        self.cycles = (self.cycles + 1) % SCANLINE_CYCLES;
        if self.cycles == 0 {
            self.scanline = (self.scanline + 1) % LINES_PER_FRAME;

            memory.ppu_write(LY_ADDR, self.scanline as u8);

            let line_is_match = memory.ppu_read(LY_ADDR) == memory.ppu_read(LYC_ADDR);
            let status = memory.ppu_read(STAT_ADDR);
            memory.ppu_write(STAT_ADDR, set_bit(status, 2, line_is_match));

            // Request interrupt if scanline matches the requested one
            if line_is_match && read_bit(status, 6) {
                Self::set_status_interrupt(memory);
            }
        }

        let mode = Self::get_lcd_mode(self.cycles, self.scanline);

        if mode != old_mode {
            Self::set_lcd_mode(memory, mode);

            match mode {
                LcdMode::VBlank => {
                    Self::set_vblank_interrupt(memory);
                },
                LcdMode::PixelTransfer => {
                    self.visible_sprites = Self::compute_sprites_for_line(memory, self.scanline);
                },
                _ => (),
            }

            if Self::status_interrupt_enabled(memory, mode) {
                Self::set_status_interrupt(memory);
            }
        }
    }

    fn get_sprite(&self, x: u8) -> Option<&Sprite> {
        self.visible_sprites.iter().find(|sprite| {
            let x_start = sprite.x as i32 - 8;
            (x_start..x_start+(NUM_PIXELS_IN_LINE as i32)).contains(&(x as i32))
        })
    }

    fn get_sprite_pixel(sprite: &Sprite, memory: &Memory, x: u8, y: u8) -> Option<u8> {
        let tile = Self::get_sprite_tile(memory, sprite.tile_no);
        let (x_start, y_start) = (sprite.x as i32 - 8, sprite.y as i32 - 16);
        // TODO: Account for tiles being flipped
        let (line_x, line_y) = ((x as i32 - x_start) as u8, (y as i32 - y_start) as u8);
        let pixel_data = get_tile_pixel_data(tile, line_x, line_y);

        if pixel_data == 0x00 {
            // 0 maps to transparency, so background color is used
            return None;
        }

        let palette = match sprite.palette_no {
            0 => memory.ppu_read(OBP0_ADDR),
            1 => memory.ppu_read(OBP1_ADDR),
            _ => panic!("Invalid palette_no: {}", sprite.palette_no),
        };

        Some(pixel_map(pixel_data, palette))
    }

    fn compute_sprites_for_line(memory: &Memory, line_no: u32) -> Vec<Sprite> {
        let sprites = memory.ppu_read_range(0xFE00..0xFEA0).chunks(4)
            .map(|bytes| Sprite::from(bytes));

        let mut sprites_on_line_enumerated: Vec<_> = sprites.filter(|sprite| {
            let y_start = sprite.y as i32 - 16;
            // TODO: Account for varying sprite height
            sprite.x != 0 && (y_start..y_start+8).contains(&(line_no as i32))
        }).enumerate().collect();

        sprites_on_line_enumerated.sort_by_key(|(i, sprite)| (sprite.x, *i));

        let num_sprites = std::cmp::min(SPRITES_PER_LINE, sprites_on_line_enumerated.len());
        sprites_on_line_enumerated.drain(0..num_sprites).map(|(_, s)| s).collect()
    }

    fn get_background_pixel(memory: &Memory, x: u8, y: u8, map: &[u8]) -> u8 {
        let (tile_x, tile_y) = (x as usize / NUM_PIXELS_IN_LINE, y as usize / NUM_PIXELS_IN_LINE);
        let tileset_index = map[tile_y * 32 + tile_x];
        let (line_x, line_y) = (x % NUM_PIXELS_IN_LINE as u8, y % NUM_PIXELS_IN_LINE as u8);

        let tile = Self::get_tile(memory, tileset_index);
        let background_palette = memory.ppu_read(BGP_ADDR);
        Self::get_tile_pixel(tile, line_x, line_y, background_palette)
    }

    fn get_tile_pixel(tile: &[u8], line_x: u8, line_y: u8, palette: u8) -> u8 {
        let pixel_data = get_tile_pixel_data(tile, line_x, line_y);
        pixel_map(pixel_data, palette)
    }

    fn get_sprite_tile(memory: &Memory, tile_index: u8) -> &[u8] {
        let start_index = 0x8000 + (tile_index as usize * TILE_NUM_BYTES);
        memory.ppu_read_range(start_index..start_index+TILE_NUM_BYTES)
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
            Self::get_background_pixel(memory, x as u8, y as u8, map)
        }).collect()
    }

    pub fn get_tileset(memory: &Memory) -> Vec<u8> {
        let num_tiles = 16 * 24;
        let num_pixels_in_tile = NUM_LINES_IN_TILE * NUM_PIXELS_IN_LINE;
        let mut pixels = vec![0; num_tiles * num_pixels_in_tile];

        let palette = memory.ppu_read(BGP_ADDR);
        for (tile_index, tile) in memory.ppu_read_range(0x8000..0x9800).chunks(TILE_NUM_BYTES).enumerate() {
            for p in 0..num_pixels_in_tile {
                let (line_x, line_y) = (p % NUM_PIXELS_IN_LINE, p / NUM_PIXELS_IN_LINE);
                let x = (tile_index % 16) * NUM_PIXELS_IN_LINE + line_x;
                let y = (tile_index / 16) * NUM_PIXELS_IN_LINE + line_y;
                pixels[y * NUM_PIXELS_IN_LINE * 16 + x] =
                    Self::get_tile_pixel(tile, line_x as u8, line_y as u8, palette);
            }
        }

        pixels
    }

    pub fn get_sprites(memory: &Memory) -> Vec<u8> {
        let sprite_data = memory.ppu_read_range(0xFE00..0xFEA0);
        let num_tiles = 10 * 4;
        let num_pixels_in_tile = NUM_LINES_IN_TILE * NUM_PIXELS_IN_LINE;
        let mut pixels = vec![0; num_tiles * num_pixels_in_tile];

        // TODO: Use sprite palette!
        let palette = memory.ppu_read(BGP_ADDR);

        for (sprite_index, sprite) in sprite_data.chunks(4).enumerate() {
            let tileset_index = sprite[2];
            let tile = Self::get_sprite_tile(memory, tileset_index);

            for p in 0..num_pixels_in_tile {
                let (line_x, line_y) = (p % NUM_PIXELS_IN_LINE, p / NUM_PIXELS_IN_LINE);
                let x = (sprite_index % 10) * NUM_PIXELS_IN_LINE + line_x;
                let y = (sprite_index / 10) * NUM_PIXELS_IN_LINE + line_y;
                pixels[y * NUM_PIXELS_IN_LINE * 10 + x] =
                    Self::get_tile_pixel(tile, line_x as u8, line_y as u8, palette);
            }
        }

        pixels
    }
}

struct Sprite {
    y: u8,
    x: u8,
    tile_no: u8,
    priority: bool,
    y_flip: bool,
    x_flip: bool,
    palette_no: u8,
}

impl std::convert::From<&[u8]> for Sprite {
    fn from(bytes: &[u8]) -> Self {
        Sprite {
            y: bytes[0],
            x: bytes[1],
            tile_no: bytes[2],
            priority: read_bit(bytes[3], 7),
            y_flip: read_bit(bytes[3], 6),
            x_flip: read_bit(bytes[3], 5),
            palette_no: read_bit(bytes[3], 4) as u8,
        }
    }
}

fn get_tile_pixel_data(tile: &[u8], line_x: u8, line_y: u8) -> u8 {
    let lower_bit = read_bit(tile[(line_y * 2) as usize], 7 - line_x) as u8;
    let upper_bit = read_bit(tile[(line_y * 2 + 1) as usize], 7 - line_x) as u8;
    upper_bit << 1 | lower_bit
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
