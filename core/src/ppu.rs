use crate::memory::MemoryAccess;
use crate::utils::{read_bit, set_bit};

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
pub const FRAME_CYCLES: u32 = SCANLINE_CYCLES * LINES_PER_FRAME;

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
const WY_ADDR: u16   = 0xFF4A;
const WX_ADDR: u16   = 0xFF4B;

pub const PPU_REGISTER_ADDRS: [u16; 11] = [
    LCDC_ADDR,
    STAT_ADDR,
    SCY_ADDR,
    SCX_ADDR,
    LY_ADDR,
    LYC_ADDR,
    BGP_ADDR,
    OBP0_ADDR,
    OBP1_ADDR,
    WY_ADDR,
    WX_ADDR,
];

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LcdMode {
    HBlank,
    VBlank,
    OAMSearch,
    PixelTransfer,
}

#[derive(Copy, Clone)]
pub enum PixelColor {
    BackgroundPixel(u8),
    SpritePixel(u8),
}

impl PixelColor {
    pub fn raw(&self) -> u8 {
        match self {
            PixelColor::BackgroundPixel(p) => *p,
            PixelColor::SpritePixel(p) => *p,
        }
    }
}

pub struct Ppu {
    screen_buffer: DoubleBuffer,
    cycles: u32,
    scanline: u32,
    visible_sprites: Vec<Sprite>,
    pub frame_complete: bool,
    regs: PpuRegisters,
    pub oam: [u8; 160],
    pub vram: [u8; 8192],
}

#[derive(Default)]
struct PpuRegisters {
    lcdc: u8,
    stat: u8,
    scy: u8,
    scx: u8,
    ly: u8,
    lyc: u8,
    bgp: u8,
    obp0: u8,
    obp1: u8,
    wy: u8,
    wx: u8,
}

impl MemoryAccess for Ppu {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            LCDC_ADDR => self.regs.lcdc,
            STAT_ADDR => self.regs.stat | 0b1000_0000,
            SCY_ADDR => self.regs.scy,
            SCX_ADDR => self.regs.scx,
            LY_ADDR => self.regs.ly,
            LYC_ADDR => self.regs.lyc,
            BGP_ADDR => self.regs.bgp,
            OBP0_ADDR => self.regs.obp0,
            OBP1_ADDR => self.regs.obp1,
            WY_ADDR => self.regs.wy,
            WX_ADDR => self.regs.wx,
            0xFE00..=0xFE9F => {
                if self.oam_blocked() {
                    0xFF
                } else {
                    self.oam[(addr as usize) - 0xFE00]
                }
            },
            0x8000..=0x9FFF => {
                if self.vram_blocked() {
                    0xFF
                } else {
                    self.vram[(addr as usize) - 0x8000]
                }
            },
            _ => panic!("Invalid ppu addr: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            LCDC_ADDR => {
                let prev_lcd_on = self.lcd_enabled();

                self.regs.lcdc = byte;

                let curr_lcd_on = self.lcd_enabled();

                if !prev_lcd_on && curr_lcd_on {
                    self.switch_on_lcd();
                }

                if prev_lcd_on && !curr_lcd_on {
                    self.switch_off_lcd();
                }
            },
            STAT_ADDR => self.regs.stat = byte,
            SCY_ADDR => self.regs.scy = byte,
            SCX_ADDR => self.regs.scx = byte,
            LY_ADDR => self.regs.ly = byte,
            LYC_ADDR => self.regs.lyc = byte,
            BGP_ADDR => self.regs.bgp = byte,
            OBP0_ADDR => self.regs.obp0 = byte,
            OBP1_ADDR => self.regs.obp1 = byte,
            WY_ADDR => self.regs.wy = byte,
            WX_ADDR => self.regs.wx = byte,
            0xFE00..=0xFE9F => {
                if !self.oam_blocked() {
                    self.oam[(addr as usize) - 0xFE00] = byte;
                }
            },
            0x8000..=0x9FFF => {
                if !self.vram_blocked() {
                    self.vram[(addr as usize) - 0x8000] = byte;
                }
            },
            _ => panic!("Invalid ppu addr: {:04x}", addr),
        }
    }
}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {
            screen_buffer: DoubleBuffer::new(),
            cycles: 0,
            scanline: 0,
            visible_sprites: Vec::new(),
            frame_complete: false,
            regs: PpuRegisters::default(),
            oam: [0; 160],
            vram: [0; 8192],
        }
    }

    pub fn lcd_enabled(&self) -> bool {
        read_bit(self.regs.lcdc, 7)
    }

    fn switch_on_lcd(&mut self) {
        self.cycles = OAM_SEARCH_CYCLES + PIXEL_TRANSFER_CYCLES;
    }

    fn switch_off_lcd(&mut self) {
        self.set_lcd_mode(LcdMode::HBlank);
        self.scanline = 0;
        self.regs.ly = 0;
    }

    fn vram_blocked(&self) -> bool {
        self.lcd_enabled() && self.get_lcd_mode() == LcdMode::PixelTransfer
    }

    fn oam_blocked(&self) -> bool {
        let lcd_mode = self.get_lcd_mode();
        self.lcd_enabled() && (lcd_mode == LcdMode::PixelTransfer || lcd_mode == LcdMode::OAMSearch)
    }

    pub fn vram_read_range(&self, start_addr: usize, end_addr: usize) -> &[u8] {
        &self.vram[(start_addr - 0x8000)..(end_addr - 0x8000)]
    }

    pub fn clock(&mut self, interrupt_flags: &mut u8) {
        if !self.lcd_enabled() {
            return;
        }

        let old_mode = Self::compute_lcd_mode(self.cycles, self.scanline);

        // Pixel transfer lasts 43 machine cycles, but we will be done in 40
        if old_mode == LcdMode::PixelTransfer && (self.cycles - OAM_SEARCH_CYCLES) < 40 {
            self.update_screen();
        }

        self.cycles = (self.cycles + 1) % SCANLINE_CYCLES;
        if self.cycles == 0 {
            self.scanline = (self.scanline + 1) % LINES_PER_FRAME;

            self.write(LY_ADDR, self.scanline as u8);

            let line_is_match = self.read(LY_ADDR) == self.read(LYC_ADDR);
            let status = self.read(STAT_ADDR);
            self.write(STAT_ADDR, set_bit(status, 2, line_is_match));

            // Request interrupt if scanline matches the requested one
            if line_is_match && read_bit(status, 6) {
                Self::set_status_interrupt(interrupt_flags);
            }
        }

        let mode = Self::compute_lcd_mode(self.cycles, self.scanline);

        if mode != old_mode {
            self.set_lcd_mode(mode);

            match mode {
                LcdMode::VBlank => {
                    Self::set_vblank_interrupt(interrupt_flags);
                    self.frame_complete = true;
                    self.screen_buffer.flip();
                },
                LcdMode::PixelTransfer => {
                    self.visible_sprites = self.compute_sprites_for_line(self.scanline);
                },
                _ => (),
            }

            if self.status_interrupt_enabled(mode) {
                Self::set_status_interrupt(interrupt_flags);
            }
        }
    }

    pub fn get_screen_buffer(&self) -> &[PixelColor] {
        self.screen_buffer.get_secondary()
    }

    pub fn get_lcd_mode(&self) -> LcdMode {
        match self.read(STAT_ADDR) & 0b11 {
            0 => LcdMode::HBlank,
            1 => LcdMode::VBlank,
            2 => LcdMode::OAMSearch,
            3 => LcdMode::PixelTransfer,
            _ => panic!("Not a valid LCD mode!"),
        }
    }

    fn update_screen(&mut self) {
        let scroll_x = self.read(SCX_ADDR);
        let scroll_y = self.read(SCY_ADDR);

        let window_x = self.read(WX_ADDR).wrapping_sub(7);
        let window_y = self.read(WY_ADDR);

        let background_enabled = read_bit(self.read(LCDC_ADDR), 0);
        let window_enabled = read_bit(self.read(LCDC_ADDR), 5);
        let sprites_enabled = read_bit(self.read(LCDC_ADDR), 1);

        // We are drawing 4 pixels per cycle
        let start_x = (self.cycles - OAM_SEARCH_CYCLES) * 4;

        for x in start_x..(start_x + 4) {
            let mut background_pixel = 0;

            if background_enabled {
                background_pixel = self.get_background_pixel(
                    (x as u8).wrapping_add(scroll_x),
                    (self.scanline as u8).wrapping_add(scroll_y),
                    self.get_background_map_memory(),
                );
            }

            if window_enabled {
                let (x, y) = (x as u8, self.scanline as u8);
                let should_draw = (window_x..LCD_WIDTH as u8).contains(&x)
                    && (window_y..LCD_WIDTH as u8).contains(&y);

                if should_draw {
                    background_pixel = self.get_background_pixel(
                        x - window_x, y - window_y, self.get_window_map_memory(),
                    );
                }
            }

            let mut pixel = PixelColor::BackgroundPixel(background_pixel);

            if sprites_enabled {
                // Finds first non-transparent sprite pixel, if any
                self.get_sprites_at_x(x as u8).iter()
                    .filter(|sprite| !(sprite.priority && (1..=3).contains(&background_pixel)))
                    .find_map(|sprite| self.get_sprite_pixel(sprite, x as u8, self.scanline as u8))
                    .map(|sprite_pixel| pixel = PixelColor::SpritePixel(sprite_pixel));
            }

            let index = self.scanline * LCD_WIDTH + x;
            self.screen_buffer.get_primary_mut()[index as usize] = pixel;
        }
    }

    fn get_sprites_at_x(&self, x: u8) -> Vec<&Sprite> {
        self.visible_sprites.iter().filter(|sprite| {
            let x_start = sprite.x as i32 - 8;
            (x_start..x_start+(NUM_PIXELS_IN_LINE as i32)).contains(&(x as i32))
        }).collect()
    }

    fn get_sprite_pixel(&self, sprite: &Sprite, x: u8, y: u8) -> Option<u8> {
        let tile = self.get_sprite_tile(sprite.tile_no);
        let (x_start, y_start) = (sprite.x as i32 - 8, sprite.y as i32 - 16);
        let (mut line_x, mut line_y) = ((x as i32 - x_start) as u8, (y as i32 - y_start) as u8);

        if sprite.x_flip {
            line_x = 7 - line_x;
        }
        if sprite.y_flip {
            line_y = 7 - line_y;
        }

        let pixel_data = get_tile_pixel_data(tile, line_x, line_y);

        if pixel_data == 0x00 {
            // 0 maps to transparency, so background color is used
            return None;
        }

        let palette = match sprite.palette_no {
            0 => self.read(OBP0_ADDR),
            1 => self.read(OBP1_ADDR),
            _ => panic!("Invalid palette_no: {}", sprite.palette_no),
        };

        Some(pixel_map(pixel_data, palette))
    }

    fn compute_sprites_for_line(&self, line_no: u32) -> Vec<Sprite> {
        let sprites = self.oam.chunks(4).map(|bytes| Sprite::from(bytes));

        let mut sprites_on_line_enumerated: Vec<_> = sprites.filter(|sprite| {
            let y_start = sprite.y as i32 - 16;
            // TODO: Account for varying sprite height
            sprite.x != 0 && (y_start..y_start+8).contains(&(line_no as i32))
        }).enumerate().collect();

        sprites_on_line_enumerated.sort_by_key(|(i, sprite)| (sprite.x, *i));

        let num_sprites = std::cmp::min(SPRITES_PER_LINE, sprites_on_line_enumerated.len());
        sprites_on_line_enumerated.drain(0..num_sprites).map(|(_, s)| s).collect()
    }

    fn get_background_pixel(&self, x: u8, y: u8, map: &[u8]) -> u8 {
        let (tile_x, tile_y) = (x as usize / NUM_PIXELS_IN_LINE, y as usize / NUM_PIXELS_IN_LINE);
        let tileset_index = map[tile_y * 32 + tile_x];
        let (line_x, line_y) = (x % NUM_PIXELS_IN_LINE as u8, y % NUM_PIXELS_IN_LINE as u8);

        let tile = self.get_tile(tileset_index);
        let background_palette = self.read(BGP_ADDR);
        Self::get_tile_pixel(tile, line_x, line_y, background_palette)
    }

    fn get_tile_pixel(tile: &[u8], line_x: u8, line_y: u8, palette: u8) -> u8 {
        let pixel_data = get_tile_pixel_data(tile, line_x, line_y);
        pixel_map(pixel_data, palette)
    }

    fn get_sprite_tile(&self, tile_index: u8) -> &[u8] {
        let start_index = 0x8000 + (tile_index as usize * TILE_NUM_BYTES);
        self.vram_read_range(start_index, start_index+TILE_NUM_BYTES)
    }

    fn get_tile(&self, tile_index: u8) -> &[u8] {
        let start_index = if read_bit(self.read(LCDC_ADDR), 4) {
            0x8000 + (tile_index as usize * TILE_NUM_BYTES)
        } else {
            ((0x9000 as i32) + ((tile_index as i8) as i32 * TILE_NUM_BYTES as i32)) as usize
        };

        self.vram_read_range(start_index, start_index+TILE_NUM_BYTES)
    }

    fn get_background_map_memory(&self) -> &[u8] {
        if read_bit(self.read(LCDC_ADDR), 3) {
            self.vram_read_range(0x9C00, 0xA000)
        } else {
            self.vram_read_range(0x9800, 0x9C00)
        }
    }

    fn get_window_map_memory(&self) -> &[u8] {
        if read_bit(self.read(LCDC_ADDR), 6) {
            self.vram_read_range(0x9C00, 0xA000)
        } else {
            self.vram_read_range(0x9800, 0x9C00)
        }
    }

    fn compute_lcd_mode(cycles: u32, scanline: u32) -> LcdMode {
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

    fn status_interrupt_enabled(&self, mode: LcdMode) -> bool {
        let status = self.read(STAT_ADDR);

        match mode {
            LcdMode::HBlank => read_bit(status, 3),
            LcdMode::VBlank => read_bit(status, 4),
            LcdMode::OAMSearch => read_bit(status, 5),
            LcdMode::PixelTransfer => false,
        }
    }

    fn set_status_interrupt(interrupt_flags: &mut u8) {
        *interrupt_flags |= 2;
    }

    fn set_vblank_interrupt(interrupt_flags: &mut u8) {
        *interrupt_flags |= 1;
    }

    fn set_lcd_mode(&mut self, mode: LcdMode) {
        let mode_bits = match mode {
            LcdMode::HBlank => 0,
            LcdMode::VBlank => 1,
            LcdMode::OAMSearch => 2,
            LcdMode::PixelTransfer => 3,
        };

        let mut status = self.read(STAT_ADDR);
        status = set_bit(status, 0, mode_bits & 0x1 > 0);
        status = set_bit(status, 1, mode_bits & 0x2 > 0);
        self.write(STAT_ADDR, status);
    }

    pub fn get_background_map(&self) -> Vec<u8> {
        let background_map_memory = self.get_background_map_memory();
        self.get_map_data(background_map_memory)
    }

    pub fn get_window_map(&self) -> Vec<u8> {
        let window_map_memory = self.get_window_map_memory();
        self.get_map_data(window_map_memory)
    }

    pub fn get_map_data(&self, map: &[u8]) -> Vec<u8> {
        (0..MAP_WIDTH * MAP_HEIGHT).map(|i| {
            let (x, y) = (i % MAP_WIDTH, i / MAP_WIDTH);
            // Since we are using u8, x and y should automatically wrap around 256
            self.get_background_pixel(x as u8, y as u8, map)
        }).collect()
    }

    pub fn get_tileset(&self) -> Vec<u8> {
        let num_tiles = 16 * 24;
        let num_pixels_in_tile = NUM_LINES_IN_TILE * NUM_PIXELS_IN_LINE;
        let mut pixels = vec![0; num_tiles * num_pixels_in_tile];

        let palette = self.read(BGP_ADDR);
        for (tile_index, tile) in self.vram_read_range(0x8000, 0x9800).chunks(TILE_NUM_BYTES).enumerate() {
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

    pub fn get_sprites(&self) -> Vec<u8> {
        let num_tiles = 10 * 4;
        let num_pixels_in_tile = NUM_LINES_IN_TILE * NUM_PIXELS_IN_LINE;
        let mut pixels = vec![0; num_tiles * num_pixels_in_tile];

        // TODO: Use sprite palette!
        let palette = self.read(BGP_ADDR);

        for (sprite_index, sprite) in self.oam.chunks(4).enumerate() {
            let tileset_index = sprite[2];
            let tile = self.get_sprite_tile(tileset_index);

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

struct DoubleBuffer {
    buffer: [PixelColor; (LCD_WIDTH * LCD_HEIGHT * 2) as usize],
    index: usize,
}

impl DoubleBuffer {
    fn new() -> DoubleBuffer {
        DoubleBuffer {
            buffer: [PixelColor::BackgroundPixel(0); (LCD_WIDTH * LCD_HEIGHT * 2) as usize],
            index: 0,
        }
    }

    fn flip(&mut self) {
        self.index = 1 - self.index;
    }

    fn get_primary_mut(&mut self) -> &mut [PixelColor] {
        self.buffer_at_index_mut(self.index)
    }

    fn get_secondary(&self) -> &[PixelColor] {
        self.buffer_at_index(1 - self.index)
    }

    fn buffer_at_index(&self, index: usize) -> &[PixelColor] {
        let size = (LCD_WIDTH * LCD_HEIGHT) as usize;
        let offset = index * size;
        &self.buffer[offset..offset+size]
    }

    fn buffer_at_index_mut(&mut self, index: usize) -> &mut [PixelColor] {
        let size = (LCD_WIDTH * LCD_HEIGHT) as usize;
        let offset = index * size;
        &mut self.buffer[offset..offset+size]
    }
}

#[derive(Debug)]
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
