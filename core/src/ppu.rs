use crate::memory::{IoRegisters, MemoryAccess};
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

const VBLANK_INTERRUPT: u8 = 0;
const STAT_INTERRUPT: u8 = 1;

// Video I/O Register addresses
const LCDC_ADDR: u16 = 0xFF40;
const STAT_ADDR: u16 = 0xFF41;
const SCY_ADDR: u16 = 0xFF42;
const SCX_ADDR: u16 = 0xFF43;
const LY_ADDR: u16 = 0xFF44;
const LYC_ADDR: u16 = 0xFF45;
const BGP_ADDR: u16 = 0xFF47;
const OBP0_ADDR: u16 = 0xFF48;
const OBP1_ADDR: u16 = 0xFF49;
const WY_ADDR: u16 = 0xFF4A;
const WX_ADDR: u16 = 0xFF4B;

pub const VBK_ADDR: u16 = 0xFF4F;

const BGPI_ADDR: u16 = 0xFF68;
const BGPD_ADDR: u16 = 0xFF69;
const OBPI_ADDR: u16 = 0xFF6A;
const OBPD_ADDR: u16 = 0xFF6B;

pub const PPU_REGISTER_ADDR_RANGE: std::ops::RangeInclusive<u16> = 0xFF40..=0xFF4B;
pub const CGB_PPU_REGISTER_ADDR_RANGE: std::ops::RangeInclusive<u16> = 0xFF68..=0xFF6B;

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LcdMode {
    HBlank,
    VBlank,
    OAMSearch,
    PixelTransfer,
}

#[derive(Copy, Clone)]
pub enum PixelColor {
    Monochrome(u8),
    Color(u8, u8, u8),
}

impl PixelColor {
    pub fn raw(&self) -> u8 {
        match self {
            PixelColor::Monochrome(p) => *p,
            PixelColor::Color(_, _, _) => 0,
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
    pub vram: Vram,
    background_color_palette: ColorPalette,
    sprite_color_palette: ColorPalette,
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
    bgpi: u8,
    obpi: u8,
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
            BGPI_ADDR => self.regs.bgpi,
            BGPD_ADDR => {
                let offset = self.regs.bgpi & 0b0011_1111;
                self.background_color_palette.read(offset as usize)
            }
            OBPI_ADDR => self.regs.obpi,
            OBPD_ADDR => {
                let offset = self.regs.obpi & 0b0011_1111;
                self.sprite_color_palette.read(offset as usize)
            }
            0xFE00..=0xFE9F => {
                if self.oam_blocked() {
                    0xFF
                } else {
                    self.oam[(addr as usize) - 0xFE00]
                }
            }
            0x8000..=0x97FF => {
                if self.vram_blocked() {
                    0xFF
                } else {
                    self.vram.read(addr)
                }
            }
            0x9800..=0x9FFF => {
                if self.vram_blocked() {
                    0xFF
                } else {
                    self.vram.read(addr)
                }
            }
            VBK_ADDR => self.vram.read(addr),
            _ => unreachable!("Invalid ppu addr: {:04x}", addr),
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
            }
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
            BGPI_ADDR => self.regs.bgpi = byte,
            BGPD_ADDR => {
                let index = self.regs.bgpi & 0b0011_1111;
                self.background_color_palette.write(index as usize, byte);

                let auto_increment = read_bit(self.regs.bgpi, 7);

                if auto_increment {
                    self.regs.bgpi = ((index + 1) % 0x40) | (1 << 7);
                }
            }
            OBPI_ADDR => self.regs.obpi = byte,
            OBPD_ADDR => {
                let index = self.regs.obpi & 0b0011_1111;
                self.sprite_color_palette.write(index as usize, byte);

                let auto_increment = read_bit(self.regs.obpi, 7);

                if auto_increment {
                    self.regs.obpi = ((index + 1) % 0x40) | (1 << 7);
                }
            }
            0xFE00..=0xFE9F => {
                if !self.oam_blocked() {
                    self.oam[(addr as usize) - 0xFE00] = byte;
                }
            }
            0x8000..=0x97FF => {
                if !self.vram_blocked() {
                    self.vram.write(addr, byte);
                }
            }
            0x9800..=0x9FFF => {
                if !self.vram_blocked() {
                    self.vram.write(addr, byte);
                }
            }
            VBK_ADDR => self.vram.write(addr, byte),
            _ => unreachable!("Invalid ppu addr: {:04x}", addr),
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
            vram: Vram::new(),
            background_color_palette: ColorPalette::new(),
            sprite_color_palette: ColorPalette::new(),
        }
    }

    pub fn lcd_enabled(&self) -> bool {
        read_bit(self.regs.lcdc, 7)
    }

    pub fn dma_read(&mut self, addr: u16) -> u8 {
        self.vram.read(addr)
    }

    pub fn dma_write(&mut self, oam_offset: usize, byte: u8) {
        self.oam[oam_offset] = byte;
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

    pub fn clock(&mut self, io_registers: &mut IoRegisters, is_cgb: bool) {
        if !self.lcd_enabled() {
            return;
        }

        let old_mode = Self::compute_lcd_mode(self.cycles, self.scanline);

        // Pixel transfer lasts 43 machine cycles, but we will be done in 40
        if old_mode == LcdMode::PixelTransfer && (self.cycles - OAM_SEARCH_CYCLES) < 40 {
            self.update_screen(is_cgb);
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
                io_registers.request_interrupt(STAT_INTERRUPT);
            }
        }

        let mode = Self::compute_lcd_mode(self.cycles, self.scanline);

        if mode != old_mode {
            self.set_lcd_mode(mode);

            match mode {
                LcdMode::VBlank => {
                    io_registers.request_interrupt(VBLANK_INTERRUPT);
                    self.frame_complete = true;
                    self.screen_buffer.flip();
                }
                LcdMode::PixelTransfer => {
                    self.visible_sprites = self.compute_sprites_for_line(self.scanline);
                }
                _ => (),
            }

            if self.status_interrupt_enabled(mode) {
                io_registers.request_interrupt(STAT_INTERRUPT);
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
            _ => unreachable!("Not a valid LCD mode!"),
        }
    }

    fn update_screen(&mut self, is_cgb: bool) {
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
            let mut background_pixel = if is_cgb {
                (
                    PixelData(0),
                    PixelData(0).with_cgb_color(0, &self.background_color_palette),
                )
            } else {
                let background_palette = self.read(BGP_ADDR);
                (PixelData(0), PixelData(0).with_color(background_palette))
            };

            if background_enabled {
                background_pixel = self.get_background_pixel_color(
                    (x as u8).wrapping_add(scroll_x),
                    (self.scanline as u8).wrapping_add(scroll_y),
                    self.get_background_map_no(),
                    is_cgb,
                );
            }

            if window_enabled {
                let (x, y) = (x as u8, self.scanline as u8);
                let should_draw = (window_x..LCD_WIDTH as u8).contains(&x)
                    && (window_y..LCD_WIDTH as u8).contains(&y);

                if should_draw {
                    background_pixel = self.get_background_pixel_color(
                        x - window_x,
                        y - window_y,
                        self.get_window_map_no(),
                        is_cgb,
                    );
                }
            }

            let mut pixel = background_pixel.1;

            if sprites_enabled {
                // Finds first non-transparent sprite pixel, if any
                self.get_sprites_at_x(x as u8)
                    .iter()
                    .filter(|sprite| sprite.has_priority(background_pixel.0))
                    .find_map(|sprite| {
                        self.get_sprite_pixel_color(sprite, x as u8, self.scanline as u8, is_cgb)
                    })
                    .map(|sprite_pixel_color| pixel = sprite_pixel_color);
            }

            let index = self.scanline * LCD_WIDTH + x;
            self.screen_buffer.get_primary_mut()[index as usize] = pixel;
        }
    }

    fn get_sprites_at_x(&self, x: u8) -> Vec<&Sprite> {
        self.visible_sprites
            .iter()
            .filter(|sprite| {
                let x_start = sprite.x as i32 - 8;
                (x_start..x_start + (NUM_PIXELS_IN_LINE as i32)).contains(&(x as i32))
            })
            .collect()
    }

    fn get_sprite_pixel_color(
        &self,
        sprite: &Sprite,
        x: u8,
        y: u8,
        is_cgb: bool,
    ) -> Option<PixelColor> {
        let (x_start, y_start) = (sprite.x as i32 - 8, sprite.y as i32 - 16);
        let (mut line_x, mut line_y) = ((x as i32 - x_start) as u8, (y as i32 - y_start) as u8);

        if sprite.x_flip {
            line_x = 7 - line_x;
        }
        if sprite.y_flip {
            if self.sprite16_mode() {
                line_y = 15 - line_y;
            } else {
                line_y = 7 - line_y;
            }
        }

        let pixel_data = if self.sprite16_mode() {
            if (0..=7).contains(&line_y) {
                let tile_no = sprite.tile_no & 0xFE; // Upper tile number
                let tile = self.get_sprite_tile(tile_no);
                tile.pixel_at(line_x, line_y)
            } else {
                let tile_no = sprite.tile_no | 0x01; // Lower tile number
                let tile = self.get_sprite_tile(tile_no);
                tile.pixel_at(line_x, line_y - 8)
            }
        } else {
            let tile = self.get_sprite_tile(sprite.tile_no);
            tile.pixel_at(line_x, line_y)
        };

        if pixel_data.0 == 0x00 {
            // 0 maps to transparency, so background color is used
            return None;
        }

        if is_cgb {
            let pixel_color = PixelData(pixel_data.0)
                .with_cgb_color(sprite.cgb_palette_no as usize, &self.sprite_color_palette);
            Some(pixel_color)
        } else {
            let palette = match sprite.palette_no {
                0 => self.read(OBP0_ADDR),
                1 => self.read(OBP1_ADDR),
                _ => unreachable!("Invalid palette_no: {}", sprite.palette_no),
            };

            Some(pixel_data.with_color(palette))
        }
    }

    fn compute_sprites_for_line(&self, line_no: u32) -> Vec<Sprite> {
        let sprites = self.oam.chunks(4).map(|bytes| Sprite::from(bytes));

        let mut sprites_on_line_enumerated: Vec<_> = sprites
            .filter(|sprite| {
                let y_start = sprite.y as i32 - 16;
                let y_length = if self.sprite16_mode() { 16 } else { 8 };

                sprite.x != 0 && (y_start..y_start + y_length).contains(&(line_no as i32))
            })
            .enumerate()
            .collect();

        sprites_on_line_enumerated.sort_by_key(|(i, sprite)| (sprite.x, *i));

        let num_sprites = std::cmp::min(SPRITES_PER_LINE, sprites_on_line_enumerated.len());
        sprites_on_line_enumerated
            .drain(0..num_sprites)
            .map(|(_, s)| s)
            .collect()
    }

    fn get_background_pixel(&self, x: u8, y: u8, map_no: usize) -> PixelData {
        let tile_number = self.vram.get_tile_number(x, y, map_no);
        let tile = self.vram.get_tile(tile_number, self.get_pattern_table());

        let (line_x, line_y) = (x % NUM_PIXELS_IN_LINE as u8, y % NUM_PIXELS_IN_LINE as u8);
        tile.pixel_at(line_x, line_y)
    }

    fn get_background_pixel_color(
        &self,
        x: u8,
        y: u8,
        map_no: usize,
        is_cgb: bool,
    ) -> (PixelData, PixelColor) {
        let pixel_data = self.get_background_pixel(x, y, map_no);

        let pixel_color = if is_cgb {
            let palette_no = self.vram.get_tile_palette_no(x, y, map_no);
            PixelData(pixel_data.0).with_cgb_color(palette_no, &self.background_color_palette)
        } else {
            let background_palette = self.read(BGP_ADDR);
            pixel_data.with_color(background_palette)
        };

        (pixel_data, pixel_color)
    }

    fn get_tile_pixel(tile: &Tile, line_x: u8, line_y: u8, palette: u8) -> PixelData {
        let raw_pixel_colour = tile.pixel_at(line_x, line_y).with_color(palette).raw();
        PixelData(raw_pixel_colour)
    }

    fn get_sprite_tile(&self, tile_index: u8) -> Tile {
        self.vram.get_tile(TileNumber(tile_index, 0), 1)
    }

    fn get_pattern_table(&self) -> usize {
        read_bit(self.read(LCDC_ADDR), 4) as usize
    }

    fn get_background_map_no(&self) -> usize {
        read_bit(self.read(LCDC_ADDR), 3) as usize
    }

    fn get_window_map_no(&self) -> usize {
        read_bit(self.read(LCDC_ADDR), 6) as usize
    }

    fn sprite16_mode(&self) -> bool {
        read_bit(self.read(LCDC_ADDR), 2)
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
        let background_map_no = self.get_background_map_no();
        self.get_map_data(background_map_no)
    }

    pub fn get_window_map(&self) -> Vec<u8> {
        let window_map_no = self.get_window_map_no();
        self.get_map_data(window_map_no)
    }

    // TODO: Iterator over x, y
    pub fn get_map_data(&self, map_no: usize) -> Vec<u8> {
        (0..MAP_WIDTH * MAP_HEIGHT)
            .map(|i| {
                let (x, y) = (i % MAP_WIDTH, i / MAP_WIDTH);
                // Since we are using u8, x and y should automatically wrap around 256
                self.get_background_pixel(x as u8, y as u8, map_no).0
            })
            .collect()
    }

    pub fn get_tileset(&self) -> Vec<u8> {
        let num_tiles = 16 * 24;
        let num_pixels_in_tile = NUM_LINES_IN_TILE * NUM_PIXELS_IN_LINE;
        let mut pixels = vec![0; num_tiles * num_pixels_in_tile];

        let palette = self.read(BGP_ADDR);
        for (tile_index, tile) in self
            .vram
            .read_tile_data_range(0x8000, 0x9800)
            .chunks(TILE_NUM_BYTES)
            .enumerate()
        {
            for p in 0..num_pixels_in_tile {
                let (line_x, line_y) = (p % NUM_PIXELS_IN_LINE, p / NUM_PIXELS_IN_LINE);
                let x = (tile_index % 16) * NUM_PIXELS_IN_LINE + line_x;
                let y = (tile_index / 16) * NUM_PIXELS_IN_LINE + line_y;
                pixels[y * NUM_PIXELS_IN_LINE * 16 + x] =
                    Self::get_tile_pixel(&tile.into(), line_x as u8, line_y as u8, palette).0;
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
                    Self::get_tile_pixel(&tile, line_x as u8, line_y as u8, palette).0;
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
            buffer: [PixelColor::Monochrome(0); (LCD_WIDTH * LCD_HEIGHT * 2) as usize],
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
        &self.buffer[offset..offset + size]
    }

    fn buffer_at_index_mut(&mut self, index: usize) -> &mut [PixelColor] {
        let size = (LCD_WIDTH * LCD_HEIGHT) as usize;
        let offset = index * size;
        &mut self.buffer[offset..offset + size]
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
    cgb_palette_no: u8,
}

impl Sprite {
    fn has_priority(&self, background_pixel: PixelData) -> bool {
        let always_above_background = !self.priority;

        // Background colour 0 is always behind sprite
        always_above_background || background_pixel.0 == 0
    }
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
            cgb_palette_no: bytes[3] & 0b111,
        }
    }
}

pub struct Vram {
    tile_data0: [u8; 0x1800],
    tile_data1: [u8; 0x1800],

    background_maps: [u8; 0x800],
    background_attributes: [u8; 0x800],
    bank_no: usize,
}

impl MemoryAccess for Vram {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0x8000..=0x97FF => match self.bank_no {
                0 => self.tile_data0[addr as usize - 0x8000],
                1 => self.tile_data1[addr as usize - 0x8000],
                _ => unreachable!("Invalid vram bank: {}", self.bank_no),
            },
            0x9800..=0x9FFF => match self.bank_no {
                0 => self.background_maps[addr as usize - 0x9800],
                1 => self.background_attributes[addr as usize - 0x9800],
                _ => unreachable!("Invalid vram bank: {}", self.bank_no),
            },
            VBK_ADDR => self.bank_no as u8,
            _ => unreachable!("Invalid VRAM address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0x8000..=0x97FF => match self.bank_no {
                0 => self.tile_data0[addr as usize - 0x8000] = byte,
                1 => self.tile_data1[addr as usize - 0x8000] = byte,
                _ => unreachable!("Invalid vram bank: {}", self.bank_no),
            },
            0x9800..=0x9FFF => match self.bank_no {
                0 => self.background_maps[addr as usize - 0x9800] = byte,
                1 => self.background_attributes[addr as usize - 0x9800] = byte,
                _ => unreachable!("Invalid vram bank: {}", self.bank_no),
            },
            VBK_ADDR => self.bank_no = (byte & 0b1) as usize,
            _ => unreachable!("Invalid VRAM address: {:04x}", addr),
        }
    }
}

struct TileNumber(u8, usize);

impl Vram {
    fn new() -> Self {
        Self {
            tile_data0: [0; 0x1800],
            tile_data1: [0; 0x1800],
            background_maps: [0; 0x800],
            background_attributes: [0; 0x800],
            bank_no: 0,
        }
    }

    // TODO: Account for CGB mode
    fn read_tile_data_range(&self, start_addr: usize, end_addr: usize) -> &[u8] {
        &self.tile_data0[(start_addr - 0x8000)..(end_addr - 0x8000)]
    }

    fn get_background_map(&self, map_no: usize) -> &[u8] {
        let addr_start = map_no * 0x400;
        &self.background_maps[addr_start..addr_start + 0x400]
    }

    fn get_background_attributes(&self, map_no: usize) -> &[u8] {
        let addr_start = map_no * 0x400;
        &self.background_attributes[addr_start..addr_start + 0x400]
    }

    fn get_tile_number(&self, x: u8, y: u8, map_no: usize) -> TileNumber {
        let (tile_x, tile_y) = (
            x as usize / NUM_PIXELS_IN_LINE,
            y as usize / NUM_PIXELS_IN_LINE,
        );
        let tile_number = self.get_background_map(map_no)[tile_y * 32 + tile_x];
        let bank_no = read_bit(
            self.get_background_attributes(map_no)[tile_y * 32 + tile_x],
            3,
        );
        TileNumber(tile_number, bank_no as usize)
    }

    fn get_tile_palette_no(&self, x: u8, y: u8, map_no: usize) -> usize {
        let (tile_x, tile_y) = (
            x as usize / NUM_PIXELS_IN_LINE,
            y as usize / NUM_PIXELS_IN_LINE,
        );
        self.get_background_attributes(map_no)[tile_y * 32 + tile_x] as usize & 0b111
    }

    fn get_tile(&self, tile_number: TileNumber, pattern_table: usize) -> Tile {
        let TileNumber(tile_number, bank_no) = tile_number;

        let start_index = match pattern_table {
            0 => {
                let pattern_table_start: i32 = 0x1000;
                let offset = (tile_number as i8) as i32 * TILE_NUM_BYTES as i32;
                (pattern_table_start + offset) as usize
            }
            1 => tile_number as usize * TILE_NUM_BYTES,
            _ => unreachable!("Invalid pattern table {}", pattern_table),
        };

        if bank_no == 0 {
            self.tile_data0[start_index..start_index + TILE_NUM_BYTES]
                .as_ref()
                .into()
        } else {
            self.tile_data1[start_index..start_index + TILE_NUM_BYTES]
                .as_ref()
                .into()
        }
    }
}

#[derive(Copy, Clone)]
struct PixelData(u8);

impl PixelData {
    fn with_color(&self, palette: u8) -> PixelColor {
        let high_bit = read_bit(palette, self.0 * 2 + 1) as u8;
        let low_bit = read_bit(palette, self.0 * 2) as u8;
        PixelColor::Monochrome(high_bit << 1 | low_bit)
    }

    fn with_cgb_color(&self, palette_no: usize, palette: &ColorPalette) -> PixelColor {
        palette.get_color(self.0, palette_no)
    }
}

struct Tile<'a> {
    bytes: &'a [u8],
}

impl Tile<'_> {
    fn pixel_at(&self, line_x: u8, line_y: u8) -> PixelData {
        let lower_bit = read_bit(self.bytes[(line_y * 2) as usize], 7 - line_x) as u8;
        let upper_bit = read_bit(self.bytes[(line_y * 2 + 1) as usize], 7 - line_x) as u8;
        PixelData(upper_bit << 1 | lower_bit)
    }
}

impl<'a> std::convert::From<&'a [u8]> for Tile<'a> {
    fn from(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }
}

struct ColorPalette {
    bytes: [u8; 64],
}

impl ColorPalette {
    fn new() -> Self {
        Self { bytes: [0; 64] }
    }

    fn read(&self, offset: usize) -> u8 {
        self.bytes[offset]
    }

    fn write(&mut self, offset: usize, val: u8) {
        self.bytes[offset] = val;
    }

    fn get_color(&self, pixel: u8, palette_no: usize) -> PixelColor {
        let palette_offset = palette_no * 8;
        let byte0 = self.bytes[palette_offset + (pixel as usize) * 2];
        let byte1 = self.bytes[palette_offset + (pixel as usize) * 2 + 1];

        Self::into_color(byte0, byte1)
    }

    fn into_color(byte0: u8, byte1: u8) -> PixelColor {
        let full_color = (byte1 as u16) << 8 | byte0 as u16;

        let r = (full_color & 0b0000_0000_0001_1111) >> 0;
        let g = (full_color & 0b0000_0011_1110_0000) >> 5;
        let b = (full_color & 0b0111_1100_0000_0000) >> 10;

        PixelColor::Color(r as u8, g as u8, b as u8)
    }
}
