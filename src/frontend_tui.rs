use std::fs::File;
use std::io::{self, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crossterm::cursor::{Hide, MoveTo, Show};
use crossterm::event::{
    self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers, KeyboardEnhancementFlags,
    PopKeyboardEnhancementFlags, PushKeyboardEnhancementFlags,
};
use crossterm::style::{Color, Print, ResetColor, SetBackgroundColor, SetForegroundColor};
use crossterm::terminal::{
    self, BeginSynchronizedUpdate, Clear, ClearType, EndSynchronizedUpdate, EnterAlternateScreen,
    LeaveAlternateScreen,
};
use crossterm::{execute, queue};

use core::emulator::{Emulator, MACHINE_CYCLES_PER_SECOND};
use core::memory::Memory;
use core::ppu::{FRAME_CYCLES, LCD_HEIGHT, LCD_WIDTH, PixelColor};

const FRAME_INTERVAL: Duration =
    Duration::from_nanos((FRAME_CYCLES as f32 / MACHINE_CYCLES_PER_SECOND as f32 * 1e9) as u64);

// Each terminal cell draws two vertically stacked pixels using an upper half block:
// the foreground colour is the top pixel, the background colour is the bottom pixel.
const HALF_BLOCK: char = '▀';
const COLS: u16 = LCD_WIDTH as u16;
const ROWS: u16 = (LCD_HEIGHT / 2) as u16;

// Without key release events, a key counts as held for this long after its last
// press/repeat event. Needs to bridge the gap between OS key repeat events.
const HOLD_TIMEOUT: Duration = Duration::from_millis(200);

const MONOCHROME_PALETTE: [(u8, u8, u8); 4] = [
    (255, 228, 204),
    (240, 100, 120),
    (48, 51, 107),
    (19, 15, 64),
];

type Rgb = (u8, u8, u8);

#[derive(Clone, Copy)]
enum Button {
    Down,
    Up,
    Left,
    Right,
    Select,
    Start,
    B,
    A,
}

impl Button {
    fn from_char(c: char) -> Option<Button> {
        match c.to_ascii_lowercase() {
            'j' => Some(Button::Down),
            'k' => Some(Button::Up),
            'h' => Some(Button::Left),
            'l' => Some(Button::Right),
            'v' => Some(Button::Select),
            'n' => Some(Button::Start),
            'd' => Some(Button::B),
            'f' => Some(Button::A),
            _ => None,
        }
    }
}

struct Input {
    // Whether the terminal reports key release events (kitty keyboard protocol)
    release_events: bool,
    pressed_at: [Option<Instant>; 8],
}

impl Input {
    fn handle(&mut self, key: KeyEvent) {
        let Some(button) = (match key.code {
            KeyCode::Char(c) => Button::from_char(c),
            _ => None,
        }) else {
            return;
        };

        self.pressed_at[button as usize] = match key.kind {
            KeyEventKind::Press | KeyEventKind::Repeat => Some(Instant::now()),
            KeyEventKind::Release => None,
        };
    }

    fn is_held(&self, button: Button, now: Instant) -> bool {
        match self.pressed_at[button as usize] {
            None => false,
            Some(t) => self.release_events || now - t < HOLD_TIMEOUT,
        }
    }
}

pub struct FrontendTui {
    emulator: Emulator,
    save_path: PathBuf,
    input: Input,
    // Last cell colours written to the terminal, used to only redraw changed cells.
    // None means the cell must be redrawn.
    prev_frame: Vec<Option<(Rgb, Rgb)>>,
    // Top-left corner of the screen in the terminal, or None if the terminal is too small
    origin: Option<(u16, u16)>,
}

impl FrontendTui {
    pub fn start(emulator: Emulator, save_path: PathBuf) -> io::Result<()> {
        let _guard = TerminalGuard::enter()?;

        let release_events = terminal::supports_keyboard_enhancement().unwrap_or(false);
        if release_events {
            execute!(
                io::stdout(),
                PushKeyboardEnhancementFlags(
                    KeyboardEnhancementFlags::DISAMBIGUATE_ESCAPE_CODES
                        | KeyboardEnhancementFlags::REPORT_EVENT_TYPES
                )
            )?;
        }

        let mut frontend = FrontendTui {
            emulator,
            save_path,
            input: Input {
                release_events,
                pressed_at: [None; 8],
            },
            prev_frame: vec![None; COLS as usize * ROWS as usize],
            origin: None,
        };

        let (width, height) = terminal::size()?;
        frontend.resize(width, height)?;

        frontend.run()
    }

    fn run(&mut self) -> io::Result<()> {
        let mut next_frame_at = Instant::now();

        loop {
            while event::poll(Duration::ZERO)? {
                match event::read()? {
                    Event::Key(key) if is_quit(&key) => return Ok(()),
                    Event::Key(key) => self.input.handle(key),
                    Event::Resize(width, height) => self.resize(width, height)?,
                    _ => {}
                }
            }

            self.update_joypad();

            loop {
                self.emulator.clock();
                if self.emulator.memory.ppu.frame_complete {
                    break;
                }
            }
            self.emulator.memory.ppu.frame_complete = false;

            save_external_ram(&mut self.emulator.memory, &self.save_path);

            self.draw()?;

            next_frame_at += FRAME_INTERVAL;
            let now = Instant::now();
            if now < next_frame_at {
                std::thread::sleep(next_frame_at - now);
            } else if now - next_frame_at > FRAME_INTERVAL * 4 {
                // Too far behind (e.g. a slow terminal), don't try to catch up
                next_frame_at = now;
            }
        }
    }

    fn update_joypad(&mut self) {
        let now = Instant::now();
        let input = &self.input;
        let joypad = &mut self.emulator.memory.joypad;

        // TODO: Trigger joypad interrupt
        joypad.down = input.is_held(Button::Down, now);
        joypad.up = input.is_held(Button::Up, now);
        joypad.left = input.is_held(Button::Left, now);
        joypad.right = input.is_held(Button::Right, now);
        joypad.select = input.is_held(Button::Select, now);
        joypad.start = input.is_held(Button::Start, now);
        joypad.b = input.is_held(Button::B, now);
        joypad.a = input.is_held(Button::A, now);
    }

    fn resize(&mut self, width: u16, height: u16) -> io::Result<()> {
        let mut stdout = io::stdout();
        queue!(stdout, ResetColor, Clear(ClearType::All))?;

        self.prev_frame.fill(None);
        self.origin = if width >= COLS && height >= ROWS {
            Some(((width - COLS) / 2, (height - ROWS) / 2))
        } else {
            let message = format!(
                "Terminal too small: need {}x{}, have {}x{}. Press q to quit.",
                COLS, ROWS, width, height
            );
            queue!(stdout, MoveTo(0, 0), Print(message))?;
            None
        };

        stdout.flush()
    }

    fn draw(&mut self) -> io::Result<()> {
        let Some((origin_x, origin_y)) = self.origin else {
            return Ok(());
        };
        // LCD is off, keep showing the last frame
        let Some(screen_buffer) = self.emulator.get_screen_buffer() else {
            return Ok(());
        };

        let width = LCD_WIDTH as usize;
        let mut buf = Vec::new();
        let mut cursor = None;
        let mut colors = None;

        queue!(buf, BeginSynchronizedUpdate)?;

        for row in 0..ROWS {
            for col in 0..COLS {
                let (x, y) = (col as usize, row as usize * 2);
                let top = color(screen_buffer[y * width + x]);
                let bottom = color(screen_buffer[(y + 1) * width + x]);

                let cell = &mut self.prev_frame[row as usize * COLS as usize + col as usize];
                if *cell == Some((top, bottom)) {
                    continue;
                }
                *cell = Some((top, bottom));

                if cursor != Some((col, row)) {
                    queue!(buf, MoveTo(origin_x + col, origin_y + row))?;
                }
                if colors != Some((top, bottom)) {
                    queue!(
                        buf,
                        SetForegroundColor(rgb(top)),
                        SetBackgroundColor(rgb(bottom))
                    )?;
                    colors = Some((top, bottom));
                }
                queue!(buf, Print(HALF_BLOCK))?;
                cursor = Some((col + 1, row));
            }
        }

        queue!(buf, EndSynchronizedUpdate)?;

        let mut stdout = io::stdout().lock();
        stdout.write_all(&buf)?;
        stdout.flush()
    }
}

/// Puts the terminal into raw mode on an alternate screen, and restores it when
/// dropped or if the program panics.
struct TerminalGuard;

impl TerminalGuard {
    fn enter() -> io::Result<Self> {
        let default_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            restore_terminal();
            default_hook(info);
        }));

        terminal::enable_raw_mode()?;
        execute!(io::stdout(), EnterAlternateScreen, Hide)?;
        Ok(TerminalGuard)
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        restore_terminal();
    }
}

fn restore_terminal() {
    // Popping keyboard flags that were never pushed is harmless
    let _ = execute!(
        io::stdout(),
        PopKeyboardEnhancementFlags,
        ResetColor,
        Show,
        LeaveAlternateScreen
    );
    let _ = terminal::disable_raw_mode();
}

fn is_quit(key: &KeyEvent) -> bool {
    if key.kind == KeyEventKind::Release {
        return false;
    }
    match key.code {
        KeyCode::Char('q') | KeyCode::Esc => true,
        KeyCode::Char('c') => key.modifiers.contains(KeyModifiers::CONTROL),
        _ => false,
    }
}

fn save_external_ram(memory: &mut Memory, save_path: &PathBuf) {
    memory
        .get_external_ram()
        .and_then(|ram| {
            File::create(save_path)
                .and_then(|mut f| f.write_all(ram))
                .ok()
        })
        .map(|_| memory.mark_external_ram_as_saved());
}

fn color(pixel: PixelColor) -> Rgb {
    match pixel {
        PixelColor::Monochrome(p) => MONOCHROME_PALETTE[p as usize],
        PixelColor::Color(r, g, b) => (r << 3, g << 3, b << 3),
    }
}

fn rgb((r, g, b): Rgb) -> Color {
    Color::Rgb { r, g, b }
}
