# gameboy

A [Game Boy](https://en.wikipedia.org/wiki/Game_Boy) emulator written in Rust.

![A demo gif of the emulator](blue_60fps.gif)

## Usage

The current build supports two frontends: [SDL](https://en.wikipedia.org/wiki/Simple_DirectMedia_Layer), and JavaScript (for the browser, using [WebAssembly](https://en.wikipedia.org/wiki/WebAssembly)).

### SDL

Ensure you have [Rust installed](https://www.rust-lang.org/tools/install), then run:
```shell
make sdl
```

You can then load a ROM and play by running:

```shell
cargo run --release path/to/rom/file.gb
```

If the game supports save files, the emulator will read from and write save data to the same filename as the rom, but with the extension `.sav` instead of `.gb`.

### JavaScript + WASM

The emulator can also be run in the browser, using a JavaScript frontend. To build the emulator and start a server, run:

```shell
make wasm
```

The emulator is then available at `localhost:8080`.

_Note: The browser frontend currently does not support save files._

## Features

- Fully-functional CPU and PPU (pixel processing unit)
- Audio
- Joypad input
- MBC1 and MBC3 support
- Save files

_The emulator works perfectly with [Pokemon Blue](https://en.wikipedia.org/wiki/Pok%C3%A9mon_Red_and_Blue)_.

TODO:
- [ ] Make CPU [cycle-accurate](https://en.wikipedia.org/wiki/Computer_architecture_simulator#Cycle-accurate_simulator)
- [ ] Add support for [Game Boy Color](https://en.wikipedia.org/wiki/Game_Boy_Color) ROMs

## Dependencies

For debugging, the emulator uses a [Rust port](https://github.com/mattbettcher/rustyPixelGameEngine) of [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine) for the display. License [here](https://github.com/mattbettcher/rustyPixelGameEngine/blob/master/LICENSE.md).

## Testing

I have been using [Blargg's test roms](https://github.com/retrio/gb-test-roms) to test my emulator. The emulator passes all of the `cpu_instrs` tests, and many of the other tests.

## References

This is a [curated list](https://gbdev.io/list.html) of useful resources, but the main ones I'm using are:
 - [Gameboy Pan Docs](http://bgb.bircd.org/pandocs.htm)
 - [The Ultimate Game Boy Talk](https://www.youtube.com/watch?v=HyzD8pNlpwI) (has a great explanation of how the PPU works)
 - [The Cycle-Accurate Game Boy Docs](https://github.com/AntonioND/giibiiadvance/blob/master/docs/TCAGBD.pdf)
 - [Gameboy sound hardware](https://gbdev.gg8.se/wiki/articles/Gameboy_sound_hardware)
