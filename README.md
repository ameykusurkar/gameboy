# gameboy

Still a work in progress, the end goal is to be a fully-functional Gameboy emulator. Currently the emulator supports simple games such as Tetris and Dr. Mario.

TODO:
 - Memory bank controller support
 - Implement sound system
 
## Demo
 
To make it easier to debug, emulator displays important information about the CPU/memory in real-time, such as the values of the registers, flags, upcoming instructions, background maps and tilesets. It also supports a "step" mode, allowing the user to execute one machine cycle at a time.
 
![A demo gif of the emulator](example.gif)

## Dependencies

The emulator uses a [Rust port](https://github.com/mattbettcher/rustyPixelGameEngine) of [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine) for the display. License [here](https://github.com/mattbettcher/rustyPixelGameEngine/blob/master/LICENSE.md).

## Testing

I have been using [Blargg's test roms](https://github.com/retrio/gb-test-roms) to test my emulator. So far, the emulator passes all of the `cpu_instrs` individual tests.

## References

This is a [curated list](https://gbdev.io/list.html) of useful resources, but the main ones I'm using are:
 - [Gameboy Pan Docs](http://bgb.bircd.org/pandocs.htm)
 - [The Ultimate Game Boy Talk](https://www.youtube.com/watch?v=HyzD8pNlpwI) (has a great explanation of how the PPU works)
 - [The Cycle-Accurate Game Boy Docs](https://github.com/AntonioND/giibiiadvance/blob/master/docs/TCAGBD.pdf)
