use crate::memory::MemoryAccess;

mod mbc1;
mod mbc3;

use crate::cartridge::mbc1::Mbc1;
use crate::cartridge::mbc3::Mbc3;

pub struct Cartridge {
    rom: Vec<u8>,
    ram: Vec<u8>,
    mbc: Box<dyn Mbc>,
}

trait Mbc {
    fn read_rom(&self, rom: &[u8], addr: u16) -> u8;
    fn write_rom(&mut self, rom: &mut [u8], addr: u16, byte: u8);

    fn read_ram(&self, ram: &[u8], addr: u16) -> u8;
    fn write_ram(&mut self, ram: &mut [u8], addr: u16, byte: u8);
}

impl Cartridge {
    pub fn new(rom: Vec<u8>) -> Self {
        let ram_size = match rom[0x0149] {
            0x00 => 0,
            0x01 => 1 << (10 + 1), // 2KB
            0x02 => 1 << (10 + 3), // 8KB
            _    => 1 << (10 + 5), // 32KB
        };

        println!("CARTRIDGE TYPE: {:02x}h", rom[0x0147]);
        let mbc: Box<dyn Mbc> = match rom[0x147] {
            0x00 => Box::new(NoMbc::default()),
            0x01..=0x03 => Box::new(Mbc1::new()),
            0x0F | 0x10 => unimplemented!("MBC Timer not implemented!"),
            0x11..=0x13 => Box::new(Mbc3::new()),
            _ => unimplemented!("Unimplemented cartridge type: {:02x}h", rom[0x147]),
        };

        Cartridge {
            rom,
            ram: vec![0; ram_size],
            mbc,
        }
    }
}

impl MemoryAccess for Cartridge {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x7FFF => {
                self.mbc.read_rom(&self.rom, addr)
            },
            0xA000..=0xBFFF => {
                self.mbc.read_ram(&self.ram, addr)
            },
            _ => unreachable!("Invalid cartridge read address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            0x0000..=0x7FFF => {
                self.mbc.write_rom(&mut self.rom, addr, byte)
            },
            0xA000..=0xBFFF => {
                self.mbc.write_ram(&mut self.ram, addr, byte)
            },
            _ => unreachable!("Invalid cartridge write address: {:04x}", addr),
        }
    }
}

#[derive(Default)]
struct NoMbc;

impl Mbc for NoMbc {
    fn read_rom(&self, rom: &[u8], addr: u16) -> u8 {
        rom[addr as usize]
    }

    fn write_rom(&mut self, _rom: &mut [u8], _addr: u16, _byte: u8) {}
    fn read_ram(&self, _ram: &[u8], _addr: u16) -> u8 { 0xFF }
    fn write_ram(&mut self, _ram: &mut [u8], _addr: u16, _byte: u8) {}
}
