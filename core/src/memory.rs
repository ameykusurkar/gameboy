use crate::ppu::{Ppu, PPU_REGISTER_ADDR_RANGE};
use crate::joypad::Joypad;
use crate::serial_transfer::SerialTransfer;
use crate::cartridge::Cartridge;
use crate::sound_controller::SoundController;
use crate::utils::{read_bit, set_bit};

const OAM_RANGE: std::ops::RangeInclusive<u16> = 0xFE00..=0xFE9F;

// Address for the timer register (TIMA). The increment frequency is specified in the TAC register.
const TIMA_ADDR: u16 = 0xFF05;
// When TIMA overflows, the data at this address will be loaded.
const TMA_ADDR: u16  = 0xFF06;
// Address for the timer control register. Bit 2 specifies if the timer is active, and bits 1-0
// specify the frequency at which to increment the timer.
const TAC_ADDR: u16  = 0xFF07;

// Address of the interrupt enable register
const IE_ADDR: u16 = 0xFFFF;
// Address of the interrupt flags register
const IF_ADDR: u16 = 0xFF0F;

pub struct Memory {
    cartridge: Cartridge,
    memory: [u8; 1 << 16],
    bootrom: [u8; 256],
    pub ppu: Ppu,
    pub joypad: Joypad,
    pub sound_controller: SoundController,
    pub serial_transfer: SerialTransfer,
    pub div: u16,
    memory_mode: MemoryMode,
    dma_state: Option<DmaState>,
    pub io_registers: IoRegisters,
}

#[derive(Debug)]
enum MemoryMode {
    Normal,
    DmaTriggered(u16),
    DmaNoop(u16),
}

#[derive(Debug)]
struct DmaState(u16, usize);

pub trait MemoryAccess {
    fn read(&self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, byte: u8);
}

impl Memory {
    pub fn new(rom: Vec<u8>, external_ram: Option<Vec<u8>>) -> Memory {
        Memory {
            cartridge: Cartridge::new(rom, external_ram),
            memory: [0; 1 << 16],
            bootrom: [0; 256],
            joypad: Joypad::default(),
            ppu: Ppu::new(),
            sound_controller: SoundController::new(),
            serial_transfer: SerialTransfer::new(),
            div: 0,
            memory_mode: MemoryMode::Normal,
            dma_state: None,
            io_registers: IoRegisters::default(),
        }
    }

    pub fn clock(&mut self) {
        self.ppu.clock(&mut self.io_registers.interrupt_flags);
        self.sound_controller.clock();

        self.dma_transfer_cycle();
        self.update_memory_mode();
    }

    pub fn load_bootrom(&mut self, buffer: &[u8]) {
        self.bootrom.copy_from_slice(buffer);
    }

    fn dma_transfer_cycle(&mut self) {
        if let Some(DmaState(start_addr, cycle)) = self.dma_state {
            let src_addr = start_addr + cycle as u16;

            let byte = match src_addr {
                0x8000..=0x9FFF => self.ppu.dma_read(src_addr),
                _ => self.memory[src_addr as usize],
            };

            self.ppu.dma_write(cycle, byte);

            self.dma_state = if cycle == 159 {
                None
            } else {
                Some(DmaState(start_addr, cycle + 1))
            }
        }
    }

    fn update_memory_mode(&mut self) {
        match self.memory_mode {
            MemoryMode::Normal => (),
            MemoryMode::DmaTriggered(start_addr) => {
                self.memory_mode = MemoryMode::DmaNoop(start_addr);
            }
            MemoryMode::DmaNoop(start_addr) => {
                self.dma_state = Some(DmaState(start_addr, 0));
                self.memory_mode = MemoryMode::Normal;
            },
        }
    }

    fn is_bootrom_active(&self) -> bool {
        (self.memory[0xFF50] & 0x01) == 0
    }

    pub fn div_cycle(&mut self) {
        self.div = self.div.wrapping_add(4);
    }

    // TODO: This method is a performance bottleneck, investigate
    // Eg. when it is called to update registers and handle interrupts
    // we don't have to compare the address against most of the address space
    pub fn cpu_read(&self, addr: u16) -> u8 {
        if self.dma_state.is_some() && OAM_RANGE.contains(&addr) {
            return 0xFF;
        }

        let addr = addr as usize;

        // TODO: Refactor how memory access is delegated
        if addr < 0x100 && self.is_bootrom_active() {
            self.bootrom[addr]
        } else if (0x0000..0x8000).contains(&addr) {
            self.cartridge.read(addr as u16)
        } else if (0xA000..0xC000).contains(&addr) {
            self.cartridge.read(addr as u16)
        } else if addr == 0xFF00 {
            self.joypad.read(addr as u16)
        } else if addr == 0xFF01 || addr == 0xFF02 {
            self.serial_transfer.read(addr as u16)
        } else if addr == 0xFF04 {
            let top_bits = self.div & 0xFF00;
            (top_bits >> 8) as u8
        } else if Self::is_io_addr(addr as u16) {
            self.io_registers.read(addr as u16)
        } else if Self::is_sound_addr(addr as u16) {
            self.sound_controller.read(addr as u16)
        } else if Self::is_ppu_addr(addr as u16) {
            self.ppu.read(addr as u16)
        } else {
            self.memory[addr]
        }
    }

    pub fn cpu_write(&mut self, addr: u16, val: u8) {
        if self.dma_state.is_some() && OAM_RANGE.contains(&addr) {
            return;
        }

        match addr {
            // Cartridge
            0x0000..=0x7FFF => {
                self.cartridge.write(addr, val);
            },
            // External RAM, also handled by cartridge
            0xA000..=0xBFFF => {
                self.cartridge.write(addr, val);
            },
            // Joypad
            0xFF00 => {
                self.joypad.write(addr, val);
            },
            0xFF01 | 0xFF02 => {
                self.serial_transfer.write(addr, val);
            },
            0xFF04 => {
                self.div = 0x00;
            },
            _ if Self::is_io_addr(addr) => {
                self.io_registers.write(addr, val);
            },
            _ if Self::is_sound_addr(addr) => {
                self.sound_controller.write(addr, val);
            },
            _ if Self::is_ppu_addr(addr) => {
                self.ppu.write(addr, val);
            },
            // DMA Transfer
            0xFF46 => {
                let start_addr = (val as u16) << 8;
                self.memory_mode = MemoryMode::DmaTriggered(start_addr);
            },
            _ => self.memory[addr as usize] = val,
        }
    }

    fn is_sound_addr(addr: u16) -> bool {
        (0xFF10..=0xFF14).contains(&addr)
            || (0xFF16..=0xFF19).contains(&addr)
            || (0xFF1A..=0xFF1E).contains(&addr)
            || (0xFF20..=0xFF23).contains(&addr)
            || (0xFF24..=0xFF26).contains(&addr)
            || (0xFF30..=0xFF3F).contains(&addr)
    }

    fn is_ppu_addr(addr: u16) -> bool {
        (0x8000..=0x9FFF).contains(&addr)
            || (OAM_RANGE).contains(&addr)
            // TODO: Move DMA logic (0xFF46) to PPU?
            || (PPU_REGISTER_ADDR_RANGE.contains(&addr) && addr != 0xFF46)
    }

    fn is_io_addr(addr: u16) -> bool {
        (0xFF05..=0xFF07).contains(&addr)
            || addr == IE_ADDR
            || addr == IF_ADDR
    }

    pub fn get_external_ram(&self) -> Option<&[u8]> {
        if self.cartridge.mbc.should_save_ram() {
            Some(&self.cartridge.ram)
        } else {
            None
        }
    }

    pub fn mark_external_ram_as_saved(&mut self) {
        self.cartridge.mbc.mark_ram_as_saved()
    }
}

#[derive(Default)]
pub struct IoRegisters {
    tima: u8,
    tma: u8,
    tac: u8,

    interrupt_enable: u8,
    interrupt_flags: u8,
}

impl IoRegisters {
    pub fn increment_timer(&mut self) {
        self.tima += 1;
    }

    pub fn reload_timer(&mut self) {
        self.tima = self.tma;
    }

    pub fn get_timer(&self) -> u8 {
        self.tima
    }

    pub fn timer_is_active(&self) -> bool {
        read_bit(self.tac, 2)
    }

    pub fn get_timer_mode(&self) -> u8 {
        self.tac & 0b11
    }

    pub fn request_interrupt(&mut self, interrupt_no: u8) {
        self.interrupt_flags |= 1 << interrupt_no;
    }

    pub fn unset_interrupt(&mut self, interrupt_no: u8) {
        self.interrupt_flags = set_bit(self.interrupt_flags, interrupt_no, false);
    }

    pub fn get_pending_interrupts(&mut self) -> u8 {
        self.interrupt_enable & self.interrupt_flags
    }
}

impl MemoryAccess for IoRegisters {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            TIMA_ADDR => self.tima,
            TMA_ADDR => self.tma,
            TAC_ADDR => self.tac,
            IE_ADDR => self.interrupt_enable,
            IF_ADDR => self.interrupt_flags,
            _ => unreachable!("Invalid IO address: {:04x}", addr),
        }
    }

    fn write(&mut self, addr: u16, byte: u8) {
        match addr {
            TIMA_ADDR => self.tima = byte,
            TMA_ADDR => self.tma = byte,
            TAC_ADDR => self.tac = byte,
            IE_ADDR => self.interrupt_enable = byte,
            IF_ADDR => self.interrupt_flags = byte,
            _ => unreachable!("Invalid IO address: {:04x}", addr),
        }
    }
}
