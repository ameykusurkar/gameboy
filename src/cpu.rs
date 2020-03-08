pub struct Cpu {
    regs: Registers,
    sp: u16,
    pc: u16,
    memory: [u8; 1 << 16],
}

#[derive(Default, Debug)]
struct Registers {
    a: u8, f: u8,
    b: u8, c: u8,
    d: u8, e: u8,
    h: u8, l: u8,
}

const ZERO_FLAG: u8       = 7;
const SUBTRACT_FLAG: u8   = 6;
const HALF_CARRY_FLAG: u8 = 5;
const CARRY_FLAG: u8      = 4;

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            regs: Registers::default(),
            sp: 0,
            pc: 0,
            memory: [0; 1 << 16],
        }
    }

    pub fn step(&mut self) {
        let opcode = self.memory[self.pc as usize];

        match opcode {
            0x21 => {
                // LD SP,nn
                let nn = self.read_u16(self.pc + 1);
                self.regs.write_hl(nn);
                self.pc += 3;

                println!("LD HL, {:04x}", nn);
            },
            0x31 => {
                // LD SP,nn
                let nn = self.read_u16(self.pc + 1);
                self.sp = nn;
                self.pc += 3;

                println!("LD SP, {:04x}", nn);
            },
            0x32 => {
                // LD (HL-), A
                let addr = self.regs.read_hl();
                self.memory[addr as usize] = self.regs.a;
                self.regs.write_hl(addr - 1);
                self.pc += 1;

                println!("LD (HL-), A");
            },
            0xAF => {
                // XOR A
                self.regs.a ^= self.regs.a;
                self.reset_flags();
                self.set_flag(ZERO_FLAG, self.regs.a == 0);
                self.pc += 1;

                println!("XOR A");
            },
            0xCB => {
                self.execute_prefixed_instruction();
            },
            _ => panic!("Unimplemented opcode {:02x}", opcode),
        }
    }

    pub fn load_bootrom(&mut self, buffer: &[u8]) {
        let bootrom_area = &mut self.memory[0..0x100];
        bootrom_area.copy_from_slice(buffer);
    }

    pub fn execute_prefixed_instruction(&mut self) {
        self.pc += 1;
        let opcode = self.memory[self.pc as usize];

        match opcode {
            0x7c => {
                // BIT 7,H
                let bit = read_bit(self.regs.h, 7);
                self.set_flag(ZERO_FLAG, !bit);
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, true);
                self.pc += 1;

                println!("BIT 7, H");
            }
            _ => panic!("Unimplemented prefixed opcode {:02x}", opcode),
        }
    }

    fn read_u16(&self, addr: u16) -> u16 {
        let lsb = self.memory[addr as usize] as u16;
        let msb = self.memory[addr as usize + 1] as u16;
        (msb << 8) | lsb
    }

    fn reset_flags(&mut self) {
        self.regs.f = 0x00;
    }

    fn set_flag(&mut self, flag_bit: u8, val: bool) {
        self.regs.f = set_bit(self.regs.f, flag_bit, val);
    }
}

impl Registers {
    fn write_hl(&mut self, val: u16) {
        self.h = ((val & 0xFF00) >> 8) as u8;
        self.l = (val & 0x00FF) as u8;
    }

    fn read_hl(&self) -> u16 {
        ((self.h as u16) << 8) | self.l as u16
    }
}

// Writes `x` to the nth bit in `byte`
fn set_bit(byte: u8, n: u8, x: bool) -> u8 {
    (byte & !(1 << n)) | ((x as u8) << n)
}

// Reads the nth bit in `byte`
fn read_bit(byte: u8, n: u8) -> bool {
    (byte & (1 << n)) > 0
}
