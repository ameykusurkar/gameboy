use crate::registers::Registers;
use crate::registers::RegisterIndex;
use crate::registers::RegisterIndex::*;

pub struct Cpu {
    regs: Registers,
    sp: u16,
    pc: u16,
    memory: [u8; 1 << 16],
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

        let old_pc = self.pc;
        print!("{:#06x}: ", self.pc);

        match opcode {
            0x04 => {
                // INC B
                self.inc_reg(B);

                println!("INC B");
            },
            0x0C => {
                // INC C
                self.inc_reg(C);

                println!("INC C");
            },
            0x05 => {
                // DEC B
                let old = self.regs[B];
                self.regs[B] -= 1;
                self.set_flag(ZERO_FLAG, self.regs[B] == 0);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
                self.pc += 1;

                println!("DEC B");
            },
            0x06 => {
                // LD B,n
                let n = self.load_reg_byte(B);

                println!("LD B, {:02x}", n);
            },
            0x0E => {
                // LD C,n
                let n = self.load_reg_byte(C);

                println!("LD C, {:02x}", n);
            },
            // TODO: Refactor LD XX, nn opcodes
            0x11 => {
                // LD DE,nn
                let nn = self.read_u16(self.pc + 1);
                self.regs.write_de(nn);
                self.pc += 3;

                println!("LD DE, {:04x}", nn);
            },
            0x17 => {
                // RLA
                let carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, carry) = rotate_left_through_carry(self.regs[A], carry);
                self.regs[A] = result;

                self.reset_flags();
                // Unlike RL X, ZERO_FLAG is RESET
                self.set_flag(ZERO_FLAG, false);
                self.set_flag(CARRY_FLAG, carry);
                self.pc += 1;

                println!("RLA");
            },
            0x1A => {
                // LD A,(DE)
                let addr = self.regs.read_de();
                self.regs[A] = self.memory[addr as usize];
                self.pc += 1;

                println!("LD A, (DE)");
            },
            0x20 => {
                // JR NZ,n
                // The offset is signed
                let offset = self.memory[self.pc as usize + 1] as i8;
                self.pc += 2;

                if !read_bit(self.regs[F], ZERO_FLAG) {
                    self.pc = ((self.pc as i32) + (offset as i32)) as u16;
                }

                println!("JR NZ, {}", offset);
            },
            0x21 => {
                // LD HL,nn
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
                self.memory[addr as usize] = self.regs[A];
                self.regs.write_hl(addr - 1);
                self.pc += 1;

                println!("LD (HL-), A");
            },
            0x3E => {
                // LD A,n
                let n = self.memory[self.pc as usize + 1];
                self.regs[A] = n;
                self.pc += 2;

                println!("LD A, {:02x}", n);
            },
            0x4F => {
                // LD C,A
                self.regs[C] = self.regs[A];
                self.pc += 1;

                println!("LD C, A");
            },
            0x77 => {
                // LD (HL),A
                let addr = self.regs.read_hl();
                self.memory[addr as usize] = self.regs[A];
                self.pc += 1;

                println!("LD (HL), A");
            },
            0xAF => {
                // XOR A
                self.regs[A] ^= self.regs[A];
                self.reset_flags();
                self.set_flag(ZERO_FLAG, self.regs[A] == 0);
                self.pc += 1;

                println!("XOR A");
            },
            0xC1 => {
                // POP BC
                self.regs.write_bc(self.read_u16(self.sp));
                self.sp += 2;
                self.pc += 1;

                println!("POP BC");
            },
            0xC5 => {
                // PUSH BC
                self.sp -= 2;
                self.write_u16(self.sp, self.regs.read_bc());
                self.pc += 1;

                println!("PUSH BC");
            },
            0xCB => {
                self.execute_prefixed_instruction();
            },
            0xCD => {
                // CALL nn
                let nn = self.read_u16(self.pc + 1);
                self.sp -= 2;
                self.write_u16(self.sp, nn);
                self.pc = nn;

                println!("CALL {:04x}", nn);
            },
            0xE0 => {
                // LDH (n),A
                let offset = self.memory[self.pc as usize + 1];
                let addr = 0xFF + (offset as u16);
                self.memory[addr as usize] = self.regs[A];
                self.pc += 2;

                println!("LDH ({:02x}), A", offset);
            },
            0xE2 => {
                // LDH (C),A
                let addr = 0xFF + (self.regs[C] as u16);
                self.memory[addr as usize] = self.regs[A];
                self.pc += 1;

                println!("LDH (C), A");
            },
            _ => panic!("Unimplemented opcode {:02x}", opcode),
        }

        if old_pc == self.pc {
            panic!("PC is still {:04x}, should have changed!", old_pc);
        }
    }

    pub fn load_bootrom(&mut self, buffer: &[u8]) {
        let bootrom_area = &mut self.memory[0..0x100];
        bootrom_area.copy_from_slice(buffer);
    }

    fn execute_prefixed_instruction(&mut self) {
        self.pc += 1;
        let opcode = self.memory[self.pc as usize];

        match opcode {
            0x11 => {
                // RL C
                let carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, carry) = rotate_left_through_carry(self.regs[C], carry);
                self.regs[C] = result;

                self.reset_flags();
                self.set_flag(ZERO_FLAG, result == 0);
                self.set_flag(CARRY_FLAG, carry);
                self.pc += 1;

                println!("RL C");
            },
            0x7c => {
                // BIT 7,H
                let bit = read_bit(self.regs[H], 7);
                self.set_flag(ZERO_FLAG, !bit);
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, true);
                self.pc += 1;

                println!("BIT 7, H");
            }
            _ => panic!("Unimplemented prefixed (CB) opcode {:02x}", opcode),
        }
    }

    fn inc_reg(&mut self, index: RegisterIndex) {
        // INC r
        let old = self.regs[index];
        self.regs[index] += 1;
        self.set_flag(ZERO_FLAG, self.regs[index] == 0);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, ((old & 0xF) + 1) > 0xF);
        self.pc += 1;
    }

    fn load_reg_byte(&mut self, index: RegisterIndex) -> u8 {
        // INC r, n
        let n = self.memory[self.pc as usize + 1];
        self.regs[index] = n;
        self.pc += 2;

        n
    }

    fn read_u16(&self, addr: u16) -> u16 {
        let addr = addr as usize;
        let lsb = self.memory[addr] as u16;
        let msb = self.memory[addr + 1] as u16;
        (msb << 8) | lsb
    }

    fn write_u16(&mut self, addr: u16, val: u16) {
        let lsb = (val & 0x00FF) as u8;
        let msb = ((val & 0xFF00) >> 8) as u8;
        let addr = addr as usize;
        self.memory[addr] = lsb;
        self.memory[addr + 1] = msb;
    }

    fn reset_flags(&mut self) {
        self.regs[F] = 0x00;
    }

    fn set_flag(&mut self, flag_bit: u8, val: bool) {
        self.regs[F] = set_bit(self.regs[F], flag_bit, val);
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

fn rotate_left_through_carry(val: u8, carry: bool) -> (u8, bool) {
    let new_carry = read_bit(val, 7);
    let new_val = set_bit(val << 1, 0, carry);
    (new_val, new_carry)
}
