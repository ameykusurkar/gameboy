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
        // TODO: Remove this
        // Temp hack to let CPU think that screen is done rendering
        self.memory[0xFF44] = 0x90;

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
                self.dec_reg(B);

                println!("DEC B");
            },
            0x06 => {
                // LD B,n
                let n = self.load_reg_byte(B);

                println!("LD B, {:02x}", n);
            },
            0x0D => {
                // DEC C
                self.dec_reg(C);

                println!("DEC C");
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
            0x1D => {
                // DEC E
                self.dec_reg(E);

                println!("DEC E");
            },
            // TODO: Refactor INC XX opcodes
            0x13 => {
                // INC DE
                self.regs.write_de(self.regs.read_de() + 1);
                self.pc += 1;

                println!("INC DE");
            },
            0x15 => {
                // DEC D
                self.dec_reg(D);

                println!("DEC D");
            },
            0x16 => {
                // LD D,n
                let n = self.load_reg_byte(D);

                println!("LD D, {:02x}", n);
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
            0x18 => {
                // JR n
                let offset = self.jump_rel();

                println!("JR {}", offset);
            },
            0x1A => {
                // LD A,(DE)
                let addr = self.regs.read_de();
                self.regs[A] = self.memory[addr as usize];
                self.pc += 1;

                println!("LD A, (DE)");
            },
            0x1E => {
                // LD E,n
                let n = self.load_reg_byte(E);

                println!("LD E, {:02x}", n);
            },
            0x20 => {
                // JR NZ,n
                let offset = self.jump_rel_condition(!read_bit(self.regs[F], ZERO_FLAG));

                println!("JR NZ, {}", offset);
            },
            0x21 => {
                // LD HL,nn
                let nn = self.read_u16(self.pc + 1);
                self.regs.write_hl(nn);
                self.pc += 3;

                println!("LD HL, {:04x}", nn);
            },
            0x22 => {
                // LD (HL+),A
                let addr = self.regs.read_hl();
                self.memory[addr as usize] = self.regs[A];
                self.regs.write_hl(addr + 1);
                self.pc += 1;

                println!("LD (HL+), A");
            },
            0x23 => {
                // INC HL
                self.regs.write_hl(self.regs.read_hl() + 1);
                self.pc += 1;

                println!("INC HL");
            },
            0x24 => {
                // INC H
                self.inc_reg(H);

                println!("INC H");
            },
            0x28 => {
                // JR Z,n
                let offset = self.jump_rel_condition(read_bit(self.regs[F], ZERO_FLAG));

                println!("JR Z, {}", offset);
            },
            0x2E => {
                // LD L,n
                let n = self.load_reg_byte(L);

                println!("LD L, {:02x}", n);
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
            0x3D => {
                // DEC A
                self.dec_reg(A);

                println!("DEC A");
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
                self.load_reg_reg(C, A);

                println!("LD C, A");
            },
            0x57 => {
                // LD D,A
                self.load_reg_reg(D, A);

                println!("LD D, A");
            },
            0x67 => {
                // LD H,A
                self.load_reg_reg(H, A);

                println!("LD H, A");
            },
            0x77 => {
                // LD (HL),A
                let addr = self.regs.read_hl();
                self.memory[addr as usize] = self.regs[A];
                self.pc += 1;

                println!("LD (HL), A");
            },
            0x7B => {
                // LD A,E
                self.load_reg_reg(A, E);

                println!("LD A, E");
            },
            0x7C => {
                // LD A,H
                self.load_reg_reg(A, H);

                println!("LD A, H");
            },
            0x90 => {
                // SUB B
                let old = self.regs[A];
                self.regs[A] -= self.regs[B];
                self.set_flag(ZERO_FLAG, self.regs[A] == 0);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) < (self.regs[B] & 0xF));
                self.set_flag(CARRY_FLAG, old < self.regs[B]);
                self.pc += 1;

                println!("SUB B");
            },
            0xAF => {
                // XOR A
                self.regs[A] ^= self.regs[A];
                self.reset_flags();
                self.set_flag(ZERO_FLAG, self.regs[A] == 0);
                self.pc += 1;

                println!("XOR A");
            },
            0xBE => {
                // CP (HL)
                let addr = self.regs.read_hl();
                let n = self.memory[addr as usize];
                let r = self.regs[A];
                self.set_flag(ZERO_FLAG, r == n);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (r & 0x0F) < (n & 0x0F));
                self.set_flag(CARRY_FLAG, r < n);
                self.pc += 1;

                println!("CP (HL)");
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
            0xC9 => {
                // RET
                self.pc = self.read_u16(self.sp);
                self.sp += 2;

                println!("RET");
            },
            0xCB => {
                self.execute_prefixed_instruction();
            },
            0xCD => {
                // CALL nn
                let nn = self.read_u16(self.pc + 1);
                self.sp -= 2;
                self.write_u16(self.sp, self.pc + 3);
                self.pc = nn;

                println!("CALL {:04x}", nn);
            },
            0xE0 => {
                // LDH (n),A
                let offset = self.memory[self.pc as usize + 1];
                let addr = 0xFF00 + (offset as u16);
                self.memory[addr as usize] = self.regs[A];
                self.pc += 2;

                println!("LDH ({:02x}), A", offset);
            },
            0xE2 => {
                // LDH (C),A
                let addr = 0xFF00 + (self.regs[C] as u16);
                self.memory[addr as usize] = self.regs[A];
                self.pc += 1;

                println!("LDH (C), A");
            },
            0xEA => {
                // LD (nn),A
                let nn = self.read_u16(self.pc + 1);
                self.memory[nn as usize] = self.regs[A];
                self.pc += 3;

                println!("LD ({:04x}), A", nn);
            },
            0xF0 => {
                // LDH A,(n)
                let offset = self.memory[self.pc as usize + 1];
                let addr = 0xFF00 + (offset as u16);
                self.regs[A] = self.memory[addr as usize];
                self.pc += 2;

                println!("LDH A, ({:02x})", offset);
            },
            0xFE => {
                // CP n
                let n = self.memory[self.pc as usize + 1];
                let r = self.regs[A];
                self.set_flag(ZERO_FLAG, r == n);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (r & 0x0F) < (n & 0x0F));
                self.set_flag(CARRY_FLAG, r < n);
                self.pc += 2;

                println!("CP {:02x}", n);
            },
            _ => panic!("Unimplemented opcode {:02x}", opcode),
        }

        println!("{:02x?}, PC: {:#06x}", self.regs, self.pc);

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

    fn dec_reg(&mut self, index: RegisterIndex) {
        // DEC r
        let old = self.regs[index];
        self.regs[index] -= 1;
        self.set_flag(ZERO_FLAG, self.regs[index] == 0);
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
        self.pc += 1;
    }
    fn load_reg_byte(&mut self, index: RegisterIndex) -> u8 {
        // LD r, n
        let n = self.memory[self.pc as usize + 1];
        self.regs[index] = n;
        self.pc += 2;

        n
    }

    fn load_reg_reg(&mut self, dest: RegisterIndex, source: RegisterIndex) {
        self.regs[dest] = self.regs[source];
        self.pc += 1;
    }

    fn jump_rel(&mut self) -> i8 {
        // JR n
        self.jump_rel_condition(true)
    }

    fn jump_rel_condition(&mut self, condition: bool) -> i8 {
        // JR cc, n
        // Offset is signed
        let offset = self.memory[self.pc as usize + 1] as i8;
        self.pc += 2;

        if condition {
            self.pc = ((self.pc as i32) + (offset as i32)) as u16;
        }

        offset
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
