use crate::registers::Registers;
use crate::registers::RegisterIndex;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex;
use crate::registers::TwoRegisterIndex::*;
use crate::registers::Flags;
use crate::registers::{ZERO_FLAG, SUBTRACT_FLAG, HALF_CARRY_FLAG, CARRY_FLAG};

use crate::memory::Memory;

pub struct Cpu {
    regs: Registers,
    sp: u16,
    pc: u16,
    ime: bool,
    memory: Memory,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            regs: Registers::default(),
            sp: 0,
            pc: 0,
            ime: false,
            memory: Memory::new(),
        }
    }

    #[allow(dead_code)]
    pub fn skip_bootrom(&mut self) {
        self.pc = 0x100;
        self.sp = 0xFFFE;
        self.memory[0xFF50] = 1;
    }

    pub fn get_memory(&self) -> &[u8] {
        &self.memory.get_memory()
    }

    pub fn step(&mut self) {
        // TODO: Remove this
        // Temp hack to let CPU think that screen is done rendering
        self.memory[0xFF44] = 0x90;

        let opcode = self.memory[self.pc];

        let old_pc = self.pc;
        print!("{:#06x}: ", self.pc);

        match opcode {
            0x00 => {
                // NOP
                self.pc += 1;

                println!("NOP");
            },
            0x01 => {
                // LD BC,nn
                let nn = self.load_rr_nn(BC);

                println!("LD BC, {:04x}", nn);
            },
            0x03 => {
                // INC BC
                self.inc_rr(BC);

                println!("INC BC");
            },
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
            0x11 => {
                // LD DE,nn
                let nn = self.load_rr_nn(DE);

                println!("LD DE, {:04x}", nn);
            },
            0x12 => {
                // LD (DE), A
                let addr = self.regs.read(DE);
                self.memory[addr] = self.regs[A];
                self.pc += 1;

                println!("LD (DE), A");
            },
            0x13 => {
                // INC DE
                self.inc_rr(DE);

                println!("INC DE");
            },
            0x14 => {
                // INC D
                self.inc_reg(D);

                println!("INC D");
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
                // Unlike RL X, zero flag is RESET
                self.regs.write_flags(Flags {
                    carry,
                    ..Flags::default()
                });
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
                let addr = self.regs.read(DE);
                self.regs[A] = self.memory[addr];
                self.pc += 1;

                println!("LD A, (DE)");
            },
            0x1C => {
                // INC E
                self.inc_reg(E);

                println!("INC E");
            },
            0x1D => {
                // DEC E
                self.dec_reg(E);

                println!("DEC E");
            },
            0x1E => {
                // LD E,n
                let n = self.load_reg_byte(E);

                println!("LD E, {:02x}", n);
            },
            0x1F => {
                // RRA
                let carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, carry) = rotate_right_through_carry(self.regs[A], carry);
                self.regs[A] = result;
                // Unlike RL X, zero flag is RESET
                self.regs.write_flags(Flags {
                    carry,
                    ..Flags::default()
                });
                self.pc += 1;

                println!("RRA");
            },
            0x20 => {
                // JR NZ,n
                let offset = self.jump_rel_condition(!read_bit(self.regs[F], ZERO_FLAG));

                println!("JR NZ, {}", offset);
            },
            0x21 => {
                // LD HL,nn
                let nn = self.load_rr_nn(HL);

                println!("LD HL, {:04x}", nn);
            },
            0x22 => {
                // LD (HL+),A
                let addr = self.regs.read(HL);
                self.memory[addr] = self.regs[A];
                self.regs.write(HL, addr + 1);
                self.pc += 1;

                println!("LD (HL+), A");
            },
            0x23 => {
                // INC HL
                self.inc_rr(HL);

                println!("INC HL");
            },
            0x24 => {
                // INC H
                self.inc_reg(H);

                println!("INC H");
            },
            0x25 => {
                // DEC H
                self.dec_reg(H);

                println!("DEC H");
            },
            0x26 => {
                // LD H,n
                let n = self.load_reg_byte(H);

                println!("LD H, {:02x}", n);
            },
            0x28 => {
                // JR Z,n
                let offset = self.jump_rel_condition(read_bit(self.regs[F], ZERO_FLAG));

                println!("JR Z, {}", offset);
            },
            0x29 => {
                // ADD HL,HL
                let old_zero = read_bit(self.regs[F], ZERO_FLAG);
                let (result, flags) = add_u16(self.regs.read(HL), self.regs.read(HL));
                self.regs.write(HL, result);
                self.regs.write_flags(Flags { zero: old_zero, ..flags });
                self.pc += 1;

                println!("ADD HL,HL");
            },
            0x2A => {
                // LD A, (HL+)
                let addr = self.regs.read(HL);
                self.regs[A] = self.memory[addr];
                self.regs.write(HL, addr + 1);
                self.pc += 1;

                println!("LD A, (HL+)");
            },
            0x2C => {
                // INC L
                self.inc_reg(L);

                println!("INC L");
            },
            0x2D => {
                // DEC L
                self.dec_reg(L);

                println!("DEC L");
            },
            0x2E => {
                // LD L,n
                let n = self.load_reg_byte(L);

                println!("LD L, {:02x}", n);
            },
            0x30 => {
                // JR NC,n
                let offset = self.jump_rel_condition(!read_bit(self.regs[F], CARRY_FLAG));

                println!("JR NC, {}", offset);
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
                let addr = self.regs.read(HL);
                self.memory[addr] = self.regs[A];
                self.regs.write(HL, addr - 1);
                self.pc += 1;

                println!("LD (HL-), A");
            },
            0x35 => {
                // DEC (HL)
                let addr = self.regs.read(HL);
                let old = self.memory[addr];
                self.memory[addr] -= 1;
                self.set_flag(ZERO_FLAG, self.memory[addr] == 0);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
                self.pc += 1;

                println!("DEC (HL)");
            },
            0x38 => {
                // JR C,n
                let offset = self.jump_rel_condition(read_bit(self.regs[F], CARRY_FLAG));

                println!("JR C, {}", offset);
            },
            0x3C => {
                // INC A
                self.inc_reg(A);

                println!("INC A");
            },
            0x3D => {
                // DEC A
                self.dec_reg(A);

                println!("DEC A");
            },
            0x3E => {
                // LD A,n
                let n = self.memory[self.pc + 1];
                self.regs[A] = n;
                self.pc += 2;

                println!("LD A, {:02x}", n);
            },
            0x40 => {
                // LD B,B
                self.load_reg_reg(B, B);

                println!("LD B, B");
            },
            0x41 => {
                // LD B,C
                self.load_reg_reg(B, C);

                println!("LD B, C");
            },
            0x42 => {
                // LD B,D
                self.load_reg_reg(B, D);

                println!("LD B, D");
            },
            0x43 => {
                // LD B,E
                self.load_reg_reg(B, E);

                println!("LD B, E");
            },
            0x44 => {
                // LD B,H
                self.load_reg_reg(B, H);

                println!("LD B, H");
            },
            0x45 => {
                // LD B,L
                self.load_reg_reg(B, L);

                println!("LD B, L");
            },
            0x46 => {
                // LD B,(HL)
                self.load_reg_hl(B);

                println!("LD B, (HL)");
            },
            0x47 => {
                // LD B,A
                self.load_reg_reg(B, A);

                println!("LD B, A");
            },
            0x48 => {
                // LD C,B
                self.load_reg_reg(C, B);

                println!("LD C, B");
            },
            0x49 => {
                // LD C,C
                self.load_reg_reg(C, C);

                println!("LD C, C");
            },
            0x4A => {
                // LD C,D
                self.load_reg_reg(C, D);

                println!("LD C, D");
            },
            0x4B => {
                // LD C,E
                self.load_reg_reg(C, E);

                println!("LD C, E");
            },
            0x4C => {
                // LD C,H
                self.load_reg_reg(C, H);

                println!("LD C, H");
            },
            0x4D => {
                // LD C,L
                self.load_reg_reg(C, L);

                println!("LD C, L");
            },
            0x4E => {
                // LD C,(HL)
                self.load_reg_hl(C);

                println!("LD C, (HL)");
            },
            0x4F => {
                // LD C,A
                self.load_reg_reg(C, A);

                println!("LD C, A");
            },
            0x56 => {
                // LD D,(HL)
                self.load_reg_hl(D);

                println!("LD D, (HL)");
            },
            0x57 => {
                // LD D,A
                self.load_reg_reg(D, A);

                println!("LD D, A");
            },
            0x5D => {
                // LD E,L
                self.load_reg_reg(E, L);

                println!("LD E, L");
            },
            0x5F => {
                // LD E,A
                self.load_reg_reg(E, A);

                println!("LD E, A");
            },
            0x67 => {
                // LD H,A
                self.load_reg_reg(H, A);

                println!("LD H, A");
            },
            0x6E => {
                // LD L,(HL)
                self.load_reg_hl(L);

                println!("LD L, (HL)");
            },
            0x6F => {
                // LD L,A
                self.load_reg_reg(L, A);

                println!("LD L, A");
            },
            0x70 => {
                // LD (HL),B
                self.load_hl_reg(B);

                println!("LD (HL), B");
            },
            0x71 => {
                // LD (HL),C
                self.load_hl_reg(C);

                println!("LD (HL), C");
            },
            0x72 => {
                // LD (HL),D
                self.load_hl_reg(D);

                println!("LD (HL), D");
            },
            0x73 => {
                // LD (HL),E
                self.load_hl_reg(E);

                println!("LD (HL), E");
            },
            0x77 => {
                // LD (HL),A
                self.load_hl_reg(A);

                println!("LD (HL), A");
            },
            0x78 => {
                // LD A,B
                self.load_reg_reg(A, B);

                println!("LD A, B");
            },
            0x79 => {
                // LD A,C
                self.load_reg_reg(A, C);

                println!("LD A, C");
            },
            0x7A => {
                // LD A,D
                self.load_reg_reg(A, D);

                println!("LD A, D");
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
            0x7D => {
                // LD A,L
                self.load_reg_reg(A, L);

                println!("LD A, L");
            },
            0x7E => {
                // LD A,(HL)
                self.load_reg_hl(A);

                println!("LD A, (HL)");
            },
            0x86 => {
                // ADD (HL)
                let addr = self.regs.read(HL);
                let n = self.memory[addr];
                let (result, flags) = add_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 1;

                println!("ADD (HL)");
            },
            0x90 => {
                // SUB B
                let (result, flags) = subtract_u8(self.regs[A], self.regs[B]);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 1;

                println!("SUB B");
            },
            0xA9 => {
                // XOR C
                self.xor_reg(C);

                println!("XOR C");
            },
            0xAD => {
                // XOR L
                self.xor_reg(L);

                println!("XOR L");
            },
            0xAE => {
                // XOR (HL)
                let addr = self.regs.read(HL);
                let n = self.memory[addr];
                let (result, flags) = xor_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 1;

                println!("XOR (HL)");
            },
            0xAF => {
                // XOR A
                self.xor_reg(A);

                println!("XOR A");
            },
            0xB0 => {
                // OR B
                self.or_reg(B);

                println!("OR B");
            },
            0xB1 => {
                // OR C
                self.or_reg(C);

                println!("OR C");
            },
            0xB6 => {
                // OR (HL)
                let addr = self.regs.read(HL);
                let n = self.memory[addr];
                let (result, flags) = or_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 1;

                println!("OR (HL)");
            },
            0xB7 => {
                // OR A
                self.or_reg(A);

                println!("OR A");
            },
            0xBB => {
                // CP E
                let (_, flags) = subtract_u8(self.regs[A], self.regs[E]);
                self.regs.write_flags(flags);
                self.pc += 1;

                println!("CP E");
            },
            0xBE => {
                // CP (HL)
                let addr = self.regs.read(HL);
                let n = self.memory[addr];
                let (_, flags) = subtract_u8(self.regs[A], n);
                self.regs.write_flags(flags);
                self.pc += 1;

                println!("CP (HL)");
            },
            0xC1 => {
                // POP BC
                self.pop(BC);

                println!("POP BC");
            },
            0xC2 => {
                // JP NZ,nn
                let addr = self.read_u16(self.pc + 1);

                if !read_bit(self.regs[F], ZERO_FLAG) {
                    self.pc = addr;
                } else {
                    self.pc += 3;
                }

                println!("JP NZ, {:04x}", addr);
            },
            0xC3 => {
                // JP nn
                let addr = self.read_u16(self.pc + 1);
                self.pc = addr;

                println!("JP {:04x}", addr);
            },
            0xC4 => {
                // CALL NZ,nn
                let nn = self.read_u16(self.pc + 1);
                self.pc += 3;

                if !read_bit(self.regs[F], ZERO_FLAG) {
                    self.sp -= 2;
                    self.write_u16(self.sp, self.pc);
                    self.pc = nn;
                }

                println!("CALL NZ, {:04x}", nn);
            },
            0xC5 => {
                // PUSH BC
                self.push(BC);

                println!("PUSH BC");
            },
            0xC6 => {
                // ADD n
                let n = self.memory[self.pc + 1];
                let (result, flags) = add_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("ADD {:02x}", n);
            },
            0xC8 => {
                // RET Z
                self.ret_condition(read_bit(self.regs[F], ZERO_FLAG));

                println!("RET Z");
            },
            0xC9 => {
                // RET
                self.ret();

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
            0xCE => {
                // ADC n
                let n = self.memory[self.pc + 1];
                let old_carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, old_flags) = add_u8(self.regs[A], n);
                let (result, mut flags) = add_u8(result, old_carry as u8);
                flags.half_carry |= old_flags.half_carry;
                flags.carry |= old_flags.carry;
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("ADC {:02x}", n);
            },
            0xD0 => {
                // RET NC
                self.ret_condition(!read_bit(self.regs[F], CARRY_FLAG));

                println!("RET NC");
            },
            0xD1 => {
                // POP DE
                self.pop(DE);

                println!("POP DE");
            },
            0xD5 => {
                // PUSH DE
                self.push(DE);

                println!("PUSH DE");
            },
            0xD6 => {
                // SUB n
                let n = self.memory[self.pc + 1];
                let (result, flags) = subtract_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("SUB {:02x}", n);
            },
            0xD8 => {
                // RET C
                self.ret_condition(read_bit(self.regs[F], CARRY_FLAG));

                println!("RET C");
            },
            0xE0 => {
                // LDH (n),A
                let offset = self.memory[self.pc + 1];
                let addr = 0xFF00 + (offset as u16);
                self.memory[addr] = self.regs[A];
                self.pc += 2;

                println!("LDH ({:02x}), A", offset);
            },
            0xE1 => {
                // POP HL
                self.pop(HL);

                println!("POP HL");
            },
            0xE2 => {
                // LDH (C),A
                let addr = 0xFF00 + (self.regs[C] as u16);
                self.memory[addr] = self.regs[A];
                self.pc += 1;

                println!("LDH (C), A");
            },
            0xE9 => {
                // JP (HL)
                let addr = self.regs.read(HL);
                self.pc = addr;

                println!("JP (HL)");
            },
            0xEA => {
                // LD (nn),A
                let nn = self.read_u16(self.pc + 1);
                self.memory[nn] = self.regs[A];
                self.pc += 3;

                println!("LD ({:04x}), A", nn);
            },
            0xE5 => {
                // PUSH HL
                self.push(HL);

                println!("PUSH HL");
            },
            0xE6 => {
                // AND n
                let n = self.memory[self.pc + 1];
                self.regs[A] &= n;
                self.reset_flags();
                self.set_flag(ZERO_FLAG, self.regs[A] == 0);
                self.set_flag(HALF_CARRY_FLAG, true);
                self.pc += 2;

                println!("AND {:02x}", n);
            },
            0xEE => {
                // XOR n
                let n = self.memory[self.pc + 1];
                let (result, flags) = xor_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("XOR {:02x}", n);
            },
            0xF0 => {
                // LDH A,(n)
                let offset = self.memory[self.pc + 1];
                let addr = 0xFF00 + (offset as u16);
                self.regs[A] = self.memory[addr];
                self.pc += 2;

                println!("LDH A, ({:02x})", offset);
            },
            0xF1 => {
                // POP AF
                self.pop(AF);

                println!("POP AF");
            },
            0xF3 => {
                // DI
                self.ime = false;
                self.pc += 1;

                println!("DI");
            },
            0xF5 => {
                // PUSH AF
                self.push(AF);

                println!("PUSH AF");
            },
            0xFA => {
                // LD A,(nn)
                let nn = self.read_u16(self.pc + 1);
                self.regs[A] = self.memory[nn];
                self.pc += 3;

                println!("LD A, ({:04x})", nn);
            },
            0xFE => {
                // CP n
                let n = self.memory[self.pc + 1];
                let (_, flags) = subtract_u8(self.regs[A], n);
                self.regs.write_flags(flags);
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
        self.memory.load_bootrom(buffer);
    }

    pub fn load_rom(&mut self, buffer: &[u8]) {
        self.memory.load_rom(buffer);
    }

    fn execute_prefixed_instruction(&mut self) {
        self.pc += 1;
        let opcode = self.memory[self.pc];

        match opcode {
            0x11 => {
                // RL C
                let carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, carry) = rotate_left_through_carry(self.regs[C], carry);
                self.regs[C] = result;
                self.regs.write_flags(Flags {
                    zero: result == 0,
                    carry,
                    ..Flags::default()
                });
                self.pc += 1;

                println!("RL C");
            },
            0x19 => {
                // RR C
                self.rr_reg(C);

                println!("RR C");
            },
            0x1A => {
                // RR D
                self.rr_reg(D);

                println!("RR D");
            },
            0x1B => {
                // RR E
                self.rr_reg(E);

                println!("RR E");
            },
            0x38 => {
                // SRL B
                let (result, carry) = shift_right_logical(self.regs[B]);
                self.regs[B] = result;
                self.regs.write_flags(Flags {
                    zero: result == 0,
                    carry,
                    ..Flags::default()
                });
                self.pc += 1;

                println!("SRL B");
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

    fn xor_reg(&mut self, index: RegisterIndex) {
        let (result, flags) = xor_u8(self.regs[A], self.regs[index]);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;
    }

    fn or_reg(&mut self, index: RegisterIndex) {
        let (result, flags) = or_u8(self.regs[A], self.regs[index]);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;
    }

    fn inc_rr(&mut self, index: TwoRegisterIndex) {
        self.regs.write(index, self.regs.read(index) + 1);
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
        let n = self.memory[self.pc + 1];
        self.regs[index] = n;
        self.pc += 2;

        n
    }

    fn load_reg_hl(&mut self, index: RegisterIndex) {
        let addr = self.regs.read(HL);
        self.regs[index] = self.memory[addr];
        self.pc += 1;
    }

    fn load_hl_reg(&mut self, index: RegisterIndex) {
        let addr = self.regs.read(HL);
        self.memory[addr] = self.regs[index];
        self.pc += 1;
    }

    fn load_rr_nn(&mut self, index: TwoRegisterIndex) -> u16 {
        let nn = self.read_u16(self.pc + 1);
        self.regs.write(index, nn);
        self.pc += 3;

        nn
    }

    fn load_reg_reg(&mut self, dest: RegisterIndex, source: RegisterIndex) {
        self.regs[dest] = self.regs[source];
        self.pc += 1;
    }

    fn rr_reg(&mut self, index: RegisterIndex) {
        // RR r
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, carry) = rotate_right_through_carry(self.regs[index], carry);
        self.regs[index] = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;
    }

    fn jump_rel(&mut self) -> i8 {
        // JR n
        self.jump_rel_condition(true)
    }

    fn jump_rel_condition(&mut self, condition: bool) -> i8 {
        // JR cc, n
        // Offset is signed
        let offset = self.memory[self.pc + 1] as i8;
        self.pc += 2;

        if condition {
            self.pc = ((self.pc as i32) + (offset as i32)) as u16;
        }

        offset
    }

    fn ret(&mut self) {
        self.ret_condition(true);
    }

    fn ret_condition(&mut self, condition: bool) {
        // RET cc
        if condition {
            self.pc = self.read_u16(self.sp);
            self.sp += 2;
        } else {
            self.pc += 1;
        }
    }

    fn push(&mut self, index: TwoRegisterIndex) {
        self.sp -= 2;
        self.write_u16(self.sp, self.regs.read(index));
        self.pc += 1;
    }

    fn pop(&mut self, index: TwoRegisterIndex) {
        self.regs.write(index, self.read_u16(self.sp));
        self.sp += 2;
        self.pc += 1;
    }

    fn read_u16(&self, addr: u16) -> u16 {
        let lsb = self.memory[addr] as u16;
        let msb = self.memory[addr + 1] as u16;
        (msb << 8) | lsb
    }

    fn write_u16(&mut self, addr: u16, val: u16) {
        let lsb = (val & 0x00FF) as u8;
        let msb = ((val & 0xFF00) >> 8) as u8;
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

fn rotate_right_through_carry(val: u8, carry: bool) -> (u8, bool) {
    let new_carry = read_bit(val, 0);
    let new_val = set_bit(val >> 1, 7, carry);
    (new_val, new_carry)
}

fn shift_right_logical(val: u8) -> (u8, bool) {
    let carry = read_bit(val, 0);
    let new_val = val >> 1;
    (new_val, carry)
}

fn subtract_u8(x: u8, y: u8) -> (u8, Flags) {
    let result = x - y;

    let flags = Flags {
        zero: result == 0,
        subtract: true,
        half_carry: (x & 0xF) < (y & 0xF),
        carry: x < y,
    };

    (result, flags)
}

fn add_u8(x: u8, y: u8) -> (u8, Flags) {
    let result = x + y;

    let flags = Flags {
        zero: result == 0,
        subtract: false,
        half_carry: (x & 0xF) + (y & 0xF) > 0xF,
        carry: (x as u32) + (y as u32) > 0xFF,
    };

    (result, flags)
}

fn add_u16(x: u16, y: u16) -> (u16, Flags) {
    let result = x + y;

    let flags = Flags {
        zero: result == 0,
        subtract: false,
        half_carry: (x & 0x0FFF) + (y & 0x0FFF) > 0x0FFF,
        carry: (x as u32) + (y as u32) > 0xFFFF,
    };

    (result, flags)
}

fn or_u8(x: u8, y: u8) -> (u8, Flags) {
    let result = x | y;

    let flags = Flags {
        zero: result == 0,
        ..Flags::default()
    };

    (result, flags)
}

fn xor_u8(x: u8, y: u8) -> (u8, Flags) {
    let result = x ^ y;

    let flags = Flags {
        zero: result == 0,
        ..Flags::default()
    };

    (result, flags)
}
