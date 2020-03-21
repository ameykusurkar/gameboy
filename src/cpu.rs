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
            0x01 | 0x11 | 0x21=> {
                self.execute_load_rr_nn(opcode);
            },
            0x03 | 0x13 | 0x23 => {
                self.execute_inc_rr(opcode);
            },
            0x04 => {
                // INC B
                self.inc_reg(B);

                println!("INC B");
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
            0x07 => {
                // RLCA
                let (result, carry) = rotate_left(self.regs[A]);
                self.regs[A] = result;
                // Unlike RLC X, zero flag is RESET
                self.regs.write_flags(Flags {
                    carry,
                    ..Flags::default()
                });
                self.pc += 1;

                println!("RLCA");
            },
            0x08 => {
                // LD (nn),SP
                let nn = self.read_u16(self.pc + 1);
                self.write_u16(nn, self.sp);
                self.pc += 3;

                println!("LD ({:04x}), SP", nn);
            },
            0x09 | 0x19 | 0x29 => {
                self.execute_add_rr(opcode);
            },
            0x0B | 0x1B | 0x2B => {
                self.execute_dec_rr(opcode);
            },
            0x0C => {
                // INC C
                self.inc_reg(C);

                println!("INC C");
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
            0x0F => {
                // RRCA
                let (result, carry) = rotate_right(self.regs[A]);
                self.regs[A] = result;
                // Unlike RRC X, zero flag is RESET
                self.regs.write_flags(Flags {
                    carry,
                    ..Flags::default()
                });
                self.pc += 1;

                println!("RRCA");
            },
            0x12 => {
                // LD (DE), A
                let addr = self.regs.read(DE);
                self.memory[addr] = self.regs[A];
                self.pc += 1;

                println!("LD (DE), A");
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
                // Unlike RR X, zero flag is RESET
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
            0x22 => {
                // LD (HL+),A
                let addr = self.regs.read(HL);
                self.memory[addr] = self.regs[A];
                self.regs.write(HL, addr + 1);
                self.pc += 1;

                println!("LD (HL+), A");
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
            0x2F => {
                // CPL
                self.regs[A] ^= 0xFF;
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, true);
                self.pc += 1;

                println!("CPL");
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
            0x36 => {
                // LD (HL),n
                let n = self.memory[self.pc + 1];
                self.memory[self.regs.read(HL)] = n;
                self.pc += 2;

                println!("LD (HL), {:02x}", n);
            },
            0x37 => {
                // SCF
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, false);
                self.set_flag(CARRY_FLAG, true);
                self.pc += 1;

                println!("SCF");
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
            0x3F => {
                // CCF
                let old_carry = read_bit(self.regs[F], CARRY_FLAG);
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, false);
                self.set_flag(CARRY_FLAG, old_carry ^ true);
                self.pc += 1;

                println!("CCF");
            },
            0x40..=0x7F => {
                self.execute_load_r_r(opcode);
            },
            0x80..=0x87 => {
                self.execute_add_reg(opcode);
            }
            0x88..=0x8F => {
                self.execute_adc_reg(opcode);
            },
            0x90..=0x97 => {
                self.execute_sub_reg(opcode);
            },
            0x98..=0x9F => {
                self.execute_sbc_reg(opcode);
            },
            0xA0..=0xA7 => {
                self.execute_and_reg(opcode);
            },
            0xA8..=0xAF => {
                self.execute_xor_reg(opcode);
            },
            0xB0..=0xB7 => {
                self.execute_or_reg(opcode);
            },
            0xB8..=0xBF => {
                self.execute_cp_reg(opcode);
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
                let (result, flags) = adc_u8(self.regs[A], n, old_carry);
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
                let (result, flags) = sub_u8(self.regs[A], n);
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
            0xDE => {
                // SBC n
                let n = self.memory[self.pc + 1];
                let old_carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, flags) = sbc_u8(self.regs[A], n, old_carry);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("SBC {:02x}", n);
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
                let (result, flags) = and_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
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
            0xF2 => {
                // LDH A,(C)
                let addr = 0xFF00 + (self.regs[C] as u16);
                self.regs[A] = self.memory[addr];
                self.pc += 1;

                println!("LDH A, (C)");
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
            0xF6 => {
                // OR n
                let n = self.memory[self.pc + 1];
                let (result, flags) = or_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("OR {:02x}", n);
            },
            0xF9 => {
                // LD SP,HL
                self.sp = self.regs.read(HL);
                self.pc += 1;

                println!("LD SP,HL");
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
                let (_, flags) = sub_u8(self.regs[A], n);
                self.regs.write_flags(flags);
                self.pc += 2;

                println!("CP {:02x}", n);
            },
            _ => panic!("Unimplemented opcode {:02x}", opcode),
        }

        println!("{:02x?}, PC: {:#06x}", self.regs, self.pc);

        // if old_pc == self.pc {
        //     panic!("PC is still {:04x}, should have changed!", old_pc);
        // }
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
            0x00..=0x07 => {
                self.execute_rlc_reg(opcode);
            },
            0x08..=0x0F => {
                self.execute_rrc_reg(opcode);
            },
            0x10..=0x17 => {
                self.execute_rl_reg(opcode);
            },
            0x18..=0x1F => {
                self.execute_rr_reg(opcode);
            },
            0x20..=0x27 => {
                self.execute_sla_reg(opcode);
            },
            0x28..=0x2F => {
                self.execute_sra_reg(opcode);
            },
            0x30..=0x37 => {
                self.execute_swap_reg(opcode);
            },
            0x38..=0x3F => {
                self.execute_srl_reg(opcode);
            },
            0x7C => {
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

    fn execute_dec_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        self.regs.write(reg, self.regs.read(reg) - 1);
        self.pc += 1;

        println!("DEC {:?}", reg);
    }

    fn execute_inc_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        self.regs.write(reg, self.regs.read(reg) + 1);
        self.pc += 1;

        println!("INC {:?}", reg);
    }

    fn execute_load_rr_nn(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        let nn = self.read_u16(self.pc + 1);
        self.regs.write(reg, nn);
        self.pc += 3;

        println!("LD {:?}, {:04x}", reg, nn);
    }

    fn execute_add_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        let old_zero = read_bit(self.regs[F], ZERO_FLAG);
        let (result, flags) = add_u16(self.regs.read(HL), self.regs.read(reg));
        self.regs.write(HL, result);
        self.regs.write_flags(Flags { zero: old_zero, ..flags });
        self.pc += 1;

        println!("INC HL, {:?}", reg);
    }

    fn execute_load_r_r(&mut self, opcode: u8) {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        let opcode_index = opcode - 0x40;
        let dst_index = order[(opcode_index / 0x08) as usize];
        let src_index = order[(opcode_index % 0x08) as usize];

        match (dst_index, src_index) {
            (Some(reg), Some(reg1)) => {
                self.load_reg_reg(reg, reg1);
                println!("LD {:?}, {:?}", reg, reg1);
            },
            (None, Some(reg)) => {
                self.load_hl_reg(reg);
                println!("LD (HL), {:?}", reg);
            },
            (Some(reg), None) => {
                self.load_reg_hl(reg);
                println!("LD {:?}, (HL)", reg);
            },
            (None, None) => {
                panic!("Cannot handle {:04x} here!", opcode);
            },
        }
    }

    fn execute_or_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let (result, flags) = or_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("OR {}", display);
    }

    fn execute_cp_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let (_, flags) = sub_u8(self.regs[A], src);
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("CP {}", display);
    }

    fn execute_add_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let (result, flags) = add_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("ADD {}", display);
    }

    fn execute_adc_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, flags) = adc_u8(self.regs[A], src, old_carry);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("ADC {}", display);
    }

    fn execute_sub_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let (result, flags) = sub_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("SUB {}", display);
    }

    fn execute_sbc_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, flags) = sbc_u8(self.regs[A], src, old_carry);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("SBC {}", display);
    }

    fn execute_and_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let (result, flags) = and_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("AND {}", display);
    }

    fn execute_xor_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_val(opcode);
        let (result, flags) = xor_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("XOR {}", display);
    }

    fn execute_rlc_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = rotate_left(*src);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("RLC {}", display);
    }

    fn execute_rrc_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = rotate_right(*src);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("RRC {}", display);
    }

    fn execute_rl_reg(&mut self, opcode: u8) {
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = rotate_left_through_carry(*src, carry);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("RL {}", display);
    }

    fn execute_rr_reg(&mut self, opcode: u8) {
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = rotate_right_through_carry(*src, carry);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("RR {}", display);
    }

    fn execute_sla_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = shift_left(*src);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("SLA {}", display);
    }

    fn execute_sra_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = shift_right_arithmetic(*src);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("SRA {}", display);
    }

    fn execute_swap_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, flags) = swap_u8(*src);
        *src = result;
        self.regs.write_flags(flags);
        self.pc += 1;

        println!("SWAP {}", display);
    }

    fn execute_srl_reg(&mut self, opcode: u8) {
        let (src, display) = self.get_source_reg_mut(opcode);
        let (result, carry) = shift_right_logical(*src);
        *src = result;
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
        self.pc += 1;

        println!("SRL {}", display);
    }

    // Many arithmetic/logical opcodes are arranged a particular order,
    // eg. ADD r, where consecutive opcodes iterate through the different
    // registers (r). There is one caveat to this: these operations treat
    // the memory address pointed to by HL like a register as well, even
    // though it is not really a register, but memory. This method hides
    // that detail so that register operations can't tell the difference.
    fn get_source_val(&self, opcode: u8) -> (u8, String) {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        match order[(opcode % 0x08) as usize] {
            Some(reg) => {
                (self.regs[reg], format!("{:?}", reg))
            },
            None => {
                let addr = self.regs.read(HL);
                (self.memory[addr], "(HL)".to_string())
            }
        }
    }
    
    fn get_source_reg_mut(&mut self, opcode: u8) -> (&mut u8, String) {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        match order[(opcode % 0x08) as usize] {
            Some(reg) => {
                (&mut self.regs[reg], format!("{:?}", reg))
            },
            None => {
                let addr = self.regs.read(HL);
                (&mut self.memory[addr], "(HL)".to_string())
            }
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

fn rotate_left(val: u8) -> (u8, bool) {
    let carry = read_bit(val, 7);
    let new_val = set_bit(val << 1, 0, carry);
    (new_val, carry)
}

fn rotate_right(val: u8) -> (u8, bool) {
    let carry = read_bit(val, 0);
    let new_val = set_bit(val >> 1, 7, carry);
    (new_val, carry)
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

fn shift_left(val: u8) -> (u8, bool) {
    let carry = read_bit(val, 7);
    let new_val = val << 1;
    (new_val, carry)
}

fn shift_right_arithmetic(val: u8) -> (u8, bool) {
    let carry = read_bit(val, 0);
    let new_val = ((val as i8) >> 1) as u8;
    (new_val, carry)
}

fn shift_right_logical(val: u8) -> (u8, bool) {
    let carry = read_bit(val, 0);
    let new_val = val >> 1;
    (new_val, carry)
}

fn sub_u8(x: u8, y: u8) -> (u8, Flags) {
    let result = x - y;

    let flags = Flags {
        zero: result == 0,
        subtract: true,
        half_carry: (x & 0xF) < (y & 0xF),
        carry: x < y,
    };

    (result, flags)
}

fn sbc_u8(x: u8, y: u8, carry: bool) -> (u8, Flags) {
    let (result, old_flags) = sub_u8(x, y);
    let (result, mut flags) = sub_u8(result, carry as u8);
    flags.half_carry |= old_flags.half_carry;
    flags.carry |= old_flags.carry;

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

fn adc_u8(x: u8, y: u8, carry: bool) -> (u8, Flags) {
    let (result, old_flags) = add_u8(x, y);
    let (result, mut flags) = add_u8(result, carry as u8);
    flags.half_carry |= old_flags.half_carry;
    flags.carry |= old_flags.carry;

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

fn and_u8(x: u8, y: u8) -> (u8, Flags) {
    let result = x & y;

    let flags = Flags {
        zero: result == 0,
        half_carry: true,
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

fn swap_u8(x: u8) -> (u8, Flags) {
    let result = (x & 0x0F) << 4 | (x & 0xF0) >> 4;

    let flags = Flags {
        zero: result == 0,
        ..Flags::default()
    };

    (result, flags)
}
