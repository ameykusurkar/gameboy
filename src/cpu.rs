use crate::registers::Registers;
use crate::registers::RegisterIndex;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex;
use crate::registers::TwoRegisterIndex::*;
use crate::registers::Flags;
use crate::registers::{ZERO_FLAG, SUBTRACT_FLAG, HALF_CARRY_FLAG, CARRY_FLAG};

use crate::memory::Memory;
use crate::instruction::{Instruction, AddressingMode};
use crate::instruction::{INSTRUCTIONS, PREFIXED_INSTRUCTIONS};
use crate::instruction::CycleCount::*;

use crate::emulator::DEBUG;

// Address of the interrupt enable register
const IE_ADDR: u16 = 0xFFFF;
// Address of the interrupt flags register
pub const IF_ADDR: u16 = 0xFF0F;
// Address the cpu jumps to when the respective interrupts are triggered
const INTERRUPT_ADDRS: [u16; 5] = [0x40, 0x48, 0x50, 0x58, 0x60];

// Address for the divider register (DIV). The incremented at 16,384 Hx.
const DIV_ADDR: u16 = 0xFF04;
// Address for the timer register (TIMA). The increment frequency is specified in the TAC register.
const TIMA_ADDR: u16 = 0xFF05;
// When TIMA overflows, the data at this address will be loaded.
const TMA_ADDR: u16  = 0xFF06;
// Address for the timer control register. Bit 2 specifies if the timer is active, and bits 1-0
// specify the frequency at which to increment the timer.
const TAC_ADDR: u16  = 0xFF07;

pub struct Cpu {
    pub regs: Registers,
    pub sp: u16,
    pub pc: u16,
    ime: bool,
    remaining_cycles: u32,
    total_clock_cycles: u32,
    halted: bool,
    pub current_instruction: &'static Instruction<'static>,
}

#[derive(Copy, Clone)]
struct RegisterHL;

struct Immediate8;

trait Operand8<T> {
    fn read_oper(&mut self, memory: &Memory, src: T) -> u8;
    fn write_oper(&mut self, memory: &mut Memory, src: T, val: u8);
}

impl Operand8<RegisterIndex> for Cpu {
    fn read_oper(&mut self, _memory: &Memory, src: RegisterIndex) -> u8 {
        self.regs[src]
    }

    fn write_oper(&mut self, _memory: &mut Memory, src: RegisterIndex, val: u8) {
        self.regs[src] = val;
    }
}

impl Operand8<RegisterHL> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: RegisterHL) -> u8 {
        let addr = self.regs.read(HL);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, _src: RegisterHL, val: u8) {
        let addr = self.regs.read(HL);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<Immediate8> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: Immediate8) -> u8 {
        let val = memory.cpu_read(self.pc);
        self.pc += 1;
        val
    }

    fn write_oper(&mut self, _memory: &mut Memory, _src: Immediate8, _val: u8) {
        // Do nothing for now
    }
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            regs: Registers::default(),
            sp: 0,
            pc: 0,
            ime: false,
            remaining_cycles: 0,
            total_clock_cycles: 0,
            halted: false,
            // This should get populated when cpu starts running
            current_instruction: &INSTRUCTIONS[0],
        }
    }

    #[allow(dead_code)]
    pub fn skip_bootrom(&mut self, memory: &mut Memory) {
        self.pc = 0x100;
        self.sp = 0xFFFE;
        memory.cpu_write(0xFF50, 1);
    }

    pub fn step(&mut self, memory: &mut Memory) {
        if self.halted && self.get_pending_interrupts(memory) > 0 {
            self.halted = false;
        }

        if !self.halted {
            if self.remaining_cycles == 0 {
                let interrupt_was_serviced = self.check_and_handle_interrupts(memory);

                if interrupt_was_serviced {
                    self.remaining_cycles += 5;
                }

                self.current_instruction = self.fetch_instruction(memory, self.pc);

                self.remaining_cycles += self.cycles_for_instruction(self.current_instruction);

                // For instructions with a conditional jump, the cpu takes extra cycles if
                // the jump does happen, which `execute` determines based on the condition.
                let extra_cycles = self.execute(memory);
                self.remaining_cycles += extra_cycles;
            }

            self.remaining_cycles -= 1;
        }

        self.total_clock_cycles += 1;
        self.update_timers(memory);
    }

    fn fetch_instruction(&self, memory: &Memory, addr: u16) -> &'static Instruction<'static> {
        let opcode = memory.cpu_read(addr);

        if opcode == 0xCB {
            let next_byte = memory.cpu_read(addr + 1);
            &PREFIXED_INSTRUCTIONS[next_byte as usize]
        } else {
            &INSTRUCTIONS[opcode as usize]
        }
    }

    // Finds the number of cycles required for the given instruction. If the instruction
    // is a conditional jump, this does not take into account the extra cycles required
    // for the jump: that is determined by the `execute` method.
    fn cycles_for_instruction(&self, instruction: &Instruction) -> u32 {
        match instruction.cycles {
            Fixed(cycles) => cycles,
            Jump(_, cycles_without_jump) => cycles_without_jump,
        }
    }

    fn update_timers(&mut self, memory: &mut Memory) {
        if self.total_clock_cycles % 64 == 0 {
            let div = memory.cpu_read(DIV_ADDR);
            memory.cpu_write(DIV_ADDR, div + 1);
        }

        let timer_control = memory.cpu_read(TAC_ADDR);
        let timer_is_active = read_bit(timer_control, 2);

        if timer_is_active {
            let cycles_per_update = match timer_control & 0b11 {
                0b00 => 256,        // 4096 Hz
                0b01 => 4,          // 262,144 Hz
                0b10 => 16,         // 65,536 Hz
                0b11..=0xFF => 64,  // 16,384 Hz
            };

            if self.total_clock_cycles % cycles_per_update == 0 {
                if memory.cpu_read(TIMA_ADDR) == 0xFF {
                    // If the timer is going to overflow, request a timer interrupt
                    let flags = memory.cpu_read(IF_ADDR);
                    memory.cpu_write(IF_ADDR, flags | 1 << 2);
                    memory.cpu_write(TIMA_ADDR, memory.cpu_read(TMA_ADDR));
                } else {
                    let tima = memory.cpu_read(TIMA_ADDR);
                    memory.cpu_write(TIMA_ADDR, tima + 1);
                }
            }
        }
    }

    pub fn execute(&mut self, memory: &mut Memory) -> u32 {
        let opcode = memory.cpu_read(self.pc);

        let mut extra_cycles = 0;

        self.pc += 1;

        if DEBUG {
            println!("{:#06x}: {}", self.pc - 1, self.build_instruction_repr(memory, self.pc, self.current_instruction));
        }

        match opcode {
            0x00 => (),
            0x01 | 0x11 | 0x21=> {
                self.execute_load_rr_nn(memory, opcode);
            },
            0x02 => {
                // LD (BC), A
                let addr = self.regs.read(BC);
                memory.cpu_write(addr, self.regs[A]);
            },
            0x03 | 0x13 | 0x23 => {
                self.execute_inc_rr(opcode);
            },
            0x04 => {
                // INC B
                self.inc_reg(B);
            },
            0x05 => {
                // DEC B
                self.dec_reg(B);
            },
            0x06 => {
                // LD B,n
                self.load_reg_byte(memory, B);
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
            },
            0x08 => {
                // LD (nn),SP
                let nn = self.read_imm_u16(memory);
                Self::write_mem_u16(memory, nn, self.sp);
            },
            0x09 | 0x19 | 0x29 => {
                self.execute_add_rr(opcode);
            },
            0x0A => {
                // LD A,(BC)
                let addr = self.regs.read(BC);
                self.regs[A] = memory.cpu_read(addr);
            },
            0x0B | 0x1B | 0x2B => {
                self.execute_dec_rr(opcode);
            },
            0x0C => {
                // INC C
                self.inc_reg(C);
            },
            0x0D => {
                // DEC C
                self.dec_reg(C);
            },
            0x0E => {
                // LD C,n
                self.load_reg_byte(memory, C);
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
            },
            0x12 => {
                // LD (DE), A
                let addr = self.regs.read(DE);
                memory.cpu_write(addr, self.regs[A]);
            },
            0x14 => {
                // INC D
                self.inc_reg(D);
            },
            0x15 => {
                // DEC D
                self.dec_reg(D);
            },
            0x16 => {
                // LD D,n
                self.load_reg_byte(memory, D);
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
            },
            0x18 => {
                // JR n
                self.jump_rel(memory);
            },
            0x1A => {
                // LD A,(DE)
                let addr = self.regs.read(DE);
                self.regs[A] = memory.cpu_read(addr);
            },
            0x1C => {
                // INC E
                self.inc_reg(E);
            },
            0x1D => {
                // DEC E
                self.dec_reg(E);
            },
            0x1E => {
                // LD E,n
                self.load_reg_byte(memory, E);
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
            },
            0x20 => {
                // JR NZ,n
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_rel_condition(memory, condition);
            },
            0x22 => {
                // LD (HL+),A
                let addr = self.regs.read(HL);
                memory.cpu_write(addr, self.regs[A]);
                self.regs.write(HL, addr + 1);
            },
            0x24 => {
                // INC H
                self.inc_reg(H);
            },
            0x25 => {
                // DEC H
                self.dec_reg(H);
            },
            0x26 => {
                // LD H,n
                self.load_reg_byte(memory, H);
            },
            0x27 => {
                // DAA
                let val = self.regs[A];
                let mut correction = 0;
                let mut carry = read_bit(self.regs[F], CARRY_FLAG);
                if read_bit(self.regs[F], HALF_CARRY_FLAG) || (!read_bit(self.regs[F], SUBTRACT_FLAG) && ((val & 0x0F) > 0x09)) {
                    correction |= 0x06;
                }

                if read_bit(self.regs[F], CARRY_FLAG) || (!read_bit(self.regs[F], SUBTRACT_FLAG) && (val > 0x99)) {
                    correction |= 0x60;
                    carry = true;
                }

                if read_bit(self.regs[F], SUBTRACT_FLAG) {
                    self.regs[A] -= correction;
                } else {
                    self.regs[A] += correction;
                }

                self.set_flag(ZERO_FLAG, self.regs[A] == 0);
                self.set_flag(HALF_CARRY_FLAG, false);
                self.set_flag(CARRY_FLAG, carry);
            },
            0x28 => {
                // JR Z,n
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_rel_condition(memory, condition);
            },
            0x2A => {
                // LD A, (HL+)
                let addr = self.regs.read(HL);
                self.regs[A] = memory.cpu_read(addr);
                self.regs.write(HL, addr + 1);
            },
            0x2C => {
                // INC L
                self.inc_reg(L);
            },
            0x2D => {
                // DEC L
                self.dec_reg(L);
            },
            0x2E => {
                // LD L,n
                self.load_reg_byte(memory, L);
            },
            0x2F => {
                // CPL
                self.regs[A] ^= 0xFF;
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, true);
            },
            0x30 => {
                // JR NC,n
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_rel_condition(memory, condition);
            },
            0x31 => {
                // LD SP,nn
                let nn = self.read_imm_u16(memory);
                self.sp = nn;
            },
            0x32 => {
                // LD (HL-), A
                let addr = self.regs.read(HL);
                memory.cpu_write(addr, self.regs[A]);
                self.regs.write(HL, addr - 1);
            },
            0x33 => {
                // INC SP
                self.sp += 1;
            },
            0x34 => {
                // INC (HL)
                let addr = self.regs.read(HL);
                let old = memory.cpu_read(addr);
                memory.cpu_write(addr, old + 1);
                self.set_flag(ZERO_FLAG, (old + 1) == 0);
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0xF);
            },
            0x35 => {
                // DEC (HL)
                let addr = self.regs.read(HL);
                let old = memory.cpu_read(addr);
                memory.cpu_write(addr, old - 1);
                self.set_flag(ZERO_FLAG, (old - 1) == 0);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
            },
            0x36 => {
                // LD (HL),n
                let n = self.read_oper(memory, Immediate8);
                memory.cpu_write(self.regs.read(HL), n);
            },
            0x37 => {
                // SCF
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, false);
                self.set_flag(CARRY_FLAG, true);
            },
            0x38 => {
                // JR C,n
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_rel_condition(memory, condition);
            },
            0x39 => {
                // ADD HL,SP
                let old_zero = read_bit(self.regs[F], ZERO_FLAG);
                let (result, flags) = add_u16(self.regs.read(HL), self.sp);
                self.regs.write(HL, result);
                self.regs.write_flags(Flags { zero: old_zero, ..flags });
            },
            0x3A => {
                // LD A, (HL-)
                let addr = self.regs.read(HL);
                self.regs[A] = memory.cpu_read(addr);
                self.regs.write(HL, addr - 1);
            },
            0x3B => {
                // DEC SP
                self.sp -= 1;
            },
            0x3C => {
                // INC A
                self.inc_reg(A);
            },
            0x3D => {
                // DEC A
                self.dec_reg(A);
            },
            0x3E => {
                // LD A,n
                let n = self.read_oper(memory, Immediate8);
                self.regs[A] = n;
            },
            0x3F => {
                // CCF
                let old_carry = read_bit(self.regs[F], CARRY_FLAG);
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, false);
                self.set_flag(CARRY_FLAG, old_carry ^ true);
            },
            0x40..=0x75 | 0x77..=0x7F => {
                self.execute_load_r_r(memory, opcode);
            },
            0x76 => {
                // HALT
                self.halted = true;
            },

            0x80 => self.execute_add(memory, B),
            0x81 => self.execute_add(memory, C),
            0x82 => self.execute_add(memory, D),
            0x83 => self.execute_add(memory, E),
            0x84 => self.execute_add(memory, H),
            0x85 => self.execute_add(memory, L),
            0x86 => self.execute_add(memory, RegisterHL),
            0x87 => self.execute_add(memory, A),

            0x88 => self.execute_adc(memory, B),
            0x89 => self.execute_adc(memory, C),
            0x8A => self.execute_adc(memory, D),
            0x8B => self.execute_adc(memory, E),
            0x8C => self.execute_adc(memory, H),
            0x8D => self.execute_adc(memory, L),
            0x8E => self.execute_adc(memory, RegisterHL),
            0x8F => self.execute_adc(memory, A),

            0x90 => self.execute_sub(memory, B),
            0x91 => self.execute_sub(memory, C),
            0x92 => self.execute_sub(memory, D),
            0x93 => self.execute_sub(memory, E),
            0x94 => self.execute_sub(memory, H),
            0x95 => self.execute_sub(memory, L),
            0x96 => self.execute_sub(memory, RegisterHL),
            0x97 => self.execute_sub(memory, A),

            0x98 => self.execute_sbc(memory, B),
            0x99 => self.execute_sbc(memory, C),
            0x9A => self.execute_sbc(memory, D),
            0x9B => self.execute_sbc(memory, E),
            0x9C => self.execute_sbc(memory, H),
            0x9D => self.execute_sbc(memory, L),
            0x9E => self.execute_sbc(memory, RegisterHL),
            0x9F => self.execute_sbc(memory, A),

            0xA0 => self.execute_and(memory, B),
            0xA1 => self.execute_and(memory, C),
            0xA2 => self.execute_and(memory, D),
            0xA3 => self.execute_and(memory, E),
            0xA4 => self.execute_and(memory, H),
            0xA5 => self.execute_and(memory, L),
            0xA6 => self.execute_and(memory, RegisterHL),
            0xA7 => self.execute_and(memory, A),

            0xA8 => self.execute_xor(memory, B),
            0xA9 => self.execute_xor(memory, C),
            0xAA => self.execute_xor(memory, D),
            0xAB => self.execute_xor(memory, E),
            0xAC => self.execute_xor(memory, H),
            0xAD => self.execute_xor(memory, L),
            0xAE => self.execute_xor(memory, RegisterHL),
            0xAF => self.execute_xor(memory, A),

            0xB0 => self.execute_or(memory, B),
            0xB1 => self.execute_or(memory, C),
            0xB2 => self.execute_or(memory, D),
            0xB3 => self.execute_or(memory, E),
            0xB4 => self.execute_or(memory, H),
            0xB5 => self.execute_or(memory, L),
            0xB6 => self.execute_or(memory, RegisterHL),
            0xB7 => self.execute_or(memory, A),

            0xB8 => self.execute_cp(memory, B),
            0xB9 => self.execute_cp(memory, C),
            0xBA => self.execute_cp(memory, D),
            0xBB => self.execute_cp(memory, E),
            0xBC => self.execute_cp(memory, H),
            0xBD => self.execute_cp(memory, L),
            0xBE => self.execute_cp(memory, RegisterHL),
            0xBF => self.execute_cp(memory, A),

            0xC0 => {
                // RET NZ
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(memory, condition);
            },
            0xC1 => {
                // POP BC
                self.pop(memory, BC);
            },
            0xC2 => {
                // JP NZ,nn
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(memory, condition);
            },
            0xC3 => {
                // JP nn
                self.jump(memory);
            },
            0xC4 => {
                // CALL NZ,nn
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(memory, condition);
            },
            0xC5 => {
                // PUSH BC
                self.push(memory, BC);
            },
            0xC6 => {
                self.execute_add(memory, Immediate8);
            },
            0xC7 | 0xD7 | 0xE7 | 0xF7 | 0xCF | 0xDF | 0xEF | 0xFF => {
                self.execute_rst(memory, opcode);
            },
            0xC8 => {
                // RET Z
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(memory, condition);
            },
            0xC9 => {
                // RET
                self.ret(memory);
            },
            0xCA => {
                // JP Z,nn
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(memory, condition);
            },
            0xCB => {
                self.execute_prefixed_instruction(memory);
            },
            0xCC => {
                // CALL Z,nn
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(memory, condition);
            },
            0xCD => {
                // CALL nn
                self.call_condition(memory, true);
            },
            0xCE => {
                self.execute_adc(memory, Immediate8);
            },
            0xD0 => {
                // RET NC
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(memory, condition);
            },
            0xD1 => {
                // POP DE
                self.pop(memory, DE);
            },
            0xD2 => {
                // JP NC,nn
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(memory, condition);
            },
            0xD4 => {
                // CALL NC,nn
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(memory, condition);
            },
            0xD5 => {
                // PUSH DE
                self.push(memory, DE);
            },
            0xD6 => {
                self.execute_sub(memory, Immediate8);
            },
            0xD8 => {
                // RET C
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(memory, condition);
            },
            0xD9 => {
                // RETI
                self.ime = true;
                self.ret(memory);
            },
            0xDA => {
                // JP C,nn
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(memory, condition);
            },
            0xDC => {
                // CALL C,nn
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(memory, condition);
            },
            0xDE => {
                self.execute_sbc(memory, Immediate8);
            },
            0xE0 => {
                // LDH (n),A
                let offset = self.read_oper(memory, Immediate8);
                let addr = 0xFF00 + (offset as u16);
                memory.cpu_write(addr, self.regs[A]);
            },
            0xE1 => {
                // POP HL
                self.pop(memory, HL);
            },
            0xE2 => {
                // LDH (C),A
                let addr = 0xFF00 + (self.regs[C] as u16);
                memory.cpu_write(addr, self.regs[A]);
            },
            0xE9 => {
                // JP (HL)
                let addr = self.regs.read(HL);
                self.pc = addr;
            },
            0xEA => {
                // LD (nn),A
                let nn = self.read_imm_u16(memory);
                memory.cpu_write(nn, self.regs[A]);
            },
            0xE5 => {
                // PUSH HL
                self.push(memory, HL);
            },
            0xE6 => {
                self.execute_and(memory, Immediate8);
            },
            0xE8 => {
                // ADD SP,n
                let n = self.read_oper(memory, Immediate8);
                let (result, flags) = self.sum_sp_n(n);
                self.sp = result;
                self.regs.write_flags(flags);
            },
            0xEE => {
                self.execute_xor(memory, Immediate8);
            },
            0xF0 => {
                // LDH A,(n)
                let offset = self.read_oper(memory, Immediate8);
                let addr = 0xFF00 + (offset as u16);
                self.regs[A] = memory.cpu_read(addr);
            },
            0xF1 => {
                // POP AF
                self.pop(memory, AF);
            },
            0xF2 => {
                // LDH A,(C)
                let addr = 0xFF00 + (self.regs[C] as u16);
                self.regs[A] = memory.cpu_read(addr);
            },
            0xF3 => {
                // DI
                self.ime = false;
            },
            0xF5 => {
                // PUSH AF
                self.push(memory, AF);
            },
            0xF6 => {
                self.execute_or(memory, Immediate8);
            },
            0xF8 => {
                // LD HL, SP + n
                let n = self.read_oper(memory, Immediate8);
                let (result, flags) = self.sum_sp_n(n);
                self.regs.write(HL, result);
                self.regs.write_flags(flags);
            },
            0xF9 => {
                // LD SP,HL
                self.sp = self.regs.read(HL);
            },
            0xFA => {
                // LD A,(nn)
                let nn = self.read_imm_u16(memory);
                self.regs[A] = memory.cpu_read(nn);
            },
            0xFB => {
                // EI
                self.ime = true;
            },
            0xFE => {
                self.execute_cp(memory, Immediate8);
            },
            _ => panic!("Unimplemented opcode {:02x}, {:?}", opcode, self.current_instruction),
        }

        if DEBUG {
            println!("{:02x?}, PC: {:#06x}, Cycles: {}", self.regs, self.pc, self.total_clock_cycles);
        }

        extra_cycles
    }

    pub fn disassemble(&self, memory: &Memory, start_addr: u16, end_addr: u16) -> Vec<(u16, String)> {
        let mut current_addr = start_addr;
        let mut instruction_reprs = Vec::new();

        loop {
            if current_addr > end_addr { break };

            let instruction = self.fetch_instruction(memory, current_addr);
            let num_bytes = instruction.num_bytes as u32;

            if current_addr as u32 + num_bytes > end_addr as u32 { break };

            let operand_start_addr = current_addr + 1 + (instruction.prefixed as u16);
            let repr = self.build_instruction_repr(memory, operand_start_addr, instruction);
            instruction_reprs.push((current_addr, repr));
            current_addr += num_bytes as u16;
        }

        instruction_reprs
    }

    // Depends on `instruction.repr` being in the correct format
    fn build_instruction_repr(&self, memory: &Memory, addr: u16, instruction: &Instruction) -> String {
        let (operand, _) = self.fetch_operand(memory, addr, instruction);
        let mut repr = String::from(instruction.repr);

        match &instruction.addressing_mode {
            AddressingMode::Implied => (),
            AddressingMode::Imm8 => {
                let index = repr.find("d").unwrap_or(repr.len());
                if index == repr.len() {
                    panic!("Cannot find in {:?}, PC: {:04x}, addr: {:04x}", instruction, self.pc, addr);
                }
                repr.replace_range(index..index+2, &format!("{:02x}", operand));
            },
            AddressingMode::Imm16 => {
                let index = repr.find("d").unwrap_or(repr.len());
                if index == repr.len() {
                    panic!("Cannot find in {:?}, PC: {:04x}, addr: {:04x}", instruction, self.pc, addr);
                }
                repr.replace_range(index..index+3, &format!("{:04x}", operand));
            },
            AddressingMode::Addr16 => {
                let index = repr.find("a").unwrap_or(repr.len());
                if index == repr.len() {
                    panic!("Cannot find in {:?}, PC: {:04x}, addr: {:04x}", instruction, self.pc, addr);
                }
                repr.replace_range(index..index+3, &format!("{:04x}", operand));
            },
            AddressingMode::ZeroPageOffset => {
                let index = repr.find("a").unwrap_or(repr.len());
                if index == repr.len() {
                    panic!("Cannot find in {:?}, PC: {:04x}, addr: {:04x}", instruction, self.pc, addr);
                }
                repr.replace_range(index..index+2, &format!("{:04x}", operand));
            },
            AddressingMode::SignedAddrOffset => {
                let index = repr.find("r").unwrap_or(repr.len());
                if index == repr.len() {
                    panic!("Cannot find in {:?}, PC: {:04x}, addr: {:04x}", instruction, self.pc,
                           addr);
                }
                repr.replace_range(index..index+2, &format!("{:04x}", operand));
            },
        };

        repr
    }

    fn fetch_operand(&self, memory: &Memory, addr: u16, instruction: &Instruction) -> (u16, u32) {
        // (operand, bytes_read)
        let lsb = memory.cpu_read(addr) as u16;
        match &instruction.addressing_mode {
            AddressingMode::Implied => (0, 0),
            AddressingMode::Imm8 => (lsb, 0),
            AddressingMode::Imm16 | AddressingMode::Addr16 => {
                let msb = memory.cpu_read(addr + 1) as u16;
                (msb << 8 | lsb, 2)
            },
            AddressingMode::ZeroPageOffset => {
                (0xFF00 | lsb, 1)
            },
            AddressingMode::SignedAddrOffset => {
                let lsb = lsb as i8;
                // The offset assumes that the PC has already been incremented
                let target_addr = ((addr + 1) as i32) + (lsb as i32);
                (target_addr as u16, 1)
            },
        }
    }

    fn check_and_handle_interrupts(&mut self, memory: &mut Memory) -> bool {
        let pending_interrupts = self.get_pending_interrupts(memory);

        if self.ime && pending_interrupts > 0 {
            for i in 0..5 {
                if read_bit(pending_interrupts, i) {
                    self.handle_interrupt(memory, i);
                    return true;
                }
            }
        }

        false
    }

    fn get_pending_interrupts(&self, memory: &Memory) -> u8 {
        memory.cpu_read(IE_ADDR) & memory.cpu_read(IF_ADDR)
    }

    fn handle_interrupt(&mut self, memory: &mut Memory, interrupt_no: u8) {
        // This routine should take 5 machine cycles
        self.ime = false;
        memory.cpu_write(IF_ADDR, set_bit(memory.cpu_read(IF_ADDR), interrupt_no, false));
        self.sp -= 2;
        Self::write_mem_u16(memory, self.sp, self.pc);
        self.pc = INTERRUPT_ADDRS[interrupt_no as usize];
    }

    fn read_imm_u16(&mut self, memory: &Memory) -> u16 {
        let lsb = self.read_oper(memory, Immediate8) as u16;
        let msb = self.read_oper(memory, Immediate8) as u16;
        (msb << 8) | lsb
    }

    fn read_mem_u16(memory: &Memory, addr: u16) -> u16 {
        let lsb = memory.cpu_read(addr) as u16;
        let msb = memory.cpu_read(addr + 1) as u16;
        (msb << 8) | lsb
    }

    fn write_mem_u16(memory: &mut Memory, addr: u16, val: u16) {
        let lsb = (val & 0x00FF) as u8;
        let msb = ((val & 0xFF00) >> 8) as u8;
        memory.cpu_write(addr, lsb);
        memory.cpu_write(addr + 1, msb);
    }

    fn execute_prefixed_instruction(&mut self, memory: &mut Memory) {
        let opcode = self.read_oper(memory, Immediate8);

        match opcode {
            0x00 => self.execute_rlc(memory, B),
            0x01 => self.execute_rlc(memory, C),
            0x02 => self.execute_rlc(memory, D),
            0x03 => self.execute_rlc(memory, E),
            0x04 => self.execute_rlc(memory, H),
            0x05 => self.execute_rlc(memory, L),
            0x06 => self.execute_rlc(memory, RegisterHL),
            0x07 => self.execute_rlc(memory, A),

            0x08 => self.execute_rrc(memory, B),
            0x09 => self.execute_rrc(memory, C),
            0x0A => self.execute_rrc(memory, D),
            0x0B => self.execute_rrc(memory, E),
            0x0C => self.execute_rrc(memory, H),
            0x0D => self.execute_rrc(memory, L),
            0x0E => self.execute_rrc(memory, RegisterHL),
            0x0F => self.execute_rrc(memory, A),

            0x10 => self.execute_rl(memory, B),
            0x11 => self.execute_rl(memory, C),
            0x12 => self.execute_rl(memory, D),
            0x13 => self.execute_rl(memory, E),
            0x14 => self.execute_rl(memory, H),
            0x15 => self.execute_rl(memory, L),
            0x16 => self.execute_rl(memory, RegisterHL),
            0x17 => self.execute_rl(memory, A),

            0x18 => self.execute_rr(memory, B),
            0x19 => self.execute_rr(memory, C),
            0x1A => self.execute_rr(memory, D),
            0x1B => self.execute_rr(memory, E),
            0x1C => self.execute_rr(memory, H),
            0x1D => self.execute_rr(memory, L),
            0x1E => self.execute_rr(memory, RegisterHL),
            0x1F => self.execute_rr(memory, A),

            0x20 => self.execute_sla(memory, B),
            0x21 => self.execute_sla(memory, C),
            0x22 => self.execute_sla(memory, D),
            0x23 => self.execute_sla(memory, E),
            0x24 => self.execute_sla(memory, H),
            0x25 => self.execute_sla(memory, L),
            0x26 => self.execute_sla(memory, RegisterHL),
            0x27 => self.execute_sla(memory, A),

            0x28 => self.execute_sra(memory, B),
            0x29 => self.execute_sra(memory, C),
            0x2A => self.execute_sra(memory, D),
            0x2B => self.execute_sra(memory, E),
            0x2C => self.execute_sra(memory, H),
            0x2D => self.execute_sra(memory, L),
            0x2E => self.execute_sra(memory, RegisterHL),
            0x2F => self.execute_sra(memory, A),

            0x30 => self.execute_swap(memory, B),
            0x31 => self.execute_swap(memory, C),
            0x32 => self.execute_swap(memory, D),
            0x33 => self.execute_swap(memory, E),
            0x34 => self.execute_swap(memory, H),
            0x35 => self.execute_swap(memory, L),
            0x36 => self.execute_swap(memory, RegisterHL),
            0x37 => self.execute_swap(memory, A),

            0x38 => self.execute_srl(memory, B),
            0x39 => self.execute_srl(memory, C),
            0x3A => self.execute_srl(memory, D),
            0x3B => self.execute_srl(memory, E),
            0x3C => self.execute_srl(memory, H),
            0x3D => self.execute_srl(memory, L),
            0x3E => self.execute_srl(memory, RegisterHL),
            0x3F => self.execute_srl(memory, A),

            0x40..=0x7F => {
                let bit = (opcode - 0x40) / 0x08;
                match opcode % 0x08 {
                    0x00 => self.execute_tst(memory, bit, B),
                    0x01 => self.execute_tst(memory, bit, C),
                    0x02 => self.execute_tst(memory, bit, D),
                    0x03 => self.execute_tst(memory, bit, E),
                    0x04 => self.execute_tst(memory, bit, H),
                    0x05 => self.execute_tst(memory, bit, L),
                    0x06 => self.execute_tst(memory, bit, RegisterHL),
                    0x07 => self.execute_tst(memory, bit, A),
                    _    => unreachable!(),
                }
            },
            0x80..=0xBF => {
                let bit = (opcode - 0x80) / 0x08;
                match opcode % 0x08 {
                    0x00 => self.execute_res(memory, bit, B),
                    0x01 => self.execute_res(memory, bit, C),
                    0x02 => self.execute_res(memory, bit, D),
                    0x03 => self.execute_res(memory, bit, E),
                    0x04 => self.execute_res(memory, bit, H),
                    0x05 => self.execute_res(memory, bit, L),
                    0x06 => self.execute_res(memory, bit, RegisterHL),
                    0x07 => self.execute_res(memory, bit, A),
                    _    => unreachable!(),
                }
            },
            0xC0..=0xFF => {
                let bit = (opcode - 0xC0) / 0x08;
                match opcode % 0x08 {
                    0x00 => self.execute_set(memory, bit, B),
                    0x01 => self.execute_set(memory, bit, C),
                    0x02 => self.execute_set(memory, bit, D),
                    0x03 => self.execute_set(memory, bit, E),
                    0x04 => self.execute_set(memory, bit, H),
                    0x05 => self.execute_set(memory, bit, L),
                    0x06 => self.execute_set(memory, bit, RegisterHL),
                    0x07 => self.execute_set(memory, bit, A),
                    _    => unreachable!(),
                }
            },
        }
    }

    fn execute_dec_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        self.regs.write(reg, self.regs.read(reg) - 1);
    }

    fn execute_inc_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        self.regs.write(reg, self.regs.read(reg) + 1);
    }

    fn execute_load_rr_nn(&mut self, memory: &Memory, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        let nn = self.read_imm_u16(memory);
        self.regs.write(reg, nn);
    }

    fn execute_add_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        let old_zero = read_bit(self.regs[F], ZERO_FLAG);
        let (result, flags) = add_u16(self.regs.read(HL), self.regs.read(reg));
        self.regs.write(HL, result);
        self.regs.write_flags(Flags { zero: old_zero, ..flags });
    }

    fn execute_load_r_r(&mut self, memory: &mut Memory, opcode: u8) {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        let opcode_index = opcode - 0x40;
        let dst_index = order[(opcode_index / 0x08) as usize];
        let src_index = order[(opcode_index % 0x08) as usize];

        match (dst_index, src_index) {
            (Some(reg), Some(reg1)) => {
                self.load_reg_reg(reg, reg1);
            },
            (None, Some(reg)) => {
                self.load_hl_reg(memory, reg);
            },
            (Some(reg), None) => {
                self.load_reg_hl(memory, reg);
            },
            (None, None) => {
                panic!("Cannot handle {:04x} here!", opcode);
            },
        }
    }

    fn execute_add<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, flags) = add_u8(self.regs[A], val);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_adc<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, flags) = adc_u8(self.regs[A], val, old_carry);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_sub<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, flags) = sub_u8(self.regs[A], val);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_sbc<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, flags) = sbc_u8(self.regs[A], val, old_carry);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_and<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        self.regs[A] &= val;
        self.regs.write_flags(Flags {
            zero: self.regs[A] == 0,
            half_carry: true,
            ..Flags::default()
        });
    }

    fn execute_xor<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        self.regs[A] ^= val;
        self.regs.write_flags(Flags {
            zero: self.regs[A] == 0,
            ..Flags::default()
        });
    }

    fn execute_or<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        self.regs[A] |= val;
        self.regs.write_flags(Flags {
            zero: self.regs[A] == 0,
            ..Flags::default()
        });
    }

    fn execute_cp<T>(&mut self, memory: &Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (_, flags) = sub_u8(self.regs[A], val);
        self.regs.write_flags(flags);
    }

    fn execute_rlc<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_left(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rrc<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_right(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rl<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_left_through_carry(val, carry);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rr<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_right_through_carry(val, carry);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_sla<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, carry) = shift_left(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_sra<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, carry) = shift_right_arithmetic(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_swap<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let result = (val & 0x0F) << 4 | (val & 0xF0) >> 4;
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            ..Flags::default()
        });
    }

    fn execute_srl<T: Copy>(&mut self, memory: &mut Memory, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let (result, carry) = shift_right_logical(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_tst<T: Copy>(&mut self, memory: &Memory, bit: u8, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let result = read_bit(val, bit);
        self.set_flag(ZERO_FLAG, !result);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, true);
    }

    fn execute_res<T: Copy>(&mut self, memory: &mut Memory, bit: u8, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let result = set_bit(val, bit, false);
        self.write_oper(memory, src, result);
    }

    fn execute_set<T: Copy>(&mut self, memory: &mut Memory, bit: u8, src: T) where
        Self: Operand8<T> {
        let val = self.read_oper(memory, src);
        let result = set_bit(val, bit, true);
        self.write_oper(memory, src, result);
    }

    fn inc_reg(&mut self, index: RegisterIndex) {
        // INC r
        let old = self.regs[index];
        self.regs[index] += 1;
        self.set_flag(ZERO_FLAG, self.regs[index] == 0);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, ((old & 0xF) + 1) > 0xF);
    }

    fn dec_reg(&mut self, index: RegisterIndex) {
        // DEC r
        let old = self.regs[index];
        self.regs[index] -= 1;
        self.set_flag(ZERO_FLAG, self.regs[index] == 0);
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
    }

    fn load_reg_byte(&mut self, memory: &Memory, index: RegisterIndex) -> u8 {
        // LD r, n
        let n = self.read_oper(memory, Immediate8);
        self.regs[index] = n;

        n
    }

    fn load_reg_hl(&mut self, memory: &Memory, index: RegisterIndex) {
        let addr = self.regs.read(HL);
        self.regs[index] = memory.cpu_read(addr);
    }

    fn load_hl_reg(&mut self, memory: &mut Memory, index: RegisterIndex) {
        let addr = self.regs.read(HL);
        memory.cpu_write(addr, self.regs[index]);
    }

    fn load_reg_reg(&mut self, dest: RegisterIndex, source: RegisterIndex) {
        self.regs[dest] = self.regs[source];
    }

    fn sum_sp_n(&mut self, n: u8) -> (u16, Flags) {
        // n is a signed value
        let n = n as i8;
        let sp = self.sp as i32;

        let result = sp + (n as i32);
        let (half_carry, carry) = if n < 0 {
            (
                (result & 0x0F) <= (sp & 0x0F),
                (result & 0xFF) <= (sp & 0xFF),
            )
        } else {
            (
                (sp & 0x0F) + (n as i32 & 0x0F) > 0x0F,
                (sp & 0xFF) + (n as i32 & 0xFF) > 0xFF,
            )
        };

        (result as u16, Flags { half_carry, carry, ..Flags::default() })
    }

    fn jump_rel(&mut self, memory: &Memory) -> i8 {
        // JR n
        self.jump_rel_condition(memory, true)
    }

    fn jump_rel_condition(&mut self, memory: &Memory, condition: bool) -> i8 {
        // JR cc, n
        // Offset is signed
        let offset = self.read_oper(memory, Immediate8) as i8;

        if condition {
            self.pc = ((self.pc as i32) + (offset as i32)) as u16;
        }

        offset
    }

    fn jump(&mut self, memory: &Memory) -> u16 {
        let addr = self.read_imm_u16(memory);
        self.pc = addr;
        addr
    }

    fn jump_condition(&mut self, memory: &Memory, condition: bool) -> u16 {
        // JP cc, nn
        let addr = self.read_imm_u16(memory);
        if condition {
            self.pc = addr;
        }
        addr
    }

    fn call_condition(&mut self, memory: &mut Memory, condition: bool) -> u16 {
        // CALL cc, nn
        let addr = self.read_imm_u16(memory);

        if condition {
            self.sp -= 2;
            Self::write_mem_u16(memory, self.sp, self.pc);
            self.pc = addr;
        }

        addr
    }

    fn ret(&mut self, memory: &Memory) {
        // RET
        self.pc = Self::read_mem_u16(memory, self.sp);
        self.sp += 2;
    }

    fn ret_condition(&mut self, memory: &Memory, condition: bool) {
        // RET cc
        if condition {
            self.pc = Self::read_mem_u16(memory, self.sp);
            self.sp += 2;
        }
    }

    fn execute_rst(&mut self, memory: &mut Memory, opcode: u8) {
        let addr = (opcode - 0xC7) as u16;
        self.sp -= 2;
        Self::write_mem_u16(memory, self.sp, self.pc);
        self.pc = addr;
    }

    fn push(&mut self, memory: &mut Memory, index: TwoRegisterIndex) {
        self.sp -= 2;
        Self::write_mem_u16(memory, self.sp, self.regs.read(index));
    }

    fn pop(&mut self, memory: &mut Memory, index: TwoRegisterIndex) {
        let nn = Self::read_mem_u16(memory, self.sp);
        self.regs.write(index, nn);
        self.sp += 2;
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
    let (result, carry) = x.overflowing_sub(y);

    let flags = Flags {
        zero: result == 0,
        subtract: true,
        half_carry: (x & 0xF) < (y & 0xF),
        carry,
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
    let (result, carry) = x.overflowing_add(y);

    let flags = Flags {
        zero: result == 0,
        subtract: false,
        half_carry: (x & 0xF) + (y & 0xF) > 0xF,
        carry,
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
