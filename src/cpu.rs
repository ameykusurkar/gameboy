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
const IF_ADDR: u16 = 0xFF0F;
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
    pub memory: Memory,
    clock_cycles: u32,
    remaining_cycles: u32,
    total_clock_cycles: u32,
    halted: bool,
    pub current_instruction: &'static Instruction<'static>,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            regs: Registers::default(),
            sp: 0,
            pc: 0,
            ime: false,
            memory: Memory::new(),
            clock_cycles: 0,
            remaining_cycles: 0,
            total_clock_cycles: 0,
            halted: false,
            // This should get populated when cpu starts running
            current_instruction: &INSTRUCTIONS[0],
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
        if self.halted && self.get_pending_interrupts() > 0 {
            self.halted = false;
        }

        if !self.halted {
            if self.remaining_cycles == 0 {
                let interrupt_was_serviced = self.check_and_handle_interrupts();

                if interrupt_was_serviced {
                    self.remaining_cycles += 5;
                }

                self.current_instruction = self.fetch_instruction(self.pc);

                self.remaining_cycles += self.cycles_for_instruction(self.current_instruction);

                // For instructions with a conditional jump, the cpu takes extra cycles if
                // the jump does happen, which `execute` determines based on the condition.
                let extra_cycles = self.execute();
                self.remaining_cycles += extra_cycles;
            }

            self.remaining_cycles -= 1;
        }

        self.total_clock_cycles += 1;
        self.update_timers();
    }

    fn fetch_instruction(&self, addr: u16) -> &'static Instruction<'static> {
        let opcode = self.memory[addr];

        if opcode == 0xCB {
            let next_byte = self.memory[addr + 1];
            &PREFIXED_INSTRUCTIONS[next_byte as usize]
        } else {
            &INSTRUCTIONS[opcode as usize]
        }
    }

    // Finds the number of cycles required for the given instruction. If the instruction
    // is a conditional jump, this does not take into account the extra cycles required
    // for the jump: that is determined by the `execute` method. For prefixed instructions,
    // we also add the cycles of the prefix instruction "CB".
    fn cycles_for_instruction(&self, instruction: &Instruction) -> u32 {
        let mut cycles = match instruction.cycles {
            Fixed(cycles) => cycles,
            Jump(_, cycles_without_jump) => cycles_without_jump,
        };

        if instruction.prefixed {
            cycles += match &INSTRUCTIONS[0xCB].cycles {
                Fixed(cycles) => *cycles,
                Jump(_, cycles_without_jump) => *cycles_without_jump,
            };
        }

        cycles
    }

    fn update_timers(&mut self) {
        if self.total_clock_cycles % 64 == 0 {
            self.memory[DIV_ADDR] += 1;
        }

        let timer_control = self.memory[TAC_ADDR];
        let timer_is_active = read_bit(timer_control, 2);

        if timer_is_active {
            let cycles_per_update = match timer_control & 0b11 {
                0b00 => 256,        // 4096 Hz
                0b01 => 4,          // 262,144 Hz
                0b10 => 16,         // 65,536 Hz
                0b11..=0xFF => 64,  // 16,384 Hz
            };

            if self.total_clock_cycles % cycles_per_update == 0 {
                if self.memory[TIMA_ADDR] == 0xFF {
                    // If the timer is going to overflow, request a timer interrupt
                    self.memory[IF_ADDR] |= 1 << 2;
                    self.memory[TIMA_ADDR] = self.memory[TMA_ADDR];
                } else {
                    self.memory[TIMA_ADDR] += 1;
                }
            }
        }
    }

    pub fn execute(&mut self) -> u32 {
        // TODO: Remove this
        // Temp hack to let CPU think that screen is done rendering
        self.memory[0xFF44] = 0x90;

        let opcode = self.memory[self.pc];

        let mut extra_cycles = 0;

        self.pc += 1;

        if DEBUG {
            println!("{:#06x}: {}", self.pc - 1, self.build_instruction_repr(self.pc, self.current_instruction));
        }

        match opcode {
            0x00 => (),
            0x01 | 0x11 | 0x21=> {
                self.execute_load_rr_nn(opcode);
            },
            0x02 => {
                // LD (BC), A
                let addr = self.regs.read(BC);
                self.write_mem(addr, self.regs[A]);
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
                self.load_reg_byte(B);
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
                let nn = self.read_imm_u16();
                self.write_mem_u16(nn, self.sp);
            },
            0x09 | 0x19 | 0x29 => {
                self.execute_add_rr(opcode);
            },
            0x0A => {
                // LD A,(BC)
                let addr = self.regs.read(BC);
                self.regs[A] = self.read_mem(addr);
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
                self.load_reg_byte(C);
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
                self.write_mem(addr, self.regs[A]);
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
                self.load_reg_byte(D);
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
                self.jump_rel();
            },
            0x1A => {
                // LD A,(DE)
                let addr = self.regs.read(DE);
                self.regs[A] = self.read_mem(addr);
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
                self.load_reg_byte(E);
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
                self.jump_rel_condition(condition);
            },
            0x22 => {
                // LD (HL+),A
                let addr = self.regs.read(HL);
                self.write_mem(addr, self.regs[A]);
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
                self.load_reg_byte(H);
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
                self.jump_rel_condition(condition);
            },
            0x2A => {
                // LD A, (HL+)
                let addr = self.regs.read(HL);
                self.regs[A] = self.read_mem(addr);
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
                self.load_reg_byte(L);
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
                self.jump_rel_condition(condition);
            },
            0x31 => {
                // LD SP,nn
                let nn = self.read_imm_u16();
                self.sp = nn;
            },
            0x32 => {
                // LD (HL-), A
                let addr = self.regs.read(HL);
                self.write_mem(addr, self.regs[A]);
                self.regs.write(HL, addr - 1);
            },
            0x33 => {
                // INC SP
                self.sp += 1;
                // 16-bit operation
                self.nop();
            },
            0x34 => {
                // INC (HL)
                let addr = self.regs.read(HL);
                let old = self.read_mem(addr);
                self.write_mem(addr, old + 1);
                self.set_flag(ZERO_FLAG, self.memory[addr] == 0);
                self.set_flag(SUBTRACT_FLAG, false);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0xF);
            },
            0x35 => {
                // DEC (HL)
                let addr = self.regs.read(HL);
                let old = self.read_mem(addr);
                self.write_mem(addr, old - 1);
                self.set_flag(ZERO_FLAG, self.memory[addr] == 0);
                self.set_flag(SUBTRACT_FLAG, true);
                self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
            },
            0x36 => {
                // LD (HL),n
                let n = self.read_imm();
                self.write_mem(self.regs.read(HL), n);
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
                self.jump_rel_condition(condition);
            },
            0x39 => {
                // ADD HL,SP
                let old_zero = read_bit(self.regs[F], ZERO_FLAG);
                let (result, flags) = add_u16(self.regs.read(HL), self.sp);
                self.regs.write(HL, result);
                self.regs.write_flags(Flags { zero: old_zero, ..flags });
                // 16-bit operation
                self.nop();
            },
            0x3A => {
                // LD A, (HL-)
                let addr = self.regs.read(HL);
                self.regs[A] = self.read_mem(addr);
                self.regs.write(HL, addr - 1);
            },
            0x3B => {
                // DEC SP
                self.sp -= 1;
                // 16-bit operation
                self.nop();
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
                let n = self.read_imm();
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
                self.execute_load_r_r(opcode);
            },
            0x76 => {
                // HALT
                self.halted = true;
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
            0xC0 => {
                // RET NZ
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(condition);
            },
            0xC1 => {
                // POP BC
                self.pop(BC);
            },
            0xC2 => {
                // JP NZ,nn
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(condition);
            },
            0xC3 => {
                // JP nn
                self.jump();
            },
            0xC4 => {
                // CALL NZ,nn
                let condition = !read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(condition);
            },
            0xC5 => {
                // PUSH BC
                self.push(BC);
            },
            0xC6 => {
                // ADD n
                let n = self.read_imm();
                let (result, flags) = add_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xC7 | 0xD7 | 0xE7 | 0xF7 | 0xCF | 0xDF | 0xEF | 0xFF => {
                self.execute_rst(opcode);
            },
            0xC8 => {
                // RET Z
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(condition);
            },
            0xC9 => {
                // RET
                self.ret();
            },
            0xCA => {
                // JP Z,nn
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(condition);
            },
            0xCB => {
                self.execute_prefixed_instruction();
            },
            0xCC => {
                // CALL Z,nn
                let condition = read_bit(self.regs[F], ZERO_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(condition);
            },
            0xCD => {
                // CALL nn
                self.call_condition(true);
            },
            0xCE => {
                // ADC n
                let n = self.read_imm();
                let old_carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, flags) = adc_u8(self.regs[A], n, old_carry);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xD0 => {
                // RET NC
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(condition);
            },
            0xD1 => {
                // POP DE
                self.pop(DE);
            },
            0xD2 => {
                // JP NC,nn
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(condition);
            },
            0xD4 => {
                // CALL NC,nn
                let condition = !read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(condition);
            },
            0xD5 => {
                // PUSH DE
                self.push(DE);
            },
            0xD6 => {
                // SUB n
                let n = self.read_imm();
                let (result, flags) = sub_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xD8 => {
                // RET C
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.ret_condition(condition);
            },
            0xD9 => {
                // RETI
                self.ime = true;
                self.ret();
            },
            0xDA => {
                // JP C,nn
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.jump_condition(condition);
            },
            0xDC => {
                // CALL C,nn
                let condition = read_bit(self.regs[F], CARRY_FLAG);
                if condition {
                    extra_cycles = self.current_instruction.cycles.get_extra_cycles();
                }
                self.call_condition(condition);
            },
            0xDE => {
                // SBC n
                let n = self.read_imm();
                let old_carry = read_bit(self.regs[F], CARRY_FLAG);
                let (result, flags) = sbc_u8(self.regs[A], n, old_carry);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xE0 => {
                // LDH (n),A
                let offset = self.read_imm();
                let addr = 0xFF00 + (offset as u16);
                self.write_mem(addr, self.regs[A]);
            },
            0xE1 => {
                // POP HL
                self.pop(HL);
            },
            0xE2 => {
                // LDH (C),A
                let addr = 0xFF00 + (self.regs[C] as u16);
                self.write_mem(addr, self.regs[A]);
            },
            0xE9 => {
                // JP (HL)
                let addr = self.regs.read(HL);
                self.pc = addr;
            },
            0xEA => {
                // LD (nn),A
                let nn = self.read_imm_u16();
                self.write_mem(nn, self.regs[A]);
            },
            0xE5 => {
                // PUSH HL
                self.push(HL);
            },
            0xE6 => {
                // AND n
                let n = self.read_imm();
                let (result, flags) = and_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xE8 => {
                // ADD SP,n
                let n = self.read_imm();
                let (result, flags) = self.sum_sp_n(n);
                self.sp = result;
                self.regs.write_flags(flags);
                // Extra IO for 16-bit operation
                self.nop();
                self.nop();
            },
            0xEE => {
                // XOR n
                let n = self.read_imm();
                let (result, flags) = xor_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xF0 => {
                // LDH A,(n)
                let offset = self.read_imm();
                let addr = 0xFF00 + (offset as u16);
                self.regs[A] = self.read_mem(addr);
            },
            0xF1 => {
                // POP AF
                self.pop(AF);
            },
            0xF2 => {
                // LDH A,(C)
                let addr = 0xFF00 + (self.regs[C] as u16);
                self.regs[A] = self.read_mem(addr);
            },
            0xF3 => {
                // DI
                self.ime = false;
            },
            0xF5 => {
                // PUSH AF
                self.push(AF);
            },
            0xF6 => {
                // OR n
                let n = self.read_imm();
                let (result, flags) = or_u8(self.regs[A], n);
                self.regs[A] = result;
                self.regs.write_flags(flags);
            },
            0xF8 => {
                // LD HL, SP + n
                let n = self.read_imm();
                let (result, flags) = self.sum_sp_n(n);
                self.regs.write(HL, result);
                self.regs.write_flags(flags);
                // 16-bit operation
                self.nop();
            },
            0xF9 => {
                // LD SP,HL
                self.sp = self.regs.read(HL);
                // 16-bit operation
                self.nop();
            },
            0xFA => {
                // LD A,(nn)
                let nn = self.read_imm_u16();
                self.regs[A] = self.read_mem(nn);
            },
            0xFB => {
                // EI
                self.ime = true;
            },
            0xFE => {
                // CP n
                let n = self.read_imm();
                let (_, flags) = sub_u8(self.regs[A], n);
                self.regs.write_flags(flags);
            },
            _ => panic!("Unimplemented opcode {:02x}, {:?}", opcode, self.current_instruction),
        }

        if DEBUG {
            println!("{:02x?}, PC: {:#06x}, Cycles: {}", self.regs, self.pc, self.total_clock_cycles);
        }

        extra_cycles
    }

    pub fn disassemble(&self, start_addr: u16, end_addr: u16) -> Vec<(u16, String)> {
        let mut current_addr = start_addr;
        let mut instruction_reprs = Vec::new();

        loop {
            if current_addr > end_addr { break };

            let instruction = self.fetch_instruction(current_addr);
            let num_bytes = instruction.num_bytes as u32;

            if current_addr as u32 + num_bytes > end_addr as u32 { break };

            let operand_start_addr = current_addr + 1 + (instruction.prefixed as u16);
            let repr = self.build_instruction_repr(operand_start_addr, instruction);
            instruction_reprs.push((current_addr, repr));
            current_addr += num_bytes as u16;
        }

        instruction_reprs
    }

    // Depends on `instruction.repr` being in the correct format
    fn build_instruction_repr(&self, addr: u16, instruction: &Instruction) -> String {
        let (operand, _) = self.fetch_operand(addr, instruction);
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

    fn fetch_operand(&self, addr: u16, instruction: &Instruction) -> (u16, u32) {
        // (operand, bytes_read)
        let lsb = self.memory[addr] as u16;
        match &instruction.addressing_mode {
            AddressingMode::Implied => (0, 0),
            AddressingMode::Imm8 => (lsb, 0),
            AddressingMode::Imm16 | AddressingMode::Addr16 => {
                let msb = self.memory[addr + 1] as u16;
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

    fn check_and_handle_interrupts(&mut self) -> bool {
        let pending_interrupts = self.get_pending_interrupts();

        if self.ime && pending_interrupts > 0 {
            for i in 0..5 {
                if read_bit(pending_interrupts, i) {
                    self.handle_interrupt(i);
                    return true;
                }
            }
        }

        false
    }

    fn get_pending_interrupts(&self) -> u8 {
        self.memory[IE_ADDR] & self.memory[IF_ADDR]
    }

    fn handle_interrupt(&mut self, interrupt_no: u8) {
        // This routine should take 5 machine cycles
        self.ime = false;
        self.nop();
        self.nop();
        self.write_mem(IF_ADDR, set_bit(self.memory[IF_ADDR], interrupt_no, false));
        self.sp -= 2;
        self.write_mem_u16(self.sp, self.pc);
        self.pc = INTERRUPT_ADDRS[interrupt_no as usize];
    }

    fn nop(&mut self) {
        self.clock_cycles += 1;
    }

    fn read_imm(&mut self) -> u8 {
        let n = self.read_mem(self.pc);
        self.pc += 1;
        n
    }

    fn read_imm_u16(&mut self) -> u16 {
        let lsb = self.read_imm() as u16;
        let msb = self.read_imm() as u16;
        (msb << 8) | lsb
    }

    fn read_mem(&mut self, addr: u16) -> u8 {
        let n = self.memory[addr];
        self.clock_cycles += 1;
        n
    }

    fn read_mem_u16(&mut self, addr: u16) -> u16 {
        let lsb = self.read_mem(addr) as u16;
        let msb = self.read_mem(addr + 1) as u16;
        (msb << 8) | lsb
    }

    fn write_mem(&mut self, addr: u16, val: u8) {
        self.memory[addr] = val;
        self.clock_cycles += 1;
    }

    fn write_mem_u16(&mut self, addr: u16, val: u16) {
        let lsb = (val & 0x00FF) as u8;
        let msb = ((val & 0xFF00) >> 8) as u8;
        self.write_mem(addr, lsb);
        self.write_mem(addr + 1, msb);
    }

    pub fn load_bootrom(&mut self, buffer: &[u8]) {
        self.memory.load_bootrom(buffer);
    }

    pub fn load_rom(&mut self, buffer: &[u8]) {
        self.memory.load_rom(buffer);
    }

    fn execute_prefixed_instruction(&mut self) {
        let opcode = self.read_imm();

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
            0x40..=0x7F => {
                self.execute_test_bit_reg(opcode);
            },
            0x80..=0xBF => {
                self.execute_reset_bit_reg(opcode);
            },
            0xC0..=0xFF => {
                self.execute_set_bit_reg(opcode);
            },
        }
    }

    fn execute_dec_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        self.regs.write(reg, self.regs.read(reg) - 1);
        // Because it is a 16-bit register operation
        self.nop();
    }

    fn execute_inc_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        self.regs.write(reg, self.regs.read(reg) + 1);
        // Because it is a 16-bit register operation
        self.nop();
    }

    fn execute_load_rr_nn(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        let nn = self.read_imm_u16();
        self.regs.write(reg, nn);
    }

    fn execute_add_rr(&mut self, opcode: u8) {
        let order = [BC, DE, HL];
        let reg = order[(opcode / 0x10) as usize];

        let old_zero = read_bit(self.regs[F], ZERO_FLAG);
        let (result, flags) = add_u16(self.regs.read(HL), self.regs.read(reg));
        self.regs.write(HL, result);
        self.regs.write_flags(Flags { zero: old_zero, ..flags });
        // Because it is a 16-bit register operation
        self.nop();
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
            },
            (None, Some(reg)) => {
                self.load_hl_reg(reg);
            },
            (Some(reg), None) => {
                self.load_reg_hl(reg);
            },
            (None, None) => {
                panic!("Cannot handle {:04x} here!", opcode);
            },
        }
    }

    fn execute_or_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, flags) = or_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_cp_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (_, flags) = sub_u8(self.regs[A], src);
        self.regs.write_flags(flags);
    }

    fn execute_add_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, flags) = add_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_adc_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, flags) = adc_u8(self.regs[A], src, old_carry);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_sub_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, flags) = sub_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_sbc_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        let (result, flags) = sbc_u8(self.regs[A], src, old_carry);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_and_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, flags) = and_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_xor_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, flags) = xor_u8(self.regs[A], src);
        self.regs[A] = result;
        self.regs.write_flags(flags);
    }

    fn execute_rlc_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, carry) = rotate_left(src);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rrc_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, carry) = rotate_right(src);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rl_reg(&mut self, opcode: u8) {
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let src = self.get_source_val(opcode);
        let (result, carry) = rotate_left_through_carry(src, carry);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rr_reg(&mut self, opcode: u8) {
        let carry = read_bit(self.regs[F], CARRY_FLAG);
        let src = self.get_source_val(opcode);
        let (result, carry) = rotate_right_through_carry(src, carry);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_sla_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, carry) = shift_left(src);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_sra_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, carry) = shift_right_arithmetic(src);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_swap_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, flags) = swap_u8(src);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(flags);
    }

    fn execute_srl_reg(&mut self, opcode: u8) {
        let src = self.get_source_val(opcode);
        let (result, carry) = shift_right_logical(src);
        self.set_dst_reg(opcode, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_test_bit_reg(&mut self, opcode: u8) {
        let bit_position = (opcode - 0x40) / 0x08;
        let src = self.get_source_val(opcode);
        let result = read_bit(src, bit_position);
        self.set_flag(ZERO_FLAG, !result);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, true);
    }

    fn execute_reset_bit_reg(&mut self, opcode: u8) {
        let bit_position = (opcode - 0x40) / 0x08;
        let src = self.get_source_val(opcode);
        let result = set_bit(src, bit_position, false);
        self.set_dst_reg(opcode, result);
    }

    fn execute_set_bit_reg(&mut self, opcode: u8) {
        let bit_position = (opcode - 0x40) / 0x08;
        let src = self.get_source_val(opcode);
        let result = set_bit(src, bit_position, true);
        self.set_dst_reg(opcode, result);
    }

    // Many arithmetic/logical opcodes are arranged a particular order,
    // eg. ADD r, where consecutive opcodes iterate through the different
    // registers (r). There is one caveat to this: these operations treat
    // the memory address pointed to by HL like a register as well, even
    // though it is not really a register, but memory. This method hides
    // that detail so that register operations can't tell the difference.
    fn get_source_val(&mut self, opcode: u8) -> u8 {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        match order[(opcode % 0x08) as usize] {
            Some(reg) => self.regs[reg],
            None => {
                let addr = self.regs.read(HL);
                self.read_mem(addr)
            }
        }
    }

    fn set_dst_reg(&mut self, opcode: u8, val: u8) {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        match order[(opcode % 0x08) as usize] {
            Some(reg) => {
                self.regs[reg] = val;
            },
            None => {
                let addr = self.regs.read(HL);
                self.write_mem(addr, val);
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
    }

    fn dec_reg(&mut self, index: RegisterIndex) {
        // DEC r
        let old = self.regs[index];
        self.regs[index] -= 1;
        self.set_flag(ZERO_FLAG, self.regs[index] == 0);
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
    }

    fn load_reg_byte(&mut self, index: RegisterIndex) -> u8 {
        // LD r, n
        let n = self.read_imm();
        self.regs[index] = n;

        n
    }

    fn load_reg_hl(&mut self, index: RegisterIndex) {
        let addr = self.regs.read(HL);
        self.regs[index] = self.read_mem(addr);
    }

    fn load_hl_reg(&mut self, index: RegisterIndex) {
        let addr = self.regs.read(HL);
        self.write_mem(addr, self.regs[index]);
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

    fn jump_rel(&mut self) -> i8 {
        // JR n
        self.jump_rel_condition(true)
    }

    fn jump_rel_condition(&mut self, condition: bool) -> i8 {
        // JR cc, n
        // Offset is signed
        let offset = self.read_imm() as i8;

        if condition {
            self.pc = ((self.pc as i32) + (offset as i32)) as u16;
        }

        offset
    }

    fn jump(&mut self) -> u16 {
        let addr = self.read_imm_u16();
        self.pc = addr;
        self.nop();
        addr
    }

    fn jump_condition(&mut self, condition: bool) -> u16 {
        // JP cc, nn
        let addr = self.read_imm_u16();
        if condition {
            self.pc = addr;
            self.nop();
        }
        addr
    }

    fn call_condition(&mut self, condition: bool) -> u16 {
        // CALL cc, nn
        let addr = self.read_imm_u16();

        if condition {
            self.nop(); // To decrement sp
            self.sp -= 2;
            self.write_mem_u16(self.sp, self.pc);
            self.pc = addr;
        }

        addr
    }

    fn ret(&mut self) {
        // RET
        self.pc = self.read_mem_u16(self.sp);
        self.sp += 2;
        self.nop();
    }

    fn ret_condition(&mut self, condition: bool) {
        // RET cc
        self.nop();
        if condition {
            self.pc = self.read_mem_u16(self.sp);
            self.sp += 2;
            self.nop();
        }
    }

    fn execute_rst(&mut self, opcode: u8) {
        let addr = (opcode - 0xC7) as u16;
        self.nop(); // To decrement sp
        self.sp -= 2;
        self.write_mem_u16(self.sp, self.pc);
        self.pc = addr;
    }

    fn push(&mut self, index: TwoRegisterIndex) {
        self.sp -= 2;
        self.write_mem_u16(self.sp, self.regs.read(index));
    }

    fn pop(&mut self, index: TwoRegisterIndex) {
        let nn = self.read_mem_u16(self.sp);
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
