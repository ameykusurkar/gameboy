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

enum Condition { NZ, Z, NC, C }
trait ConditionEvaluate {
    fn eval_condition(&self, condition: Condition) -> bool;
}

impl ConditionEvaluate for Cpu {
    fn eval_condition(&self, condition: Condition) -> bool {
        match condition {
            Condition::NZ => !read_bit(self.regs[F], ZERO_FLAG),
            Condition::Z => read_bit(self.regs[F], ZERO_FLAG),
            Condition::NC => !read_bit(self.regs[F], CARRY_FLAG),
            Condition::C => read_bit(self.regs[F], CARRY_FLAG),
        }
    }
}

#[derive(Copy, Clone)]
struct RegisterHL;

struct RegisterHLI;
struct RegisterHLD;

struct Immediate8;
struct Immediate16;
struct Addr16;
struct HighPageAddr;
struct HighPageC;
struct SP;

struct AddrReg16(TwoRegisterIndex);

trait Operand8<T> {
    fn read_oper(&mut self, memory: &Memory, src: T) -> u8;
    fn write_oper(&mut self, memory: &mut Memory, src: T, val: u8);
}

trait Operand16<T> {
    fn read16(&mut self, memory: &Memory, src: T) -> u16;
    fn write16(&mut self, memory: &mut Memory, src: T, val: u16);
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

impl Operand8<RegisterHLI> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: RegisterHLI) -> u8 {
        let addr = self.regs.read(HL);
        self.regs.write(HL, addr + 1);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, _src: RegisterHLI, val: u8) {
        let addr = self.regs.read(HL);
        self.regs.write(HL, addr + 1);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<RegisterHLD> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: RegisterHLD) -> u8 {
        let addr = self.regs.read(HL);
        self.regs.write(HL, addr - 1);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, _src: RegisterHLD, val: u8) {
        let addr = self.regs.read(HL);
        self.regs.write(HL, addr - 1);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<AddrReg16> for Cpu {
    fn read_oper(&mut self, memory: &Memory, src: AddrReg16) -> u8 {
        let AddrReg16(reg) = src;
        let addr = self.regs.read(reg);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, src: AddrReg16, val: u8) {
        let AddrReg16(reg) = src;
        let addr = self.regs.read(reg);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<HighPageAddr> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: HighPageAddr) -> u8 {
        let offset = self.read_oper(memory, Immediate8);
        let addr = 0xFF00 | (offset as u16);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, _src: HighPageAddr, val: u8) {
        let offset = self.read_oper(memory, Immediate8);
        let addr = 0xFF00 | (offset as u16);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<HighPageC> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: HighPageC) -> u8 {
        let offset = self.read_oper(memory, C);
        let addr = 0xFF00 | (offset as u16);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, _src: HighPageC, val: u8) {
        let offset = self.read_oper(memory, C);
        let addr = 0xFF00 | (offset as u16);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<Immediate8> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: Immediate8) -> u8 {
        let val = memory.cpu_read(self.pc);
        self.pc += 1;
        val
    }

    fn write_oper(&mut self, _memory: &mut Memory, _src: Immediate8, _val: u8) {}
}

impl Operand8<Addr16> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: Addr16) -> u8 {
        let addr = self.read16(memory, Immediate16);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, _src: Addr16, val: u8) {
        let addr = self.read16(memory, Immediate16);
        memory.cpu_write(addr, val);
    }
}

impl Operand16<TwoRegisterIndex> for Cpu {
    fn read16(&mut self, _memory: &Memory, src: TwoRegisterIndex) -> u16 {
        self.regs.read(src)
    }

    fn write16(&mut self, _memory: &mut Memory, src: TwoRegisterIndex, val: u16) {
        self.regs.write(src, val);
    }
}

impl Operand16<SP> for Cpu {
    fn read16(&mut self, _memory: &Memory, _src: SP) -> u16 {
        self.sp
    }

    fn write16(&mut self, _memory: &mut Memory, _src: SP, val: u16) {
        self.sp = val;
    }
}

impl Operand16<Immediate16> for Cpu {
    fn read16(&mut self, memory: &Memory, _src: Immediate16) -> u16 {
        let lsb = self.read_oper(memory, Immediate8) as u16;
        let msb = self.read_oper(memory, Immediate8) as u16;
        (msb << 8) | lsb
    }

    fn write16(&mut self, _memory: &mut Memory, _src: Immediate16, _val: u16) {}
}

impl Operand16<Addr16> for Cpu {
    fn read16(&mut self, memory: &Memory, _src: Addr16) -> u16 {
        let addr = self.read16(memory, Immediate16);
        let lsb = memory.cpu_read(addr) as u16;
        let msb = memory.cpu_read(addr + 1) as u16;
        (msb << 8) | lsb
    }

    fn write16(&mut self, memory: &mut Memory, _src: Addr16, val: u16) {
        let addr = self.read16(memory, Immediate16);
        let lsb = (val & 0x00FF) as u8;
        let msb = ((val & 0xFF00) >> 8) as u8;
        memory.cpu_write(addr, lsb);
        memory.cpu_write(addr + 1, msb);
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
        let opcode = self.read_oper(memory, Immediate8);

        let mut extra_cycles = 0;

        if DEBUG {
            println!("{:#06x}: {}", self.pc - 1, self.build_instruction_repr(memory, self.pc, self.current_instruction));
        }

        match opcode {
            0x00 => (), // NOP

            0x01 => self.execute_load16(memory, BC, Immediate16),
            0x11 => self.execute_load16(memory, DE, Immediate16),
            0x21 => self.execute_load16(memory, HL, Immediate16),
            0x31 => self.sp = self.read16(memory, Immediate16),

            0x02 => self.execute_load(memory, AddrReg16(BC), A),
            0x12 => self.execute_load(memory, AddrReg16(DE), A),

            0x06 => self.execute_load(memory, B, Immediate8),
            0x0E => self.execute_load(memory, C, Immediate8),
            0x16 => self.execute_load(memory, D, Immediate8),
            0x1E => self.execute_load(memory, E, Immediate8),
            0x26 => self.execute_load(memory, H, Immediate8),
            0x2E => self.execute_load(memory, L, Immediate8),
            0x36 => self.execute_load(memory, RegisterHL, Immediate8),
            0x3E => self.execute_load(memory, A, Immediate8),

            0x0A => self.execute_load(memory, A, AddrReg16(BC)),
            0x1A => self.execute_load(memory, A, AddrReg16(DE)),

            0x08 => self.write16(memory, Addr16, self.sp),

            0xE0 => self.execute_load(memory, HighPageAddr, A),
            0xF0 => self.execute_load(memory, A, HighPageAddr),

            0xE2 => self.execute_load(memory, HighPageC, A),
            0xF2 => self.execute_load(memory, A, HighPageC),

            0xEA => self.execute_load(memory, Addr16, A),
            0xFA => self.execute_load(memory, A, Addr16),

            0x22 => self.execute_load(memory, RegisterHLI, A),
            0x2A => self.execute_load(memory, A, RegisterHLI),
            0x32 => self.execute_load(memory, RegisterHLD, A),
            0x3A => self.execute_load(memory, A, RegisterHLD),

            0xF9 => self.sp = self.regs.read(HL),

            0x40..=0x75 | 0x77..=0x7F => {
                self.execute_load_reg_reg(memory, opcode);
            },

            0x18 => self.execute_jr(memory),
            0x20 => extra_cycles = self.execute_jr_cc(memory, Condition::NZ),
            0x28 => extra_cycles = self.execute_jr_cc(memory, Condition::Z),
            0x30 => extra_cycles = self.execute_jr_cc(memory, Condition::NC),
            0x38 => extra_cycles = self.execute_jr_cc(memory, Condition::C),

            0xC3 => self.execute_jp(memory),
            0xC2 => extra_cycles = self.execute_jp_cc(memory, Condition::NZ),
            0xCA => extra_cycles = self.execute_jp_cc(memory, Condition::Z),
            0xD2 => extra_cycles = self.execute_jp_cc(memory, Condition::NC),
            0xDA => extra_cycles = self.execute_jp_cc(memory, Condition::C),
            0xE9 => self.pc = self.regs.read(HL),

            0xC9 => self.execute_ret(memory),
            0xC0 => extra_cycles = self.execute_ret_cc(memory, Condition::NZ),
            0xC8 => extra_cycles = self.execute_ret_cc(memory, Condition::Z),
            0xD0 => extra_cycles = self.execute_ret_cc(memory, Condition::NC),
            0xD8 => extra_cycles = self.execute_ret_cc(memory, Condition::C),
            0xD9 => {
                self.ime = true;
                self.execute_ret(memory);
            },

            0xCD => self.execute_call(memory),
            0xC4 => extra_cycles = self.execute_call_cc(memory, Condition::NZ),
            0xCC => extra_cycles = self.execute_call_cc(memory, Condition::Z),
            0xD4 => extra_cycles = self.execute_call_cc(memory, Condition::NC),
            0xDC => extra_cycles = self.execute_call_cc(memory, Condition::C),

            0xC7 | 0xD7 | 0xE7 | 0xF7 | 0xCF | 0xDF | 0xEF | 0xFF => {
                let addr = (opcode - 0xC7) as u16;
                self.execute_rst(memory, addr);
            },

            0xC1 => self.execute_pop(memory, BC),
            0xD1 => self.execute_pop(memory, DE),
            0xE1 => self.execute_pop(memory, HL),
            0xF1 => self.execute_pop(memory, AF),

            0xC5 => self.execute_push(memory, BC),
            0xD5 => self.execute_push(memory, DE),
            0xE5 => self.execute_push(memory, HL),
            0xF5 => self.execute_push(memory, AF),

            0x07 => {
                self.execute_rlc(memory, A);
                self.set_flag(ZERO_FLAG, false);
            },
            0x0F => {
                self.execute_rrc(memory, A);
                self.set_flag(ZERO_FLAG, false);
            },
            0x17 => {
                self.execute_rl(memory, A);
                self.set_flag(ZERO_FLAG, false);
            },
            0x1F => {
                self.execute_rr(memory, A);
                self.set_flag(ZERO_FLAG, false);
            },

            0x04 => self.execute_inc(memory, B),
            0x0C => self.execute_inc(memory, C),
            0x14 => self.execute_inc(memory, D),
            0x1C => self.execute_inc(memory, E),
            0x24 => self.execute_inc(memory, H),
            0x2C => self.execute_inc(memory, L),
            0x34 => self.execute_inc(memory, RegisterHL),
            0x3C => self.execute_inc(memory, A),

            0x05 => self.execute_dec(memory, B),
            0x0D => self.execute_dec(memory, C),
            0x15 => self.execute_dec(memory, D),
            0x1D => self.execute_dec(memory, E),
            0x25 => self.execute_dec(memory, H),
            0x2D => self.execute_dec(memory, L),
            0x35 => self.execute_dec(memory, RegisterHL),
            0x3D => self.execute_dec(memory, A),

            0x80 => self.execute_add(memory, B),
            0x81 => self.execute_add(memory, C),
            0x82 => self.execute_add(memory, D),
            0x83 => self.execute_add(memory, E),
            0x84 => self.execute_add(memory, H),
            0x85 => self.execute_add(memory, L),
            0x86 => self.execute_add(memory, RegisterHL),
            0x87 => self.execute_add(memory, A),
            0xC6 => self.execute_add(memory, Immediate8),

            0x88 => self.execute_adc(memory, B),
            0x89 => self.execute_adc(memory, C),
            0x8A => self.execute_adc(memory, D),
            0x8B => self.execute_adc(memory, E),
            0x8C => self.execute_adc(memory, H),
            0x8D => self.execute_adc(memory, L),
            0x8E => self.execute_adc(memory, RegisterHL),
            0x8F => self.execute_adc(memory, A),
            0xCE => self.execute_adc(memory, Immediate8),

            0x90 => self.execute_sub(memory, B),
            0x91 => self.execute_sub(memory, C),
            0x92 => self.execute_sub(memory, D),
            0x93 => self.execute_sub(memory, E),
            0x94 => self.execute_sub(memory, H),
            0x95 => self.execute_sub(memory, L),
            0x96 => self.execute_sub(memory, RegisterHL),
            0x97 => self.execute_sub(memory, A),
            0xD6 => self.execute_sub(memory, Immediate8),

            0x98 => self.execute_sbc(memory, B),
            0x99 => self.execute_sbc(memory, C),
            0x9A => self.execute_sbc(memory, D),
            0x9B => self.execute_sbc(memory, E),
            0x9C => self.execute_sbc(memory, H),
            0x9D => self.execute_sbc(memory, L),
            0x9E => self.execute_sbc(memory, RegisterHL),
            0x9F => self.execute_sbc(memory, A),
            0xDE => self.execute_sbc(memory, Immediate8),

            0xA0 => self.execute_and(memory, B),
            0xA1 => self.execute_and(memory, C),
            0xA2 => self.execute_and(memory, D),
            0xA3 => self.execute_and(memory, E),
            0xA4 => self.execute_and(memory, H),
            0xA5 => self.execute_and(memory, L),
            0xA6 => self.execute_and(memory, RegisterHL),
            0xA7 => self.execute_and(memory, A),
            0xE6 => self.execute_and(memory, Immediate8),

            0xA8 => self.execute_xor(memory, B),
            0xA9 => self.execute_xor(memory, C),
            0xAA => self.execute_xor(memory, D),
            0xAB => self.execute_xor(memory, E),
            0xAC => self.execute_xor(memory, H),
            0xAD => self.execute_xor(memory, L),
            0xAE => self.execute_xor(memory, RegisterHL),
            0xAF => self.execute_xor(memory, A),
            0xEE => self.execute_xor(memory, Immediate8),

            0xB0 => self.execute_or(memory, B),
            0xB1 => self.execute_or(memory, C),
            0xB2 => self.execute_or(memory, D),
            0xB3 => self.execute_or(memory, E),
            0xB4 => self.execute_or(memory, H),
            0xB5 => self.execute_or(memory, L),
            0xB6 => self.execute_or(memory, RegisterHL),
            0xB7 => self.execute_or(memory, A),
            0xF6 => self.execute_or(memory, Immediate8),

            0xB8 => self.execute_cp(memory, B),
            0xB9 => self.execute_cp(memory, C),
            0xBA => self.execute_cp(memory, D),
            0xBB => self.execute_cp(memory, E),
            0xBC => self.execute_cp(memory, H),
            0xBD => self.execute_cp(memory, L),
            0xBE => self.execute_cp(memory, RegisterHL),
            0xBF => self.execute_cp(memory, A),
            0xFE => self.execute_cp(memory, Immediate8),

            0x27 => self.execute_daa(),
            0x2F => self.execute_cpl(),
            0x37 => self.execute_scf(),
            0x3F => self.execute_ccf(),

            0x03 => self.execute_inc16(BC),
            0x13 => self.execute_inc16(DE),
            0x23 => self.execute_inc16(HL),
            0x33 => self.sp += 1,

            0x0B => self.execute_dec16(BC),
            0x1B => self.execute_dec16(DE),
            0x2B => self.execute_dec16(HL),
            0x3B => self.sp -= 1,

            0x09 => self.execute_add16(memory, BC),
            0x19 => self.execute_add16(memory, DE),
            0x29 => self.execute_add16(memory, HL),
            0x39 => self.execute_add16(memory, SP),

            0xE8 => self.execute_add_sp_imm8(memory, SP),
            0xF8 => self.execute_add_sp_imm8(memory, HL),

            0xCB => self.execute_prefixed_instruction(memory),
            0xF3 => self.ime = false,
            0xFB => self.ime = true,
            0x76 => self.halted = true,

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

    fn execute_inc16(&mut self, reg: TwoRegisterIndex) {
        self.regs.write(reg, self.regs.read(reg) + 1);
    }

    fn execute_dec16(&mut self, reg: TwoRegisterIndex) {
        self.regs.write(reg, self.regs.read(reg) - 1);
    }

    fn execute_add16<T>(&mut self, memory: &Memory, src: T) where
    Self: Operand16<T> {
        let old_zero = read_bit(self.regs[F], ZERO_FLAG);
        let (result, flags) = add_u16(self.regs.read(HL), self.read16(memory, src));
        self.regs.write(HL, result);
        self.regs.write_flags(Flags { zero: old_zero, ..flags });
    }

    fn execute_add_sp_imm8<T>(&mut self, memory: &mut Memory, dst: T) where
    Self: Operand16<T> {
        let n = self.read_oper(memory, Immediate8);
        let (result, flags) = self.sum_sp_n(n);
        self.write16(memory, dst, result);
        self.regs.write_flags(flags);
    }

    fn execute_load_reg_reg(&mut self, memory: &mut Memory, opcode: u8) {
        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        let opcode_index = opcode - 0x40;
        let dst_index = order[(opcode_index / 0x08) as usize];
        let src_index = order[(opcode_index % 0x08) as usize];

        match (dst_index, src_index) {
            (Some(reg), Some(reg1)) => self.execute_load(memory, reg, reg1),
            (None, Some(reg)) => self.execute_load(memory, RegisterHL, reg),
            (Some(reg), None) => self.execute_load(memory, reg, RegisterHL),
            (None, None) => unreachable!(),
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

    fn execute_daa(&mut self) {
        let val = self.regs[A];
        let mut correction = 0;
        let mut carry = read_bit(self.regs[F], CARRY_FLAG);
        if read_bit(self.regs[F], HALF_CARRY_FLAG)
            || (!read_bit(self.regs[F], SUBTRACT_FLAG) && ((val & 0x0F) > 0x09)) {
            correction |= 0x06;
        }

        if read_bit(self.regs[F], CARRY_FLAG)
            || (!read_bit(self.regs[F], SUBTRACT_FLAG) && (val > 0x99)) {
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
    }

    fn execute_cpl(&mut self) {
        self.regs[A] ^= 0xFF;
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, true);
    }

    fn execute_scf(&mut self) {
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, false);
        self.set_flag(CARRY_FLAG, true);
    }

    fn execute_ccf(&mut self) {
        let old_carry = read_bit(self.regs[F], CARRY_FLAG);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, false);
        self.set_flag(CARRY_FLAG, old_carry ^ true);
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

    fn execute_inc<T: Copy>(&mut self, memory: &mut Memory, src: T)
        where Self: Operand8<T> {
        let old = self.read_oper(memory, src);
        let result = old + 1;
        self.write_oper(memory, src, result);
        self.set_flag(ZERO_FLAG, result == 0);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, ((old & 0xF) + 1) > 0xF);
    }

    fn execute_dec<T: Copy>(&mut self, memory: &mut Memory, src: T)
        where Self: Operand8<T> {
        let old = self.read_oper(memory, src);
        let result = old - 1;
        self.write_oper(memory, src, result);
        self.set_flag(ZERO_FLAG, result == 0);
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
    }

    fn execute_load<D, S>(&mut self, memory: &mut Memory, dst: D, src: S) where
    Self: Operand8<D> + Operand8<S> {
        let val = self.read_oper(memory, src);
        self.write_oper(memory, dst, val);
    }

    fn execute_load16<D, S>(&mut self, memory: &mut Memory, dst: D, src: S) where
    Self: Operand16<D> + Operand16<S> {
        let val = self.read16(memory, src);
        self.write16(memory, dst, val);
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

    fn execute_jr(&mut self, memory: &Memory) {
        let offset = self.read_oper(memory, Immediate8) as i8;
        self.pc = ((self.pc as i32) + (offset as i32)) as u16;
    }

    fn execute_jr_cc(&mut self, memory: &Memory, condition: Condition) -> u32 {
        // Offset is signed
        let offset = self.read_oper(memory, Immediate8) as i8;

        if self.eval_condition(condition) {
            self.pc = ((self.pc as i32) + (offset as i32)) as u16;
            self.current_instruction.cycles.get_extra_cycles()
        } else {
            0
        }
    }

    fn execute_jp(&mut self, memory: &Memory) {
        let addr = self.read16(memory, Immediate16);
        self.pc = addr;
    }

    fn execute_jp_cc(&mut self, memory: &Memory, condition: Condition) -> u32 {
        let addr = self.read16(memory, Immediate16);

        if self.eval_condition(condition) {
            self.pc = addr;
            self.current_instruction.cycles.get_extra_cycles()
        } else {
            0
        }
    }

    fn execute_call(&mut self, memory: &mut Memory) {
        let addr = self.read16(memory, Immediate16);
        self.sp -= 2;
        Self::write_mem_u16(memory, self.sp, self.pc);
        self.pc = addr;
    }

    fn execute_call_cc(&mut self, memory: &mut Memory, condition: Condition) -> u32 {
        let addr = self.read16(memory, Immediate16);

        if self.eval_condition(condition) {
            self.sp -= 2;
            Self::write_mem_u16(memory, self.sp, self.pc);
            self.pc = addr;
            self.current_instruction.cycles.get_extra_cycles()
        } else {
            0
        }
    }

    fn execute_ret(&mut self, memory: &Memory) {
        self.pc = Self::read_mem_u16(memory, self.sp);
        self.sp += 2;
    }

    fn execute_ret_cc(&mut self, memory: &Memory, condition: Condition) -> u32 {
        if self.eval_condition(condition) {
            self.pc = Self::read_mem_u16(memory, self.sp);
            self.sp += 2;
            self.current_instruction.cycles.get_extra_cycles()
        } else {
            0
        }
    }

    fn execute_rst(&mut self, memory: &mut Memory, addr: u16) {
        self.sp -= 2;
        Self::write_mem_u16(memory, self.sp, self.pc);
        self.pc = addr;
    }

    fn execute_push(&mut self, memory: &mut Memory, index: TwoRegisterIndex) {
        self.sp -= 2;
        Self::write_mem_u16(memory, self.sp, self.regs.read(index));
    }

    fn execute_pop(&mut self, memory: &mut Memory, index: TwoRegisterIndex) {
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
