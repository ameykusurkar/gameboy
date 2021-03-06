use crate::registers::Flags;
use crate::registers::RegisterIndex;
use crate::registers::RegisterIndex::*;
use crate::registers::Registers;
use crate::registers::TwoRegisterIndex;
use crate::registers::TwoRegisterIndex::*;
use crate::registers::{CARRY_FLAG, HALF_CARRY_FLAG, SUBTRACT_FLAG, ZERO_FLAG};

use crate::instruction::{AddressingMode, Instruction};
use crate::instruction::{INSTRUCTIONS, PREFIXED_INSTRUCTIONS};
use crate::memory::Memory;
use crate::new_instruction::{
    AddressSource, AluOperation, MemoryOperation, MicroInstruction, NewInstructionIterator,
    RegisterOperation, REGISTRY,
};

use crate::utils::{read_bit, set_bit};

const INTERRUPT_ADDRS: [u16; 5] = [0x40, 0x48, 0x50, 0x58, 0x60];

pub struct Cpu {
    pub regs: Registers,
    ime: bool,
    remaining_cycles: u32,
    total_clock_cycles: u32,
    halted: bool,
    pub current_instruction: &'static Instruction<'static>,
    current_new_instruction: Option<NewInstructionIterator<'static>>,
    prefixed_mode: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum Condition {
    NZ,
    Z,
    NC,
    C,
}

trait ConditionEvaluate {
    fn eval_condition(&self, condition: Condition) -> bool;
}

impl ConditionEvaluate for Cpu {
    fn eval_condition(&self, condition: Condition) -> bool {
        match condition {
            Condition::NZ => !read_bit(self.regs.read(F), ZERO_FLAG),
            Condition::Z => read_bit(self.regs.read(F), ZERO_FLAG),
            Condition::NC => !read_bit(self.regs.read(F), CARRY_FLAG),
            Condition::C => read_bit(self.regs.read(F), CARRY_FLAG),
        }
    }
}

struct Immediate8;

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
        self.regs.read(src)
    }

    fn write_oper(&mut self, _memory: &mut Memory, src: RegisterIndex, val: u8) {
        self.regs.write(src, val);
    }
}

impl Operand8<AddrReg16> for Cpu {
    fn read_oper(&mut self, memory: &Memory, src: AddrReg16) -> u8 {
        let AddrReg16(reg) = src;
        let addr = self.regs.read16(reg);
        memory.cpu_read(addr)
    }

    fn write_oper(&mut self, memory: &mut Memory, src: AddrReg16, val: u8) {
        let AddrReg16(reg) = src;
        let addr = self.regs.read16(reg);
        memory.cpu_write(addr, val);
    }
}

impl Operand8<Immediate8> for Cpu {
    fn read_oper(&mut self, memory: &Memory, _src: Immediate8) -> u8 {
        let val = memory.cpu_read(self.regs.read16(PC));
        self.regs.write16(PC, self.regs.read16(PC).wrapping_add(1));
        val
    }

    fn write_oper(&mut self, _memory: &mut Memory, _src: Immediate8, _val: u8) {}
}

impl Operand16<TwoRegisterIndex> for Cpu {
    fn read16(&mut self, _memory: &Memory, src: TwoRegisterIndex) -> u16 {
        self.regs.read16(src)
    }

    fn write16(&mut self, _memory: &mut Memory, src: TwoRegisterIndex, val: u16) {
        self.regs.write16(src, val);
    }
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            regs: Registers::default(),
            ime: false,
            remaining_cycles: 0,
            total_clock_cycles: 0,
            halted: false,
            // This should get populated when cpu starts running
            current_instruction: &INSTRUCTIONS[0],
            current_new_instruction: None,
            prefixed_mode: false,
        }
    }

    #[allow(dead_code)]
    pub fn skip_bootrom(&mut self, memory: &mut Memory) {
        self.regs.write16(PC, 0x100);
        self.regs.write16(SP, 0xFFFE);
        memory.cpu_write(0xFF50, 1);
    }

    pub fn step(&mut self, memory: &mut Memory) {
        if self.halted && memory.io_registers.get_pending_interrupts() > 0 {
            self.halted = false;
        }

        if !self.halted {
            if self.remaining_cycles == 0 {
                // If we are in prefixed mode, that means that the instruction is not yet complete
                // TODO: Find a cleaner way to do this
                if !self.prefixed_mode {
                    let interrupt_was_serviced = self.check_and_handle_interrupts(memory);

                    if interrupt_was_serviced {
                        self.remaining_cycles += 5;
                    }
                }

                let opcode = self.read_oper(memory, Immediate8);

                match REGISTRY.fetch(opcode, self.prefixed_mode) {
                    Some(instruction) => {
                        if instruction.num_cycles() > 0 {
                            self.remaining_cycles += instruction.num_cycles() as u32;
                            self.current_new_instruction = Some(instruction.into_iter());
                        } else {
                            // We don't do anything for a NOOP/PREFIXED instruction,
                            // but add one cycle to simulate fetching the PC
                            self.remaining_cycles += 1;
                        }

                        if self.prefixed_mode {
                            self.prefixed_mode = false;
                        } else {
                            self.prefixed_mode = opcode == 0xCB;
                        }
                    }
                    None => unimplemented!(
                        "Unimplemented opcode {:02x}, prefixed: {}",
                        opcode,
                        self.prefixed_mode
                    ),
                }
            }

            if self.current_new_instruction.is_some() {
                match self.current_new_instruction.as_mut().unwrap().next() {
                    Some(micro_instruction) => {
                        let should_proceed = self.execute_micro(memory, &micro_instruction);

                        if !should_proceed {
                            // Consumes the iterator
                            self.current_new_instruction.as_mut().unwrap().last();
                            self.remaining_cycles = 1;
                        }

                        // TODO: Clean up!
                        if self.current_new_instruction.as_mut().unwrap().is_empty() {
                            self.current_new_instruction = None;

                            if micro_instruction.has_memory_access() {
                                // Add an extra cycle to simulate fetching the next instruction, as
                                // it already has a memory access in the current micro instruction.
                                // Will be removed once all instructions follow the new format.
                                self.remaining_cycles += 1;
                            }
                        }
                    }
                    // TODO: Clean up!
                    None => panic!(),
                }
            }

            self.remaining_cycles -= 1;
        }

        self.total_clock_cycles += 1;
        self.update_timers(memory);
    }

    fn execute_micro(&mut self, memory: &mut Memory, micro_instruction: &MicroInstruction) -> bool {
        micro_instruction
            .memory_operation
            .map(|memory_op| match memory_op {
                MemoryOperation::Read(address_source, mem_reg) => {
                    let val = match address_source {
                        AddressSource::Immediate => self.read_oper(memory, Immediate8),
                        AddressSource::HighPage(src_reg) => {
                            memory.cpu_read(0xFF00 | self.regs.read(src_reg) as u16)
                        }
                        AddressSource::Reg(reg) => self.read_oper(memory, AddrReg16(reg)),
                        AddressSource::RegWithOffset(reg, offset) => {
                            let addr = self.regs.read16(reg) + offset;
                            memory.cpu_read(addr)
                        }
                    };

                    self.regs.write(mem_reg, val);
                }
                MemoryOperation::Write(address_source, mem_reg) => {
                    let addr = match address_source {
                        AddressSource::Immediate => panic!(),
                        AddressSource::HighPage(src_reg) => 0xFF00 | self.regs.read(src_reg) as u16,
                        AddressSource::Reg(reg) => self.regs.read16(reg),
                        AddressSource::RegWithOffset(reg, offset) => self.regs.read16(reg) + offset,
                    };

                    memory.cpu_write(addr, self.regs.read(mem_reg));
                }
                MemoryOperation::Noop => (),
            });

        micro_instruction
            .register_operation
            .map(|register_op| match register_op {
                RegisterOperation::Load(dst, src) => {
                    let val = self.read_oper(memory, src);
                    self.write_oper(memory, dst, val);
                }
                RegisterOperation::Load16(dst, src) => {
                    self.regs.write16(dst, self.regs.read16(src));
                }
                RegisterOperation::IncReg16(reg) => {
                    self.regs
                        .write16(reg, self.regs.read16(reg).wrapping_add(1));
                }
                RegisterOperation::DecReg16(reg) => {
                    self.regs
                        .write16(reg, self.regs.read16(reg).wrapping_sub(1));
                }
                RegisterOperation::AddReg16(reg) => {
                    self.execute_add16(memory, reg);
                }
                RegisterOperation::AddSP8(reg16, reg) => {
                    self.execute_add_sp(memory, reg16, reg);
                }
                RegisterOperation::SignedAddReg16(reg16, reg) => {
                    let offset = (self.regs.read(reg) as i8) as i32;
                    let orig_val = self.regs.read16(reg16) as i32;
                    self.regs.write16(reg16, (orig_val + offset) as u16);
                }
                RegisterOperation::LoadRstAddress(addr) => {
                    self.regs.write16(PC, addr);
                }
                RegisterOperation::Alu(alu_operation, reg) => match alu_operation {
                    AluOperation::Inc => self.execute_inc(memory, reg),
                    AluOperation::Dec => self.execute_dec(memory, reg),
                    AluOperation::Add => self.execute_add(memory, reg),
                    AluOperation::Adc => self.execute_adc(memory, reg),
                    AluOperation::Sub => self.execute_sub(memory, reg),
                    AluOperation::Sbc => self.execute_sbc(memory, reg),
                    AluOperation::And => self.execute_and(memory, reg),
                    AluOperation::Xor => self.execute_xor(memory, reg),
                    AluOperation::Or => self.execute_or(memory, reg),
                    AluOperation::Cp => self.execute_cp(memory, reg),
                    AluOperation::Daa => self.execute_daa(),
                    AluOperation::Scf => self.execute_scf(),
                    AluOperation::Cpl => self.execute_cpl(),
                    AluOperation::Ccf => self.execute_ccf(),

                    AluOperation::Rlc => self.execute_rlc(memory, reg),
                    AluOperation::Rrc => self.execute_rrc(memory, reg),
                    AluOperation::Rl => self.execute_rl(memory, reg),
                    AluOperation::Rr => self.execute_rr(memory, reg),
                    AluOperation::Sla => self.execute_sla(memory, reg),
                    AluOperation::Sra => self.execute_sra(memory, reg),
                    AluOperation::Swap => self.execute_swap(memory, reg),
                    AluOperation::Srl => self.execute_srl(memory, reg),
                    AluOperation::Tst(bit) => self.execute_tst(memory, bit, reg),
                    AluOperation::Res(bit) => self.execute_res(memory, bit, reg),
                    AluOperation::Set(bit) => self.execute_set(memory, bit, reg),

                    AluOperation::Rlca => {
                        self.execute_rlc(memory, reg);
                        self.set_flag(ZERO_FLAG, false);
                    }
                    AluOperation::Rrca => {
                        self.execute_rrc(memory, reg);
                        self.set_flag(ZERO_FLAG, false);
                    }
                    AluOperation::Rla => {
                        self.execute_rl(memory, reg);
                        self.set_flag(ZERO_FLAG, false);
                    }
                    AluOperation::Rra => {
                        self.execute_rr(memory, reg);
                        self.set_flag(ZERO_FLAG, false);
                    }
                },
            });

        micro_instruction.set_interrupts.map(|val| {
            self.ime = val;
        });

        if micro_instruction.set_halted {
            self.halted = true;
        }

        micro_instruction
            .check_condition
            .map_or(true, |condition| self.eval_condition(condition))
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

    fn update_timers(&mut self, memory: &mut Memory) {
        memory.div_cycle();

        if memory.io_registers.timer_is_active() {
            let cycles_per_update = match memory.io_registers.get_timer_mode() {
                0b00 => 256,       // 4096 Hz
                0b01 => 4,         // 262,144 Hz
                0b10 => 16,        // 65,536 Hz
                0b11..=0xFF => 64, // 16,384 Hz
            };

            if self.total_clock_cycles % cycles_per_update == 0 {
                if memory.io_registers.get_timer() == 0xFF {
                    // If the timer is going to overflow, request a timer interrupt
                    memory.io_registers.request_interrupt(2);
                    memory.io_registers.reload_timer();
                } else {
                    memory.io_registers.increment_timer();
                }
            }
        }
    }

    pub fn disassemble(
        &self,
        memory: &Memory,
        start_addr: u16,
        end_addr: u16,
    ) -> Vec<(u16, String)> {
        let mut current_addr = start_addr;
        let mut instruction_reprs = Vec::new();

        loop {
            if current_addr > end_addr {
                break;
            };

            let instruction = self.fetch_instruction(memory, current_addr);
            let num_bytes = instruction.num_bytes as u32;

            if current_addr as u32 + num_bytes > end_addr as u32 {
                break;
            };

            let operand_start_addr = current_addr + 1 + (instruction.prefixed as u16);
            let repr = self.build_instruction_repr(memory, operand_start_addr, instruction);
            instruction_reprs.push((current_addr, repr));
            current_addr += num_bytes as u16;
        }

        instruction_reprs
    }

    // Depends on `instruction.repr` being in the correct format
    fn build_instruction_repr(
        &self,
        memory: &Memory,
        addr: u16,
        instruction: &Instruction,
    ) -> String {
        let (operand, _) = self.fetch_operand(memory, addr, instruction);
        let mut repr = String::from(instruction.repr);

        match &instruction.addressing_mode {
            AddressingMode::Implied => (),
            AddressingMode::Imm8 => {
                repr.find("d").map(|index| {
                    repr.replace_range(index..index + 2, &format!("{:02x}", operand));
                });
            }
            AddressingMode::Imm16 => {
                repr.find("d").map(|index| {
                    repr.replace_range(index..index + 3, &format!("{:04x}", operand));
                });
            }
            AddressingMode::Addr16 => {
                repr.find("a").map(|index| {
                    repr.replace_range(index..index + 3, &format!("{:04x}", operand));
                });
            }
            AddressingMode::ZeroPageOffset => {
                repr.find("a").map(|index| {
                    repr.replace_range(index..index + 2, &format!("{:04x}", operand));
                });
            }
            AddressingMode::SignedAddrOffset => {
                repr.find("r").map(|index| {
                    repr.replace_range(index..index + 2, &format!("{:04x}", operand));
                });
            }
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
            }
            AddressingMode::ZeroPageOffset => (0xFF00 | lsb, 1),
            AddressingMode::SignedAddrOffset => {
                let lsb = lsb as i8;
                // The offset assumes that the PC has already been incremented
                let target_addr = ((addr + 1) as i32) + (lsb as i32);
                (target_addr as u16, 1)
            }
        }
    }

    fn check_and_handle_interrupts(&mut self, memory: &mut Memory) -> bool {
        let pending_interrupts = memory.io_registers.get_pending_interrupts();

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

    fn handle_interrupt(&mut self, memory: &mut Memory, interrupt_no: u8) {
        // This routine should take 5 machine cycles
        self.ime = false;
        memory.io_registers.unset_interrupt(interrupt_no);
        self.regs.write16(SP, self.regs.read16(SP) - 2);
        Self::write_mem_u16(memory, self.regs.read16(SP), self.regs.read16(PC));
        self.regs
            .write16(PC, INTERRUPT_ADDRS[interrupt_no as usize]);
    }

    fn execute_add16<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand16<T>,
    {
        let old_zero = read_bit(self.regs.read(F), ZERO_FLAG);
        let (result, flags) = add_u16(self.regs.read16(HL), self.read16(memory, src));
        self.regs.write16(HL, result);
        self.regs.write_flags(Flags {
            zero: old_zero,
            ..flags
        });
    }

    fn execute_add_sp<T>(&mut self, memory: &mut Memory, dst: T, src: RegisterIndex)
    where
        Self: Operand16<T>,
    {
        let n = self.regs.read(src);
        let (result, flags) = self.sum_sp_n(n);
        self.write16(memory, dst, result);
        self.regs.write_flags(flags);
    }

    fn execute_add<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, flags) = add_u8(self.regs.read(A), val);
        self.regs.write(A, result);
        self.regs.write_flags(flags);
    }

    fn execute_adc<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let old_carry = read_bit(self.regs.read(F), CARRY_FLAG);
        let (result, flags) = adc_u8(self.regs.read(A), val, old_carry);
        self.regs.write(A, result);
        self.regs.write_flags(flags);
    }

    fn execute_sub<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, flags) = sub_u8(self.regs.read(A), val);
        self.regs.write(A, result);
        self.regs.write_flags(flags);
    }

    fn execute_sbc<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let old_carry = read_bit(self.regs.read(F), CARRY_FLAG);
        let (result, flags) = sbc_u8(self.regs.read(A), val, old_carry);
        self.regs.write(A, result);
        self.regs.write_flags(flags);
    }

    fn execute_and<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        self.regs.write(A, self.regs.read(A) & val);
        self.regs.write_flags(Flags {
            zero: self.regs.read(A) == 0,
            half_carry: true,
            ..Flags::default()
        });
    }

    fn execute_xor<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        self.regs.write(A, self.regs.read(A) ^ val);
        self.regs.write_flags(Flags {
            zero: self.regs.read(A) == 0,
            ..Flags::default()
        });
    }

    fn execute_or<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        self.regs.write(A, self.regs.read(A) | val);
        self.regs.write_flags(Flags {
            zero: self.regs.read(A) == 0,
            ..Flags::default()
        });
    }

    fn execute_cp<T>(&mut self, memory: &Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (_, flags) = sub_u8(self.regs.read(A), val);
        self.regs.write_flags(flags);
    }

    fn execute_daa(&mut self) {
        let val = self.regs.read(A);
        let mut correction = 0;
        let mut carry = read_bit(self.regs.read(F), CARRY_FLAG);
        if read_bit(self.regs.read(F), HALF_CARRY_FLAG)
            || (!read_bit(self.regs.read(F), SUBTRACT_FLAG) && ((val & 0x0F) > 0x09))
        {
            correction |= 0x06;
        }

        if read_bit(self.regs.read(F), CARRY_FLAG)
            || (!read_bit(self.regs.read(F), SUBTRACT_FLAG) && (val > 0x99))
        {
            correction |= 0x60;
            carry = true;
        }

        if read_bit(self.regs.read(F), SUBTRACT_FLAG) {
            self.regs
                .write(A, self.regs.read(A).wrapping_sub(correction));
        } else {
            self.regs
                .write(A, self.regs.read(A).wrapping_add(correction));
        }

        self.set_flag(ZERO_FLAG, self.regs.read(A) == 0);
        self.set_flag(HALF_CARRY_FLAG, false);
        self.set_flag(CARRY_FLAG, carry);
    }

    fn execute_cpl(&mut self) {
        self.regs.write(A, self.regs.read(A) ^ 0xFF);
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, true);
    }

    fn execute_scf(&mut self) {
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, false);
        self.set_flag(CARRY_FLAG, true);
    }

    fn execute_ccf(&mut self) {
        let old_carry = read_bit(self.regs.read(F), CARRY_FLAG);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, false);
        self.set_flag(CARRY_FLAG, old_carry ^ true);
    }

    fn execute_rlc<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_left(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rrc<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_right(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rl<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let carry = read_bit(self.regs.read(F), CARRY_FLAG);
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_left_through_carry(val, carry);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_rr<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let carry = read_bit(self.regs.read(F), CARRY_FLAG);
        let val = self.read_oper(memory, src);
        let (result, carry) = rotate_right_through_carry(val, carry);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_sla<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, carry) = shift_left(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_sra<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, carry) = shift_right_arithmetic(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_swap<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let result = (val & 0x0F) << 4 | (val & 0xF0) >> 4;
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            ..Flags::default()
        });
    }

    fn execute_srl<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let (result, carry) = shift_right_logical(val);
        self.write_oper(memory, src, result);
        self.regs.write_flags(Flags {
            zero: result == 0,
            carry,
            ..Flags::default()
        });
    }

    fn execute_tst<T: Copy>(&mut self, memory: &Memory, bit: u8, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let result = read_bit(val, bit);
        self.set_flag(ZERO_FLAG, !result);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, true);
    }

    fn execute_res<T: Copy>(&mut self, memory: &mut Memory, bit: u8, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let result = set_bit(val, bit, false);
        self.write_oper(memory, src, result);
    }

    fn execute_set<T: Copy>(&mut self, memory: &mut Memory, bit: u8, src: T)
    where
        Self: Operand8<T>,
    {
        let val = self.read_oper(memory, src);
        let result = set_bit(val, bit, true);
        self.write_oper(memory, src, result);
    }

    fn execute_inc<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let old = self.read_oper(memory, src);
        let result = old.wrapping_add(1);
        self.write_oper(memory, src, result);
        self.set_flag(ZERO_FLAG, result == 0);
        self.set_flag(SUBTRACT_FLAG, false);
        self.set_flag(HALF_CARRY_FLAG, ((old & 0xF) + 1) > 0xF);
    }

    fn execute_dec<T: Copy>(&mut self, memory: &mut Memory, src: T)
    where
        Self: Operand8<T>,
    {
        let old = self.read_oper(memory, src);
        let result = old.wrapping_sub(1);
        self.write_oper(memory, src, result);
        self.set_flag(ZERO_FLAG, result == 0);
        self.set_flag(SUBTRACT_FLAG, true);
        self.set_flag(HALF_CARRY_FLAG, (old & 0xF) == 0);
    }

    fn sum_sp_n(&mut self, n: u8) -> (u16, Flags) {
        // n is a signed value
        let n = n as i8;
        let sp = self.regs.read16(SP) as i32;

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

        (
            result as u16,
            Flags {
                half_carry,
                carry,
                ..Flags::default()
            },
        )
    }

    fn write_mem_u16(memory: &mut Memory, addr: u16, val: u16) {
        let lsb = (val & 0x00FF) as u8;
        let msb = ((val & 0xFF00) >> 8) as u8;
        memory.cpu_write(addr, lsb);
        memory.cpu_write(addr + 1, msb);
    }

    fn set_flag(&mut self, flag_bit: u8, val: bool) {
        self.regs
            .write(F, set_bit(self.regs.read(F), flag_bit, val));
    }
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
    let result = x.wrapping_add(y);

    let flags = Flags {
        zero: result == 0,
        subtract: false,
        half_carry: (x & 0x0FFF) + (y & 0x0FFF) > 0x0FFF,
        carry: (x as u32) + (y as u32) > 0xFFFF,
    };

    (result, flags)
}
