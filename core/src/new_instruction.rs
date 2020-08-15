use crate::registers::RegisterIndex;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex;
use crate::registers::TwoRegisterIndex::*;
use crate::cpu::Condition;

use lazy_static::lazy_static;

lazy_static! {
    pub static ref REGISTRY: InstructionRegistry = InstructionRegistry::new();
}

#[derive(Clone)]
pub struct NewInstruction {
    micro_instructions: std::collections::VecDeque<MicroInstruction>,
}

impl NewInstruction {
    fn new() -> NewInstruction {
        NewInstruction {
            micro_instructions: std::collections::VecDeque::new(),
        }
    }

    fn push(mut self, micro_instruction: MicroInstruction) -> Self {
        (&mut self).micro_instructions.push_back(micro_instruction);
        self
    }

    pub fn complete(&mut self) {
        self.micro_instructions.clear()
    }

    pub fn num_cycles(&self) -> usize {
        self.micro_instructions.len()
    }

    pub fn next_micro(&mut self) -> Option<MicroInstruction> {
        self.micro_instructions.pop_front()
    }

    fn set_interrupt_value(mut self, val: bool) -> Self {
        match self.micro_instructions.pop_back() {
            Some(mut micro_instruction) => {
                micro_instruction.set_interrupts = Some(val);
                self.micro_instructions.push_back(micro_instruction);
            },
            None => unimplemented!("No micro instruction found to enable interrupts!"),
        }

        self
    }

    fn and_enable_interrupts(self) -> Self {
        self.set_interrupt_value(true)
    }

    fn and_disable_interrupts(self) -> Self {
        self.set_interrupt_value(false)
    }

    fn append_register_operation(mut self, reg_op: RegisterOperation) -> Self {
        match self.micro_instructions.pop_back() {
            Some(mut micro_instruction) => {
                match micro_instruction {
                    MicroInstruction { register_operation: None, .. } => {
                        micro_instruction.register_operation = Some(reg_op);
                        self.micro_instructions.push_back(micro_instruction);
                    }
                    _ => unimplemented!("Cannot add register operation!"),
                }
            },
            None => unimplemented!("No micro instruction found to add register operation!"),
        }

        self
    }

    fn append_condition_check(mut self, condition: Condition) -> Self {
        match self.micro_instructions.pop_back() {
            Some(mut micro_instruction) => {
                match micro_instruction {
                    MicroInstruction { check_condition: None, .. } => {
                        micro_instruction.check_condition = Some(condition);
                        self.micro_instructions.push_back(micro_instruction);
                    }
                    _ => unimplemented!("Cannot add condition check!"),
                }
            },
            None => unimplemented!("No micro instruction found to add condition check!"),
        }

        self
    }

    fn and_proceed_if(self, condition: Condition) -> Self {
        self.append_condition_check(condition)
    }

    fn noop(self) -> Self {
        self.push(
            MicroInstruction::default().with_memory_operation(MemoryOperation::Noop)
        )
    }

    fn empty(self) -> Self {
        self.push(MicroInstruction::default())
    }

    fn and_halt(mut self) -> Self {
        match self.micro_instructions.pop_back() {
            Some(mut micro_instruction) => {
                micro_instruction.set_halted = true;
                self.micro_instructions.push_back(micro_instruction);
            },
            None => unimplemented!("No micro instruction found to set halted!"),
        }

        self
    }

    fn load(self, addr_source: AddressSource, reg: RegisterIndex) -> Self {
        self.push(
            MicroInstruction::default()
                .with_memory_operation(MemoryOperation::Read(addr_source, reg))
        )
    }

    fn load_imm(self, reg: RegisterIndex) -> Self {
        self.load(AddressSource::Immediate, reg)
    }

    fn and_move_rst_addr(self, addr: u16) -> Self {
        self.append_register_operation(RegisterOperation::LoadRstAddress(addr))
    }

    fn and_move_reg(self, dst: RegisterIndex, src: RegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::Load(dst, src))
    }

    fn move_reg16(self, dst: TwoRegisterIndex, src: TwoRegisterIndex) -> Self {
        self.noop().and_move_reg16(dst, src)
    }

    fn and_move_reg16(self, dst: TwoRegisterIndex, src: TwoRegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::Load16(dst, src))
    }

    fn and_signed_add_reg16(self, reg16: TwoRegisterIndex, reg: RegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::SignedAddReg16(reg16, reg))
    }

    fn store(self, addr_source: AddressSource, reg: RegisterIndex) -> Self {
        self.push(
            MicroInstruction::default()
                .with_memory_operation(MemoryOperation::Write(addr_source, reg))
        )
    }

    fn and_inc16(self, reg16: TwoRegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::IncReg16(reg16))
    }

    fn and_dec16(self, reg16: TwoRegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::DecReg16(reg16))
    }

    fn and_add16(self, reg16: TwoRegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::AddReg16(reg16))
    }

    fn and_add_sp(self, reg16: TwoRegisterIndex, reg: RegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::AddSP8(reg16, reg))
    }

    fn and_alu(self, alu_operation: AluOperation, reg: RegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::Alu(alu_operation, reg))
    }
}

#[derive(Debug, Clone, Default)]
pub struct MicroInstruction {
    pub memory_operation: Option<MemoryOperation>,
    pub register_operation: Option<RegisterOperation>,
    pub check_condition: Option<Condition>,
    pub set_interrupts: Option<bool>,
    pub set_halted: bool,
}

impl MicroInstruction {
    pub fn has_memory_access(&self) -> bool {
        self.memory_operation.is_some()
    }

    fn with_memory_operation(mut self, memory_operation: MemoryOperation) -> Self {
        self.memory_operation = Some(memory_operation);
        self
    }
}

#[derive(Debug, Copy, Clone)]
pub enum MemoryOperation {
    Noop,
    Read(AddressSource, RegisterIndex),
    Write(AddressSource, RegisterIndex),
}

#[derive(Debug, Copy, Clone)]
pub enum RegisterOperation {
    Load(RegisterIndex, RegisterIndex),
    Load16(TwoRegisterIndex, TwoRegisterIndex),
    IncReg16(TwoRegisterIndex),
    DecReg16(TwoRegisterIndex),
    AddReg16(TwoRegisterIndex),
    AddSP8(TwoRegisterIndex, RegisterIndex),
    SignedAddReg16(TwoRegisterIndex, RegisterIndex),
    LoadRstAddress(u16),
    Alu(AluOperation, RegisterIndex),
}

#[derive(Debug, Copy, Clone)]
pub enum AluOperation {
    Inc, Dec, Add, Adc, Sub, Sbc, And, Xor, Or, Cp, Daa, Scf, Cpl, Ccf,
    Rlc, Rrc, Rl, Rr, Sla, Sra, Swap, Srl, Tst(u8), Res(u8), Set(u8),
    Rlca, Rrca, Rla, Rra,
}

#[derive(Debug, Copy, Clone)]
pub enum AddressSource {
    Immediate,
    HighPage(RegisterIndex),
    Reg(TwoRegisterIndex),
    RegWithOffset(TwoRegisterIndex, u16),
}

pub struct InstructionRegistry {
    instruction_map: std::collections::HashMap<u8, NewInstruction>,
    prefixed_instruction_map: std::collections::HashMap<u8, NewInstruction>,
}

impl InstructionRegistry {
    pub fn new() -> Self {
        let mut instruction_map = std::collections::HashMap::new();

        for (opcode, reg16) in [(0x01, BC), (0x11, DE), (0x21, HL), (0x31, SP)].iter() {
            let (high_reg, low_reg) = reg16.split_index();

            instruction_map.insert(
                *opcode,
                NewInstruction::new().load_imm(low_reg).load_imm(high_reg),
            );
        }

        instruction_map.insert(0x00, NewInstruction::new());
        instruction_map.insert(0xCB, NewInstruction::new());

        instruction_map.insert(0x03, build_inc16_instruction(BC));
        instruction_map.insert(0x13, build_inc16_instruction(DE));
        instruction_map.insert(0x23, build_inc16_instruction(HL));
        instruction_map.insert(0x33, build_inc16_instruction(SP));

        instruction_map.insert(0x0B, build_dec16_instruction(BC));
        instruction_map.insert(0x1B, build_dec16_instruction(DE));
        instruction_map.insert(0x2B, build_dec16_instruction(HL));
        instruction_map.insert(0x3B, build_dec16_instruction(SP));

        instruction_map.insert(0x09, build_add16_instruction(BC));
        instruction_map.insert(0x19, build_add16_instruction(DE));
        instruction_map.insert(0x29, build_add16_instruction(HL));
        instruction_map.insert(0x39, build_add16_instruction(SP));

        instruction_map.insert(0xE8, build_add_sp_n_instruction());
        instruction_map.insert(0xF8, build_add_hl_sp_n_instruction());

        instruction_map.insert(0x08, build_load_addr16_sp_instruction());

        instruction_map.insert(0x02, build_load_reg16addr_reg_instruction(AddressSource::Reg(BC)));
        instruction_map.insert(0x12, build_load_reg16addr_reg_instruction(AddressSource::Reg(DE)));

        instruction_map.insert(0x0A, build_load_reg_reg16addr_instruction(AddressSource::Reg(BC)));
        instruction_map.insert(0x1A, build_load_reg_reg16addr_instruction(AddressSource::Reg(DE)));

        instruction_map.insert(0x06, build_load_r_n_instruction(B));
        instruction_map.insert(0x0E, build_load_r_n_instruction(C));
        instruction_map.insert(0x16, build_load_r_n_instruction(D));
        instruction_map.insert(0x1E, build_load_r_n_instruction(E));
        instruction_map.insert(0x26, build_load_r_n_instruction(H));
        instruction_map.insert(0x2E, build_load_r_n_instruction(L));
        instruction_map.insert(0x36, build_load_rhl_n_instruction());
        instruction_map.insert(0x3E, build_load_r_n_instruction(A));

        instruction_map.insert(0xE0, build_load_high_addr_a_instruction(A));
        instruction_map.insert(0xF0, build_load_a_high_addr_instruction(A));

        instruction_map.insert(0xE2, build_load_high_addr_c_a_instruction(A));
        instruction_map.insert(0xF2, build_load_a_high_addr_c_instruction(A));

        instruction_map.insert(0xEA, build_load_addr16_a_instruction(A));
        instruction_map.insert(0xFA, build_load_a_addr16_instruction(A));

        instruction_map.insert(0xF9, build_load_rr_rr_instruction(SP, HL));

        instruction_map.insert(0x22, build_load_rhl_inc_r_instruction(A));
        instruction_map.insert(0x2A, build_load_r_rhl_inc_instruction(A));
        instruction_map.insert(0x32, build_load_rhl_dec_r_instruction(A));
        instruction_map.insert(0x3A, build_load_r_rhl_dec_instruction(A));

        instruction_map.insert(0xC1, build_pop_instruction(BC));
        instruction_map.insert(0xD1, build_pop_instruction(DE));
        instruction_map.insert(0xE1, build_pop_instruction(HL));
        instruction_map.insert(0xF1, build_pop_instruction(AF));

        instruction_map.insert(0xC5, build_push_instruction(BC));
        instruction_map.insert(0xD5, build_push_instruction(DE));
        instruction_map.insert(0xE5, build_push_instruction(HL));
        instruction_map.insert(0xF5, build_push_instruction(AF));

        instruction_map.insert(0x18, build_jump_relative_instruction());
        instruction_map.insert(0x20, build_conditional_jump_relative_instruction(Condition::NZ));
        instruction_map.insert(0x28, build_conditional_jump_relative_instruction(Condition::Z));
        instruction_map.insert(0x30, build_conditional_jump_relative_instruction(Condition::NC));
        instruction_map.insert(0x38, build_conditional_jump_relative_instruction(Condition::C));

        instruction_map.insert(0xC3, build_jump_instruction());
        instruction_map.insert(0xC2, build_conditional_jump_instruction(Condition::NZ));
        instruction_map.insert(0xCA, build_conditional_jump_instruction(Condition::Z));
        instruction_map.insert(0xD2, build_conditional_jump_instruction(Condition::NC));
        instruction_map.insert(0xDA, build_conditional_jump_instruction(Condition::C));

        instruction_map.insert(0xE9, NewInstruction::new().empty().and_move_reg16(PC, HL));

        instruction_map.insert(0xC9, build_return_instruction());
        instruction_map.insert(0xC0, build_conditional_return_instruction(Condition::NZ));
        instruction_map.insert(0xC8, build_conditional_return_instruction(Condition::Z));
        instruction_map.insert(0xD0, build_conditional_return_instruction(Condition::NC));
        instruction_map.insert(0xD8, build_conditional_return_instruction(Condition::C));
        instruction_map.insert(0xD9, build_return_instruction().and_enable_interrupts());

        instruction_map.insert(0xCD, build_call_instruction());
        instruction_map.insert(0xC4, build_conditional_call_instruction(Condition::NZ));
        instruction_map.insert(0xCC, build_conditional_call_instruction(Condition::Z));
        instruction_map.insert(0xD4, build_conditional_call_instruction(Condition::NC));
        instruction_map.insert(0xDC, build_conditional_call_instruction(Condition::C));

        instruction_map.insert(0xF3, NewInstruction::new().empty().and_disable_interrupts());
        instruction_map.insert(0xFB, NewInstruction::new().empty().and_enable_interrupts());

        for opcode in [0xC7, 0xD7, 0xE7, 0xF7, 0xCF, 0xDF, 0xEF, 0xFF].iter() {
            let addr = (opcode - 0xC7) as u16;
            instruction_map.insert(*opcode, build_reset_instruction(addr));
        }

        let order = [
            Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
        ];

        for opcode in 0x40..=0x7F {
            let opcode_index = opcode - 0x40;
            let dst_index = order[(opcode_index / 0x08) as usize];
            let src_index = order[(opcode_index % 0x08) as usize];

            match (dst_index, src_index) {
                (Some(reg), Some(reg1)) => instruction_map.insert(opcode, build_load_r_r_instruction(reg, reg1)),
                (None, Some(reg)) => instruction_map.insert(opcode, build_load_rhl_r_instruction(reg)),
                (Some(reg), None) => instruction_map.insert(opcode, build_load_r_rhl_instruction(reg)),
                (None, None) => None,
            };
        }

        instruction_map = build_alu_instructions(instruction_map, (0x04..=0x3C).step_by(8).collect(), AluOperation::Inc);
        instruction_map = build_alu_instructions(instruction_map, (0x05..=0x3D).step_by(8).collect(), AluOperation::Dec);

        instruction_map = build_alu_instructions(instruction_map, (0x80..=0x87).collect(), AluOperation::Add);
        instruction_map.insert(0xC6, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Add, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0x88..=0x8F).collect(), AluOperation::Adc);
        instruction_map.insert(0xCE, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Adc, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0x90..=0x97).collect(), AluOperation::Sub);
        instruction_map.insert(0xD6, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Sub, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0x98..=0x9F).collect(), AluOperation::Sbc);
        instruction_map.insert(0xDE, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Sbc, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0xA0..=0xA7).collect(), AluOperation::And);
        instruction_map.insert(0xE6, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::And, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0xA8..=0xAF).collect(), AluOperation::Xor);
        instruction_map.insert(0xEE, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Xor, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0xB0..=0xB7).collect(), AluOperation::Or);
        instruction_map.insert(0xF6, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Or, TempLow));

        instruction_map = build_alu_instructions(instruction_map, (0xB8..=0xBF).collect(), AluOperation::Cp);
        instruction_map.insert(0xFE, NewInstruction::new().load_imm(TempLow).empty().and_alu(AluOperation::Cp, TempLow));

        instruction_map.insert(0x07, NewInstruction::new().empty().and_alu(AluOperation::Rlca, A));
        instruction_map.insert(0x0F, NewInstruction::new().empty().and_alu(AluOperation::Rrca, A));
        instruction_map.insert(0x17, NewInstruction::new().empty().and_alu(AluOperation::Rla, A));
        instruction_map.insert(0x1F, NewInstruction::new().empty().and_alu(AluOperation::Rra, A));

        instruction_map.insert(0x27, NewInstruction::new().empty().and_alu(AluOperation::Daa, A));
        instruction_map.insert(0x2F, NewInstruction::new().empty().and_alu(AluOperation::Cpl, A));
        instruction_map.insert(0x37, NewInstruction::new().empty().and_alu(AluOperation::Scf, A));
        instruction_map.insert(0x3F, NewInstruction::new().empty().and_alu(AluOperation::Ccf, A));

        instruction_map.insert(0x76, NewInstruction::new().empty().and_halt());

        let mut prefixed_instruction_map = std::collections::HashMap::new();

        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x00..=0x07).collect(), AluOperation::Rlc);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x08..=0x0F).collect(), AluOperation::Rrc);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x10..=0x17).collect(), AluOperation::Rl);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x18..=0x1F).collect(), AluOperation::Rr);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x20..=0x27).collect(), AluOperation::Sla);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x28..=0x2F).collect(), AluOperation::Sra);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x30..=0x37).collect(), AluOperation::Swap);
        prefixed_instruction_map = build_alu_instructions(prefixed_instruction_map, (0x38..=0x3F).collect(), AluOperation::Srl);

        for start_opcode in (0x40..=0x7F).step_by(8) {
            let bit = (start_opcode - 0x40) / 0x08;

            prefixed_instruction_map = build_alu_instructions(
                prefixed_instruction_map,
                (start_opcode..start_opcode+8).collect(),
                AluOperation::Tst(bit),
            );
        }

        for start_opcode in (0x80..=0xBF).step_by(8) {
            let bit = (start_opcode - 0x80) / 0x08;

            prefixed_instruction_map = build_alu_instructions(
                prefixed_instruction_map,
                (start_opcode..start_opcode+8).collect(),
                AluOperation::Res(bit),
            );
        }

        for start_opcode in (0xC0..=0xFF).step_by(8) {
            let bit = (start_opcode - 0xC0) / 0x08;

            prefixed_instruction_map = build_alu_instructions(
                prefixed_instruction_map,
                // Use an inclusive range to prevent overflow
                (start_opcode..=start_opcode+7).collect(),
                AluOperation::Set(bit as u8),
            );
        }

        InstructionRegistry { instruction_map, prefixed_instruction_map }
    }

    pub fn fetch(&self, opcode: u8, prefixed_mode: bool) -> Option<NewInstruction> {
        if prefixed_mode {
            self.prefixed_instruction_map.get(&opcode).map(|instr| instr.to_owned())
        } else {
            self.instruction_map.get(&opcode).map(|instr| instr.to_owned())
        }
    }
}

fn build_load_r_n_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new().load_imm(reg)
}

fn build_load_rhl_n_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .store(AddressSource::Reg(HL), TempLow)
}

fn build_load_reg16addr_reg_instruction(reg: AddressSource) -> NewInstruction {
    NewInstruction::new().store(reg, A)
}

fn build_load_reg_reg16addr_instruction(reg: AddressSource) -> NewInstruction {
    NewInstruction::new().load(reg, A)
}

fn build_load_r_r_instruction(dst_reg: RegisterIndex, src_reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new().empty().and_move_reg(dst_reg, src_reg)
}

fn build_load_rr_rr_instruction(dst_reg: TwoRegisterIndex, src_reg: TwoRegisterIndex) -> NewInstruction {
    NewInstruction::new().move_reg16(dst_reg, src_reg)
}

fn build_load_r_rhl_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new().load(AddressSource::Reg(HL), reg)
}

fn build_load_rhl_r_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new().store(AddressSource::Reg(HL), reg)
}

fn build_load_rhl_inc_r_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .store(AddressSource::Reg(HL), reg)
        .empty().and_inc16(HL)
}

fn build_load_r_rhl_inc_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .load(AddressSource::Reg(HL), reg)
        .empty().and_inc16(HL)
}

fn build_load_rhl_dec_r_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .store(AddressSource::Reg(HL), reg)
        .empty().and_dec16(HL)
}

fn build_load_r_rhl_dec_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .load(AddressSource::Reg(HL), reg)
        .empty().and_dec16(HL)
}

fn build_load_high_addr_a_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .store(AddressSource::HighPage(TempLow), reg)
}

fn build_load_a_high_addr_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load(AddressSource::HighPage(TempLow), reg)
}

fn build_load_high_addr_c_a_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new().store(AddressSource::HighPage(C), reg)
}

fn build_load_a_high_addr_c_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new().load(AddressSource::HighPage(C), reg)
}

fn build_load_addr16_a_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .store(AddressSource::Reg(Temp16), reg)
}

fn build_load_a_addr16_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .load(AddressSource::Reg(Temp16), reg)
}

fn build_load_addr16_sp_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .store(AddressSource::Reg(Temp16), SPLow)
        .store(AddressSource::RegWithOffset(Temp16, 1), SPHigh)
}

fn build_pop_instruction(reg16: TwoRegisterIndex) -> NewInstruction {
    let (high_reg, low_reg) = reg16.split_index();

    NewInstruction::new()
        .load(AddressSource::Reg(SP), low_reg)
        .and_inc16(SP)
        .load(AddressSource::Reg(SP), high_reg)
        .and_inc16(SP)
}

fn build_push_instruction(reg16: TwoRegisterIndex) -> NewInstruction {
    let (high_reg, low_reg) = reg16.split_index();

    NewInstruction::new()
        .noop()
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), high_reg)
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), low_reg)
}

fn build_jump_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .move_reg16(PC, Temp16)
}

fn build_conditional_jump_instruction(condition: Condition) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .and_proceed_if(condition)
        .move_reg16(PC, Temp16)
}

fn build_jump_relative_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .noop().and_signed_add_reg16(PC, TempLow)
}

fn build_conditional_jump_relative_instruction(condition: Condition) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .and_proceed_if(condition)
        .noop().and_signed_add_reg16(PC, TempLow)
}

fn build_return_instruction() -> NewInstruction {
    NewInstruction::new()
        .load(AddressSource::Reg(SP), PCLow)
        .and_inc16(SP)
        .load(AddressSource::Reg(SP), PCHigh)
        .and_inc16(SP)
        .noop()
}

fn build_conditional_return_instruction(condition: Condition) -> NewInstruction {
    NewInstruction::new()
        .noop()
        .and_proceed_if(condition)
        .load(AddressSource::Reg(SP), PCLow)
        .and_inc16(SP)
        .load(AddressSource::Reg(SP), PCHigh)
        .and_inc16(SP)
        .noop()
}

fn build_call_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .noop()
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), PCHigh)
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), PCLow)
        .and_move_reg16(PC, Temp16)
}

fn build_conditional_call_instruction(condition: Condition) -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .load_imm(TempHigh)
        .and_proceed_if(condition)
        .noop()
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), PCHigh)
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), PCLow)
        .and_move_reg16(PC, Temp16)
}

fn build_reset_instruction(addr: u16) -> NewInstruction {
    NewInstruction::new()
        .noop()
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), PCHigh)
        .and_dec16(SP)
        .store(AddressSource::Reg(SP), PCLow)
        .and_move_rst_addr(addr)
}

fn build_inc16_instruction(reg16: TwoRegisterIndex) -> NewInstruction {
    NewInstruction::new().noop().and_inc16(reg16)
}

fn build_dec16_instruction(reg16: TwoRegisterIndex) -> NewInstruction {
    NewInstruction::new().noop().and_dec16(reg16)
}

fn build_add16_instruction(reg16: TwoRegisterIndex) -> NewInstruction {
    // TODO: Actually split alu instruction instead of using a noop
    NewInstruction::new().noop().empty().and_add16(reg16)
}

fn build_add_sp_n_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .noop().and_add_sp(SP, TempLow)
        .noop()
}

fn build_add_hl_sp_n_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
        .noop().and_add_sp(HL, TempLow)
}

fn build_alu_instructions(
    mut instruction_map: std::collections::HashMap<u8, NewInstruction>,
    opcode_range: Vec<u8>, alu_operation: AluOperation) -> std::collections::HashMap<u8, NewInstruction> {
    let order = [
        Some(B), Some(C), Some(D), Some(E), Some(H), Some(L), None, Some(A),
    ];

    for (reg_index, opcode) in order.iter().zip(opcode_range.iter()) {
        let mut instruction = NewInstruction::new();

        instruction = match reg_index {
            Some(reg) => instruction.empty().and_alu(alu_operation, *reg),
            None => {
                match alu_operation {
                    AluOperation::Inc | AluOperation::Dec => {
                        instruction
                            .load(AddressSource::Reg(HL), TempLow)
                            .and_alu(alu_operation, TempLow)
                            .store(AddressSource::Reg(HL), TempLow)
                    },
                    AluOperation::Rlc | AluOperation::Rrc | AluOperation::Rl
                        | AluOperation::Rr | AluOperation::Sla | AluOperation::Sra
                        | AluOperation::Swap | AluOperation::Srl | AluOperation::Res(_)
                        | AluOperation::Set(_) => {
                            instruction
                                .load(AddressSource::Reg(HL), TempLow)
                                .and_alu(alu_operation, TempLow)
                                .store(AddressSource::Reg(HL), TempLow)
                        },
                    _ => {
                        instruction
                            .load(AddressSource::Reg(HL), TempLow)
                            .and_alu(alu_operation, TempLow)
                    },
                }
            }
        };

        instruction_map.insert(*opcode, instruction);
    }

    instruction_map
}
