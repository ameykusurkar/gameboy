use crate::registers::RegisterIndex;
use crate::registers::RegisterIndex::*;
use crate::registers::TwoRegisterIndex;
use crate::registers::TwoRegisterIndex::*;

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

    pub fn num_cycles(&self) -> usize {
        self.micro_instructions.len()
    }

    pub fn next_micro(&mut self) -> Option<MicroInstruction> {
        self.micro_instructions.pop_front()
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

    fn noop(self) -> Self {
        self.push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Noop),
                register_operation: None,
        })
    }

    fn empty(self) -> Self {
        self.push(
            MicroInstruction { memory_operation: None, register_operation: None }
        )
    }

    fn load(self, addr_source: AddressSource, reg: RegisterIndex) -> Self {
        self.push(MicroInstruction {
            memory_operation: Some(MemoryOperation::Read(addr_source, reg)),
            register_operation: None,
        })
    }

    fn load_imm(self, reg: RegisterIndex) -> Self {
        self.load(AddressSource::Immediate, reg)
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
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Write(addr_source, reg)),
                register_operation: None,
        })
    }

    fn and_inc16(self, reg16: TwoRegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::IncReg16(reg16))
    }

    fn and_dec16(self, reg16: TwoRegisterIndex) -> Self {
        self.append_register_operation(RegisterOperation::DecReg16(reg16))
    }
}

#[derive(Debug, Clone)]
pub struct MicroInstruction {
    pub memory_operation: Option<MemoryOperation>,
    pub register_operation: Option<RegisterOperation>,
}

impl MicroInstruction {
    pub fn has_memory_access(&self) -> bool {
        self.memory_operation.is_some()
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
    SignedAddReg16(TwoRegisterIndex, RegisterIndex),
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

        // TODO: POP AF
        instruction_map.insert(0xC1, build_pop_instruction(BC));
        instruction_map.insert(0xD1, build_pop_instruction(DE));
        instruction_map.insert(0xE1, build_pop_instruction(HL));

        instruction_map.insert(0xC5, build_push_instruction(BC));
        instruction_map.insert(0xD5, build_push_instruction(DE));
        instruction_map.insert(0xE5, build_push_instruction(HL));
        instruction_map.insert(0xF5, build_push_instruction(AF));

        instruction_map.insert(0xC3, build_jump_instruction());

        instruction_map.insert(0x18, build_jump_relative_instruction());

        instruction_map.insert(0xC9, build_return_instruction());

        instruction_map.insert(0xCD, build_call_instruction());

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

        InstructionRegistry { instruction_map }
    }

    pub fn fetch(&self, opcode: u8) -> Option<NewInstruction> {
        self.instruction_map.get(&opcode).map(|instr| instr.to_owned())
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

fn build_jump_relative_instruction() -> NewInstruction {
    NewInstruction::new()
        .load_imm(TempLow)
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
