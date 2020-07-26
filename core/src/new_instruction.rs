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
    Read(AddressSource, RegisterIndex),
    Write(AddressSource, RegisterIndex),
}

#[derive(Debug, Copy, Clone)]
pub enum RegisterOperation {
    Load(RegisterIndex, RegisterIndex),
}

#[derive(Debug, Copy, Clone)]
pub enum AddressSource {
    Immediate,
    HighPage(RegisterIndex),
    Reg(TwoRegisterIndex),
}

pub struct InstructionRegistry {
    instruction_map: std::collections::HashMap<u8, NewInstruction>,
}

impl InstructionRegistry {
    pub fn new() -> Self {
        let mut instruction_map = std::collections::HashMap::new();

        instruction_map.insert(0x01, build_load_rr_addr16_instruction(BC));
        instruction_map.insert(0x11, build_load_rr_addr16_instruction(DE));
        instruction_map.insert(0x21, build_load_rr_addr16_instruction(HL));
        instruction_map.insert(0x31, build_load_rr_addr16_instruction(SP));

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
    NewInstruction::new()
        .push(MicroInstruction {
            memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, reg)),
            register_operation: None,
        })
}

fn build_load_rhl_n_instruction() -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation:
                    Some(MemoryOperation::Read(AddressSource::Immediate, TempLow)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation:
                    Some(MemoryOperation::Write(AddressSource::Reg(HL), TempLow)),
                register_operation: None,
        })
}

fn build_load_reg16addr_reg_instruction(reg: AddressSource) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation:
                    Some(MemoryOperation::Write(reg, A)),
                register_operation: None,
        })
}

fn build_load_reg_reg16addr_instruction(reg: AddressSource) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation:
                    Some(MemoryOperation::Read(reg, A)),
                register_operation: None,
        })
}

fn build_load_r_r_instruction(dst_reg: RegisterIndex, src_reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: None,
                register_operation: Some(RegisterOperation::Load(dst_reg, src_reg)),
        })
}

fn build_load_r_rhl_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Reg(HL), reg)),
                register_operation: None,
        })
}

fn build_load_rhl_r_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Write(AddressSource::Reg(HL), reg)),
                register_operation: None,
        })
}

fn build_load_high_addr_a_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, TempLow)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Write(AddressSource::HighPage(TempLow), reg)),
                register_operation: None,
        })
}

fn build_load_a_high_addr_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, TempLow)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::HighPage(TempLow), reg)),
                register_operation: None,
        })
}

fn build_load_high_addr_c_a_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Write(AddressSource::HighPage(C), reg)),
                register_operation: None,
        })
}

fn build_load_a_high_addr_c_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::HighPage(C), reg)),
                register_operation: None,
        })
}

fn build_load_addr16_a_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, TempLow)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, TempHigh)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Write(AddressSource::Reg(Temp16), reg)),
                register_operation: None,
        })
}

fn build_load_a_addr16_instruction(reg: RegisterIndex) -> NewInstruction {
    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, TempLow)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, TempHigh)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Reg(Temp16), reg)),
                register_operation: None,
        })
}

fn build_load_rr_addr16_instruction(reg16: TwoRegisterIndex) -> NewInstruction {
    let (high_reg, low_reg) = reg16.split_index();

    NewInstruction::new()
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, low_reg)),
                register_operation: None,
        })
        .push(
            MicroInstruction {
                memory_operation: Some(MemoryOperation::Read(AddressSource::Immediate, high_reg)),
                register_operation: None,
        })
}
