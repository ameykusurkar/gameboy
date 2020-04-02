import json

with open("opcodes.json") as f:
    opcodes = json.loads(f.read())

def write_constructed_opcodes(filename, opcodes):
    with open("construct.txt", "w") as f:
        for opcode in opcodes:
            cycles = opcode["cycles"]
            opcode["bytes"] = opcode["bytes"] or 0
            if opcode["repr"] != "UNIMPLEMENTED":
                cycles = [int(c) for c in cycles.split("/")]
                for c in cycles:
                    if c % 4 != 0:
                        raise RuntimeError(f"Cycles {cycles} has to be a multiple for 4")
                cycles = [x // 4 for x in cycles]
                cycles_str = f"Fixed({cycles[0]})" if len(cycles) == 1 else f"Jump({', '.join(map(str, cycles))})"
            else:
                cycles_str = "Fixed(0)"
            instr = f"opcode: 0x{opcode['opcode']}, repr: \"{opcode['repr']}\", num_bytes: {opcode['bytes']}, cycles: {cycles_str}, addressing_mode: {opcode['addressing_mode']}"
            f.write(f"Instruction {{ {instr} }}")
            f.write(",\n")

with open("prefixed_opcodes.json") as f:
    prefixed_opcodes = json.loads(f.read())

with open("prefixed_construct.txt", "w") as f:
    for opcode in prefixed_opcodes:
        cycles = opcode["cycles"]
        opcode["bytes"] = opcode["bytes"] or 0
        if opcode["repr"] != "UNIMPLEMENTED":
            cycles = [int(c) for c in cycles.split("/")]
            for c in cycles:
                if c % 4 != 0:
                    raise RuntimeError(f"Cycles {cycles} has to be a multiple for 4")
            cycles = [x // 4 for x in cycles]
            cycles_str = f"Fixed({cycles[0]})" if len(cycles) == 1 else f"Jump({', '.join(map(str, cycles))})"
        else:
            cycles_str = "Fixed(0)"
        instr = f"opcode: 0x{opcode['opcode']}, repr: \"{opcode['repr']}\", num_bytes: {opcode['bytes']}, cycles: {cycles_str}, addressing_mode: {opcode['addressing_mode']}"
        f.write(f"Instruction {{ {instr} }}")
        f.write(",\n")
