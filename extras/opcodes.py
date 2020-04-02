from bs4 import BeautifulSoup
import re
import json

def get_table_content(table):
    rows = [row.find_all("td")[1:] for row in table.find_all("tr")[1:]]

    opcodes_content = []
    for row in rows:
        for cell in row:
            opcodes_content.append(cell.contents)
    return opcodes_content

def generate_opcode_json(opcodes_content):
    opcodes = []
    for i, opcode in enumerate(opcodes_content):
        hex_i = f"{i:02X}"
        if len(opcode) == 1:
            opcodes.append({
                "opcode": hex_i,
                "repr": "UNIMPLEMENTED",
                "bytes": None,
                "cycles": None,
                "addressing_mode": addressing_mode,
                "z": None, "n": None, "h": None, "c": None,
            })
        else:
            instruction, _, bytes_cycles, _, flags = opcode
            num_bytes, cycles = filter(lambda x: bool(x), bytes_cycles.split())
            z, n, h, c = flags.split()
            if instruction == "STOP 0":
                addressing_mode = "Implied"
            elif num_bytes == "1":
                addressing_mode = "Implied"
            elif instruction.endswith("d8"):
                addressing_mode = "Imm8"
            elif hex_i in ["E0", "E2", "F0", "F2"]:
                addressing_mode = "ZeroPageOffset"
            elif instruction.endswith("d16"):
                addressing_mode = "Imm16"
            elif "a16" in instruction:
                addressing_mode = "Addr16"
            elif instruction.endswith("r8"):
                addressing_mode = "SignedAddrOffset"
            else:
                print(instruction)
                raise RuntimeError(f"Unknown addressing mode for {hex_i}")
            opcodes.append({
                "opcode": hex_i,
                "repr": instruction,
                "bytes": num_bytes,
                "cycles": cycles,
                "addressing_mode": addressing_mode,
                "z": z, "n": n, "h": h, "c": c,
            })
    return opcodes

def generate_prefixed_opcode_json(opcodes_content):
    opcodes = []
    for i, opcode in enumerate(opcodes_content):
        hex_i = f"{i:02X}"
        instruction, _, bytes_cycles, _, flags = opcode
        num_bytes, cycles = filter(lambda x: bool(x), bytes_cycles.split())
        z, n, h, c = flags.split()
        addressing_mode = "Implied"
        opcodes.append({
            "opcode": hex_i,
            "repr": instruction,
            "bytes": num_bytes,
            "cycles": cycles,
            "addressing_mode": addressing_mode,
            "z": z, "n": n, "h": h, "c": c,
        })
    return opcodes

html_doc = open("opcodes.html")
soup = BeautifulSoup(html_doc, 'html.parser')

tables = soup.body.find_all("table")

opcode_table = tables[0]
opcode_content = get_table_content(opcode_table)
opcodes = generate_opcode_json(opcode_content)

with open("opcodes.json", "w") as f:
    f.write(json.dumps(opcodes, indent=2))

prefixed_opcode_table = tables[1]
prefixed_opcode_content = get_table_content(prefixed_opcode_table)
prefixed_opcodes = generate_prefixed_opcode_json(prefixed_opcode_content)

with open("prefixed_opcodes.json", "w") as f:
    f.write(json.dumps(prefixed_opcodes, indent=2))
