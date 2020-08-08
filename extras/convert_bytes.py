import sys

if len(sys.argv) < 2:
    exit("Error: please provide filename to convert")

filename = sys.argv[1]
f = open(filename, "rb")
contents = f.read()

for i in range(0, len(contents), 16):
    row = contents[i:i+16]
    row = ", ".join(f"0x{b:02X}" for b in row)
    print(row)
