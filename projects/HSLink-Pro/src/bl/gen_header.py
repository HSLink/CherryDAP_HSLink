import sys

if len(sys.argv) < 3:
    print("Usage: python bin2h.py input.bin output.h")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, "rb") as f:
    binary_data = f.read()

with open(output_file, "w") as f:
    f.write("#ifndef BINFILE_H\n#define BINFILE_H\n\n")
    f.write("const unsigned char binfile[] = {\n")
    for i, byte in enumerate(binary_data):
        if i % 12 == 0:
            f.write("    ")
        f.write(f"0x{byte:02x}, ")
        if i % 12 == 11 or i == len(binary_data) - 1:
            f.write("\n")
    f.write("};\n\n")
    f.write(f"const unsigned int binfile_size = {len(binary_data)};\n\n")
    f.write("#endif // BINFILE_H\n")
