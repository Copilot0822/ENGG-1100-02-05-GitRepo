import subprocess
import struct
import os
import sys

# ========== CONFIGURE FOR YOUR MACHINE ==========
AVRDUDE = "avrdude"   # or full path to avrdude.exe
MCU = "m328p"
PROGRAMMER = "arduino"
BAUD = "115200"
PORT = "COM3"         # change to your port
EEPROM_SIZE = 1024    # Uno EEPROM in bytes
# ===============================================

EEPROM_BIN = "eeprom.bin"

FLOAT_NAMES = [
    "kP",
    "kI",
    "kD",
    "kFf",
    "maxVel",
    "maxAccel",
    "maxJerk",
    "driveSetpoint",
    "returnSetpoint",
    "shootSetpoint",
    "shooterPower",
]
N_FLOATS = len(FLOAT_NAMES)
START_ADDR = 0   # must match Arduino EEPROM_ADDR


def run_avrdude(extra_args):
    cmd = [
        AVRDUDE,
        "-p", MCU,
        "-c", PROGRAMMER,
        "-P", PORT,
        "-b", BAUD,
        "-D",  # don't erase/write flash
    ] + extra_args

    print("Running:", " ".join(cmd))
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    print(result.stdout)
    if result.returncode != 0:
        raise RuntimeError("avrdude failed")


def read_eeprom_to_bin():
    # Read raw EEPROM into binary file
    run_avrdude(["-U", f"eeprom:r:{EEPROM_BIN}:r"])
    if not os.path.exists(EEPROM_BIN):
        raise FileNotFoundError(EEPROM_BIN)
    data = open(EEPROM_BIN, "rb").read()
    if len(data) < EEPROM_SIZE:
        data = data + b"\xFF" * (EEPROM_SIZE - len(data))
    return bytearray(data)


def write_bin_to_eeprom(data: bytes):
    if len(data) != EEPROM_SIZE:
        raise ValueError(f"Expected {EEPROM_SIZE} bytes, got {len(data)}")
    open(EEPROM_BIN, "wb").write(data)
    run_avrdude(["-U", f"eeprom:w:{EEPROM_BIN}:r"])


def read_floats():
    data = read_eeprom_to_bin()
    values = []
    for i in range(N_FLOATS):
        addr = START_ADDR + i * 4
        chunk = data[addr:addr+4]
        val = struct.unpack("<f", chunk)[0]  # little-endian float32
        values.append(val)
    return values


def write_floats(values):
    if len(values) != N_FLOATS:
        raise ValueError(f"Need {N_FLOATS} values")
    data = read_eeprom_to_bin()
    for i, val in enumerate(values):
        addr = START_ADDR + i * 4
        chunk = struct.pack("<f", val)
        data[addr:addr+4] = chunk
    write_bin_to_eeprom(bytes(data))


def print_current():
    vals = read_floats()
    print("Current EEPROM config:")
    for name, val in zip(FLOAT_NAMES, vals):
        print(f"  {name:14s} = {val}")


def main():
    if len(sys.argv) == 1:
        # Just dump current values
        print_current()
        return

    if sys.argv[1] == "set":
        # CLI usage:
        #   python eeprom_tool.py set kP 1.0 kI 0.1 maxVel 200 ...
        vals = read_floats()
        name_to_index = {name: i for i, name in enumerate(FLOAT_NAMES)}

        if (len(sys.argv) - 2) % 2 != 0:
            print("Usage: python eeprom_tool.py set name1 val1 [name2 val2 ...]")
            sys.exit(1)

        for i in range(2, len(sys.argv), 2):
            name = sys.argv[i]
            val_str = sys.argv[i+1]
            if name not in name_to_index:
                print(f"Unknown name: {name}")
                print("Valid names:", ", ".join(FLOAT_NAMES))
                sys.exit(1)
            val = float(val_str)
            idx = name_to_index[name]
            vals[idx] = val

        write_floats(vals)
        print("Written new values.")
        print_current()
    else:
        print("Usage:")
        print("  python eeprom_tool.py          # show all values")
        print("  python eeprom_tool.py set kP 1.0 maxVel 150 shooterPower 0.6")
        sys.exit(1)


if __name__ == "__main__":
    main()
