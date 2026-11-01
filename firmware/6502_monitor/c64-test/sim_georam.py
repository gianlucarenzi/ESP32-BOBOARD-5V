"""Run georam_test.prg on py65 against a model of the C64 GeoRAM.

Host-side check of the test program itself: the model follows the firmware
(env c64: $DE00-$DEFF window, $DFFE page bits 0-5, $DFFF block masked to
the size, write-only registers, open bus on IO2 reads); each fault breaks
one rule and the test must report it.  KERNAL CHROUT ($FFD2) is trapped.

Requires py65 (pip install py65).

Usage: sim_georam.py PRG [fault] [blocks]
faults: none, page_reg_ignored, block_reg_ignored, page_8bit, no_alias,
        stuck_d3, alias_a6, no_offset_ff, small, block_mask_low, nodev
        (nodev: no GeoRAM, the open bus reads $00)
"""
import sys
from py65.devices.mpu6502 import MPU
from py65.memory import ObservableMemory

prg = sys.argv[1]
fault = sys.argv[2] if len(sys.argv) > 2 else "none"
blocks = int(sys.argv[3]) if len(sys.argv) > 3 else 16
if fault == "small":
    blocks = 8                                 # a 128 KB unit: SIZE expects 16
FLOAT = 0xFF                                   # open bus


class GeoRAM:
    def __init__(self):
        self.page = 0
        self.block = 0
        pages = 256 if fault == "page_8bit" else 64
        self.ppb = pages
        self.ram = [bytearray(256) for _ in range(blocks * pages)]

    def index(self):
        p = self.page if fault == "page_8bit" else self.page & 0x3F
        b = self.block
        if fault == "no_alias":
            if b >= blocks:
                return None
        elif fault == "block_mask_low":
            b &= blocks // 2 - 1               # two blocks share the RAM
        else:
            b &= blocks - 1
        return b * self.ppb + p

    def read(self, addr):
        if fault == "nodev":
            return 0x00
        if addr >= 0xDF00:
            return FLOAT                       # registers are write-only
        off = addr & 0xFF
        if fault == "alias_a6":
            off &= ~0x40
        i = self.index()
        if i is None:
            return FLOAT
        v = self.ram[i][off]
        return v | 0x08 if fault == "stuck_d3" else v

    def write(self, addr, v):
        if addr >= 0xDF00:
            if addr == 0xDFFE and fault != "page_reg_ignored":
                self.page = v
            elif addr == 0xDFFF and fault != "block_reg_ignored":
                self.block = v
            return
        off = addr & 0xFF
        if fault == "alias_a6":
            off &= ~0x40
        if fault == "no_offset_ff" and off == 0xFF:
            return
        i = self.index()
        if i is not None:
            self.ram[i][off] = v


geo = GeoRAM()
mem = ObservableMemory()
io = list(range(0xDE00, 0xE000))
mem.subscribe_to_read(io, lambda a: geo.read(a))
mem.subscribe_to_write(io, lambda a, v: geo.write(a, v))

data = open(prg, "rb").read()
load = data[0] | data[1] << 8
for k, b in enumerate(data[2:]):
    mem[load + k] = b
# BASIC stub "10 SYS nnnn": the address follows the SYS token ($9E).
i = data.index(0x9E, 2) + 1
run = int(bytes(c for c in data[i:i + 6] if 0x30 <= c <= 0x39).decode())

CHROUT = 0xFFD2
mem[CHROUT] = 0x60                             # RTS, output trapped below
mpu = MPU(memory=mem, pc=run)
mpu.sp = 0xFF
out = []
for steps in range(100_000_000):
    if mpu.pc == CHROUT:
        c = mpu.a
        out.append("\n" if c == 13 else "" if c == 147 else chr(c) if 0x20 <= c < 0x60 else "?")
    pc_before = mpu.pc
    mpu.step()
    if mpu.pc == pc_before:                    # JMP halt
        break
print("--- fault: %s, %d blocks, %d steps" % (fault, blocks, steps))
print("".join(out).rstrip())
