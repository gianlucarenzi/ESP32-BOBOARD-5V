"""Run banked_ram_test.xex on py65 against a model of the banked RAM device.

Host-side check of the test program itself: the model follows the firmware
(two windows, latch bit 7, bank register $xxFD, RAM $xx00-$xx7F, nothing
driven or stored while deselected); each fault breaks one rule and the test
must report it.  CIO output (CIOV) is trapped and printed.

Requires py65 (pip install py65).

Usage: sim_banked_ram.py XEX [fault]
faults: none, bankreg_ignored, latch_ignored, stuck_d3, alias_a6,
        shared_windows, bank_alias_7bit, writes_when_deselected
"""
import sys
from py65.devices.mpu6502 import MPU
from py65.memory import ObservableMemory

xex, fault = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "none")

class Device:
    def __init__(self, shared=False):
        self.sel = [0, 0]
        self.bank = [0, 0]
        self.ram = [[bytearray(128) for _ in range(256)] for _ in range(2)]
        if shared:
            self.ram[1] = self.ram[0]

    def win(self, addr):
        return {0xD1: 0, 0xD5: 1}.get(addr >> 8)

    def read(self, addr):
        w, off = self.win(addr), addr & 0xFF
        if fault == "alias_a6":
            off &= ~0x40
        if not self.sel[w] and fault != "latch_ignored":
            return 0xFF                      # floating bus
        if off < 0x80:
            b = self.bank[w]
            if fault == "bank_alias_7bit":
                b &= 0x7F
            v = self.ram[w][b][off]
            return v | 0x08 if fault == "stuck_d3" else v
        if off == 0xFD:
            return self.bank[w]
        return 0xFF

    def write(self, addr, v):
        w, off = self.win(addr), addr & 0xFF
        if fault == "alias_a6" and off < 0x80:
            off &= ~0x40
        if off == 0xFF:
            if fault != "latch_ignored":
                self.sel[w] = 1 if v & 0x80 else 0
            return
        if not self.sel[w] and fault not in ("latch_ignored", "writes_when_deselected"):
            return
        if off < 0x80:
            b = self.bank[w]
            if fault == "bank_alias_7bit":
                b &= 0x7F
            self.ram[w][b][off] = v
        elif off == 0xFD and fault != "bankreg_ignored":
            self.bank[w] = v

dev = Device(shared=(fault == "shared_windows"))
mem = ObservableMemory()
mem.subscribe_to_read(list(range(0xD100, 0xD200)) + list(range(0xD500, 0xD600)), lambda a: dev.read(a))
mem.subscribe_to_write(list(range(0xD100, 0xD200)) + list(range(0xD500, 0xD600)), lambda a, v: dev.write(a, v))

data = open(xex, "rb").read()
i, run = 0, None
while i < len(data):
    if data[i:i + 2] == b"\xff\xff":
        i += 2
    start = data[i] | data[i + 1] << 8
    end = data[i + 2] | data[i + 3] << 8
    i += 4
    seg = data[i:i + end - start + 1]
    i += len(seg)
    for k, b in enumerate(seg):
        mem[start + k] = b
    if start <= 0x2E0 <= end:
        run = mem[0x2E0] | mem[0x2E1] << 8

CIOV = 0xE456
mem[CIOV] = 0x60                             # RTS, output trapped below
mpu = MPU(memory=mem, pc=run)
mpu.sp = 0xFF
out = []
for steps in range(30_000_000):
    if mpu.pc == CIOV:
        ch = mpu.a
        out.append("\n" if ch == 0x9B else ("" if ch == 0x7D else chr(ch)))
    pc_before = mpu.pc
    mpu.step()
    if mpu.pc == pc_before:                  # JMP halt
        break
text = "".join(out)
print("--- fault: %s, %d steps" % (fault, steps))
print(text.rstrip())
