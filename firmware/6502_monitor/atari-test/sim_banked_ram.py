"""Run banked_ram_test.xex on py65 against a model of the banked RAM device.

Host-side check of the test program itself: the model follows the firmware
(two windows, latch bit 7, 16-bit bank register $xxFD-$xxFE, bank count
$xxFB-$xxFC read-only, RAM $xx00-$xx7F, banks beyond the count not driven,
nothing driven or stored while deselected); each fault breaks one rule and
the test must report it.  CIO output (CIOV) is trapped and printed.

Requires py65 (pip install py65).

Usage: sim_banked_ram.py XEX [fault] [banks per window]
faults: none, bankreg_ignored, bank_hi_ignored, latch_ignored, stuck_d3,
        alias_a6, shared_windows, writes_when_deselected, limit_wraps,
        count_writable
"""
import sys
from py65.devices.mpu6502 import MPU
from py65.memory import ObservableMemory

xex = sys.argv[1]
fault = sys.argv[2] if len(sys.argv) > 2 else "none"
banks = int(sys.argv[3]) if len(sys.argv) > 3 else 300
FLOAT = 0xFF                                  # undriven bus


class Device:
    def __init__(self):
        self.sel = [0, 0]
        self.bank = [0, 0]
        self.count = [banks, banks]
        self.ram = [[bytearray(128) for _ in range(banks)] for _ in range(2)]
        if fault == "shared_windows":
            self.ram[1] = self.ram[0]

    @staticmethod
    def win(addr):
        return {0xD1: 0, 0xD5: 1}[addr >> 8]

    def bank_ram(self, w):
        b = self.bank[w]
        if fault == "bank_hi_ignored":
            b &= 0xFF
        if b >= min(self.count[w], banks):     # count may be corrupted by a fault
            if fault == "limit_wraps":
                return self.ram[w][b % banks]
            return None
        return self.ram[w][b]

    def read(self, addr):
        w, off = self.win(addr), addr & 0xFF
        if fault == "alias_a6" and off < 0x80:
            off &= ~0x40
        if not self.sel[w] and fault != "latch_ignored":
            return FLOAT
        if off < 0x80:
            r = self.bank_ram(w)
            if r is None:
                return FLOAT
            return r[off] | 0x08 if fault == "stuck_d3" else r[off]
        regs = {0xFB: self.count[w] & 0xFF, 0xFC: self.count[w] >> 8,
                0xFD: self.bank[w] & 0xFF, 0xFE: self.bank[w] >> 8}
        return regs.get(off, FLOAT)

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
            r = self.bank_ram(w)
            if r is not None:
                r[off] = v
        elif off == 0xFD and fault != "bankreg_ignored":
            self.bank[w] = (self.bank[w] & 0xFF00) | v
        elif off == 0xFE and fault != "bankreg_ignored":
            self.bank[w] = (self.bank[w] & 0x00FF) | v << 8
        elif off == 0xFB and fault == "count_writable":
            self.count[w] = (self.count[w] & 0xFF00) | v
        elif off == 0xFC and fault == "count_writable":
            self.count[w] = (self.count[w] & 0x00FF) | v << 8


dev = Device()
mem = ObservableMemory()
win_addrs = list(range(0xD100, 0xD200)) + list(range(0xD500, 0xD600))
mem.subscribe_to_read(win_addrs, lambda a: dev.read(a))
mem.subscribe_to_write(win_addrs, lambda a, v: dev.write(a, v))

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
mem[CIOV] = 0x60                              # RTS, output trapped below
mpu = MPU(memory=mem, pc=run)
mpu.sp = 0xFF
out = []
for steps in range(100_000_000):
    if mpu.pc == CIOV:
        ch = mpu.a
        out.append("\n" if ch == 0x9B else ("" if ch == 0x7D else chr(ch)))
    pc_before = mpu.pc
    mpu.step()
    if mpu.pc == pc_before:                   # JMP halt
        break
print("--- fault: %s, %d banks per window, %d steps" % (fault, banks, steps))
print("".join(out).rstrip())
