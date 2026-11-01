"""Run banked_ram_test.xex on py65 against a model of the banked RAM device.

Host-side check of the test program itself: the model follows the firmware
(GeoRAM style: one latch at a time, registers in the selected page, RAM in
the other one, $D1FF protected, one RAM and one 16-bit bank register, count
read-only, banks beyond the count not driven, nothing driven or stored while
deselected); each fault breaks one rule and the test must report it.  CIO
output (CIOV) is trapped and printed.

Requires py65 (pip install py65).

Usage: sim_banked_ram.py XEX [fault] [banks]
faults: none, bankreg_ignored, bank_hi_ignored, both_latches, stuck_d3,
        alias_a6, separate_ram, writes_when_deselected, limit_wraps,
        count_writable, d1ff_unprotected
"""
import sys
from py65.devices.mpu6502 import MPU
from py65.memory import ObservableMemory

xex = sys.argv[1]
fault = sys.argv[2] if len(sys.argv) > 2 else "none"
banks = int(sys.argv[3]) if len(sys.argv) > 3 else 300
FLOAT = 0xFF                                  # undriven bus
D1, D5 = 0, 1


class Device:
    def __init__(self):
        self.ctl = None                       # page holding the registers
        self.last = D1                        # writes_when_deselected
        self.bank = 0
        self.count = banks
        ram = [bytearray(256) for _ in range(banks)]
        self.ram = [ram, [bytearray(256) for _ in range(banks)]
                    if fault == "separate_ram" else ram]

    @staticmethod
    def page(addr):
        return {0xD1: D1, 0xD5: D5}[addr >> 8]

    def size(self, w):
        return 255 if w == D1 and fault != "d1ff_unprotected" else 256

    def bank_ram(self, w):
        b = self.bank & 0xFF if fault == "bank_hi_ignored" else self.bank
        if b >= min(self.count, banks):       # count may be corrupted by a fault
            if fault == "limit_wraps":
                return self.ram[w][b % banks]
            return None
        return self.ram[w][b]

    def read(self, addr):
        w, off = self.page(addr), addr & 0xFF
        if self.ctl is None:
            return FLOAT
        if w == self.ctl:
            regs = {0xFB: self.count & 0xFF, 0xFC: self.count >> 8,
                    0xFD: self.bank & 0xFF, 0xFE: self.bank >> 8}
            return regs.get(off, FLOAT)
        if fault == "alias_a6":
            off &= ~0x40
        r = self.bank_ram(w)
        if r is None or off >= self.size(w):
            return FLOAT
        return r[off] | 0x08 if fault == "stuck_d3" else r[off]

    def write(self, addr, v):
        w, off = self.page(addr), addr & 0xFF
        ctl = self.ctl
        if off == 0xFF and (ctl is None or ctl == w or fault == "both_latches"):
            self.ctl = w if v & 0x80 else None
            if self.ctl is not None:
                self.last = self.ctl
            return
        if ctl is None:
            if fault != "writes_when_deselected":
                return
            ctl = self.last
        if w == ctl:
            if off == 0xFD and fault != "bankreg_ignored":
                self.bank = (self.bank & 0xFF00) | v
            elif off == 0xFE and fault != "bankreg_ignored":
                self.bank = (self.bank & 0x00FF) | v << 8
            elif off == 0xFB and fault == "count_writable":
                self.count = (self.count & 0xFF00) | v
            elif off == 0xFC and fault == "count_writable":
                self.count = (self.count & 0x00FF) | v << 8
            return
        if fault == "alias_a6":
            off &= ~0x40
        r = self.bank_ram(w)
        if r is not None and off < self.size(w):
            r[off] = v


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
print("--- fault: %s, %d banks, %d steps" % (fault, banks, steps))
print("".join(out).rstrip())
