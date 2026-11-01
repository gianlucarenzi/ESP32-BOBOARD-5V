"""Run banked_ram_test.xex on py65 against a model of the banked RAM device.

Host-side check of the test program itself: the model follows the firmware
(registers in $D1xx: count $D1FB-$D1FC read-only, 16-bit bank $D1FD-$D1FE,
latch $D1FF bit 7; RAM window $D500-$D5FF; banks beyond the count not
driven; nothing driven or stored while deselected); each fault breaks one
rule and the test must report it.  CIO output (CIOV) is trapped and printed.

Requires py65 (pip install py65).

Usage: sim_banked_ram.py XEX [fault] [banks]
faults: none, bankreg_ignored, bank_hi_ignored, latch_ignored, latch_any_bit,
        stuck_d3, alias_a6, writes_when_deselected, limit_wraps,
        count_writable, offset_ff_dead, ram_lost_on_deselect
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
        self.sel = 0
        self.bank = 0
        self.count = banks
        self.ram = [bytearray(256) for _ in range(banks)]

    def selected(self):
        return self.sel or fault == "latch_ignored"

    def bank_ram(self):
        b = self.bank & 0xFF if fault == "bank_hi_ignored" else self.bank
        if b >= min(self.count, banks):       # count may be corrupted by a fault
            if fault == "limit_wraps":
                return self.ram[b % banks]
            return None
        return self.ram[b]

    def read(self, addr):
        off = addr & 0xFF
        if not self.selected():
            return FLOAT
        if addr >> 8 == 0xD1:
            regs = {0xFB: self.count & 0xFF, 0xFC: self.count >> 8,
                    0xFD: self.bank & 0xFF, 0xFE: self.bank >> 8}
            return regs.get(off, FLOAT)
        if fault == "alias_a6":
            off &= ~0x40
        r = self.bank_ram()
        if r is None:
            return FLOAT
        return r[off] | 0x08 if fault == "stuck_d3" else r[off]

    def write(self, addr, v):
        off = addr & 0xFF
        if addr >> 8 == 0xD1 and off == 0xFF:
            new = 1 if (v != 0 if fault == "latch_any_bit" else v & 0x80) else 0
            if fault == "ram_lost_on_deselect" and self.sel and not new:
                self.ram = [bytearray(256) for _ in range(banks)]
            self.sel = new
            return
        if not self.selected() and fault != "writes_when_deselected":
            return
        if addr >> 8 == 0xD1:
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
        if fault == "offset_ff_dead" and off == 0xFF:
            return
        r = self.bank_ram()
        if r is not None:
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
