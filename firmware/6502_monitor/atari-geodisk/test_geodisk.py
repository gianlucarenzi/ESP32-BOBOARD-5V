"""Host test of GEODISK.SYS on py65.

A minimal Atari around the 6502: RAM everywhere (the ROM / RAM switch of
PORTB bit 0 is only recorded), SIOV at $E459 = JMP to a fake SIO that
answers every call with a timeout ($8A), CIO output trapped, DOSINI pointing
to a stub that counts its calls, and the 6502_monitor device (registers in
$D1xx, RAM window $D500-$D5FF).  The installer runs through INITAD, then the
test drives SIO calls for D9: and other drives and checks the results.

Requires py65 (pip install py65).

Usage: test_geodisk.py [GEODISK.SYS] [banks] [MEMLO, hex]
"""
import sys
from py65.devices.mpu6502 import MPU
from py65.memory import ObservableMemory

PATH = sys.argv[1] if len(sys.argv) > 1 else "GEODISK.SYS"
BANKS = int(sys.argv[2]) if len(sys.argv) > 2 else 300
MEMLO0 = int(sys.argv[3], 16) if len(sys.argv) > 3 else 0x1F40   # MyDOS-like

DDEVIC, DUNIT, DCOMND, DSTATS = 0x300, 0x301, 0x302, 0x303
DBUFLO, DBYTLO, DAUX1 = 0x304, 0x308, 0x30A
SHPDVS, MEMLO, DOSINI, PORTB = 0x248, 0x2E7, 0x0C, 0xD301
SIOV, CIOV = 0xE459, 0xE456
FAKE_SIO, OLD_DOSINI, DOSINI_CALLS = 0xC933, 0x3000, 0x3100
BUF = 0x4000
SENT = 0x0600                                 # return address sentinel
fails = []


def check(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        fails.append(what)


class Device:
    def __init__(self, banks):
        self.sel, self.bank, self.count = 0, 0, banks
        self.ram = [bytearray(256) for _ in range(banks)]

    def read(self, a):
        if not self.sel:
            return 0xFF
        if a >> 8 == 0xD1:
            return {0xFB: self.count & 0xFF, 0xFC: self.count >> 8,
                    0xFD: self.bank & 0xFF, 0xFE: self.bank >> 8}.get(a & 0xFF, 0xFF)
        return self.ram[self.bank][a & 0xFF] if self.bank < self.count else 0xFF

    def write(self, a, v):
        if a == 0xD1FF:
            self.sel = 1 if v & 0x80 else 0
        elif not self.sel:
            return
        elif a == 0xD1FD:
            self.bank = (self.bank & 0xFF00) | v
        elif a == 0xD1FE:
            self.bank = (self.bank & 0xFF) | v << 8
        elif a >> 8 == 0xD5 and self.bank < self.count:
            self.ram[self.bank][a & 0xFF] = v


def machine(banks):
    dev = Device(banks)
    mem = ObservableMemory()
    io = list(range(0xD100, 0xD200)) + list(range(0xD500, 0xD600))
    mem.subscribe_to_read(io, dev.read)
    mem.subscribe_to_write(io, dev.write)
    mem[PORTB] = 0xFF
    for i, b in enumerate([0x4C, FAKE_SIO & 0xFF, FAKE_SIO >> 8]):      # SIOV: JMP
        mem[SIOV + i] = b
    for i, b in enumerate([0xA0, 0x8A, 0x8C, 0x03, 0x03, 0x60]):        # LDY #$8A, STY DSTATS, RTS
        mem[FAKE_SIO + i] = b
    for i, b in enumerate([0xEE, 0x00, 0x31, 0x60]):                    # INC $3100, RTS
        mem[OLD_DOSINI + i] = b
    mem[DOSINI], mem[DOSINI + 1] = OLD_DOSINI & 0xFF, OLD_DOSINI >> 8
    mem[MEMLO], mem[MEMLO + 1] = MEMLO0 & 0xFF, MEMLO0 >> 8
    mem[CIOV] = 0x60
    return dev, mem


def load(mem, path):
    data, i, init = open(path, "rb").read(), 0, None
    while i < len(data):
        if data[i:i + 2] == b"\xff\xff":
            i += 2
        s, e = data[i] | data[i + 1] << 8, data[i + 2] | data[i + 3] << 8
        seg = data[i + 4:i + 5 + e - s]
        i += 4 + len(seg)
        for k, b in enumerate(seg):
            mem[s + k] = b
        if s <= 0x2E2 <= e:
            init = mem[0x2E2] | mem[0x2E3] << 8
    return init


def call(mem, addr, out=None):
    """JSR addr; returns when it RTSes to the sentinel."""
    mpu = MPU(memory=mem, pc=addr)
    mpu.sp = 0xFF
    mem[0x1FF], mem[0x1FE] = (SENT - 1) >> 8, (SENT - 1) & 0xFF
    mpu.sp = 0xFD
    for _ in range(50_000_000):
        if mpu.pc == CIOV and out is not None:
            out.append("\n" if mpu.a == 0x9B else chr(mpu.a))
        if mpu.pc == SENT:
            return mpu
        mpu.step()
    raise RuntimeError("no return from $%04X" % addr)


def sio(mem, unit, cmd, sector=0, length=256, device=0x31):
    mem[DDEVIC], mem[DUNIT], mem[DCOMND] = device, unit, cmd
    mem[DBUFLO], mem[DBUFLO + 1] = BUF & 0xFF, BUF >> 8
    mem[DBYTLO], mem[DBYTLO + 1] = length & 0xFF, length >> 8
    mem[DAUX1], mem[DAUX1 + 1] = sector & 0xFF, sector >> 8
    mpu = call(mem, SIOV)
    return mpu.y, mem[DSTATS], bool(mpu.p & 0x80)


# ---- install ------------------------------------------------------------
print("--- install, %d banks" % BANKS)
dev, mem = machine(BANKS)
init = load(mem, PATH)
out = []
call(mem, init, out)
print("".join(out).rstrip())
base = (MEMLO0 + 0xFF) & 0xFF00                 # page aligned
handler = mem[SIOV + 1] | mem[SIOV + 2] << 8
res_end = mem[MEMLO] | mem[MEMLO + 1] << 8
check(bytes(mem[base + 14 + i] for i in range(4)) == b"GEOD", "resident copy at $%04X (signature)" % base)
check(base < handler < res_end, "SIOV -> handler $%04X" % handler)
check(base < res_end < base + 0x300, "MEMLO raised to $%04X" % res_end)
check(mem[PORTB] & 1 == 0, "OS ROM switched off (PORTB bit 0 = 0)")
check((mem[DOSINI] | mem[DOSINI + 1] << 8) != OLD_DOSINI, "DOSINI hooked")
check(dev.sel == 0, "device deselected after the install")
check("D9: %d SECTORS" % BANKS in "".join(out), "banner with the sector count")

# ---- other drives and devices go to the original SIO ---------------------
print("--- other drives")
check(sio(mem, 1, 0x52, 1)[:2] == (0x8A, 0x8A), "D1: read goes to the original SIO")
check(sio(mem, 9, 0x52, 1, device=0x40)[:2] == (0x8A, 0x8A), "device $40 unit 9 goes to the original SIO")

# ---- D9: -------------------------------------------------------------------
print("--- D9:")
y, st, neg = sio(mem, 9, 0x53, length=4)
check((y, st, neg) == (1, 1, False), "status: success")
check([mem[BUF + i] for i in range(4)] == [0x20, 0xFF, 0xE0, 0x00], "status bytes: double density")
y, st, neg = sio(mem, 9, 0x4E, length=12)
pc = [mem[BUF + i] for i in range(12)]
check(y == 1 and pc[2] << 8 | pc[3] == BANKS and pc[6:8] == [1, 0] and pc[5] == 4,
      "PERCOM: %d sectors per track, 256 bytes, MFM" % (pc[2] << 8 | pc[3]))
check(sio(mem, 9, 0x4F, length=12)[0] == 1, "write PERCOM accepted")

for b in dev.ram:                              # dirty the RAM, format clears it
    b[:] = bytes([0x5A]) * 256
y, st, neg = sio(mem, 9, 0x21, length=256)
check(y == 1 and all(mem[BUF + i] == 0xFF for i in range(256)), "format: success, no bad sectors")
check(all(not any(b) for b in dev.ram), "format: every sector zeroed")


def pattern(s, n):
    return bytes(((i * 7) ^ s ^ (s >> 8)) & 0xFF for i in range(n))


for s in range(1, BANKS + 1):                  # sectors 1-3: 128 bytes, as on DD disks
    n = 128 if s <= 3 else 256
    for i, b in enumerate(pattern(s, n)):
        mem[BUF + i] = b
    if sio(mem, 9, 0x57 if s & 1 else 0x50, s, n)[0] != 1:
        check(False, "write sector %d" % s)
        break
else:
    check(True, "write sectors 1-%d (W and P)" % BANKS)
check(dev.ram[0][:128] == pattern(1, 128) and dev.ram[0][128:] == bytes(128),
      "sector 1 = bank 0, 128 bytes, the rest untouched")
check(dev.ram[BANKS - 1] == pattern(BANKS, 256), "last sector = last bank")

ok = True
for s in range(1, BANKS + 1):
    n = 128 if s <= 3 else 256
    for i in range(256):
        mem[BUF + i] = 0
    y = sio(mem, 9, 0x52, s, n)[0]
    if y != 1 or bytes(mem[BUF + i] for i in range(n)) != pattern(s, n):
        ok = False
        print("    sector %d: Y=$%02X" % (s, y))
        break
check(ok, "read back sectors 1-%d" % BANKS)

check(sio(mem, 9, 0x52, 0) == (0x90, 0x90, True), "sector 0: error $90, N set")
check(sio(mem, 9, 0x52, BANKS + 1) == (0x90, 0x90, True), "sector %d: error $90" % (BANKS + 1))
check(sio(mem, 9, 0x58)[:2] == (0x8B, 0x8B), "unknown command: NAK $8B")
check(dev.sel == 0 and mem[SHPDVS] == 0, "device and SHPDVS deselected after the calls")

# ---- RESET: DOSINI -------------------------------------------------------
print("--- RESET")
mem[MEMLO], mem[MEMLO + 1] = 0x00, 0x07       # warm start resets MEMLO
mem[PORTB] |= 1                                # and may switch the ROM back on
calls = mem[DOSINI_CALLS]
call(mem, mem[DOSINI] | mem[DOSINI + 1] << 8)
check(mem[DOSINI_CALLS] == calls + 1, "original DOSINI called")
check((mem[MEMLO] | mem[MEMLO + 1] << 8) == res_end, "MEMLO above the handler again")
check(mem[PORTB] & 1 == 0, "OS copy in RAM switched on again")
check(sio(mem, 9, 0x52, 4)[0] == 1, "D9: still works")

# ---- second load -----------------------------------------------------------
out = []
call(mem, init, out)
check("ALREADY INSTALLED" in "".join(out), "second load: ALREADY INSTALLED")

# ---- no device -------------------------------------------------------------
print("--- no device")
dev, mem = machine(0)
init = load(mem, PATH)
out = []
call(mem, init, out)
check("NO DEVICE" in "".join(out), "no banks: NO DEVICE, nothing installed")
check((mem[SIOV + 1] | mem[SIOV + 2] << 8) == FAKE_SIO and mem[PORTB] & 1, "SIOV and ROM untouched")

print("\n%s" % ("ALL CHECKS PASSED" if not fails else "%d CHECKS FAILED" % len(fails)))
sys.exit(1 if fails else 0)
