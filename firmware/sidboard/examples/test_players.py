"""Host test of the SID dump players on py65.

cc65 player: a minimal Atari around the 6502 (RAM, CIOV trapped by a fake
CIO with E: and D:SIDDUMP.BIN, RTCLOK advanced every "frame", CH = no key)
and a model of the sidboard firmware (sid-atari-cctl: $D500-$D51E always on;
sid-atari-pbi: $D100-$D11E while $D1FF bit 7 is set).  Every register write is
recorded with the frame it falls in, then compared with the dump.

BASIC player: its machine-language routine (the DATA bytes) is run alone
with the BASIC USR calling convention.

Requires py65.  Usage: test_players.py (from examples/, after make in cc65/)
"""
import re
import sys
from py65.devices.mpu6502 import MPU
from py65.memory import ObservableMemory

DUMP = open("SIDDUMP.BIN", "rb").read()
FRAMES = len(DUMP) // 25
CIOV, SENT = 0xE456, 0x0700
INSTR_PER_TICK = 6000
fails = []


def check(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        fails.append(what)


class Sid:
    def __init__(self, pbi):
        self.pbi, self.sel = pbi, 0 if pbi else 1
        self.base = 0xD100 if pbi else 0xD500
        self.regs = bytearray(31)
        self.writes = []          # (tick, reg, value)
        self.stray = []           # writes outside the SID page / while deselected
        self.latch = []
        self.tick = 0

    def read(self, a):
        off = a & 0xFF
        if a >> 8 == self.base >> 8 and self.sel and off < 31:
            return self.regs[off]
        return 0xFF

    def write(self, a, v):
        off = a & 0xFF
        if self.pbi and a == 0xD1FF:
            self.sel = 1 if v & 0x80 else 0
            self.latch.append((self.tick, v))
            return
        if a >> 8 == self.base >> 8 and self.sel and off < 31:
            self.regs[off] = v
            self.writes.append((self.tick, off, v))
        else:
            self.stray.append((self.tick, a, v))


def load_xex(mem, path):
    data, i, run, inits = open(path, "rb").read(), 0, None, []
    while i < len(data):
        if data[i:i + 2] == b"\xff\xff":
            i += 2
        s, e = data[i] | data[i + 1] << 8, data[i + 2] | data[i + 3] << 8
        seg = data[i + 4:i + 5 + e - s]
        i += 4 + len(seg)
        for k, b in enumerate(seg):
            mem[s + k] = b
        if s <= 0x2E2 <= e:
            inits.append(mem[0x2E2] | mem[0x2E3] << 8)
            mem[0x2E2] = mem[0x2E3] = 0
        if s <= 0x2E0 <= e:
            run = mem[0x2E0] | mem[0x2E1] << 8
    return run, inits


class FakeCio:
    def __init__(self, mem):
        self.mem, self.files, self.out = mem, {}, []

    def call(self, mpu):
        m, x = self.mem, mpu.x
        cmd = m[0x342 + x]
        buf = m[0x344 + x] | m[0x345 + x] << 8
        ln = m[0x348 + x] | m[0x349 + x] << 8
        st = 1
        if cmd == 3:                                   # OPEN
            name = bytearray()
            while m[buf + len(name)] not in (0x9B, 0) and len(name) < 40:
                name.append(m[buf + len(name)])
            name = name.decode("latin-1")
            if name.startswith("E"):
                self.files[x] = None
                m[0x340 + x] = 0
            elif name == "D:SIDDUMP.BIN":
                self.files[x] = [DUMP, 0]
                m[0x340 + x] = 1
            else:
                st = 170                               # file not found
        elif cmd in (5, 7):                            # GET RECORD / CHARS
            f = self.files.get(x)
            if not f:
                st = 136
            else:
                n = min(ln, len(f[0]) - f[1])
                for k in range(n):
                    m[buf + k] = f[0][f[1] + k]
                f[1] += n
                m[0x348 + x], m[0x349 + x] = n & 0xFF, n >> 8
                st = 1 if n == ln else 136
        elif cmd in (9, 11):                           # PUT RECORD / CHARS
            if ln == 0:                                # single character in A
                self.out.append(mpu.a)
            else:
                self.out += [m[buf + k] for k in range(ln)]
        elif cmd == 12:                                # CLOSE
            self.files.pop(x, None)
            m[0x340 + x] = 0xFF
        else:
            st = 1
        m[0x343 + x] = st
        mpu.y = st
        mpu.p = (mpu.p & ~0x82) | (0x80 if st & 0x80 else 0) | (0 if st else 2)

    def text(self):
        return bytes(self.out).decode("latin-1").replace("\x9b", "\n")


def run_cc65(path, pbi):
    sid = Sid(pbi)
    mem = ObservableMemory()
    pages = list(range(0xD100, 0xD200)) + list(range(0xD500, 0xD600))
    mem.subscribe_to_read(pages, sid.read)
    mem.subscribe_to_write(pages, sid.write)
    mem[CIOV] = 0x60
    for a, v in ((0x2E5, 0x1F), (0x2E6, 0xBC), (0x2E7, 0x00), (0x2E8, 0x1F), (0x6A, 0xC0),
                 (0x2FC, 0xFF), (0x0A, SENT & 0xFF), (0x0B, SENT >> 8)):
        mem[a] = v
    mem[SENT], mem[SENT + 1], mem[SENT + 2] = 0x4C, SENT & 0xFF, SENT >> 8   # JMP *
    for iocb in range(1, 8):                                    # IOCB 1-7 free, as after OS init
        mem[0x340 + iocb * 16] = 0xFF
    cio = FakeCio(mem)
    run, inits = load_xex(mem, path)

    def go(addr, limit):
        mpu = MPU(memory=mem, pc=addr)
        mpu.sp = 0xFF
        mem[0x1FF], mem[0x1FE] = (SENT - 1) >> 8, (SENT - 1) & 0xFF
        mpu.sp = 0xFD
        for n in range(limit):
            if mpu.pc == CIOV:
                cio.call(mpu)
            if mpu.pc == SENT:
                return True
            if n % INSTR_PER_TICK == 0:
                mem[0x14] = (mem[0x14] + 1) & 0xFF
                sid.tick += 1
            mpu.step()
        return False

    for a in inits:
        go(a, 1_000_000)
    done = go(run, 60_000_000)
    return sid, cio.text(), done


for path, pbi in (("cc65/sid_dump_player.xex", False), ("cc65/sid_dump_player_pbi.xex", True)):
    print("--- %s (%s)" % (path, "sid-atari-pbi" if pbi else "sid-atari-cctl"))
    sid, text, done = run_cc65(path, pbi)
    print("    " + text.strip().replace("\n", "\n    "))
    check(done, "program returned to DOS")
    frames = {}
    for tick, reg, v in sid.writes:
        frames.setdefault(tick, []).append((reg, v))
    body = [w for w in sid.writes]
    regs_written = [(r, v) for _, r, v in body]
    expect = [(r, DUMP[f * 25 + r]) for f in range(FRAMES) for r in range(25)]
    silence = [(4, 0), (0x0B, 0), (0x12, 0), (0x18, 0)]
    check(regs_written == expect + silence,
          "%d frames x 25 registers in order, then gates off and volume 0" % FRAMES)
    ticks = [t for t, _, _ in body[:FRAMES * 25]]
    per_frame = [ticks[k * 25:(k + 1) * 25] for k in range(FRAMES)]
    check(all(len(set(fr)) == 1 for fr in per_frame), "each frame written within one VBlank")
    starts = [fr[0] for fr in per_frame]
    check(all(b - a == 1 for a, b in zip(starts, starts[1:])), "one frame per VBlank (50 Hz)")
    check(not sid.stray, "no write outside the SID page (%d)" % len(sid.stray))
    if pbi:
        check([v for _, v in sid.latch] == [0x80, 0x00], "$D1FF: $80 before playing, $00 at the end")
    check("Played %d frames" % FRAMES in text, "message: Played %d frames" % FRAMES)

# ---- BASIC player: the machine-language USR routine ------------------------
print("--- atari-basic/sid_dump_player.bas: USR routine")
src = open("atari-basic/sid_dump_player.bas").read()
data_lines = re.findall(r"^\d+ DATA (.*)$", src, re.M)
code = [int(v) for line in data_lines for v in line.split(",")]
for pbi in (False, True):
    sid = Sid(pbi)
    sid.sel = 1
    mem = ObservableMemory()
    pages = list(range(0xD100, 0xD200)) + list(range(0xD500, 0xD600))
    mem.subscribe_to_read(pages, sid.read)
    mem.subscribe_to_write(pages, sid.write)
    ML, FR = 0x0600, 0x4000
    for k, b in enumerate(code):
        mem[ML + k] = b
    for k in range(25):
        mem[FR + k] = DUMP[25 * 7 + k]
    base = sid.base
    # USR(ADR, frame, base): stack = argc, frame hi/lo, base hi/lo
    mpu = MPU(memory=mem, pc=ML)
    mpu.sp = 0xFF
    stack = [(SENT - 1) >> 8, (SENT - 1) & 0xFF, base & 0xFF, base >> 8, FR & 0xFF, FR >> 8, 2]
    for b in stack:                                    # push in this order
        mem[0x100 + mpu.sp] = b
        mpu.sp -= 1
    for _ in range(10000):
        if mpu.pc == SENT:
            break
        mpu.step()
    check(mpu.pc == SENT and mpu.sp == 0xFF, "%s: USR returns with a clean stack" % ("PBI" if pbi else "CCTL"))
    check(sorted((r, v) for _, r, v in sid.writes) == [(r, DUMP[25 * 7 + r]) for r in range(25)],
          "%s: the 25 registers of a frame written to $%04X (from $18 down to $00)"
          % ("PBI" if pbi else "CCTL", base))

print("\n%s" % ("ALL CHECKS PASSED" if not fails else "%d CHECKS FAILED" % len(fails)))
sys.exit(1 if fails else 0)
