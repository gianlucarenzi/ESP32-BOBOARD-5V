#!/usr/bin/env python3
"""Make SIDDUMP.BIN (raw frames of 25 SID registers) from a dump header such
as Commando_dmp.h (const uint8_t sidData[] = { ... };).

Usage: mkdump.py [Commando_dmp.h] [SIDDUMP.BIN]
Copy the result to the Atari disk as D:SIDDUMP.BIN.
"""
import re
import sys

src = sys.argv[1] if len(sys.argv) > 1 else "Commando_dmp.h"
dst = sys.argv[2] if len(sys.argv) > 2 else "SIDDUMP.BIN"
text = open(src).read()
body = text[text.index("sidData"):]
body = body[body.index("{") + 1:body.index("}")]
data = bytes(int(v, 0) for v in re.findall(r"0x[0-9A-Fa-f]+|\d+", body))
m = re.search(r"sidLength\s*=\s*(\d+)", text)
if m:
    assert len(data) == int(m.group(1)), (len(data), m.group(1))
assert len(data) % 25 == 0, "not a whole number of 25-byte frames"
open(dst, "wb").write(data)
print("%s: %d bytes, %d frames (%.1f s at 50 Hz)" % (dst, len(data), len(data) // 25, len(data) / 25 / 50))
