# GeoRAM test (Commodore 64)

`georam_test.prg` tests the 6502_monitor GeoRAM (PlatformIO env `c64`) from
the C64 side:

| Address | Function |
|---|---|
| `$DE00-$DEFF` (/IO1) | 256-byte window on the RAM |
| `$DFFE` (/IO2) | page within the 16 KB block, bits 0-5 (W) |
| `$DFFF` (/IO2) | 16 KB block (W) |

The registers are write-only: the program keeps a copy.  It works on any
GeoRAM-compatible unit; `EXPECT_BLOCKS` in the source (16 = 256 KB, 0 = any
size) is the size it expects.

## Build and run

```sh
make            # cl65 -t c64 -C c64-asm.cfg -u __EXEHDR__ -> georam_test.prg
```

`LOAD"GEORAM_TEST",8` and `RUN` (BASIC stub `SYS 2061`).  It prints through
the KERNAL (CHROUT), runs in about 30 s for 256 KB and stops in an endless
loop: RUN/STOP + RESTORE or reset to leave.  **It overwrites the whole
GeoRAM.**

## Tests

| Test | What it checks |
|---|---|
| (presence) | block 0 page 0 offset 1 keeps `$A5` and `$5A`; otherwise `NO GEORAM` and stop (the C64 open bus can read `$00`) |
| `SIZE` | blocks 255..0 get their own number at page 0 offset 0; the first block that does not read it back is the size (block register masked to the size, as on a real GeoRAM); printed and checked against `EXPECT_BLOCKS` |
| `ALIAS` | block = size is block 0 (skipped with 256 blocks) |
| `PAGE REG` | the 64 pages of block 0 marked, read back with page register bits 6-7 set: only bits 0-5 count |
| `MARCH` | every page of every block filled and verified, then the complement: byte 0 = page EOR key, byte 1 = block EOR key, byte n = n EOR page EOR block EOR key; all 256 offsets, `$DEFF` included |

Output on success:

```
GEORAM TEST
WINDOW $DE00, PAGE $DFFE, BLOCK $DFFF

SIZE     $10 BLOCKS OK
ALIAS    OK
PAGE REG OK
MARCH    OK

ALL TESTS PASSED
```

On failure each test prints `FAIL $nnnnnn` (errors, 24 bit) and its first
four errors as `ERR B=bb P=pp O=oo E=ee G=gg` (block, page, offset,
expected, got); the total is printed at the end.  If the firmware fell back
to 128 KB (banner), SIZE reports `E=10 G=08`: set `EXPECT_BLOCKS = 8` or `0`.

## Host simulation

`sim_georam.py` runs the PRG on [py65](https://github.com/mnaberez/py65)
against a model of the GeoRAM that follows the firmware, with CHROUT trapped
to the terminal:

```sh
pip install py65
make sim        # must print ALL TESTS PASSED
make simall     # every injected fault must end with ERRORS or NO GEORAM
```

Faults: page register ignored, block register ignored, page register on 8
bits, blocks beyond the size not aliased, D3 stuck at 1, A6 not decoded,
offset `$FF` not stored, a 128 KB unit, two blocks sharing the RAM, no
GeoRAM at all (open bus reading `$00`).  The simulation checks the test
program, not the bus timing: that needs the real board.

The Atari side has its own test, [`../atari-test`](../atari-test/README.md),
for the GeoRAM-style mode of the Atari build.
