# Banked RAM test (Atari)

`banked_ram_test.xex` tests the 6502_monitor banked RAM (GeoRAM style) from
the Atari side, in both modes:

| Latch written | Registers | RAM window |
|---|---|---|
| `$D1FF` ← `$80` | `$D1xx` | `$D500-$D5FF` (256 bytes) |
| `$D5FF` ← `$80` | `$D5xx` | `$D100-$D1FE` (255 bytes, `$D1FF` protected) |

## Build and run

```sh
make            # cl65 -t atari -C atari-asm.cfg -> banked_ram_test.xex
```

Load the XEX from DOS (or any XEX loader).  It prints on the screen through
CIO (E:), runs in about 30 s for 220 KB of banks and stops in an endless
loop: press RESET to leave.  The device is deselected at the end.

If the firmware is built with `-D D1FF_RAM=1`, set `D1FF_RAM = 1` in the
source too: the RAM window in `$D1xx` is then 256 bytes and the last test
expects `$D1FF` to be RAM.

**Do not run it with a cartridge that bank-switches through CCTL** (`$D5xx`:
XEGS, AtariMax, SpartaDOS X, ...): the device needs both pages and the test
writes the whole `$D5xx` page.

## Tests

For each mode (registers `$D1xx` / RAM `$D5xx`, then registers `$D5xx` /
RAM `$D1xx`):

| Test | What it checks |
|---|---|
| `COUNT` | number of banks `$xxFB-$xxFC` (printed): not zero, read-only, the same in both modes |
| `SHARED` | second mode only, before any RAM write: the pattern written through the first page reads the same through the second one (one RAM) |
| `BANK REG` | 16-bit bank register `$xxFD-$xxFE` written and read back: low byte `$00-$FF` with high byte `$00`, `$55`, `$AA`, `$FF` |
| `EXCL` | `$80` written to the latch offset of the RAM page does not move the registers: one latch at a time |
| `PROTECT` | deselected with `$7F` (bit 7 = 0, other bits set), writes to RAM and bank register (in both pages) are ignored; reselected with `$81`, the old values are back |
| `MARCH` | every bank filled and verified, then the complement: bytes 0-1 hold the bank number, the others `offset EOR bank lo EOR bank hi EOR key`.  Stuck data bits, address aliasing, banks that alias each other |
| `LIMIT` | writes to bank = count (no RAM behind it) do not land in the first or the last bank |

Then:

| Test | What it checks |
|---|---|
| `D1FF PROTECTED` | bank 0 byte `$FF` written as `$A5` through `$D5FF`; with the RAM in `$D1xx`, `$DA` (bit 7 set) written to `$D1FF` must neither reach the RAM nor act as a latch; `$D5FF` still reads `$A5` (`$DA` with `D1FF_RAM = 1`) |

Output on success (the count depends on the board):

```
BANKED RAM TEST (GEORAM STYLE)
COUNT $xxFB BANK $xxFD LATCH $xxFF

SELECT $D1FF, RAM $D500-$D5FF
 COUNT    $012C OK
 BANK REG OK
 EXCL     OK
 PROTECT  OK
 MARCH    OK
 LIMIT    OK

SELECT $D5FF, RAM $D100-$D1FE
 COUNT    $012C OK
 SHARED   OK
 BANK REG OK
 EXCL     OK
 PROTECT  OK
 MARCH    OK
 LIMIT    OK

D1FF PROTECTED OK

ALL TESTS PASSED
```

On failure each test prints `FAIL $nnnnnn` (errors, 24 bit) and its first
four errors as `ERR D5 B=bbbb O=oo E=ee G=gg` (page, bank, offset,
expected, got); the total is printed at the end.

## Host simulation

`sim_banked_ram.py` runs the XEX on [py65](https://github.com/mnaberez/py65)
against a model of the device that follows the firmware rules, with CIO
trapped to the terminal:

```sh
pip install py65
make sim        # must print ALL TESTS PASSED
make simall     # every injected fault must end with ERRORS
```

The model has 300 banks (`sim_banked_ram.py XEX none N` for N), so the high
byte of the bank register is used.  Faults: bank register ignored, bank high
byte ignored, both latches active at once, D3 stuck at 1, A6 not decoded,
a separate RAM per page, writes accepted while deselected, banks beyond the
count wrapping onto existing ones, bank count writable, `$D1FF` not
protected.  All of them are detected.  The simulation checks the test
program, not the bus timing: that needs the real board.

The ESP32 log shows every access; a full run is about 9 accesses to the RAM
page per byte of banked RAM, so most log events are dropped (and counted)
at 115200 baud.
