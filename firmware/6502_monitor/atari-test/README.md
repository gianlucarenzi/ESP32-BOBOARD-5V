# Banked RAM test (Atari)

`banked_ram_test.xex` tests the 6502_monitor banked RAM (GeoRAM style, fixed
roles) from the Atari side:

| Address | Function |
|---|---|
| `$D500-$D5FF` | 256 bytes of the current bank (R/W), while selected |
| `$D1FB-$D1FC` | number of banks (R, 16 bit) |
| `$D1FD-$D1FE` | bank register (R/W, 16 bit) |
| `$D1FF` | PBI select latch (W): bit 7 = device ID `$80` |

## Build and run

```sh
make            # cl65 -t atari -C atari-asm.cfg -> banked_ram_test.xex
```

Load the XEX from DOS (or any XEX loader).  It prints on the screen through
CIO (E:), runs in about 20 s for 220 KB of banks and stops in an endless
loop: press RESET to leave.  The device is deselected at the end.

**Do not run it with a cartridge that bank-switches through CCTL** (`$D5xx`:
XEGS, AtariMax, SpartaDOS X, ...): the test writes the whole `$D5xx` page.

## Tests

| Test | What it checks |
|---|---|
| `COUNT` | number of banks `$D1FB-$D1FC` (printed): not zero, read-only |
| `BANK REG` | 16-bit bank register `$D1FD-$D1FE` written and read back: low byte `$00-$FF` with high byte `$00`, `$55`, `$AA`, `$FF` |
| `PROTECT` | deselected with `$7F` (bit 7 = 0, other bits set), writes to the RAM window and the bank register are ignored; reselected with `$81`, the old values are back |
| `MARCH` | every bank filled and verified, then the complement: bytes 0-1 hold the bank number, the others `offset EOR bank lo EOR bank hi EOR key`; all 256 offsets, `$D5FF` included.  Stuck data bits, address aliasing, banks that alias each other |
| `LIMIT` | writes to bank = count (no RAM behind it) do not land in the first or the last bank |
| `KEEP` | deselect and reselect: every bank still holds the last pattern |

Output on success (the count depends on the board):

```
BANKED RAM TEST (GEORAM STYLE)
RAM $D500-$D5FF, COUNT $D1FB
BANK $D1FD, SELECT $D1FF

COUNT    $012C OK
BANK REG OK
PROTECT  OK
MARCH    OK
LIMIT    OK
KEEP     OK

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
byte ignored, latch ignored (always selected), latch on any bit instead of
bit 7, D3 stuck at 1, A6 not decoded, writes accepted while deselected,
banks beyond the count wrapping onto existing ones, bank count writable,
offset `$FF` not stored, RAM lost on deselect.  The simulation checks the
test program, not the bus timing: that needs the real board.

The ESP32 log shows every access; a full run is about 5 accesses to the
window per byte of banked RAM, so most log events are dropped (and counted)
at 115200 baud.
