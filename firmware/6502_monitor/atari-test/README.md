# Banked RAM test (Atari)

`banked_ram_test.xex` tests the 6502_monitor banked RAM from the Atari side,
in both windows (`$D1xx` PBI, `$D5xx` CCTL).

## Build and run

```sh
make            # cl65 -t atari -C atari-asm.cfg -> banked_ram_test.xex
```

Load the XEX from DOS (or any XEX loader).  It prints on the screen through
CIO (E:), runs in about one second and stops in an endless loop: press RESET
to leave.  Both windows are deselected at the end.

**Do not run it with a cartridge that bank-switches through CCTL** (`$D5xx`:
XEGS, AtariMax, SpartaDOS X, ...): the test writes the whole `$D5xx` page.
Set `TEST_CCTL = 0` in the source to test only `$D1xx`.

## Tests

| Test | What it checks |
|---|---|
| `COUNT` | number of banks `$xxFB-$xxFC` (printed): not zero, read-only |
| `BANK REG` | 16-bit bank register `$xxFD-$xxFE` written and read back: low byte `$00-$FF` with high byte `$00`, `$55`, `$AA`, `$FF` |
| `PROTECT` | deselected with `$7F` (bit 7 = 0, other bits set), writes to RAM and bank register are ignored; reselected with `$81`, the old values are back |
| `MARCH` | every bank filled and verified, then the complement: bytes 0-1 hold the bank number, the others `offset EOR bank lo EOR bank hi EOR key`.  Stuck data bits, address aliasing, banks that alias each other |
| `LIMIT` | writes to bank = count (no RAM behind it) do not land in the first or the last bank |
| `D1 KEEP` | after the `$D5xx` tests and a deselect, `$D1xx` still holds its last pattern: windows independent, RAM kept while deselected |

The number of banks comes from the ESP32 heap; run time grows with it, a
few seconds per 64 KB.

Output on success (the count depends on the board):

```
BANKED RAM TEST
RAM $xx00-$xx7F, COUNT $xxFB
BANK $xxFD, SELECT $xxFF

D1 COUNT    $012C OK
D1 BANK REG OK
D1 PROTECT  OK
D1 MARCH    OK
D1 LIMIT    OK
D5 COUNT    $012C OK
D5 BANK REG OK
D5 PROTECT  OK
D5 MARCH    OK
D5 LIMIT    OK
D1 KEEP     OK

ALL TESTS PASSED
```

On failure each test prints `FAIL $nnnnnn` (errors, 24 bit) and its first
four errors as `ERR D1 B=bbbb O=oo E=ee G=gg` (window, bank, offset,
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

The model has 300 banks per window (`sim_banked_ram.py XEX none N` for N),
so the high byte of the bank register is used.  Faults: bank register
ignored, bank high byte ignored, latch ignored, D3 stuck at 1, A6 not
decoded, windows sharing one RAM, writes accepted while deselected, banks
beyond the count wrapping onto existing ones, bank count writable.  All of
them are detected.  The simulation checks the test
program, not the bus timing: that needs the real board.

The ESP32 log shows every access; a full run is about 4.5 accesses to the windows per byte of banked RAM, so most
log events are dropped (and counted) at 115200 baud.
