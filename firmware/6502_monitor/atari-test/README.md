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
| `BANK REG` | bank register `$xxFD` written and read back, `$00-$FF` |
| `PROTECT` | deselected with `$7F` (bit 7 = 0, other bits set), writes to RAM and bank register are ignored; reselected with `$81`, the old values are back |
| `MARCH` | all 256 banks filled with `offset EOR bank EOR key` and verified, then the complement: stuck data bits, address aliasing, banks that alias each other |
| `D1 KEEP` | after the `$D5xx` tests and a deselect, `$D1xx` still holds its last pattern: windows independent, RAM kept while deselected |

Output on success:

```
BANKED RAM TEST
WINDOW $xx00-$xx7F, BANK $xxFD

D1 BANK REG OK
D1 PROTECT  OK
D1 MARCH    OK
D5 BANK REG OK
D5 PROTECT  OK
D5 MARCH    OK
D1 KEEP     OK

ALL TESTS PASSED
```

On failure each test prints `FAIL $nnnnnn` (errors, 24 bit) and its first
four errors as `ERR D1 B=bb O=oo E=ee G=gg` (window, bank, offset, expected,
got); the total is printed at the end.

## Host simulation

`sim_banked_ram.py` runs the XEX on [py65](https://github.com/mnaberez/py65)
against a model of the device that follows the firmware rules, with CIO
trapped to the terminal:

```sh
pip install py65
make sim        # must print ALL TESTS PASSED
make simall     # every injected fault must end with ERRORS
```

Faults: bank register ignored, latch ignored, D3 stuck at 1, A6 not decoded,
windows sharing one RAM, banks aliased on 7 bits, writes accepted while
deselected.  All of them are detected.  The simulation checks the test
program, not the bus timing: that needs the real board.

The ESP32 log shows every access; a full run is ~300k accesses to the windows, so most
log events are dropped (and counted) at 115200 baud.
