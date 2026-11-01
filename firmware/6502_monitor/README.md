# Atari Banked RAM PBI / CCTL Peripheral & Bus Monitor

ESP32 firmware for the ESP32-BOBOARD-5V that behaves as a banked RAM
expansion in the Atari I/O area, GeoRAM style with fixed roles: the PBI page
`$D1xx` holds the registers, the CCTL page `$D500-$D5FF` is a 256-byte window
on the RAM.  Every access to both pages is logged over a lock-free ring
buffer.

## Features
- **Two pages**, selected by the Atari MMU / PLA (on a C64 the same two
  select pins are IO1 / IO2, see below):
    - `~D1XX` → `$D100-$D1FF`: registers
    - `~CCTL` → `$D500-$D5FF`: RAM window

  Only A0-A7 are sampled: the page comes from the select line.
- **Map:**

  | Address | Function |
  |---|---|
  | `$D500-$D5FF` | 256 bytes of the current bank (R/W) |
  | `$D1FB-$D1FC` | number of banks (R, 16 bit, little-endian) |
  | `$D1FD-$D1FE` | bank register (R/W, 16 bit, little-endian) |
  | `$D1FF` | PBI select latch (W): bit 7 = device ID `$80`; never driven on reads (PBI interrupt status) |
  | `$D100-$D1FA` | reserved: never driven, writes ignored |

- **PBI rules:** a write to `$D1FF` selects the device when bit 7 is 1 and
  deselects it when bit 7 is 0 (each PBI device latches only its own ID
  bit).  Deselected, the device never drives the bus and ignores every
  write but the latch.
- **Banks:** 256 bytes each, allocated at boot from the free ESP32 DRAM heap
  (every byte-addressable block, minus `BANK_HEAP_RESERVE` = 24 KB for the
  system): an estimated 200-230 KB in total, to be confirmed on the board
  (the exact count is printed at boot and readable at `$D1FB-$D1FC`).  A
  bank number at or beyond the count selects no RAM: the window is not
  driven and writes are ignored.  Linear address: bank = address >> 8,
  offset = address & `$FF`, as on a GeoRAM.
- **State:** power-up deselected, bank 0, RAM `$00`.  The device does not
  see the Atari RESET, so its state is kept across it; RAM is not kept
  across an ESP32 reset.
- **No ROM, no MPD, no EXTSEL:** the OS PBI scan finds no signature at
  `$D803` / `$D80B` and skips the device; programs select it directly by
  writing `$80` to `$D1FF` (and `$00` to release it).  The scan (power-up
  and every RESET) writes `$80` then `$00` to `$D1FF`: the device ends
  deselected.
- **Bus monitor:** every access to both pages is logged, selected or not;
  reads of other devices are sampled just before PHI2 falls.
- **Serial output** (`LogTask` on Core 0, the only writer on Serial, 115200
  baud by default, `-D LOG_BAUD=...`; each line is one `Serial.write()`, so
  lines never interleave):
    ```
    [    1.234567] [D1FF - PBI_SELECT          ] W $80  SELECTED
    [    1.234601] [D1FD - BANK_SELECT_L       ] W $03  SEL
    [    1.234620] [D1FE - BANK_SELECT_H       ] W $01  SEL
    [    1.234640] [D505 - RAM bank $0103 +$05 ] W $AA  SEL
    [    1.234680] [D505 - RAM bank $0103 +$05 ] R $AA  SEL DRV
    [    1.300000] [D105 - RESERVED            ] R $FF  SEL
    ```
  `SEL` = device selected, `DRV` = byte driven by the device, `-` =
  deselected (the logged byte is whatever was on the bus).
  Timestamps come from the Core 1 cycle counter (CCOUNT extended to 64 bit),
  in µs since the bus loop started.  Events are dropped and counted when the
  1024-entry ring is full: a block copy through the window easily fills it.
- **Bus loop:** IRAM-resident `MonitorTask` owns Core 1 (interrupts masked,
  interrupt watchdog disabled, no FreeRTOS calls).  `~D1XX` is not
  PHI2-qualified, so A0-A7, R/W and the page selects are decoded during PHI1
  (`BUS_ADDR_SETTLE_NS`, default 200 ns after PHI2 falls) and the read byte
  (RAM of the current bank or a register) is already in `GPIO.out` when
  PHI2 rises: one store turns D0-D7 on.  The PHI1 decode is inline assembly
  (22 instructions from the `GPIO.in` load to the `GPIO.out` store for a RAM
  read), with the device state in a fixed-layout struct (RAM pointer of the
  current bank, NULL while deselected or beyond the count; count and bank
  registers; latch) and a 256-entry table for
  the byte-to-pins conversion.  A
  select that appears only on the rising edge is handled from that sample
  (late drive).  Write data is sampled `BUS_WR_SAMPLE_NS` (default 120 ns)
  after PHI2 rises.  Optimizations ported from Vera-Module-RBL-XE.
- **Hardware Target:** NodeMCU DevKit V1 (ESP32-WROOM, 30 pins).

## Atari Test Program
[`atari-test/`](atari-test/README.md): `banked_ram_test.xex` (cc65) checks
bank count, 16-bit bank register, deselect protection, a march over every
bank (all 256 offsets), the bank limit and the RAM kept across a deselect.  `make sim` / `make simall` run it on
py65 against a model of the device, with injected faults.

## Commodore 64: GeoRAM (env `c64`)
`pio run -e c64` builds the same board as a GeoRAM-compatible expansion for
the C64 expansion port (`-D BUS_C64=1`):

| Address | Function |
|---|---|
| `$DE00-$DEFF` (/IO1) | 256-byte window on the RAM (R/W) |
| `$DFFE` (/IO2) | page within the 16 KB block, bits 0-5 (W) |
| `$DFFF` (/IO2) | block (W) |

Always enabled, no latch; the registers are write-only as on the original
GeoRAM (software keeps a copy) and the rest of IO2 is not decoded.  RAM
address = block × 16384 + page × 256 + offset.

**Size: 256 KB (16 blocks)** by default, `-D GEORAM_KB=...` for another power
of two.  As on a real GeoRAM the block register is masked to the size, so
block 16 aliases block 0: size detection sees a 256 KB GeoRAM (VICE has the
same size).  The DRAM heap alone holds ~210 KB, so the remaining pages come
from the free IRAM (~70 KB, 32-bit access only): every RAM access of the C64
loop is a 32-bit load, and a read-modify-write for stores.  If the heap
turns out to be too small, the size drops to 128 KB and the banner says so.
The original geoRAM was 512 KB: software that needs exactly that will not
work.

The VIC-II owns the bus during PHI1, so the C64 loop works while PHI2 is
high, with the scheme of Vera-Module-RBL-C64: after PHI2 rises it polls
/IO1 / /IO2 for at most `IO_QUALIFY_NS` (default 200 ns; the PLA asserts
them only for CPU accesses, with the address already stable), samples A0-A7
as soon as one is LOW and drives a read byte at once, until PHI2 falls
(needed 100 ns before the fall, ~490 ns after the rise).  Write data is
sampled `BUS_WR_SAMPLE_NS` (default 300 ns) after the rise.  Timing to be
measured on the real bus.

Wiring (C64 port pinout checked against c64-wiki.com and retroisle.com):

| Signal | C64 expansion port | J3 |
|---|---|---|
| /IO1 | 7 | 12 (GPIO23, `~CCTL` input) |
| /IO2 | 10 | 11 (GPIO22, `~D1XX` input) |
| D0 / D1 / D2 / D3 | 21 / 20 / 19 / 18 | 2 / 3 / 5 / 6 |
| D4 / D5 / D6 / D7 | 17 / 16 / 15 / 14 | 22 / 23 / 8 / 9 |
| A0 / A1 / A2 / A3 | Y / X / W / V | 18 / 19 / 20 / 21 |
| A4 / A5 / A6 / A7 | U / T / S / R | 16 / 17 / 10 / 15 |
| Φ2 | E | 1 |
| R/W | 5 | 7 |
| GND | 1, 22, A, Z | 29, 30 |
| +5V | 2, 3 | 31, 32 |

Test program: [`c64-test/`](c64-test/README.md), `georam_test.prg`
(cc65): presence, size by block aliasing, page register bits 0-5, a march
over every page of every block; `make sim` / `make simall` run it on py65
against a model of the GeoRAM, with injected faults.

Log lines: `[DE05 - RAM blk $03 pg $10 +$05] R $AA  SEL DRV`,
`GEORAM_PAGE`, `GEORAM_BLOCK`.  As on the Atari, the module is programmed
only disconnected from the computer, from the PC.

## Hardware Setup
Refer to the following documents for detailed pinout:
- [General Pin Mapping](PIN_MAPPING.md) (NodeMCU DevKit V1)
- [Hardware Architecture](HARDWARE.md)
- Wiring infographics, J3 to the computer (generated by
  [`docs/wiring.py`](docs/wiring.py)): [Atari 130XE](docs/wiring_atari.svg),
  [Commodore 64](docs/wiring_c64.svg); breadboard view in the Fritzing
  style ([`docs/breadboard.py`](docs/breadboard.py)):
  [Atari 130XE](docs/breadboard_atari.svg), [Commodore 64](docs/breadboard_c64.svg)

## Development
This project uses **PlatformIO**.
- Style: **Allman** (braces on new line).
- Column Limit: **80**.
- Tool: `clang-format`.

### Configuration
Two build environments: `nodemcu-32s` (Atari) and `c64` (GeoRAM, see
above).  Bus timing can be
tuned with `-D BUS_ADDR_SETTLE_NS=...`, `-D BUS_WR_SAMPLE_NS=...` and
`-D BUS_PHI2_HIGH_MAX_NS=...` after measuring the real bus.

Build and flash:
```sh
pio run -e nodemcu-32s --target upload
```

**Programming rule:** the ESP32 module is always programmed on its own,
disconnected from the Atari and connected only to the PC (serial bootloader).

**VDD_SDIO efuse:** GPIO12 is not used by the firmware but is still wired to
U1 / J3 pin 4 (former A8), and at every chip reset the ESP32 ROM samples it to
select the flash voltage (HIGH = 1.8 V: the WROOM-32 flash does not boot).
Before every upload, `efuse_vdd_sdio.py` reads the efuses and, if they are
still blank, burns VDD_SDIO to a fixed 3.3 V
(`espefuse.py set_flash_voltage 3.3V`, **irreversible**), so that GPIO12 is
ignored from then on.  Already set: nothing to do.  Any other combination:
the upload stops without burning anything.  Controlled by
`custom_efuse_vdd_sdio = 3.3V` in `platformio.ini` (empty value = no check).
On first use the script creates `.pio/espefuse-venv` with the espefuse
dependencies (needs network access once).  See [HARDWARE.md](HARDWARE.md#3-esp32-strapping-pins).

**Serial monitor:** `monitor_rts = 0` and `monitor_dtr = 0`, so opening the
monitor does not reset the ESP32 through the NodeMCU auto-reset circuit.  The
debug UART is also on J4: read-only use needs GND (pin 1) and TX0 (pin 3,
3.3 V); leave RX0 (pin 2) unconnected.
