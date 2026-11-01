# Project Logs & Architectural Decisions

## 2026-11-01: ROM-less Banked RAM (GeoRAM Style, Fixed Roles), GEODISK D9:, C64 GeoRAM and Test, Bus Loop from RBL-XE, Serial Log, Bootstrap

### GEODISK.SYS: D9: RAM Disk for MyDOS (`atari-geodisk/`)
- **Choices** (asked): MyDOS 4.5x (handles D9: and large disks), hook through the OS copied to the RAM under the ROM and the SIO vector, 256-byte sectors (one bank each, about 850 sectors).  The device has no PBI ROM, so the OS cannot see it as a disk by itself.
- **Installer** (`geodisk.s`, INITAD, back to DOS): finds the device (`$80` to `$D1FF`, count at `$D1FB-$D1FC`; 0 or `$FFFF` = no device, nothing changed), refuses a second install (signature of the copy SIOV points to), copies the resident part to MEMLO page aligned and raises MEMLO, copies `$C000-$CFFF` and `$D800-$FFFF` page by page through a buffer (ROM on to read, off to write; IRQ and NMI off meanwhile), leaves the ROM off, stores the original SIOV target in the handler and points SIOV (`$E459`) at it; DOSINI hook.
- **Resident handler** (`geodisk_res.s`, 423 bytes): device `$31` unit 9 only, everything else to the original SIO.  R / W / P (sector n = bank n-1, the DOS length: 128 bytes for sectors 1-3 of a DD disk), S (double density), PERCOM read (1 track of *count* sectors, 256 bytes, MFM) and write (ignored), format (all sectors zeroed, no bad sectors), NAK `$8B` for the rest, `$90` out of range.  Selects with `SHPDVS` too (PBI convention: another device's IRQ restores the shadow) and deselects at the end.  Buffer addresses are patched into the copy loops: no zero page used.  DOSINI: old DOSINI, PORTB bit 0 off again, MEMLO above the handler.
- **Relocation**: `mkgeodisk.py` assembles the resident part at `$2000` and `$2100` (`cl65 -t none`), every byte that differs by +1 is a high byte to relocate (59); the copy is page aligned, so low bytes never change.  Then the installer at `$6000` and the Atari executable with an INITAD segment.
- **Test** (`test_geodisk.py`, py65): RAM-only machine with SIOV = JMP to a fake SIO (timeout), DOSINI stub, CIO trapped and the device model; 31 checks (install, other drives untouched, status, PERCOM, format, write / read back every sector, errors, RESET path, second load, no device) pass at MEMLO `$1F40`, `$2345`, `$2C00`.  Cheap local labels (`@loop`) cannot span the self-modified labels in ca65: renamed.
- **Not verified**: the real XL OS (DSKINV and MyDOS going through SIOV, RESET behaviour), the MyDOS drive configuration dialogue; to try on a 130XE.

### Product Wording, Breadboard Drawings, Mermaid Timing Diagram
- **Wording** in the READMEs: *200+ KB paged RAM expansion for the Atari 130XE, 256-byte window at $D500 (GeoRAM-style)* and *256 KB GeoRAM-compatible RAM expansion for the C64*.  Not "GeoRAM" alone (Berkeley Softworks product, 512 KB only), and the Atari unit is not the PORTB extended memory of the 130XE / RAMBO; sizes to be confirmed by the boot banner.  The C64 section anchor changed: links in HARDWARE and PIN-MAPPING updated.
- **Breadboard view** (`docs/breadboard.py`, `breadboard_atari.svg`, `breadboard_c64.svg`): Fritzing-style drawings (green board with the J3 header, edge connectors with gold contacts, thick wires in Fritzing colours routed in straight segments).  Connector contacts sit 12 / 24 px off the J3 grid, so no vertical run overlaps another.  `wiring.py` split into data and drawing so both scripts share the pinouts.  ImageMagick's own SVG renderer draws neither polylines nor rotated text: previews made with cairosvg.
- **Mermaid**: all eight README diagrams parsed with mermaid 10 / 11 and rendered with mermaid-cli + Firefox.  The gantt had `dateFormat X` (seconds) with `axisFormat %L` (ms), so every tick read `000`, and made-up times: replaced by the real Atari cycle of the firmware in ns (PHI1 0-280, sample at 200, decode, drive 360-559, write sample at 400, 6502 data setup from 459).  GitHub still showed "Unable to render rich display" for that gantt (its mermaid build rejects something mermaid 10 / 11 accept: `dateFormat x` or `tickInterval`), so the gantt is gone: `docs/timing.py` draws `timing_atari.svg`, a waveform timing diagram (PHI2, address / R/W, selects, ESP32 sample and decode, D0-D7 read and write with the sample points), embedded as an image in both READMEs.
- **Mermaid shown as text**: after that, GitHub showed the flowchart and sequence blocks as plain text.  Cause: a stray code fence (```` ``` ````) left right after the serial output example when its banner was replaced; it opened a new code block and shifted every fence after it.  Removed; a fence-balance check over every Markdown file of the project now passes.
- **Mermaid still not rendered**: with the fences fixed GitHub answered "Unable to render rich display" for the flowcharts too, although mermaid 10 / 11 parse and draw them locally.  The three README diagrams (block diagram, monitor flow, select protocol, IT and EN) are now PNGs drawn by mermaid-cli with Firefox at 2x, sources in `docs/diagrams/*.mmd`, `render.sh` to redraw them.  SVG output was tried first: with plain-text labels (`htmlLabels: false`, a root option in mermaid 11) the words are placed with the widths measured by the browser that drew them and overlap with other fonts; with HTML labels the SVG uses `foreignObject`, which GitHub does not show in an image.  The READMEs no longer contain mermaid blocks.

### Atari: Fixed Roles, Registers in $D1xx, RAM Window in $D5xx (supersedes the one-latch-at-a-time mode below)
- **Map**: `$D500-$D5FF` = 256 bytes of the current bank; `$D1FB-$D1FC` count (R), `$D1FD-$D1FE` 16-bit bank register (R/W), `$D1FF` PBI select latch (W, bit 7 = ID `$80`, never driven), `$D100-$D1FA` reserved.  Same layout as the C64 GeoRAM (registers in IO2, window in IO1).
- **Why**: the swapped mode (registers in `$D5xx`, RAM in `$D1xx`) only added trouble: `$D1FF` had to be protected (PBI interrupt status, read by the OS on every IRQ, driven by other PBI devices) or made RAM with `D1FF_RAM`, the window shrank to 255 bytes and the latch had two homes.  With fixed roles `$D1FF` is always the PBI latch, the window always 256 bytes, and the OS PBI scan (`$80` then `$00` to `$D1FF` at power-up and RESET) simply leaves the device deselected.  `D1FF_RAM` and the `$D5FF` latch are gone.  Drawback unchanged: cartridges that bank-switch through CCTL cannot be used while the device is selected.
- **Bus loop**: `DevState` is 12 bytes (RAM pointer of the current bank, NULL while deselected or beyond the count; count / bank registers; latch).  PHI1 assembly: `~CCTL` → RAM pointer → byte; `~D1XX` → latch → `$FB-$FE` registers.  22 instructions from the `GPIO.in` load to the `GPIO.out` store for a RAM read (was 25 / 27).  Log: `PBI_SELECT ... SELECTED`, `RAM bank $bbbb +$oo` for `$D5xx`.
- **Test program** simplified: COUNT, BANK REG, PROTECT, MARCH (all 256 offsets), LIMIT, KEEP (pattern kept across a deselect); EXCL, SHARED and D1FF are gone.  Simulated with 300 banks: ALL TESTS PASSED in 2.6 M instructions (about 20 s on the Atari for 220 KB); faults: bank register ignored, bank high byte ignored, latch ignored, latch on any bit, D3 stuck, A6 not decoded, writes while deselected, banks beyond the count wrapping, count writable, offset `$FF` not stored, RAM lost on deselect: all eleven detected (count writable now runs to the end: the test keeps its own copy of the count).
- **Docs**: firmware README, HARDWARE, PIN_MAPPING (md, tex, pdf), board PIN-MAPPING and READMEs (IT / EN: map table, banner, log example, flowchart, sequence diagram), `atari-test/README.md`.  The wiring infographics are unchanged (same pins).

### Wiring Infographics, Connector Pinouts Checked Online
- **Fritzing tried first**: Fritzing 1.0 in batch mode (`xvfb-run fritzing -svg`) does not build the generic headers (`module id generic_female_pin_header_N_100mil not found in database`), and its library has no 32-pin header, C64 expansion port or Atari connectors (the fixed generic headers stop at 20 pins and are obsolete).
- **`docs/wiring.py`** instead: writes `wiring_atari.svg` (J3 → 130XE cartridge slot + ECI) and `wiring_c64.svg` (J3 → C64 expansion port).  J3 pins from the netlist (5 V side); one coloured arrow per used J3 pin to its destination (`CART 10 D0`, `ECI 3 ~D1XX`, `C64 7 ~IO1`), the computer connector drawn with the J3 pin number in each used pin; colours per group (data, address, PHI2 / R/W, selects, GND, +5V); J3 26 (EN) and 27-28 (+3V3) marked "do not connect".  A first version with curved wires was unreadable (22 crossings): replaced by arrows.  Linked from the firmware and board READMEs.
- **Pinouts checked online** (atarimania.com Atari 8-bit FAQ, computersmuseum.com 130XE, c64-wiki.com, retroisle.com, pinouts.ru): cartridge slot and C64 port were right; **ECI lettered row was wrong** (A reserved, B ~IRQ, C ~HALT, D-F A13-A15, H GND; GND had been put on F), fixed.  Cross-checked with the RetroBitLab KiCad library (`~/Progetti/RetroBitLab-Library`): symbols `ATARI-CARTRIDGE`, `ECIBUS` and `C64-EXPANSION-PORT` match the infographics pin for pin (ECI A = RSRVD, H = GND); footprints `ATARI-LEFT-CARTRIDGE` and `ATARI-XE-ECI-SLOT` put pins 1-15 / 1-7 on the back and A-S / A-H on the front, with 1 opposite A, as drawn.  **The 800XL PBI carries no ~D1XX**: on the 800XL $D1xx has to be decoded from A8-A15 with external logic, not on this board (the README said it came from the PBI: fixed).
- **C64 test**: the presence check prints the value read back (`ERR B=00 P=00 O=01 E=A5 G=AD` for D3 stuck at 1, `G=00` with no unit), so a stuck data line is no longer reported as a missing GeoRAM only.

### C64 GeoRAM Test Program
- **`c64-test/georam_test.s`** (ca65, `cl65 -t c64 -C c64-asm.cfg -u __EXEHDR__`, BASIC stub `SYS 2061`, output through CHROUT): presence check (`$DE01` keeps `$A5` / `$5A`: the C64 open bus can read `$00`, which a size-only check would take for 256 blocks), SIZE (blocks 255..0 marked with their number, first block not reading itself = size; `EXPECT_BLOCKS` = 16), ALIAS (block = size is block 0), PAGE REG (only bits 0-5 count), MARCH over every page of every block with all 256 offsets, then the complement.  Registers are write-only: the program keeps copies.  About 30 s for 256 KB on the C64; it overwrites the whole GeoRAM.
- **Strings in lower case**: with `-t c64` ca65 maps `'A'` to PETSCII `$C1`, a graphic character in the default upper case / graphics set; lower case source maps to `$41-$5A`, shown as upper case.
- **Host simulation** (`sim_georam.py`, py65, CHROUT trapped, start address from the BASIC stub): correct model ALL TESTS PASSED; faults: page / block register ignored, page register on 8 bits, no aliasing beyond the size, D3 stuck, A6 not decoded, offset `$FF` not stored, a 128 KB unit, two blocks sharing the RAM, no GeoRAM: all detected (SIZE for register / mask / size faults, ALIAS for no aliasing, PAGE REG for the page register, MARCH for data and address lines; no GeoRAM and D3 stuck stop at the presence check with `NO GEORAM`, since `$A5` reads back as `$AD`).
- **Atari**: no new program; `atari-test/banked_ram_test.xex` already covers the GeoRAM-style mode of the Atari build.

### C64 GeoRAM Checked Against Vera-Module-RBL-C64
- **Same behaviour**: 256 KB, `$DE00-$DEFF` window, `$DFFE` page (`& $3F`), `$DFFF` block (`& $0F`, block 16 aliases block 0), write-only registers not driven on reads, decoded only at `$DFFE` / `$DFFF` (no `$DF80-$DFFD` mirrors), always enabled, zeroed at power-up and kept across a C64 reset, write sampled 300 ns after PHI2 rises.
- **Different storage**: RBL-C64 runs on an ESP32-S3 with a static 256 KB array in DRAM; the ESP32 classic cannot hold that much `.bss`, hence DRAM + IRAM with 32-bit accesses here.
- **Adopted their bus scheme**: instead of sampling at a fixed 200 ns after PHI2 rises, the loop polls /IO1 / /IO2 for at most `IO_QUALIFY_NS` (200 ns) and samples A0-A7 and drives the read byte as soon as one is LOW: the byte is on the bus earlier, with more margin before t_DSU.  On the ESP32 classic each poll is an APB read of `GPIO.in` (RBL-C64 uses the S3 dedicated GPIO, one sample every 5 instructions).

### C64 GeoRAM: 256 KB with DRAM + IRAM
- **Why 256 KB**: the official geoRAM was 512 KB only; later clones and VICE use powers of two (64 KB - 4 MB).  A 192 KB GeoRAM never existed and would break size detection, which writes a marker in block 0 and looks for where it aliases.  13 blocks (DRAM only) has the same problem, so the size is now a power of two (`GEORAM_KB`, default 256) and the block register is masked to it, as on the real hardware.
- **Memory**: DRAM (~210 KB) is not enough for 256 KB.  The rest comes from the free IRAM (`MALLOC_CAP_EXEC`; `_iram_end` = 0x4008E348, ~71 KB up to 0x400A0000, 2 KB left to the system).  IRAM takes only 32-bit accesses, so the C64 loop reads with `l32i` + shift and writes with load / mask / `s32i` (DRAM pages too: one code path).  The load is inline assembly, so GCC cannot narrow it to `l8ui`; checked in the disassembly.  Banks are zeroed with 32-bit stores (`heap_caps_calloc` could use byte stores).  At 1 MHz the extra instructions cost nothing.
- **Fallback**: if DRAM + IRAM hold less than `GEORAM_KB`, the largest power of two that fits (128 KB) is used; the banner prints the size, the DRAM / IRAM split and the free memory.  The Atari build is unchanged (DRAM only, 8-bit loads in the PHI1 assembly).

### Commodore 64 Build: GeoRAM Compatible (env `c64`)
- **Map**: `-D BUS_C64=1`.  /IO1 `$DE00-$DEFF` = 256-byte RAM window, /IO2 `$DFFE` = page in the 16 KB block (bits 0-5), `$DFFF` = block; write-only registers, no latch, always enabled, the rest of IO2 not decoded.  Chosen over the Atari map so that GEOS and existing GeoRAM / NeoRAM software can run.
- **Wiring**: /IO1 → J3 pin 12 (GPIO23, `~CCTL` input), /IO2 → J3 pin 11 (GPIO22, `~D1XX` input): the crossed wiring would also give a 256-byte IO1 window in the Atari map (the `$D1FF` protection lands on IO2).
- **Size**: first version: only whole 16 KB blocks, about 13 (~210 KB), blocks beyond not driven; superseded by the 256 KB version above.
- **Bus loop**: separate `MonitorTask` in plain C.  The VIC-II owns PHI1 (AEC low), so everything happens with PHI2 high: decode `BUS_ADDR_SETTLE_NS` = 200 ns after the rise, read byte driven until the fall (6510 needs it 100 ns before the fall, ~490 ns after the rise), write sampled `BUS_WR_SAMPLE_NS` = 300 ns after the rise (t_MDS <= 200 ns).  VIC-II DMA cycles do not assert /IO1 / /IO2.  Not measured yet.
- **Log**: `RAM blk $bb pg $pp +$oo`, `GEORAM_PAGE`, `GEORAM_BLOCK`, `UNUSED`; page shown as `$DE` / `$DF`.

### GeoRAM-Style Mode: One Latch at a Time (supersedes the two independent windows below)
The device becomes a GeoRAM-like expansion: one page holds the registers, the other one is a 256-byte window on the RAM (on a C64 the same two select pins would be IO1 / IO2).
- **Latch rule**: deselected, `$D1FF` and `$D5FF` are both latches and nothing else is decoded.  `$80` to `$D1FF` → registers `$D1xx`, RAM `$D500-$D5FF`; `$80` to `$D5FF` → registers `$D5xx`, RAM `$D100-$D1FE`.  Bit 7 = 0 to the latch of the register page deselects.  The latch offset of the RAM page is RAM: to move the registers, deselect first.
- **Register page**: `$xxFB-$xxFC` count (R), `$xxFD-$xxFE` bank (R/W, 16 bit), `$xxFF` latch; `$xx00-$xxFA` reserved.  One RAM and one bank register for both modes.
- **`$D1FF` protected by default**: with the RAM in `$D1xx`, `$D1FF` is never driven and writes there are ignored (255-byte window).  Why: `$D1FF` is the PBI interrupt / select register; the XL/XE OS reads it on every IRQ (`PDVI AND PDIMSK`, harmless with PDIMSK = 0), writes `$01`..`$80`, `$00` there in the PBI scan at power-up and RESET, and other PBI devices (1090, Black Box, U1MB PBI) drive it on reads: answering with RAM would be bus contention.  `-D D1FF_RAM=1` makes it RAM (256 bytes), only without other PBI devices; then the scan on RESET overwrites byte `$FF` of the current bank.
- **RESET**: the ESP32 does not see the Atari RESET; its state is kept.  The PBI scan writes `$80` then `$00` to `$D1FF`: deselected afterwards, unless the registers are in `$D5xx`.
- **Banks**: 256 bytes each (the old 128-byte window split is gone), one pool from the heap, count capped at `$FFFF`; estimated 200-230 KB in total, to be confirmed by the boot banner.
- **Bus loop**: `DevState` (24 bytes): RAM pointer per page (set only for the RAM page, NULL beyond the count), RAM page size per page (256, or 255 for `$D1xx`), count / bank registers, register page.  PHI1 assembly: RAM pointer → bound check against the page size → byte; otherwise registers if this is the register page.  25 instructions from the `GPIO.in` load to the `GPIO.out` store for a RAM read in `$D1xx` (27 in `$D5xx`), was 29.
- **Log**: `SELECTED: registers $D1xx, RAM $D5xx` on latch writes; `SEL` = device selected; `RAM bank $bbbb +$oo` on the RAM page, register names on the register page.
- **C64**: GeoRAM uses `$DExx` (IO1) as the window and `$DFFE` (page in block, 0-63) / `$DFFF` (16 KB block) as write-only registers, always enabled.  Our latch at `$xxFF` clashes with the GeoRAM block register at `$DFFF`: to run existing GeoRAM software the C64 build would need a mode without latch and with page / block registers: done in the `c64` build (above).
- **Test program** rewritten: per mode COUNT (same count in both modes), SHARED (second mode reads the first mode's pattern: one RAM), BANK REG, EXCL (`$80` to the latch offset of the RAM page does not move the registers), PROTECT, MARCH, LIMIT; then D1FF PROTECTED.  `D1FF_RAM` in the source must match the firmware.  Simulated with 300 banks: passes; faults: bank register ignored, bank high byte ignored, both latches active, D3 stuck, A6 not decoded, separate RAM per page, writes while deselected, banks beyond the count wrapping, count writable, `$D1FF` not protected: all ten detected.  With `count_writable` COUNT fails at once (`ERR D1 B=0000 O=FB E=2C G=D3`) and the run then stops at the 100 M instruction limit (corrupted count).  The first version ran SHARED after PROTECT, which had written bank 0: moved before any RAM write of the second mode.

### Documentation
- **C64 in PIN-MAPPING**: new section "Commodore 64 (env `c64`, GeoRAM 256 KB)" in `PIN_MAPPING.md` / `.tex` / `.pdf` and in the board `PIN-MAPPING.md` (IT): /IO1 → J3 pin 12 (`~CCTL` input), /IO2 → J3 pin 11 (`~D1XX` input), with the GeoRAM map.  To keep the PDF on one page: `titlesec` compact section spacing, `enumitem` `nosep` lists, `\intextsep` 4 pt, vertical margin 0.45 in.
- **C64 in the board README** (IT / EN): section "3. Commodore 64: GeoRAM 256 KB" (map, size and aliasing, DRAM + IRAM, bus cycle, full expansion port ↔ J3 wiring, banner and log example, 6502 usage, what is left to verify on the hardware); `[env:c64]` in the `platformio.ini` excerpt, `pio run -e nodemcu-32s` / `pio run -e c64`, programming rule "disconnected from the Atari or the C64"; GeoRAM among the main applications.
- **PIN-MAPPING, GeoRAM style**: notes rewritten for one latch at a time (`$80` to `$D1FF` / `$D5FF`, RAM in the other page, `$D1FF` protected) and `~D1XX` / `~CCTL` called pages instead of windows; PDF still one page (shorter notes).  The GPIO 12 row of the "Unused" table ran past the right margin: note shortened to "U1 / J3 pin 4 (former A8); strapping ignored (VDD_SDIO efuses)", no overfull boxes left.
- **C64 wiring** (firmware README, `c64` section): /IO1 → J3 pin 12, /IO2 → J3 pin 11, D0-D7, A0-A7, PHI2, R/W, GND, +5V against the expansion port pins (standard pinout, to be checked against the schematic).
- **PIN-MAPPING** (`PIN_MAPPING.md` / `.tex` / `.pdf` here, `PIN-MAPPING.md` in the board root): pins unchanged; the notes now list the window map (RAM `$xx00-$xx7F`, bank count `$xxFB-$xxFC` R, bank register `$xxFD-$xxFE` R/W, latch `$xxFF` W bit 7) and the reads that drive D0-D7 (RAM, bank count, bank register).  PDF kept on one page (vertical margin 0.5 in, less space under the title).
- **READMEs** (board root IT / EN, firmware, `atari-test`, `HARDWARE.md`): register map with the 16-bit bank count and bank register, heap banks with the per-window estimate and the boot banner (counts as placeholders until measured).


### 16-bit Bank Register, Banks from the Heap
- **Map**: `$xxFD-$xxFE` bank register (16 bit, little-endian: `$xxFD` keeps its meaning), `$xxFB-$xxFC` number of banks of the window (read-only, writes ignored), `$xx80-$xxFA` reserved.  A bank at or beyond the count selects no RAM: not driven, writes ignored.
- **Allocation**: the static 64 KB array is gone.  At boot `alloc_banks()` takes every byte-addressable DRAM heap block (`MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL`, largest first, at least 4 KB) until `BANK_HEAP_RESERVE` (24 KB) is left for the system, carves 128-byte banks and lists them in a bank → pointer table; the banks are split evenly between the two windows.  Estimate from the linker symbols: ~250 KB of heap minus the 24 KB reserve and the table (4 bytes per bank) → ~100-120 KB per window, to be confirmed on the board; the exact count, heap blocks used and heap left are printed in the banner (LogTask waits for the allocation).  Static DRAM use: 39 KB.
- **Bus loop**: `WinState` grows to 12 bytes (RAM pointer of the current bank, latch, count / bank registers at +8).  PHI1 assembly: RAM pointer NULL → not driven (+1 instruction); `$FB-$FE` read through one index into the registers.  29 instructions from the `GPIO.in` load to the `GPIO.out` store for a RAM read.  A bank register write recomputes the pointer (`ws_map()`, inlined).
- **Test program**: COUNT and LIMIT tests added, 16-bit BANK REG (low byte $00-$FF with high byte $00 / $55 / $AA / $FF), the MARCH covers every bank with the bank number in bytes 0-1 of each bank.  Simulated with 300 banks per window: passes, and ten injected faults (now also bank high byte ignored, banks beyond the count wrapping onto existing ones, bank count writable) are all detected.  With `count_writable` the COUNT test fails at once (`ERR D1 B=0000 O=FB E=2C G=D3`); the run then stops at the simulator's 100 M instruction limit, because the corrupted count ($FED3) makes the later passes walk ~65000 banks.  Run time on the Atari: a few seconds per 64 KB.

### Atari Test Program, DRAM_ATTR on Large Arrays
- **`atari-test/banked_ram_test.s`** (ca65, `cl65 -t atari -C atari-asm.cfg`, 825-byte XEX at $2E00): per window BANK REG (write / read back $00-$FF), PROTECT (deselect with $7F, writes ignored, reselect with $81), MARCH (all 256 banks with `offset EOR bank EOR key`, then the complement); then D1 KEEP (pattern still there after the $D5xx tests).  Errors counted on 24 bits, first four per test printed as `ERR D1 B=bb O=oo E=ee G=gg`.  `TEST_CCTL = 0` skips $D5xx (cartridges that bank-switch through CCTL).
- **Host simulation** (`sim_banked_ram.py`, py65): the XEX runs against a model of the device with CIO trapped.  Correct model: ALL TESTS PASSED in 1.67 M instructions (~1 s on the Atari).  Seven injected faults (bank register ignored, latch ignored, D3 stuck, A6 not decoded, windows sharing RAM, banks aliased on 7 bits, writes while deselected) are all detected.  The first version printed a blank line between errors, used the 8 detailed lines up in the first failing test and wrapped its 16-bit error count past 65535: all fixed.
- **`DRAM_ATTR` removed from the large arrays**: it placed the zeroed `bank_ram` (64 KB) and `log_ring` (16 KB) in `.data`, i.e. in the flash image and in the boot-time copy.  Plain globals are already in DRAM on the ESP32: firmware.bin 351 KB → 268 KB.
- **RAM available for the 6502** (from the linker symbols, to be measured at runtime): 64 KB used now; ~190 KB more of byte-addressable DRAM heap after boot (BT memory released, task stacks subtracted), in several blocks (largest ~112 KB) → ~250 KB in total with a per-bank pointer table and a 16-bit bank register; ~70 KB of IRAM on top, 32-bit access only.  No PSRAM on the WROOM-32.

### Banked RAM in the $D1xx / $D5xx Windows (supersedes the register file below)
The ROM-less peripheral becomes a small banked RAM expansion.  Each window ($D100-$D1FF, $D500-$D5FF) is independent, with its own latch, bank register and RAM:
- **Map**: `$xx00-$xx7F` RAM window (128 bytes of the current bank), `$xxFD` bank register (R/W, 256 banks), `$xxFF` select latch (W, bit 7 = device ID $80, never driven on reads), `$xx80-$xxFC` and `$xxFE` reserved (never driven, writes ignored).  RAM and bank register respond only while the window is selected.
- **Why 128 bytes** (instead of 253 = $00-$FC): on a 6502 the whole window is one `LDX #$7F` ... `DEX` / `BPL` loop, and a linear address splits into bank / offset with shifts (address >> 7, address & $7F) instead of a division by 253.  The upper half stays free for future control registers.
- **Memory**: 256 x 128 bytes = 32 KB per window, separate for the two windows: 64 KB of `.bss` in DRAM (static DRAM use 104 KB of 320 KB).  Power-up: bank 0, RAM $00; not kept across an ESP32 reset.
- **Bus loop**: the per-byte `GPIO.out` words of the old register file (4 bytes per byte) do not scale to 64 KB.  The PHI1 decode is now inline assembly: window state in a fixed-layout struct (`WinState`: RAM pointer of the current bank, latch, bank), byte-to-pins conversion through a 256-entry `drive_lut`.  28 instructions from the `GPIO.in` load to the `GPIO.out` store for a RAM read (the same logic in C was ~50).  A bank register write only updates the RAM pointer.
- **Log**: offsets shown as `RAM bank $bb +$oo`, `BANK_SELECT`, `PBI_SELECT` / `CCTL_SELECT`, `RESERVED`; the VERA register names and the DCSEL tracking are gone.  A block copy through the window fills the 1024-entry ring quickly: dropped events are counted.
- **Timing estimate**: settle 200 ns + ~150 ns of GPIO loads and decode = byte on the bus ~70-100 ns after PHI2 rises, ~80 ns before the 6502 read setup limit (~179 ns after the rise).  To be measured.

### ROM-less PBI / CCTL Peripheral (supersedes the ROM and MPD / EXTSEL parts below)
On this board MPD (GPIO0) and EXTSEL (GPIO3) never reached the Atari: GPIO0 is not on the 30-pin DevKit headers, GPIO3 only goes to J4 and the USB bridge TX.  Only one TXS0108E channel is free (U4 channel 8), and without MPD the PBI ROM at $D800 would collide with the internal Math Pack.  Decision: drop the ROM, keep the device in the I/O windows only.
- **Signals**: D0-D7, A0-A7, PHI2, R/W, `~D1XX` (GPIO22) and `~CCTL` (GPIO23, the former ROMSEL channel) = 20 inputs, no output pins.  `~D1XX` / `~CCTL` come from the Atari MMU / PLA decode; `~D1XX` is not PHI2-qualified.  Freed: GPIO12 (A8, still wired), GPIO25 (A9), GPIO26 (A10).
- **Device**: two windows, $D100-$D1FF and $D500-$D5FF, each with its own select latch at $D1FF / $D5FF (bit 7 = device ID $80: each device latches only its own bit) and its own 256-byte register file, written only by the Atari (power-up $00).  Selected: writes to $xx00-$xxFE are stored, reads are answered with the stored byte.  Not selected: never drives the bus.  $D1FF / $D5FF are never driven on reads ($D1FF is the PBI interrupt status).  The OS PBI scan finds no ROM signature and skips the device; programs select it by writing $80 to $D1FF.
- **Bus loop**: decode during PHI1, as on RBL-XE: A0-A7, R/W and the selects are sampled `BUS_ADDR_SETTLE_NS` (200 ns) after PHI2 falls, the byte is written to `GPIO.out` with the outputs still off, and the PHI2 rising edge only needs `GPIO.enable_w1ts` (`beqz` + `s32i` after the sample that sees the edge).  A select seen only on the rising edge (PHI2-qualified decode, e.g. if `~CCTL` turns out to be) is taken from that sample and driven late.  Writes sampled `BUS_WR_SAMPLE_NS` after PHI2 rises; the settle time counts from the earlier of the detected and the predicted PHI2 falling edge (`BUS_PHI2_HIGH_MAX_NS`).  The log event of a cycle is pushed during the next PHI1.
- **Log**: every access to both windows, selected or not, tagged `SEL` / `SEL DRV` / `-`; latch changes as `PBI_SELECT` / `CCTL_SELECT`.  VERA register names kept for offsets $00-$1A, DCSEL tracked per window.
- **Removed**: `6502/` (ROM sources, Makefile, linker script, font), `pre_build.py`, `include/pbi-driver.h`, the cc65 dependency, `BUS_MODE` and the `nodemcu-32s-cctl` environment: one firmware serves both windows.
- **Bootstrap**: unchanged and simpler: no output pins at start-up, D0-D7 driven only by the bus loop.  GPIO12 is unused but still wired to U1 / J3 pin 4, so the VDD_SDIO efuse check stays.
- **To measure on hardware**: how long before PHI2 falls D0-D7 are stable on reads (6502 setup ~100 ns), and whether 200 ns of settle cover t_ADS + MMU / PLA + TXS0108E delays.

### Bus Loop Optimizations Ported from Vera-Module-RBL-XE
Commit `a1e09b9` ports the RBL-XE (ESP32-S3) bus loop optimizations, adapted to the ESP32 classic (Xtensa LX6: no dedicated GPIO, no `ee.*` instructions, scattered bus pins):
- **Core 1 belongs to the bus loop**: interrupts masked (`portDISABLE_INTERRUPTS()`), interrupt watchdog MWDT1 disabled (it checks CPU1 too and would reset the chip after 300 ms), no FreeRTOS call inside the loop.
- **Lock-free log ring** replaces the FreeRTOS queue: 1024 entries, single producer / single consumer, events dropped and counted when full. No `xQueueSend()` and no `esp_timer_get_time()` in the bus loop: the timestamp is the Core 1 CCOUNT extended to 64 bit (wrap every 17.9 s detected on each PHI2 rising edge), printed in µs since the bus loop started. If PHI2 stops for more than 17.9 s, absolute timestamps shift by multiples of 17.9 s; short intervals stay exact.
- **Critical path in inline assembly**: PHI2 wait, `GPIO.in` / `GPIO.in1` sampling, MPD and EXTSEL taken directly from the ROMSEL / SEL_N bits with `extui`, A0-A10 from two lookup tables (`addr_lut_hi[256]` for GPIO 32-39, `addr_lut_lo[128]` for GPIO 21-27, plus A8 on GPIO 12), D0-D7 from `rom_out[2048]` (precomputed `GPIO.out` words). One `GPIO.out` store carries D0-D7 + MPD + EXTSEL, then `GPIO.enable_w1ts`; no `memw`. About 21 instructions from the detected PHI2 edge to the bus drive, against ~45 for the same logic in C and the earlier 11 single-bit extractions plus a `memw` on every volatile access.
- **Write data sampled at a fixed time**: `BUS_WR_SAMPLE_NS` (default 120 ns, `-D` tunable) after the detected PHI2 rising edge, via CCOUNT. Before, $D1FF and register writes were sampled right after the decode, probably before t_MDS. VERA read data is the last sample taken before PHI2 falls.
- **Not ported**: RBL-XE decodes during PHI1, before PHI2 rises. This board has no address decoder (no 74HC138 in the netlist, unlike the 2026-05-06 entry): ROMSEL and SEL_N arrive on J3 pins 12 and 11 through U3 (TXS0108E), from the circuit connected to J3. Whether they are PHI2-qualified depends on that circuit, so sampling stays on the PHI2 rising edge.

### Bug Fixes Found During the Port
- **ROM in IRAM**: `pbi_rom`, `drive_lut` and the generated `pbi_driver[]` were `IRAM_ATTR` (0x4008xxxx). ESP32 IRAM only supports 32-bit accesses: the 8-bit loads of the bus loop raise LoadStoreError. All hot-loop data is now `DRAM_ATTR`; `6502/Makefile` generates `const uint8_t pbi_driver[]` (flash rodata, copied once into `pbi_rom[]` at boot) and regenerates the header when the Makefile changes.
- **Log never printed**: in Arduino `loop()` runs on Core 1 (`CONFIG_ARDUINO_RUNNING_CORE 1`), the same core as `MonitorTask`, which has the highest priority and never yields: `loop()` starved. The log printer is now `LogTask`, pinned to Core 0.

### PBI Latch Rule for ROMSEL / MPD
Like every PBI device, the board answers in the ROMSEL window ($D800-$DFFF) only when its latch is active: MPD is asserted and the ROM is driven only with the latch on; with the latch off MPD and EXTSEL stay HIGH and the Atari Math Pack stays visible. The same rule applies in CCTL mode, with the latch at $D5FF instead of $D1FF. One extra `bnez` in the critical path. The README's CCTL description ("latch always active, MPD / EXTSEL unused") was wrong and has been corrected.

### Serial Log: Single Writer, One Write per Line
Commit `86cd2a3`:
- **UART TX path** (arduino-esp32 2.0.14): 128-byte hardware FIFO, no TX ring buffer (`setTxBufferSize(0)`), so `Serial.write()` blocks `LogTask` until the last byte is in the FIFO. Only Core 0 blocks; the bus loop keeps filling the ring.
- **Ordering**: `uartWriteBuf()` holds the UART mutex for the whole call, so one `write()` is never split; `Print::println()` is two writes and `Print::printf()` mallocs every line longer than 64 bytes. Every line is now formatted into a 128-byte local buffer (`log_line()`) and sent with a single `Serial.write()`.
- **Single writer**: `setup()` no longer prints. `LogTask` prints the banner, the bus events, the dropped-event count and the bus loop start failure (flag set by `setup()`) in one sequence. ESP-IDF system logs (level ERROR, panic only) bypass the Arduino mutex.
- **Baud rate**: `LOG_BAUD`, default 115200. A higher value (e.g. 921600) only drops fewer events during VERA bursts (~160 lines/s at 115200, ~1280 at 921600) and needs a matching `monitor_speed`; it does not affect ordering.


### Bootstrap: VDD_SDIO Efuse, Monitor RTS/DTR, Output Glitch
Programming rule: the ESP32 module is always programmed on its own, disconnected from the Atari and connected only to the PC (serial bootloader).  With it, GPIO2 (PHI2) and GPIO0 no longer matter for the download mode.  Strapping analysis for a reset while on the Atari bus (details in HARDWARE.md §3):
- **GPIO12 = A8 (MTDI)** is sampled at every chip reset and selects the flash voltage: HIGH = 1.8 V, the WROOM-32 flash does not boot.  A8 is random at reset and the TXS0108E pull-ups hold it HIGH on an idle bus.  **`efuse_vdd_sdio.py`** (PlatformIO `post:` script, runs before every upload) reads `XPD_SDIO_FORCE/REG/TIEH`: all set → nothing to do; all blank → `espefuse.py set_flash_voltage 3.3V` (irreversible) and read back; any other state → upload stopped, nothing burned.  `custom_efuse_vdd_sdio = 3.3V` in `platformio.ini` enables it.  espefuse needs `ecdsa`, `bitstring`, `reedsolo`, `cryptography`, missing from the PlatformIO Python: a private virtualenv is created in `.pio/espefuse-venv` on first use.  Note: espefuse reports `XPD_SDIO_TIEH` as `"1.8V"` / `"3.3V"`, not as a boolean.  Tested with virtual efuses (`--virt`: blank, already set, partial) and with an unreachable port (upload stopped before writing the flash); not yet on a real module.
- **Serial monitor**: `monitor_rts = 0` / `monitor_dtr = 0`, so opening the monitor does not reset the ESP32 through the NodeMCU auto-reset circuit while it sits on the Atari bus.
- **Output glitch**: `setup()` wrote MPD / EXTSEL HIGH after `pinMode(OUTPUT)`, driving them LOW for a few µs (the `GPIO.out` bits are 0 after reset).  The level is now written first; `gpio_set_level()` works while the pin is still an input.
- **Open**: MPD (GPIO0) and EXTSEL (GPIO3) are not routed to J3 on this board (GPIO0 is not on the 30-pin headers, GPIO3 only reaches J4 and the USB bridge TX), unlike the board README, which assigns them to the third TXS0108E.
---

## 2026-06-12: Firmware Simplification, Full 2 KB Decode, VERA Logging

### Firmware Simplification — Drop RESET / VCS / RAM Emulation
Commit `b58b0f6` removed ~165 lines of obsolete logic:
- **Dropped**: Atari RESET output, VCS output pin, RAMSEL input, the entire 512-byte emulated RAM at $D600–$D7FF.
- **Kept**: ROMSEL handler (asserts MPD, drives `pbi_rom[A0-A7]` on reads) and EXTSEL handler ($D1XX + latch).
- **Motivation**: the RAM emulation was never required by the VERA driver; removing it freed three GPIO pins and reduced the interrupt-path footprint significantly.

### Full 2 KB ROM Decode — A8, A9, A10 Wired
Commit `75b3f3e`: GPIOs 12 / 25 / 26 were freed from RESET / VCS / RAMSEL and wired to **A8 / A9 / A10**.
- `pbi_rom[]` expanded from 256 bytes to **2048 bytes** (`IRAM_ATTR`).
- `decode_addr()` now returns an **11-bit** address (A0-A10), removing the previous $D6xx/$D7xx aliasing.
- Pin definitions updated: `PIN_A8 = 12  // formerly RESET`, `PIN_A9 = 25  // formerly VCS`, `PIN_A10 = 26  // formerly RAMSEL`.
- All pin-mapping documents (PIN_MAPPING.md, PIN_MAPPING.tex, PIN_MAPPING.pdf, PIN-MAPPING.md, READMEs) regenerated to match.

### NodeMCU DevKit V1 — Sole Hardware Target
Commit `8395e40` removed the ESP32-PICO-D4 build environment from `platformio.ini`. The project now targets **NodeMCU-32S** exclusively (`nodemcu-32s` for PBI, `nodemcu-32s-cctl` for CCTL).

### FreeRTOS Log Queue
- Added a 64-entry `LogEvt` queue (Core 1 → Core 0) carrying microsecond-precision timestamps (`esp_timer_get_time()`).
- Two event types: `EVT_LATCH` (PBI latch state change) and `EVT_REG` (every $D100–$D1FE access including R/W direction and data byte).
- Core 0 `loop()` drains the queue and prints without ever blocking the bus handler.

### VERA Register Name Resolution
- Added `vera_reg_name(offset, dcsel)` lookup table in `loop()` (Core 0 only, not IRAM-resident).
- Covers all registers defined in `vera_common.inc`: `VERA_ADDR_L/M/H`, `VERA_DATA0/1`, `VERA_CTRL`, `VERA_IEN`, `VERA_ISR`; muxed DC/FX registers ($09–$0C, 7 DCSEL banks); Layer 1 registers ($14–$1A).
- DCSEL state tracked by shadowing writes to `VERA_CTRL` (offset $05, bits [2:1]) — no struct change needed.
- New serial format: `[    0.012390] [D103 - VERA_DATA0           ] W $FF`

### 6502 ROM Source Restructure
- Renamed `pbi-driver.s` → `6502/src/vera_pbi_handler.s`; extracted shared VERA symbols into `6502/src/vera_common.inc`.
- Added `pre_build.py` PlatformIO `pre:` hook — runs `make -C 6502/` before any C++ compilation.
- Makefile assembles with `ca65`, links at $D800 (2 KB) with `ld65`, converts to `include/pbi-driver.h` via `hexdump`.
- Two `platformio.ini` environments: `nodemcu-32s` (PBI, `BUS_MODE=0`) and `nodemcu-32s-cctl` (CCTL, `BUS_MODE=1`).

---

## 2026-05-08: Multi-device Bus Support
- **EXTSEL_N (EXSEL):** Switched to **Open-Drain (OD)** mode. This allows the signal to be shared with other PBI devices without hardware contention.
- **Terminology Fix:** Updated documentation to reflect that EXTSEL_N disables motherboard **memory** (not just ROM).
- **MPD & IRQ Analysis:** Confirmed IRQ is handled by the VERA FPGA and MPD is driven by the 74HCT138 hardware decoder. Clarified that sharing MPD would require hardware modifications (e.g., diodes) due to the push-pull nature of the '138.
- **Boot Integrity:** Maintained internal pull-ups and established High-Z state at boot for EXTSEL_N to ensure it doesn't interfere with the Atari startup until explicitly driven.

## 2026-05-06: High-Performance PBI Driver Implementation

### Overview
Successfully implemented a high-performance ESP32 firmware for the Atari Parallel Bus Interface (PBI). The final architecture prioritizes low latency and thread safety using a combination of external hardware decoding and FreeRTOS synchronization.

### Final Hardware Mapping
The system uses an external **74HC138** decoder for the `$D800-$DFFF` range to reduce ESP32 pin usage and latency.

#### 74HC138 Wiring (External Decoder)
| Pin 74HC138 | Signal / Connection | Function |
| :--- | :--- | :--- |
| 1 (A) | **A11** (Atari) | Address selector bit 0 |
| 2 (B) | **A12** (Atari) | Address selector bit 1 |
| 3 (C) | **A14** (Atari) | Address selector bit 2 |
| 4 (G2A) | **A13** (Atari) | Enable (Active LOW) - Must be 0 |
| 5 (G2B) | **GND** | Enable (Active LOW) - Always GND |
| 6 (G1) | **A15** (Atari) | Enable (Active HIGH) - Must be 1 |
| 7 (Y7) | **ROM_SEL / MPD** | **Output**: Active LOW for $D800-$DFFF |
| 8 (GND) | **GND** | Common Ground |
| 16 (VCC) | **+5V** (Atari) | Power Supply |

**Connection Notes:**
- **MPD:** Connect Pin 7 (Y7) directly to Atari Bus MPD line.
- **ROM_SEL:** Connect Pin 7 (Y7) to ESP32 **GPIO 4** (via 5V->3.3V Level Shifter).

#### ESP32 Pinout
| Signal | ESP32 GPIO | Direction | Description |
| :--- | :--- | :--- | :--- |
| **Data Bus (D0-D7)** | 18, 19, 21-23, 25-27 | Bidirectional | Direct register access for latency < 50ns |
| **Address (A0-A5)** | 32-36, 39 | Input | Lower 6 bits for ROM indexing |
| **Address (A6-A10)** | 16, 17, 14, 12, 13 | Input | Higher 5 bits for 2KB ROM indexing |
| **PHI2** | 2 | Input | 6502 Clock (Sync trigger) |
| **R/W** | 15 | Input | High = Read, Low = Write |
| **D1XX** | 5 | Input | Device Selection Page (Active Low) |
| **ROM_SEL** | 4 | Input | Active Low from 74HC138 ($D800-$DFFF) |
| **EXTSEL** | 0 | Output | External Select (Active Low) to Atari |
| **MPD** | - | Hardware | Managed directly by 74HC138 Y7 output |
| **Debug Serial TX** | 1 (TX) | Output | Standard Serial Debug enabled |
| **DEV_SEL / CS** | 3 | Output | Chip Select hardware ($D100-$D11F, A0-A4) |
| **Internal Regs** | - | Software | Registri ESP32 ($D120-$D1FE, A5-A7 coinvolti) |



### Key Architectural Decisions

1.  **Direct GPIO Register Access:**
    - Used `GPIO.in`, `GPIO.out_w1ts`, and `GPIO.out_w1tc` instead of `digitalRead/Write` or `gpio_config`.
    - Reduced I/O latency from microseconds to ~20ns, fitting within the 279ns Atari timing window.

2.  **Multicore Optimization (FreeRTOS):**
    - **Core 1 (MonitorTask):** Dedicated to the high-speed bus monitor loop. Uses a local state for `pbi_enabled` to avoid cache coherence overhead.
    - **Core 0 (System):** Handles Serial Debugging and FreeRTOS system tasks.
    - **Queue Communication:** Used `xQueueSend` and `xQueueReceive` for thread-safe state synchronization between cores. This avoids the "bruttissima" shared global variable approach.

3.  **Hardware Decoding ($D800-$DFFF):**
    - Offloading address decoding to a **74HC138** freed up 5 GPIOs and removed critical timing bottlenecks from the firmware.
    - `MPD` is now handled at hardware speed (~10ns), preventing bus contention before the ESP32 can even react.

4.  **Serial Debug Recovery:**
    - By moving `EXTSEL` and `MPD` away from GPIO 1 and 3, standard hardware serial debugging was restored, facilitating further development.

### Performance Verification
- **PHI2 sync:** Confirmed stable triggering on rising edge.
- **Data Injection:** 2KB ROM emulation serving `pbi_driver` array correctly within the PHI2 HIGH window.
- **Device Selection:** $D1FF write logic correctly enables/disables the PBI driver response.

## 2026-05-07: VERA Text Mode Boot Screen

### Overview
The 6502 PBI ROM now initializes the external VERA chip mapped at `$D100-$D11F` during driver startup. The boot sequence configures a text screen in a Commander X16-inspired style for Atari PBI/ECI usage.

### Video Initialization
- **VERA register model:** The driver now uses CX16-compatible VERA register naming, remapped to the Atari PBI window at `$D100`.
- **Screen mode:** Layer 1 text mode configured for **80x25** cells with **8x16** character geometry.
- **Display style:** Blue background, black border, white text.
- **Banner text:** The ROM writes:
  - `**** COMMANDER X16 VERA ****`
  - `PBI VIDEO INTERFACE`
  - `READY.`

### Font Handling
- The VERA bitstream does **not** provide a guaranteed ready-to-use system font after reset.
- For this reason, the ROM now loads a **minimal 8x16 font subset** into VERA VRAM at startup.
- The embedded glyph set is intentionally compact and currently covers only the characters required by the boot banner.

### ROM Size Constraint
- The 6502 PBI ROM linker configuration is now limited to **2KB** (`$0800` bytes).
- This matches the ESP32-side ROM fetch logic, which serves the driver with `offset & 0x7FF`.
- Current generated ROM size: **2048 bytes**.
