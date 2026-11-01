# Atari PBI ROM Emulator & VERA Bus Monitor

High-performance ESP32-based firmware that emulates the Atari 8-bit Parallel Bus Interface (PBI) ROM slot ($D800-$DFFF) and logs VERA video-card register accesses via a lock-free ring buffer.

## Features
- **Bus Modes:**
    - **PBI Mode (Default):** Full Atari PBI device emulation. Serves a 2 KB ROM at $D800-$DFFF (A0-A10, no aliasing). Like every PBI device, it answers only when its latch is active: then it asserts MPD and serves the ROM on ROMSEL, and asserts EXTSEL on $D1XX. Latch enabled by writing $80 to $D1FF, disabled by writing $00.
    - **CCTL Mode:** Cartridge Control mode. Same rule as PBI, with the latch at $D5FF instead of $D1FF: write $80 to $D5FF to enable, any other value to disable. Only with the latch active it asserts MPD and serves the ROM on ROMSEL, and asserts EXTSEL on the CCTL window ($D5XX).
- **VERA Register Logging:** Every access to $D100-$D1FE is captured by Core 1 and forwarded via a 1024-entry lock-free ring buffer to `LogTask` on Core 0, which owns Serial. Each event carries a timestamp from the Core 1 cycle counter (CCOUNT extended to 64 bit), printed in microseconds since the bus loop started. Write data is sampled `BUS_WR_SAMPLE_NS` (default 120 ns) after PHI2 rises; VERA read data just before PHI2 falls. Events are dropped and counted when the ring is full.
- **Serial Output Format:**
    ```
    [    0.001234] [VCS ] Latch ENABLED  ($80 written to $D1FF)
    [    0.002345] [D103 - VERA_DATA0           ] R $FF
    [    0.002346] [D105 - VERA_CTRL            ] W $04
    [    0.002347] [D109 - VERA_FX_CTRL         ] W $20
    ```
  Registers $09-$0C are muxed by DCSEL (bits `[2:1]` of `VERA_CTRL`); the logger tracks the last write to `$D105` to resolve the correct name automatically.
- **Fast Response:** IRAM-resident `MonitorTask` owns Core 1 (interrupts masked, interrupt watchdog disabled, no FreeRTOS calls). The critical path from the PHI2 rising edge to the data bus drive is inline assembly: table-driven A0-A10 decode, precomputed `GPIO.out` words for the ROM, one `GPIO.out` store for D0-D7 + MPD + EXTSEL, no `memw`. All hot-loop data is in DRAM (ESP32 IRAM is not byte-addressable). Ported from Vera-Module-RBL-XE.
- **Hardware Target:** NodeMCU DevKit V1 (ESP32-WROOM).

## 6502 ROM Build Pipeline

The PBI ROM served at $D800-$DFFF is a real 2 KB 6502 binary assembled before the ESP32 build:

| Path | Description |
|---|---|
| `6502/src/vera_pbi_handler.s` | PBI ROM source: Atari header, INIT routine, VERA text-mode boot screen |
| `6502/src/vera_common.inc` | Shared VERA register definitions and screen-layout constants |
| `6502/pbi-driver.ld` | Linker script: places CODE/RODATA at $D800, size 2 KB |
| `6502/Makefile` | Assembles with `ca65`, links with `ld65`, emits `pbi-driver.h` via `hexdump` |
| `pre_build.py` | PlatformIO `pre:` hook — runs `make -C 6502/` before any C++ source is compiled |
| `include/pbi-driver.h` | Auto-generated C header: `const uint8_t pbi_driver[2048]` (flash, copied to DRAM at boot) |

**Toolchain requirement:** `cc65` suite (`ca65`, `ld65`) must be on `$PATH`.

Build flow:
```
PlatformIO build → pre_build.py → make -C 6502/ → ca65 → ld65 → hexdump → pbi-driver.h
                                                                                  ↓
                                                              ESP32 C++ build includes it
```

The ROM INIT routine:
1. Registers the device in `PDVMSK` and sets `CRITIC = 0`.
2. Probes for VERA hardware readiness (`WAIT_VERA`).
3. Configures VERA Layer 1 in 128×64-cell tilemap mode (80 visible columns, 1:1 VGA scale).
4. Loads a compact 27-glyph boot font into VRAM at $1F000.
5. Clears the screen and prints a version line (VERA firmware revision from `DC_HSCALE`/`DC_VSCALE`/`DC_BORDER`) and a host-type line (ATARI XL / ATARI XE detected via `PORTB` bank-switching probe).

## Hardware Setup
Refer to the following documents for detailed pinout:
- [General Pin Mapping](PIN_MAPPING.md) (NodeMCU DevKit V1)
- [Hardware Architecture](HARDWARE.md)

## Development
This project uses **PlatformIO**.
- Style: **Allman** (braces on new line).
- Column Limit: **80**.
- Tool: `clang-format`.

### Configuration
Two build environments are defined in `platformio.ini`:

| Environment | `BUS_MODE` | Description |
|---|---|---|
| `nodemcu-32s` | `BUS_MODE_PBI` (0) | Default — full PBI emulation |
| `nodemcu-32s-cctl` | `BUS_MODE_CCTL` (1) | Cartridge Control mode |

Build and flash:
```sh
pio run -e nodemcu-32s --target upload        # PBI mode
pio run -e nodemcu-32s-cctl --target upload   # CCTL mode
```
