# Atari PBI ROM Emulator & VERA Bus Monitor

High-performance ESP32-based firmware that emulates the Atari 8-bit Parallel Bus Interface (PBI) ROM slot ($D800-$DFFF) and logs VERA video-card register accesses via a FreeRTOS queue.

## Features
- **Bus Modes:**
    - **PBI Mode (Default):** Full Atari PBI device emulation. Serves a 2 KB ROM at $D800-$DFFF (A0-A10, no aliasing). Asserts MPD on ROMSEL; asserts EXTSEL when the PBI latch is active. Latch enabled by writing $80 to $D1FF, disabled by writing $00.
    - **CCTL Mode:** Cartridge Control mode. VCS latch always active; MPD, EXTSEL, and latch control ($D1FF) are not used.
- **VERA Register Logging:** Every access to $D100-$D1FE is captured by Core 1 and forwarded via a 64-entry FreeRTOS queue to Core 0 for Serial output. Each event carries a microsecond-precision timestamp (`esp_timer_get_time()`).
- **Serial Output Format:**
    ```
    [    0.001234] [VCS ] Latch ENABLED  ($80 written to $D1FF)
    [    0.002345] [D103 - VERA_DATA0           ] R $FF
    [    0.002346] [D105 - VERA_CTRL            ] W $04
    [    0.002347] [D109 - VERA_FX_CTRL         ] W $20
    ```
  Registers $09-$0C are muxed by DCSEL (bits `[2:1]` of `VERA_CTRL`); the logger tracks the last write to `$D105` to resolve the correct name automatically.
- **Fast Response:** IRAM-resident `MonitorTask` on Core 1 with direct GPIO register access (`GPIO.in`, `GPIO.out_w1ts/c`) for < 50 ns bus latency.
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
| `include/pbi-driver.h` | Auto-generated C header: `IRAM_ATTR uint8_t pbi_driver[2048]` |

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
