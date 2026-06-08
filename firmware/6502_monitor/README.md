# Atari 6502 Bus Monitor & RAM Emulator

High-performance ESP32-based firmware for monitoring and emulating the Atari 8-bit bus (XL/XE Parallel Bus Interface).

## Features
- **Bus Modes:**
    - **PBI Mode (Default):** Full Atari PBI device emulation. Supports $D1XX page, ROMSEL ($D800-$DFFF), RAMSEL ($D600-$D7FF), MPD, and EXSEL signals.
    - **CCTL Mode:** Cartridge Control mode. Simplified operation with VCS (Virtual Chip Select) always active. Does not use MPD, ROMSEL, RAMSEL, or EXSEL signals.
- **RAM Emulation:** 512 bytes of internal IRAM used for $D600-$D7FF area in PBI mode.
- **Mirroring:** Due to pin constraints, A8 is not decoded; range $D6xx and $D7xx are mirrored (256 bytes).
- **Fast Response:** Optimized IRAM-resident tasks and LUTs for < 50ns bus latency.
- **Universal Support:** Native mapping for NodeMCU DevKit V1 and ESP32-PICO-D4 SiP.

## Hardware Setup
Refer to the following documents for detailed pinout:
- [General Pin Mapping](PIN_MAPPING.md) (Standard NodeMCU)
- [ESP32-PICO-D4 Mapping](PICO_D4_MAPPING.md) (Optimized SiP)
- [Hardware Architecture](HARDWARE.md)

## Development
This project uses **PlatformIO**. 
- Style: **Allman** (braces on new line).
- Column Limit: **80**.
- Tool: `clang-format`.

### Configuration
Edit `BUS_MODE` and `HARDWARE_TARGET` in `src/main.cpp` or use build flags:
- `BUS_MODE`: `BUS_MODE_PBI` (Default) or `BUS_MODE_CCTL`.
- `HARDWARE_TARGET`: `TARGET_NODEMCU` (Default) or `TARGET_PICO_D4`.
