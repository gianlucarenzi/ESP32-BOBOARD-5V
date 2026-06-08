# 6502 Monitor for ESP32

This project implements a high-speed bus monitor for a 6502 CPU (specifically for Atari 8-bit systems) using an ESP32 NodeMCU DevKit V1. It operates at 1.79 MHz and provides interface logic for both PBI (Parallel Bus Interface) and CCTL (Cartridge Control) modes.

## Key Features
- **High Performance:** Optimized with direct register access and IRAM-resident tasks to meet 1.79 MHz bus timings (~280ns window).
- **Dual Mode Support:**
  - **PBI Mode:** Monitors $D100-$D1FF, $D600-$D7FF (RAMSEL), and $D800-$DFFF (ROMSEL). Handles device activation (VCS) and memory disabling (EXTSEL/MPD).
  - **CCTL Mode:** Monitors $D500-$D5FF for cartridge port integration.
- **Safe Boot Design:** Pin mapping avoids critical ESP32 bootstrap pins (GPIO 0 and 12 for input) to ensure reliable startup.
- **Serial Debugging:** Real-time status updates via UART0 TX (115200 bps).

## Hardware Setup
All signals must pass through 3.3V $\leftrightarrow$ 5V level shifters (e.g., TXS0108E).

For a detailed pinout and wiring guide, see [PIN_MAPPING.md](6502_monitor/PIN_MAPPING.md) or the professional [LaTeX reference](6502_monitor/PIN_MAPPING.tex).

### Signal Summary (24 Signals)
- **Data Bus:** D0-D7 (GPIO 4, 5, 13, 14, 16, 17, 18, 19)
- **Address Bus:** A0-A7 (GPIO 34, 35, 36, 39, 32, 33, 21, 27)
- **Control In:** PHI2, R/W, SEL_N, ROMSEL, RAMSEL
- **Control Out:** EXTSEL (on RX0), VCS, MPD

## Installation
1. Install [PlatformIO](https://platformio.org/).
2. Open the `6502_monitor` directory.
3. Configure the desired mode in `src/main.cpp`:
   ```cpp
   #define BUS_MODE BUS_MODE_PBI // or BUS_MODE_CCTL
   ```
4. Build and upload:
   ```bash
   pio run -t upload
   ```
