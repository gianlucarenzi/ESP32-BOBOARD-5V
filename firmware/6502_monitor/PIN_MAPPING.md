# 6502 Monitor - ESP32 Pin Mapping Reference

This document details the final pin assignment for the ESP32 NodeMCU DevKit V1 interfacing with a 6502 (Atari) system at 1.79 MHz.

## 1. Data Bus (Bidirectional)
All data bus pins are located in **GPIO Bank 0** for optimized 8-bit parallel access.

| 6502 Signal | ESP32 GPIO | Board Label | Level Shifter |
| :--- | :--- | :--- | :--- |
| **D0** | GPIO 4 | D4 | Yes (5V <-> 3.3V) |
| **D1** | GPIO 5 | D5 | Yes |
| **D2** | GPIO 13 | D13 | Yes |
| **D3** | GPIO 14 | D14 | Yes |
| **D4** | GPIO 16 | RX2 | Yes |
| **D5** | GPIO 17 | TX2 | Yes |
| **D6** | GPIO 18 | D18 | Yes |
| **D7** | GPIO 19 | D19 | Yes |

## 2. Address Bus (Inputs)
A0-A3 are connected to "Input Only" pins for maximum safety.

| 6502 Signal | ESP32 GPIO | Board Label | Caracteristics |
| :--- | :--- | :--- | :--- |
| **A0** | GPIO 34 | D34 | **Input Only** |
| **A1** | GPIO 35 | D35 | **Input Only** |
| **A2** | GPIO 36 | VP | **Input Only** |
| **A3** | GPIO 39 | VN | **Input Only** |
| **A4** | GPIO 32 | D32 | Input/Output |
| **A5** | GPIO 33 | D33 | Input/Output |
| **A6** | GPIO 21 | D21 | Input/Output |
| **A7** | GPIO 27 | D27 | Input/Output |

## 3. Control Signals (Inputs)
| 6502 Signal | ESP32 GPIO | Board Label | Description |
| :--- | :--- | :--- | :--- |
| **PHI2** | GPIO 2 | D2 | System Clock (Synchronization) |
| **R/W** | GPIO 15 | D15 | Read (H) / Write (L) |
| **SEL_N** | GPIO 22 | D22 | D1XX_N (PBI) or CCTL_N (Cart) |
| **ROMSEL** | GPIO 23 | D23 | $D800-$DFFF Range Select (PBI) |
| **RAMSEL** | GPIO 26 | D26 | $D600-$D7FF Range Select (PBI) |

## 4. Control Signals (Outputs)
| 6502 Signal | ESP32 GPIO | Board Label | Description |
| :--- | :--- | :--- | :--- |
| **EXTSEL** | GPIO 3 | **RX0** | Disable Atari Memory (Active Low) |
| **VCS** | GPIO 25 | D25 | Device Select (Active Low) |
| **MPD** | GPIO 12 | **D12** | Math Pack Disable (Active Low) |

## 5. System & Debug
| Signal | ESP32 GPIO | Board Label | Description |
| :--- | :--- | :--- | :--- |
| **Debug TX** | GPIO 1 | TX0 | Serial output (115200 bps) |
| **GND** | GND | GND | Common Ground with Atari |

---

## ⚠️ Critical Hardware Notes

1. **GPIO 12 (MPD) Safety:** This is a bootstrap pin. It **MUST be LOW** during ESP32 power-on/reset. If the Atari system pulls this pin HIGH at boot, the ESP32 will fail to start. Ensure a pull-down resistor or that the level shifter is inactive during ESP32 reset.
2. **RX0 (GPIO 3) Usage:** UART0 Receive is disabled in software. This pin is strictly used as a digital output for **EXTSEL**. Do not connect a serial source (PC RX) to this pin if the Atari is connected.
3. **Level Shifters:** All signals must pass through 3.3V <-> 5V level shifters (like TXS0108E).
4. **PBI vs CCTL Mode:** Toggle the mode in `main.cpp` via `#define BUS_MODE`.
