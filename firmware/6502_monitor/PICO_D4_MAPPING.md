# 6502 Monitor - ESP32-PICO-D4 Physical Pin Mapping

This document provides the specific mapping for the **ESP32-PICO-D4** SiP (System-in-Package). Use these physical pin numbers for your schematic and PCB design.

## 1. Data Bus (GPIO Bank 0)
| 6502 Signal | GPIO (Code) | **Physical Pin (QFN48)** | Note |
| :--- | :--- | :--- | :--- |
| **D0** | GPIO 4 | **24** | I/O |
| **D1** | GPIO 5 | **29** | I/O |
| **D2** | GPIO 13 | **20** | I/O |
| **D3** | GPIO 14 | **17** | I/O |
| **D4** | GPIO 18 | **30** | I/O |
| **D5** | GPIO 19 | **31** | I/O |
| **D6** | GPIO 21 | **33** | I/O |
| **D7** | GPIO 22 | **36** | I/O |

## 2. Address Bus (Inputs)
A0-A3 and A6-A7 are strictly Input-Only on the ESP32.

| 6502 Signal | GPIO (Code) | **Physical Pin (QFN48)** | Type |
| :--- | :--- | :--- | :--- |
| **A0** | GPIO 34 | **5** | **Input Only** |
| **A1** | GPIO 35 | **6** | **Input Only** |
| **A2** | GPIO 36 | **2** | **Input Only** (VP) |
| **A3** | GPIO 39 | **3** | **Input Only** (VN) |
| **A4** | GPIO 32 | **12** | Input/Output |
| **A5** | GPIO 33 | **13** | Input/Output |
| **A6** | GPIO 37 | **4** | **Input Only** |
| **A7** | GPIO 38 | **7** | **Input Only** |

## 3. Control Signals
| 6502 Signal | GPIO (Code) | **Physical Pin (QFN48)** | Dir | Description |
| :--- | :--- | :--- | :--- | :--- |
| **PHI2** | GPIO 2 | **22** | IN | System Clock |
| **R/W** | GPIO 15 | **21** | IN | Read/Write |
| **SEL_N** | GPIO 23 | **37** | IN | D1XX_N or CCTL_N |
| **ROMSEL** | GPIO 25 | **10** | IN | $D800-$DFFF Range Select |
| **RAMSEL** | GPIO 26 | **11** | IN | $D600-$D7FF Range Select |
| **EXTSEL** | GPIO 3 | **34** | OUT | Disable Atari Memory (Active Low) |
| **VCS** | GPIO 27 | **16** | OUT | Device Select (Active Low) |
| **MPD** | GPIO 0 | **25** | OUT | Math Pack Disable (Active Low) |
| **RESET** | GPIO 12 | **18** | OUT | **Atari RESET Control** (Active Low) |

## 4. System & Debug
| Signal | GPIO (Code) | **Physical Pin (QFN48)** | Description |
| :--- | :--- | :--- | :--- |
| **Debug TX** | GPIO 1 | **35** | Serial Output (TX0) |
| **Debug RX** | GPIO 3 | **34** | Serial Input (RX0) - Shared with EXTSEL |
| **GND** | GND | **EPAD (49)** | **Exposed Thermal Pad** |

---

## ⚠️ ESP32-PICO-D4 Design Warnings

1. **Internal Flash Pins (FORBIDDEN):** Do NOT connect physical pins **19, 23, 15, 14, 28, and 1** to anything. They are wired internally to the SiP flash memory.
2. **Strapping Pins:**
   - **Pin 18 (GPIO 12/MTDI):** Must be LOW at boot for 3.3V Flash voltage selection.
   - **Pin 25 (GPIO 0/BOOT):** Must be HIGH for normal execution (Atari pull-up handles this).
3. **Thermal Management:** The **Exposed Pad (Pin 49)** must be soldered to a solid Ground plane with multiple vias to dissipate heat from the 240 MHz CPU.
4. **Decoupling:** Use 0.1µF and 10µF capacitors as close as possible to the VDD pins.
