# 6502 Monitor - ESP32 Pin Mapping Reference (Atari 130XE Validated)

## 1. Data Bus (Bidirectional)
| 6502 Signal | ESP32 GPIO | Board Label |
| :--- | :--- | :--- |
| **D0** | GPIO 4 | D4 |
| **D1** | GPIO 5 | D5 |
| **D2** | GPIO 13 | D13 |
| **D3** | GPIO 14 | D14 |
| **D4** | GPIO 16 | RX2 |
| **D5** | GPIO 17 | TX2 |
| **D6** | GPIO 18 | D18 |
| **D7** | GPIO 19 | D19 |

## 2. Address Bus (Inputs)
| 6502 Signal | ESP32 GPIO | Board Label | Characteristics |
| :--- | :--- | :--- | :--- |
| **A0** | GPIO 34 | D34 | **Input Only** |
| **A1** | GPIO 35 | D35 | **Input Only** |
| **A2** | GPIO 36 | VP | **Input Only** |
| **A3** | GPIO 39 | VN | **Input Only** |
| **A4** | GPIO 32 | D32 | |
| **A5** | GPIO 33 | D33 | |
| **A6** | GPIO 21 | D21 | |
| **A7** | GPIO 27 | D27 | |

## 3. Control Signals
| 6502 Signal | ESP32 GPIO | Board Label | Dir | Description |
| :--- | :--- | :--- | :--- | :--- |
| **PHI2** | GPIO 2 | D2 | IN | System Clock |
| **R/W** | GPIO 15 | D15 | IN | Read/Write |
| **SEL_N** | GPIO 22 | D22 | IN | D1XX_N or CCTL_N |
| **ROMSEL** | GPIO 23 | D23 | IN | $D800-$DFFF Range Select |
| **RAMSEL** | GPIO 26 | D26 | IN | $D600-$D7FF Range Select |
| **EXTSEL** | GPIO 3 | **RX0** | OUT | Disable Atari Memory (Active Low) |
| **VCS** | GPIO 25 | D25 | OUT | Device Select (Active Low) |
| **MPD** | **GPIO 0** | **BOOT**| OUT | Math Pack Disable (Active Low) |
| **RESET** | **GPIO 12** | **D12** | OUT | **Atari RESET Control** (Active Low) |

## 4. System
| Signal | ESP32 GPIO | Board Label | Description |
| :--- | :--- | :--- | :--- |
| **Debug TX** | GPIO 1 | TX0 | Serial output (115200 bps) |
| **GND** | GND | GND | Common Ground |

---

## ⚠️ Hardware Validation & Boot Sequence
1. **Startup:** Upon power-on, the ESP32 immediately pulls **GPIO 12 (RESET)** LOW, holding the Atari 130XE in reset.
2. **GPIO 0 (MPD):** The Atari's internal pull-up ensures GPIO 0 is HIGH, allowing the ESP32 to boot normally.
3. **Initialization:** The ESP32 configures all GPIOs and starts the 1.79 MHz MonitorTask.
4. **Release:** After 100ms, the ESP32 drives **GPIO 12** HIGH, allowing the Atari to start its boot process.
5. **Level Shifters:** Ensure the Level Shifter for GPIO 12 is powered and functional during this sequence.
