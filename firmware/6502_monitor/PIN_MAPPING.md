# 6502 Monitor — ESP32 Pin Mapping (NodeMCU DevKit V1)

## 1. Data Bus (Bidirectional)
| 6502 Signal | ESP32 GPIO | Board Label |
| :--- | :--- | :--- |
| **D0** | GPIO 4  | D4  |
| **D1** | GPIO 5  | D5  |
| **D2** | GPIO 13 | D13 |
| **D3** | GPIO 14 | D14 |
| **D4** | GPIO 16 | RX2 |
| **D5** | GPIO 17 | TX2 |
| **D6** | GPIO 18 | D18 |
| **D7** | GPIO 19 | D19 |

## 2. Address Bus (A0-A10 — full 2 KB decode)
| 6502 Signal | ESP32 GPIO | Board Label | Note |
| :--- | :--- | :--- | :--- |
| **A0** | GPIO 34 | D34 | **Input Only** |
| **A1** | GPIO 35 | D35 | **Input Only** |
| **A2** | GPIO 36 | VP  | **Input Only** |
| **A3** | GPIO 39 | VN  | **Input Only** |
| **A4** | GPIO 32 | D32 | |
| **A5** | GPIO 33 | D33 | |
| **A6** | GPIO 21 | D21 | |
| **A7** | GPIO 27 | D27 | |
| **A8** | GPIO 12 | D12 | |
| **A9** | GPIO 25 | D25 | |
| **A10**| GPIO 26 | D26 | |

## 3. Control Signals
| Signal | ESP32 GPIO | Board Label | Dir | Description |
| :--- | :--- | :--- | :--- | :--- |
| **PHI2**   | GPIO 2  | D2       | IN  | 6502 System Clock (1.79 MHz) |
| **R/W**    | GPIO 15 | D15      | IN  | Read/Write |
| **SEL\_N** | GPIO 22 | D22      | IN  | $D1XX or CCTL selection (Active Low) |
| **ROMSEL** | GPIO 23 | D23      | IN  | $D800–$DFFF range (Active Low) |
| **EXTSEL** | GPIO 3  | **RX0**  | OUT | Disable Atari internal memory (Active Low) |
| **MPD**    | GPIO 0  | **BOOT** | OUT | Math Pack Disable (Active Low) |

## 4. Serial Debug
| Signal | ESP32 GPIO | Board Label | Description |
| :--- | :--- | :--- | :--- |
| **TX** | GPIO 1 | TX0 | Console output (115200 bps) |

---

## Notes
- **A0-A10 decoded**: full 11-bit address covers the entire 2 KB $D800–$DFFF range without aliasing.
- **GPIO 0 (MPD / BOOT)**: the Atari's internal pull-up (isolated by TXS0108E) keeps GPIO 0 HIGH during ESP32 boot.
- **GPIO 3 (EXTSEL / RX0)**: shared with UART RX; not usable for serial input while firmware is running.
- All signals pass through **TXS0108E** bidirectional level shifters (3.3 V ↔ 5 V).
