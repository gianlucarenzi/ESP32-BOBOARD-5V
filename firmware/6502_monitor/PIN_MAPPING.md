# 6502 Monitor — ESP32 Pin Mapping (NodeMCU DevKit V1)

Banked RAM PBI / CCTL peripheral (GeoRAM style): 20 bus signals, all through the TXS0108E
level translators.

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

D0-D7 are inputs; the firmware drives them only while answering a read of a
read of the RAM page or of a register (bank count, bank register).

GeoRAM style, one latch at a time: `$80` to `$D1FF` puts the registers in
`$D1xx` and the RAM in `$D500-$D5FF`; `$80` to `$D5FF` puts the registers in
`$D5xx` and the RAM in `$D100-$D1FE` (`$D1FF` protected).  Register page:
count `$xxFB-$xxFC` (R, 16 bit), bank `$xxFD-$xxFE` (R/W, 16 bit), latch
`$xxFF` (W, bit 7).

## 2. Address Bus (A0-A7)
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

## 3. Control Signals
| Signal | ESP32 GPIO | Board Label | Dir | Description |
| :--- | :--- | :--- | :--- | :--- |
| **PHI2**    | GPIO 2  | D2  | IN | 6502 System Clock (1.79 MHz) |
| **R/W**     | GPIO 15 | D15 | IN | Read/Write |
| **~D1XX**   | GPIO 22 | D22 | IN | $D100–$D1FF page, from the Atari MMU / PLA (Active Low, not PHI2-qualified) |
| **~CCTL**   | GPIO 23 | D23 | IN | $D500–$D5FF page, from the Atari MMU / PLA (Active Low) |

## 4. Serial Debug
| Signal | ESP32 GPIO | Board Label | Description |
| :--- | :--- | :--- | :--- |
| **TX** | GPIO 1 | TX0 | Console output (115200 bps, `LOG_BAUD`); also on J4 pin 3 |

## 5. Unused
| ESP32 GPIO | Board Label | Note |
| :--- | :--- | :--- |
| GPIO 12 | D12 | Still wired to U1 / J3 pin 4 (former A8). Strapping pin: ignored once the VDD_SDIO efuses are set |
| GPIO 25 | D25 | Free TXS0108E channel (former A9) |
| GPIO 26 | D26 | Free TXS0108E channel (former A10) |
| GPIO 3  | RX0 | J4 pin 2 only, not routed to J3; leave unconnected |
| GPIO 0  | BOOT | Not on the 30-pin headers |

U4 channel 8 (A8 pin 9 / B8 pin 12) and J3 pins 24-25 are also unconnected.

---

## Notes
- **A0-A7 only**: the page ($D1 or $D5) comes from the select line.
- **GPIO 3 (RX0)**: not used; J4 is read-only (GND + TX0).
- All signals pass through **TXS0108E** bidirectional level shifters (3.3 V ↔ 5 V).
