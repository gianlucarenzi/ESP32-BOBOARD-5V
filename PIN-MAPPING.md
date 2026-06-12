# ESP32-BOBOARD-5V — Pin Mapping

**Hardware**: ESP32-BOBOARD-5V (NodeMCU-32S + 3× TXS0108EPW level shifters)  
**Atari target**: 800XL PBI / 65XE-130XE ECI+CART  
**MCU**: ESP32-WROOM (3.3 V)

---

## GPIO Assignment (Firmware)

| Segnale | GPIO ESP32 | Etichetta Board | Dir | Descrizione |
| :--- | :---: | :--- | :---: | :--- |
| **D0** |  4 | D4       | I/O | Bus Dati bit 0 |
| **D1** |  5 | D5       | I/O | Bus Dati bit 1 |
| **D2** | 13 | D13      | I/O | Bus Dati bit 2 |
| **D3** | 14 | D14      | I/O | Bus Dati bit 3 |
| **D4** | 16 | RX2      | I/O | Bus Dati bit 4 |
| **D5** | 17 | TX2      | I/O | Bus Dati bit 5 |
| **D6** | 18 | D18      | I/O | Bus Dati bit 6 |
| **D7** | 19 | D19      | I/O | Bus Dati bit 7 |
| **A0** | 34 | D34      | IN  | Bus Indirizzi bit 0 (**Input Only**) |
| **A1** | 35 | D35      | IN  | Bus Indirizzi bit 1 (**Input Only**) |
| **A2** | 36 | VP       | IN  | Bus Indirizzi bit 2 (**Input Only**) |
| **A3** | 39 | VN       | IN  | Bus Indirizzi bit 3 (**Input Only**) |
| **A4** | 32 | D32      | IN  | Bus Indirizzi bit 4 |
| **A5** | 33 | D33      | IN  | Bus Indirizzi bit 5 |
| **A6** | 21 | D21      | IN  | Bus Indirizzi bit 6 |
| **A7** | 27 | D27      | IN  | Bus Indirizzi bit 7 |
| **A8** | 12 | D12      | IN  | Bus Indirizzi bit 8 |
| **A9** | 25 | D25      | IN  | Bus Indirizzi bit 9 |
| **A10**| 26 | D26      | IN  | Bus Indirizzi bit 10 |
| **PHI2**   |  2 | D2       | IN  | Clock 6502 (1.79 MHz) |
| **R/W**    | 15 | D15      | IN  | Read/Write |
| **SEL\_N** | 22 | D22      | IN  | Selezione $D1XX / CCTL (Active Low) |
| **ROMSEL** | 23 | D23      | IN  | Range $D800–$DFFF (Active Low) |
| **EXTSEL** |  3 | **RX0**  | OUT | Disabilita memoria interna Atari (Active Low) |
| **MPD**    |  0 | **BOOT** | OUT | Math Pack Disable (Active Low) |
| **TX debug** | 1 | TX0   | OUT | Console seriale (115200 bps) |

---

## Note Tecniche

- **A0–A10 decodificati**: 11 bit di indirizzo coprono l'intero range $D800–$DFFF (2 KB) senza aliasing.
- **GPIO 0 (MPD/BOOT)**: il pull-up interno dell'Atari (isolato dal TXS0108E) mantiene GPIO 0 alto durante il boot dell'ESP32.
- **GPIO 3 (EXTSEL/RX0)**: condiviso con il pin RX della UART; non utilizzabile per input seriale durante l'esecuzione.
- **Bus Dati (LUT)**: GPIO non contigui → LUT da 256 entry precalcolata per scrivere sul bus alla massima velocità.
- **Bus Indirizzi A0–A5**: nel registro `GPIO_IN1_REG` (GPIO 32–39), lettura atomica in un ciclo.
- **Bus Indirizzi A6–A10**: nel registro `GPIO_IN_REG` (GPIO 0–31), campionati insieme agli altri segnali di controllo.
- Tutti i segnali Atari↔ESP32 passano attraverso i **TXS0108EPW** (3.3 V / 5 V).
