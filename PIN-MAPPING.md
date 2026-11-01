# ESP32-BOBOARD-5V — Pin Mapping

**Hardware**: ESP32-BOBOARD-5V (NodeMCU-32S + 3× TXS0108EPW level shifters)  
**Atari target**: 800XL PBI / 65XE-130XE ECI+CART  
**MCU**: ESP32-WROOM (3.3 V)

---

## GPIO Assignment (Firmware)

Firmware `6502_monitor`: RAM a banchi PBI / CCTL senza ROM (stile GeoRAM), 20 segnali di bus.

| Segnale | GPIO ESP32 | Etichetta Board | TXS0108E | Dir | Descrizione |
| :--- | :---: | :--- | :---: | :---: | :--- |
| **D0** |  4 | D4       | U1 | I/O | Bus Dati bit 0 |
| **D1** |  5 | D5       | U1 | I/O | Bus Dati bit 1 |
| **D2** | 13 | D13      | U1 | I/O | Bus Dati bit 2 |
| **D3** | 14 | D14      | U1 | I/O | Bus Dati bit 3 |
| **D4** | 16 | RX2      | U4 | I/O | Bus Dati bit 4 |
| **D5** | 17 | TX2      | U4 | I/O | Bus Dati bit 5 |
| **D6** | 18 | D18      | U1 | I/O | Bus Dati bit 6 |
| **D7** | 19 | D19      | U3 | I/O | Bus Dati bit 7 |
| **A0** | 34 | D34      | U4 | IN  | Bus Indirizzi bit 0 (**Input Only**) |
| **A1** | 35 | D35      | U4 | IN  | Bus Indirizzi bit 1 (**Input Only**) |
| **A2** | 36 | VP       | U4 | IN  | Bus Indirizzi bit 2 (**Input Only**) |
| **A3** | 39 | VN       | U4 | IN  | Bus Indirizzi bit 3 (**Input Only**) |
| **A4** | 32 | D32      | U3 | IN  | Bus Indirizzi bit 4 |
| **A5** | 33 | D33      | U4 | IN  | Bus Indirizzi bit 5 |
| **A6** | 21 | D21      | U3 | IN  | Bus Indirizzi bit 6 |
| **A7** | 27 | D27      | U3 | IN  | Bus Indirizzi bit 7 |
| **PHI2**    |  2 | D2  | U1 | IN  | Clock 6502 (1.79 MHz) |
| **R/W**     | 15 | D15 | U1 | IN  | Read/Write |
| **~D1XX**   | 22 | D22 | U3 | IN  | Pagina $D100–$D1FF, dalla MMU/PLA (Active Low, non qualificato da PHI2) |
| **~CCTL**   | 23 | D23 | U3 | IN  | Pagina $D500–$D5FF, dalla MMU/PLA (Active Low) |
| **TX debug** | 1 | TX0 | —  | OUT | Console seriale (115200 bps), anche su J4 pin 3 |

### Non usati dal firmware

| GPIO | Etichetta | Note |
| :---: | :--- | :--- |
| 12 | D12 | Ancora collegato a U1 / J3 pin 4. Pin di strapping (MTDI): ignorato con gli efuse VDD_SDIO a 3,3 V |
| 25 | D25 | Canale U3 libero |
| 26 | D26 | Canale U3 libero |
| 3  | RX0 | Solo J4 pin 2, non portato su J3: lasciare scollegato |
| 0  | BOOT | Non presente sui connettori del DevKit a 30 pin |

Liberi anche il canale 8 di U4 (pin 9 / 12) e i pin 24–25 di J3.

---

## Note Tecniche

- **Solo A0–A7**: la pagina ($D1 o $D5) è data dal segnale di selezione.
- **D0–D7** sono ingressi; il firmware li pilota solo per rispondere a una lettura della pagina RAM o di un registro (numero di banchi, registro del banco).
- **Stile GeoRAM, un solo latch alla volta**: `$80` in `$D1FF` mette i registri in `$D1xx` e la RAM in `$D500–$D5FF`; `$80` in `$D5FF` mette i registri in `$D5xx` e la RAM in `$D100–$D1FE` (`$D1FF` protetto).  Pagina dei registri: numero di banchi `$xxFB–$xxFC` (R, 16 bit), registro del banco `$xxFD–$xxFE` (R/W, 16 bit), latch `$xxFF` (W, bit 7).
- **Bus Indirizzi A0–A5**: nel registro `GPIO_IN1_REG` (GPIO 32–39); A6–A7 in `GPIO_IN_REG`, insieme ai segnali di controllo.
- **Bus Dati**: GPIO non contigui → lettura con shift (`decode_data()`), scrittura tramite la tabella `drive_lut[256]` di parole `GPIO.out`.
- **J4 (debug UART)**: GND (pin 1), RX0 (pin 2, da lasciare scollegato), TX0 (pin 3, 3,3 V).
- Tutti i segnali Atari↔ESP32 passano attraverso i **TXS0108EPW** (3.3 V / 5 V); `TXS_EN` è tenuto alto da R1/R3/R4, SW1 lo porta a GND.
