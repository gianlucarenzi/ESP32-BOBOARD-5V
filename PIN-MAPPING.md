# ESP32-BOBOARD-5V — Pin Mapping

**Hardware**: ESP32-BOBOARD-5V (NodeMCU-32S + 3× TXS0108EPW level shifters)  
**Atari target**: 800XL PBI / 65XE-130XE ECI+CART  
**MCU**: ESP32 (3.3 V)

---

## Mappatura Hardware (Configurazione Corrente)

Questa è la configurazione fisica dei segnali sul connettore **J3**, traslati a 5V tramite i level shifter **U1**, **U3** e **U4**.

### Segnali Debug / Programmazione
I seguenti pin sono collegati **solo** al connettore di debug e **non** passano attraverso i level shifter verso l'Atari:
*   **TX0 (GPIO 1)**: Console Seriale Out (printf / Log)
*   **RX0 (GPIO 3)**: Console Seriale In (Programmazione / Comandi)

---

## Tabella Connettore J3 (Atari ↔ BOBOARD)

| Pin J3 | Segnale Atari | GPIO ESP32 | IC Shifter | Note |
| :---: | :--- | :---: | :---: | :--- |
| **1** | **D0** | **2** | U1 | Bus Dati (Bidirezionale) |
| **2** | **D1** | **4** | U1 | |
| **3** | **D2** | **5** | U1 | |
| **4** | **D3** | **12** | U1 | |
| **5** | **D4** | **13** | U1 | |
| **6** | **D5** | **14** | U1 | |
| **7** | **D6** | **15** | U1 | |
| **8** | **D7** | **18** | U1 | |
| **9** | **~MPD** | **19** | U3 | Math Pack Disable (Output) |
| **10** | **~EXTSEL** | **21** | U3 | External Select (Output) |
| **11** | **A8** | **22** | U3 | Address MSB (Input) |
| **12** | **PHI2** | **23** | U3 | Clock Atari (Input) |
| **13** | **R/~W** | **25** | U3 | Direzione Bus (Input) |
| **14** | **CCTL** | **26** | U3 | Selezione $D5xx (Input) |
| **15** | **~D1XX** | **27** | U3 | Selezione $D1xx (Input) |
| **16** | **A0** | **32** | U3 | Indirizzo LSB |
| **17** | **A1** | **33** | U4 | |
| **18** | **A2** | **34** | U4 | |
| **19** | **A3** | **35** | U4 | |
| **20** | **A4** | **36** | U4 | |
| **21** | **A5** | **39** | U4 | |
| **22** | **A6** | **16** | U4 | |
| **23** | **A7** | **17** | U4 | |
| **24, 25**| **NC** | — | — | Non collegati |
| **26** | **EN** | — | — | Reset ESP32 (Tasto Enable) |
| **27, 28**| **3.3V** | — | — | Alimentazione regolata |
| **29, 30**| **GND** | — | — | Massa comune |
| **31, 32**| **5V** | — | — | Alimentazione dall'Atari |

---

## Dettagli Tecnici Ottimizzazione

*   **Bus Dati**: Poiché i GPIO su **U1** non sono contigui, il firmware utilizza una **LUT (Look-Up Table)** di 256 byte per scrivere sul bus alla massima velocità possibile.
*   **Bus Indirizzi**: Gli indirizzi da **A0** a **A5** sono raggruppati nel registro `GPIO_IN1_REG` (GPIO 32-39), permettendo una lettura atomica molto veloce durante il ciclo di clock dell'Atari.
*   **Stabilità**: Il pin **GPIO 0** (necessario per il boot) non è utilizzato nel bus Atari per evitare che la macchina entri in modalità programmazione accidentalmente all'accensione.
