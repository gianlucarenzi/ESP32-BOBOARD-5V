# Sidboard — ESP32 Pin Mapping

Target board: **NodeMCU-32S** (ESP32-WROOM-32, 38 pin)

---

## Atari Bus Interface

### Data Bus (bidirectional)

| GPIO | NodeMCU | Direction | Atari signal | Note |
|------|---------|-----------|--------------|------|
| 4    | D2      | In/Out    | D0           | Data bit 0 |
| 5    | D1      | In/Out    | D1           | Data bit 1 |
| 13   | D7      | In/Out    | D2           | Data bit 2 |
| 14   | D5      | In/Out    | D3           | Data bit 3 |
| 16   | RX2     | In/Out    | D4           | Data bit 4 |
| 17   | TX2     | In/Out    | D5           | Data bit 5 |
| 18   | D5/SCK  | In/Out    | D6           | Data bit 6 |
| 19   | D6/MISO | In/Out    | D7           | Data bit 7 |

### Address Bus (input only)

| GPIO | NodeMCU | Direction | Atari signal | GPIO_IN register |
|------|---------|-----------|--------------|-----------------|
| 34   | —       | Input     | A0           | GPIO_IN1 bit 2  |
| 35   | —       | Input     | A1           | GPIO_IN1 bit 3  |
| 36   | —       | Input     | A2           | GPIO_IN1 bit 4  |
| 39   | —       | Input     | A3           | GPIO_IN1 bit 7  |
| 32   | —       | Input     | A4           | GPIO_IN1 bit 0  |

> GPIOs 32–39 are read via `GPIO_IN1_REG` (upper bank). GPIOs 34–39 are input-only (no internal pull-up/down).

### Control Signals

| GPIO | NodeMCU | Direction | Signal  | Description |
|------|---------|-----------|---------|-------------|
| 2    | D4      | Input     | PHI2    | 6502 Phase 2 clock — 1.79 MHz PAL; rising edge triggers bus sample |
| 15   | D8      | Input     | R/W     | Read/Write — High = Read, Low = Write |
| 22   | D6/SCL  | Input     | SEL_N   | CCTL active-low select — asserted for $D5xx page |

### Audio Output

| GPIO | NodeMCU | Direction | Signal   | Description |
|------|---------|-----------|----------|-------------|
| 25   | —       | Output    | DAC1     | SID audio — 8-bit, 22050 Hz, mono; feed via RC filter to audio mixer |

---

## Address Decode Logic

Only A0–A4 are wired (5 bits → 32 locations):

```
SEL_N LOW  → CCTL active ($D5xx page)
offset 0x00–0x1E  →  SID registers  ($D500–$D51E)
offset 0x1F       →  Latch control   ($D5FF, A0–A4 all high)
```

Latch control byte: `bit 0 = 1` → enable SID; `bit 0 = 0` → disable + silence.

---

## Serial Debug

| GPIO | NodeMCU | Direction | Signal | Description |
|------|---------|-----------|--------|-------------|
| 1    | TX0     | Output    | TX     | UART0 TX — 115200 baud, TX-only mode |

> RX (GPIO 3) is not used. Serial is initialised with `Serial.begin(115200, SERIAL_8N1, -1, 1)` to keep GPIO 3 free.

---

## Free GPIOs

Pins available for future use:

| GPIO | NodeMCU | Capability | Former use |
|------|---------|------------|------------|
| 0    | D3      | Output / Boot strap | — (boot pull-up required) |
| 3    | RX0     | Input / UART0 RX | — |
| 12   | D6      | Input / ADC2_CH5 / Touch5 | A8 (address bus) |
| 21   | D6/SDA  | In/Out / I²C SDA | A6 (address bus) |
| 23   | D7/MOSI | In/Out / SPI MOSI | ROMSEL |
| 26   | —       | Output / **DAC2** | A10 (address bus) |
| 27   | —       | In/Out / ADC2_CH7 / Touch7 | A7 (address bus) |
| 33   | —       | Input / ADC1_CH5 / Touch8 | A5 (address bus) |

> **GPIO 26** is ESP32 DAC2 — available as a second audio channel or CV output.

---

## Reserved / Not Available

| GPIO | Reason |
|------|--------|
| 6–11 | Internal SPI flash — hardwired, do not use |
| 34–39 | Input-only (no output drive, no pull-up/down) |

---

## NodeMCU-32S Physical Layout

Pin order follows the standard NodeMCU-32S 38-pin silk-screen (left column top→bottom, then right column top→bottom).

```
                      ┌────────────────────┐
              3V3  ───┤  1              38 ├───  GND
               EN  ───┤  2              37 ├───  GPIO23   free (was ROMSEL)
           GPIO39  ───┤  3  A3          36 ├───  GPIO22   SEL_N (CCTL)
           GPIO36  ───┤  4  A2          35 ├───  GPIO1    TX debug
           GPIO35  ───┤  5  A1          34 ├───  GPIO3    free
           GPIO34  ───┤  6  A0          33 ├───  GPIO21   free (was A6)
              GND  ───┤  7              32 ├───  GPIO19   D7
           GPIO32  ───┤  8  A4          31 ├───  GPIO18   D6
           GPIO33  ───┤  9  free(wasA5) 30 ├───  GPIO5    D1
           GPIO27  ───┤ 10  free(wasA7) 29 ├───  GPIO17   D5
           GPIO12  ───┤ 11  free(wasA8) 28 ├───  GPIO16   D4
              GND  ───┤ 12              27 ├───  GPIO4    D0
           GPIO25  ───┤ 13  DAC1 audio  26 ├───  GPIO0    free (boot strap)
           GPIO26  ───┤ 14  free(DAC2)  25 ├───  GPIO2    PHI2
              GND  ───┤ 15              24 ├───  GPIO15   R/W
           GPIO14  ───┤ 16  free(wasA?) 23 ├───  GPIO13   D2
              GND  ───┤ 17              22 ├───  GND
           GPIO12  ───┤ 18  free        21 ├───  (flash — do not use)
              5V   ───┤ 19              20 ├───  (flash — do not use)
                      └────────────────────┘
```

> Rows 20–22 (GPIO 6–11) are tied to internal SPI flash and must not be used.

---

## SID Register Map ($D500–$D51E)

| Offset | Address | Register    | Description |
|--------|---------|-------------|-------------|
| $00    | $D500   | V1_FREQ_LO  | Voice 1 frequency low byte |
| $01    | $D501   | V1_FREQ_HI  | Voice 1 frequency high byte |
| $02    | $D502   | V1_PW_LO    | Voice 1 pulse width low |
| $03    | $D503   | V1_PW_HI    | Voice 1 pulse width high |
| $04    | $D504   | V1_CTRL     | Voice 1 control (bit 0 = GATE, bit 6 = SQR) |
| $05    | $D505   | V1_AD       | Voice 1 Attack/Decay |
| $06    | $D506   | V1_SR       | Voice 1 Sustain/Release |
| $07    | $D507   | V2_FREQ_LO  | Voice 2 frequency low byte |
| $08    | $D508   | V2_FREQ_HI  | Voice 2 frequency high byte |
| $09    | $D509   | V2_PW_LO    | Voice 2 pulse width low |
| $0A    | $D50A   | V2_PW_HI    | Voice 2 pulse width high |
| $0B    | $D50B   | V2_CTRL     | Voice 2 control |
| $0C    | $D50C   | V2_AD       | Voice 2 Attack/Decay |
| $0D    | $D50D   | V2_SR       | Voice 2 Sustain/Release |
| $0E    | $D50E   | V3_FREQ_LO  | Voice 3 frequency low byte |
| $0F    | $D50F   | V3_FREQ_HI  | Voice 3 frequency high byte |
| $10    | $D510   | V3_PW_LO    | Voice 3 pulse width low |
| $11    | $D511   | V3_PW_HI    | Voice 3 pulse width high |
| $12    | $D512   | V3_CTRL     | Voice 3 control |
| $13    | $D513   | V3_AD       | Voice 3 Attack/Decay |
| $14    | $D514   | V3_SR       | Voice 3 Sustain/Release |
| $15    | $D515   | FC_LO       | Filter cutoff low (not emulated) |
| $16    | $D516   | FC_HI       | Filter cutoff high (not emulated) |
| $17    | $D517   | RES_FILT    | Filter resonance/routing (not emulated) |
| $18    | $D518   | MODE_VOL    | Master volume (bits 3:0, range 0–15) |
| $19    | $D519   | OSC3        | Voice 3 oscillator readout |
| $1A    | $D51A   | ENV3        | Voice 3 envelope readout |
| $1B–$1E | $D51B–$D51E | —     | Unused |
| **$1F** | **$D5FF** | **LATCH** | **bit 0=1 → enable; bit 0=0 → disable+reset** |
