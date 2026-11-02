# Sidboard — ESP32 Pin Mapping

Board: **ESP32-BOBOARD-5V** with a NodeMCU-32S (ESP32-WROOM-32).  Same pins as
the 6502_monitor firmware; every Atari signal goes through the TXS0108E level
translators to J3 (5 V side).

## Atari Bus

| Signal | GPIO | Board label | J3 pin | Dir | Note |
|---|---|---|---|---|---|
| D0 | 4  | D4  | 2  | I/O | driven only while the SID answers a read |
| D1 | 5  | D5  | 3  | I/O | |
| D2 | 13 | D13 | 5  | I/O | |
| D3 | 14 | D14 | 6  | I/O | |
| D4 | 16 | RX2 | 22 | I/O | |
| D5 | 17 | TX2 | 23 | I/O | |
| D6 | 18 | D18 | 8  | I/O | |
| D7 | 19 | D19 | 9  | I/O | |
| A0 | 34 | D34 | 18 | IN | input only |
| A1 | 35 | D35 | 19 | IN | input only |
| A2 | 36 | VP  | 20 | IN | input only |
| A3 | 39 | VN  | 21 | IN | input only |
| A4 | 32 | D32 | 16 | IN | |
| A5 | 33 | D33 | 17 | IN | |
| A6 | 21 | D21 | 10 | IN | |
| A7 | 27 | D27 | 15 | IN | |
| PHI2 | 2 | D2 | 1 | IN | 6502 clock |
| R/W | 15 | D15 | 7 | IN | high = read |
| `~D1XX` | 22 | D22 | 11 | IN | **sid-atari-pbi**: $D100-$D1FF page (Atari MMU, active low) |
| `~CCTL` | 23 | D23 | 12 | IN | **sid-atari-cctl**: $D500-$D5FF page (cartridge slot pin 15, active low) |

## Commodore 64 Expansion Port (sid-c64-io1 / sid-c64-io2)

Same GPIOs and J3 pins as above for D0-D7, A0-A7, PHI2 and R/W; the select
line is the one of the build:

| Signal | GPIO | J3 pin | C64 port pin | Build |
|---|---|---|---|---|
| `/IO1` | 23 | 12 | 7 | **sid-c64-io1**: $DE00-$DEFF |
| `/IO2` | 22 | 11 | 10 | **sid-c64-io2**: $DF00-$DFFF |

Wiring: [`../6502_monitor/docs/wiring_c64.svg`](../6502_monitor/docs/wiring_c64.svg)
(it shows both lines: the GeoRAM build uses both).

Wiring to the 130XE cartridge slot and ECI: see the infographics in
[`../6502_monitor/docs/`](../6502_monitor/docs/wiring_atari.svg).  Never
connect J3 26 (EN) and 27-28 (+3V3) to the computer.

## Audio and Debug

| Signal | GPIO | Note |
|---|---|---|
| DAC1 | 25 | SID audio, 8 bit, 22.2 kHz; RC filter to the audio mixer / POKEY ECI AUDIO IN |
| TX | 1 | debug log, 115200 baud, also on J4 pin 3 (RX0 on J4 pin 2: leave unconnected) |

## Not Used

| GPIO | Note |
|---|---|
| 12 | still wired to U1 / J3 pin 4; strapping pin, ignored once the VDD_SDIO efuses are set |
| 26 | free TXS0108E channel (J3 pin 14); ESP32 DAC2 |
| 0, 3 | not routed to J3 |

## Address Decode

```
sid-atari-cctl:  ~CCTL low  -> $D500 + A0-A7
             $00-$1E  SID registers (always on)
             $1F-$FF  not decoded
sid-atari-pbi:   ~D1XX low  -> $D100 + A0-A7
             $00-$1E  SID registers, while selected
             $FF      PBI select latch (W): bit 7 = 1 select, 0 deselect; never driven
             $1F-$FE  not decoded
sid-c64-io1: /IO1 low -> $DE00 + A0-A7     sid-c64-io2: /IO2 low -> $DF00 + A0-A7
             $00-$1E  SID registers (always on)
             $1F-$FF  not decoded
```
