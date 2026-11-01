# Sidboard — SID Sound for the Atari 8-bit (ESP32-BOBOARD-5V)

Firmware for the **ESP32-BOBOARD-5V** (NodeMCU-32S) on the Atari 8-bit bus:

- **SID 6581/8580 sound** (cSIDLight: 3 square-wave voices, frequency, gate,
  master volume; ESP32 DAC1 on GPIO 25, 22.2 kHz)
- two builds: **CCTL** (`$D500`, always on) or **PBI device** (`$D100`,
  select latch `$D1FF`)
- the bus loop of the [6502_monitor](../6502_monitor/README.md) firmware:
  PHI1 decode in assembly on Core 1, interrupts masked, lock-free log
- timestamped serial log of every access to the SID page

---

## Builds

| Environment | SID registers | Select | Wiring |
|---|---|---|---|
| `sid-cctl` | `$D500-$D51E` | always on, no latch, no configuration | Atari `~CCTL` → J3 pin 12 |
| `sid-pbi` | `$D100-$D11E` | PBI device ID `$80`: write `$80` to `$D1FF` to select, `$00` to deselect | `~D1XX` → J3 pin 11 |
| `sid-demo` | as `sid-cctl` | plus a built-in tune (`AUDIO_DEMO`, Commando at 50 Hz) | as `sid-cctl` |

Only A0-A7 are sampled: the page comes from the select line.  Offsets
`$1F-$FE` of the page are not decoded (never driven, writes ignored).
Reads of `$00-$1E` return the stored register byte.

**`sid-pbi`:** `$D1FF` is never driven (it is the PBI interrupt status read
by the OS).  Deselected, the SID neither answers nor stores writes, but it
**keeps playing**: deselecting only removes it from the bus.  The OS PBI
scan at power-up and RESET writes `$80` then `$00` to `$D1FF`, so the SID
ends deselected with its registers unchanged.  There is no PBI ROM (the
board has no MPD / EXTSEL), so the OS does not install a handler: programs
access the registers directly.

**`sid-cctl`:** `$D5xx` belongs to the SID: cartridges that bank-switch
through CCTL (XEGS, AtariMax, SpartaDOS X, ...) cannot be used with it; use
`sid-pbi` then.

---

## Pin Mapping

Same pins as the 6502_monitor firmware (J3 of the ESP32-BOBOARD-5V, 5 V side
of the TXS0108E); full table in [PIN_MAPPING.md](PIN_MAPPING.md), wiring
infographics in [`../6502_monitor/docs/`](../6502_monitor/docs/wiring_atari.svg).

| Signal | GPIO | J3 pin |
|---|---|---|
| D0-D7 | 4, 5, 13, 14, 16, 17, 18, 19 | 2, 3, 5, 6, 22, 23, 8, 9 |
| A0-A7 | 34, 35, 36, 39, 32, 33, 21, 27 | 18, 19, 20, 21, 16, 17, 10, 15 |
| PHI2 / R/W | 2 / 15 | 1 / 7 |
| `~D1XX` (sid-pbi) | 22 | 11 |
| `~CCTL` (sid-cctl) | 23 | 12 |
| DAC1 audio | 25 | (module pin) |
| TX debug | 1 | J4 pin 3 |

> **Changed from the previous firmware:** CCTL was read on GPIO 22 with
> A0-A4 only; it is now on GPIO 23 (J3 pin 12) with A0-A7, as on the
> 6502_monitor firmware and the board netlist.

---

## SID Register Map

Base `$D500` (sid-cctl) or `$D100` (sid-pbi).

| Offset | Register | Emulated |
|--------|----------|----------|
| `$00/$01` | V1 Freq Lo/Hi | yes |
| `$02/$03` | V1 PW Lo/Hi | no |
| `$04` | V1 CTRL | GATE (bit 0) |
| `$05/$06` | V1 AD/SR | no |
| `$07-$0D` | Voice 2 (as V1) | as V1 |
| `$0E-$14` | Voice 3 (as V1) | as V1 |
| `$15/$16` | FC Lo/Hi | no |
| `$17` | RES/FILT | no |
| `$18` | MODE/VOL | volume (bits 3-0) |
| `$19/$1A` | POTX / POTY | no (read back the stored byte) |
| `$1B/$1C` | OSC3 / ENV3 | no (read back the stored byte) |
| `$1D/$1E` | unused | stored |
| `$FF` | `PBI_SELECT` (sid-pbi only) | latch, bit 7 |

---

## Architecture

```
Core 0                                    Core 1 (interrupts masked)
──────────────────────────────            ─────────────────────────────────────
LogTask   only writer on Serial            MonitorTask (IRAM, highest priority)
          drains the log ring               PHI2 falls: release D0-D7, log
SidTask   process() every 1 ms             + BUS_ADDR_SETTLE_NS (200 ns):
          registers -> synthesis              sample A0-A7, R/W, select
esp_timer 22.2 kHz _tick() -> DAC1            read of $00-$1E: byte to GPIO.out
                                            PHI2 rises: one store drives D0-D7
                                            write: sample + BUS_WR_SAMPLE_NS
                                              (120 ns) -> register / latch
```

- **Bus loop** (from the 6502_monitor firmware, itself from
  Vera-Module-RBL-XE): Core 1 runs only MonitorTask with interrupts masked
  and the interrupt watchdog off.  The PHI1 decode is inline assembly:
  address from two lookup tables, register byte placed in `GPIO.out` while
  the outputs are still off, turned on with one store at the PHI2 rising
  edge.  A select seen only on the rising edge is handled from that sample.
  GPIO accesses without `memw`.
- **Registers:** the bus loop stores straight into the cSIDLight register
  array (`regs()`); SidTask turns changes into synthesis parameters within
  1 ms.
- **Log:** lock-free single-producer / single-consumer ring (1024 events)
  from Core 1 to LogTask; one `Serial.write()` per line, events dropped and
  counted when the ring is full.

Serial output:

```
[sidboard] SID PBI device, ID $80: $D100-$D11E, select $D1FF (~D1XX, J3 pin 11)
[sidboard] cSIDLight: 3 voices, DAC1 on GPIO 25, 22222 Hz
[sidboard] Bus loop on Core 1 (settle 200 ns, write sample 120 ns), log 115200 baud
[    1.234567] [D1FF - PBI_SELECT  ] W $80  SELECTED
[    1.234601] [D100 - V1_FREQ_LO  ] W $3C  SEL
[    1.234620] [D104 - V1_CTRL     ] W $11  SEL
[    1.300000] [D104 - V1_CTRL     ] R $11  SEL DRV
```

`SEL` = the SID answers, `DRV` = byte driven by the board, `-` = deselected
(the logged byte is whatever was on the bus).

---

## cSIDLight Audio Engine

Defined in `include/csidlight.h`. Self-contained — no external audio library.

### Parameters

| Parameter | Value |
|-----------|-------|
| Target sample rate | 22050 Hz |
| Timer period | 45 µs (`SID_TIMER_US`, integer truncation of 45.351 µs) |
| Actual sample rate | 22222 Hz (`SID_ACTUAL_RATE = 1000000 / SID_TIMER_US`) |
| Output pin | GPIO 25 (ESP32 DAC1) |
| Voices | 3 independent square-wave oscillators |
| Emulated | Frequency, GATE, master volume |
| Not emulated | ADSR envelope, waveform selection, filter, ring-mod, sync |

### Frequency Formula (PAL clock = 985248 Hz)

`esp_timer` fires every 45 µs (integer microseconds), giving an actual sample rate of 22222 Hz rather than the nominal 22050 Hz. `_phaseIncFor()` uses `SID_ACTUAL_RATE` to match the real timer cadence and keep pitch accurate.

```
freq_hz  = (reg16 × 985248) / 2^24
phaseInc = (freq_hz × 2^32) / 22222     ← SID_ACTUAL_RATE, not 22050
```

At each tick the oscillator phase always advances, regardless of gate state (as on a real SID):

```
phase[i] += phaseInc[i]                 ← unconditional
if voiceVol[i] != 0:
    square = (phase[i] & 0x80000000) ? +voiceVol[i] : -voiceVol[i]
    mix   += square
mix = clamp(128 + mix, 0, 255)
dacWrite(25, mix)
```

### Master Volume

Volume nibble is multiplied before dividing to avoid integer truncation loss:

```
voiceVol = (reg[$18] & 0x0F) × 128 / 48   // max 40 per voice; 3 × 40 = 120 ≤ 128
```

### API Surface

| Method | Caller | Description |
|--------|--------|-------------|
| `write(reg, val)` | AUDIO_DEMO timer | Single `volatile` byte store — no locking (the bus loop stores into `regs()` directly) |
| `process()` | SidTask (Core 0, every 1 ms) | Snapshot diff → update synthesis params |
| `regs()` | `setup()` | Register array, read and written directly by the bus loop |
| `reset()` | not used | Zero all registers, silence voices |
| `_tick()` | `esp_timer` (Core 0) | Generate one audio sample, write DAC |

---

## Audio Circuit

GPIO 25 is the ESP32 DAC1 output (0–3.3 V, 8-bit resolution, 22050 Hz).

**Signal path to audio mixer:**

```
GPIO 25 (DAC1)
    │
   10 µF  ──── DC block capacitor
    │
  100 kΩ  ──── audio input / POKEY ECI AUDIO IN
                │
               10 nF ─── GND      (low-pass, fc ≈ 1.75 kHz)
```

If connected to POKEY ECI AUDIO IN, the SID signal mixes directly into the POKEY audio mixer and appears on the Atari speaker output together with POKEY audio.

---

## Audio Circuit

GPIO 25 is the ESP32 DAC1 output (0–3.3 V, 8-bit resolution, 22.2 kHz).

```
GPIO 25 (DAC1)
    │
   10 µF  ──── DC block capacitor
    │
  100 kΩ  ──── audio input / POKEY ECI AUDIO IN
                │
               10 nF ─── GND      (low-pass, fc ≈ 1.75 kHz)
```

If connected to POKEY ECI AUDIO IN, the SID signal mixes into the POKEY
audio and appears on the Atari speaker output.

---

## Build and Flash

```bash
pio run -e sid-cctl -t upload     # SID at $D500
pio run -e sid-pbi  -t upload     # SID at $D100, PBI device
pio run -e sid-demo -t upload     # built-in tune, no Atari needed
pio device monitor
```

**The ESP32 module is programmed only disconnected from the Atari, from the
PC.**  As on the 6502_monitor firmware, `efuse_vdd_sdio.py` checks the
VDD_SDIO efuses before every upload and burns them to 3.3 V if blank
(**irreversible**; GPIO12 is still wired to J3 pin 4), and the serial
monitor does not toggle RTS / DTR.  Bus timing can be tuned with
`-D BUS_ADDR_SETTLE_NS=...`, `-D BUS_WR_SAMPLE_NS=...`,
`-D BUS_PHI2_HIGH_MAX_NS=...`.

---

## Usage from the Atari

### sid-cctl (Atari BASIC)

```basic
10 POKE 54528,100   : REM $D500 V1 FREQ LO
20 POKE 54529,10    : REM $D501 V1 FREQ HI
30 POKE 54552,15    : REM $D518 MODE/VOL: max volume
40 POKE 54532,17    : REM $D504 V1 CTRL: GATE + SQR
50 X=PEEK(54532)    : REM read back V1 CTRL
```

### sid-pbi (Atari BASIC)

```basic
10 POKE 53759,128   : REM $D1FF: select the SID (PBI ID $80)
20 POKE 53504,100   : REM $D100 V1 FREQ LO
30 POKE 53505,10    : REM $D101 V1 FREQ HI
40 POKE 53528,15    : REM $D118 MODE/VOL
50 POKE 53508,17    : REM $D104 V1 CTRL: GATE + SQR
60 POKE 53759,0     : REM deselect: the note keeps playing
```

### cc65

```c
#define SID ((volatile unsigned char *)0xD500)   /* 0xD100 for sid-pbi */
/* sid-pbi only: */ *(volatile unsigned char *)0xD1FF = 0x80;
SID[0x00] = 0x3C;  SID[0x01] = 0x0A;   /* V1 frequency */
SID[0x18] = 0x0F;                      /* volume */
SID[0x04] = 0x11;                      /* GATE + SQR */
```

### Dump players (`examples/`)

A dump is a flat list of frames of 25 bytes (SID registers `$00-$18`), one
frame per PAL VBlank.  `examples/mkdump.py` turns `Commando_dmp.h` into
`SIDDUMP.BIN` (27500 bytes, 1100 frames, 22 s); copy it to the Atari disk.

| Player | Build / use | Notes |
|---|---|---|
| `cc65/sid_dump_player.xex` | `make` in `examples/cc65` | sid-cctl, SID at `$D500` |
| `cc65/sid_dump_player_pbi.xex` | same | sid-pbi: `$80` to `$D1FF`, SID at `$D100`, `$00` at the end |
| `atari-basic/sid_dump_player.bas` | `PBI=0` / `PBI=1` in line 50 | a 23-byte USR routine copies each frame (BASIC alone is too slow for 50 Hz); loading 27500 bytes with `GET` takes about a minute |

Both read `D:SIDDUMP.BIN` into RAM, write one frame per VBlank, stop on
any key, then gate the voices off, set the volume to 0 and (sid-pbi)
deselect.  `python3 test_players.py` (in `examples/`, needs py65) runs
both cc65 players on a fake Atari (CIO with E: and the dump file, RTCLOK,
the sidboard model) and the BASIC USR routine: all 1100 frames written,
one per VBlank, nothing outside the SID page.  The BASIC program itself is
not run (no BASIC interpreter in the test).

The old players wrote through an `I:` CIO handler from a PBI ROM
(`6502/pbi-driver.s`) that this board cannot load (no MPD / EXTSEL): they
were rewritten for direct register access.

---

## File Structure

```
sidboard/
├── platformio.ini       sid-cctl, sid-pbi, sid-demo
├── efuse_vdd_sdio.py    VDD_SDIO efuse check before upload (from 6502_monitor)
├── PIN_MAPPING.md       pins and register map
├── Logs.md              decisions and change log
├── src/main.cpp         bus loop (Core 1), SidTask, LogTask, setup
├── include/csidlight.h  cSIDLight audio engine (header-only)
├── 6502/                legacy PBI ROM / I: handler (not loadable on this board)
└── examples/            Commando_dmp.h, mkdump.py, players (cc65, BASIC), test_players.py
```
