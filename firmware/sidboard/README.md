# Sidboard — ESP32 PBI/SID Firmware for Atari 8-bit

Firmware for an **ESP32 (NodeMCU-32S)** expansion board connected to an Atari 8-bit computer via the **PBI (Parallel Bus Interface)** and **ECI (External Cartridge Interface)**. It provides:

- **SID 6581/8580 sound emulation** (3 square-wave voices, 22050 Hz, via ESP32 DAC)
- **ROM device handler** (`I:`) mapped at `$D800` and exposed to the Atari OS
- **Real-time bus monitoring** synchronized to the 6502 PHI2 clock
- **Zero FreeRTOS overhead** in the hot bus-sampling path

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Pin Mapping](#pin-mapping)
3. [Memory Map](#memory-map)
4. [Architecture](#architecture)
5. [SID Register Map](#sid-register-map)
6. [cSIDLight Audio Engine](#csidlight-audio-engine)
7. [PBI ROM Driver (6502)](#pbi-rom-driver-6502)
8. [Audio Circuit](#audio-circuit)
9. [Build and Flash](#build-and-flash)
10. [Usage from Atari Side](#usage-from-atari-side)
11. [Examples](#examples)
12. [File Structure](#file-structure)

---

## Hardware Requirements

| Component | Detail |
|-----------|--------|
| MCU | ESP32 (NodeMCU-32S) @ 240 MHz |
| Atari interface | PBI connector (11-bit address bus, 8-bit data bus, control lines) |
| Address decoder | 74HC138 — decodes `$D800–$DFFF` → `ROM_SEL` (Y7, active low) |
| Level translator | TXS0108 — 3.3 V ↔ 5 V for data and address lines |
| Audio output | GPIO 25 → ECI AUDIO IN on POKEY (see [Audio Circuit](#audio-circuit)) |

---

## Pin Mapping

### Control / Bus

| Signal | GPIO | Direction | Notes |
|--------|------|-----------|-------|
| PHI2 | 2 | Input | Atari 6502 phase-2 clock; rising edge triggers bus sample |
| R/W | 15 | Input | High = Read, Low = Write |
| D1XX | 5 | Input, Active Low | Page `$D1xx` chip select |
| ROM_SEL | 4 | Input, Active Low | `$D800–$DFFF` range, driven by 74HC138 Y7 |
| EXTSEL | 0 | Output, Active Low | Asserted to tell Atari that PBI device responds |

### Data Bus (bidirectional)

| Data bit | GPIO |
|----------|------|
| D0 | 18 |
| D1 | 19 |
| D2 | 21 |
| D3 | 22 |
| D4 | 23 |
| D5 | 3  |
| D6 | 26 |
| D7 | 27 |

> GPIO 3 is used for D5 instead of GPIO 25, freeing GPIO 25 for the DAC audio output.

### Address Bus (read-only)

| Address bits | GPIO |
|-------------|------|
| A0–A4 | 32–36 (GPIO_IN1) |
| A5 | 39 (bit 7 of GPIO_IN1) |
| A6–A7 | 16, 17 |
| A8 | 14 |
| A9–A10 | 12, 13 |

### Audio

| Signal | GPIO | Notes |
|--------|------|-------|
| DAC out | 25 | ESP32 DAC1; feeds ECI AUDIO IN via RC filter |
| Serial TX | 1 | Debug output only (TX-only mode) |

---

## Memory Map

```
Atari address   ESP32 handling
─────────────   ───────────────────────────────────────────────
$D100–$D11F     SID registers (emulated by cSIDLight, 32 bytes)
$D120–$D1FE     ESP32 internal registers (general purpose)
$D1FF           Control register — bit 0: 1=enable device, 0=disable+reset SID
$D800–$DFFF     ROM device handler — pbi_driver[] array (6502 code for `I:`)
```

`EXTSEL` is asserted (pulled low) any time `$D1xx` is accessed, or when PBI is enabled and `$D800–$DFFF` is accessed.

---

## Architecture

### Dual-Core Split

```
Core 0 (system)              Core 1 (dedicated)
────────────────             ─────────────────────────────────────────
loop()                       MonitorTask  (priority 10, IRAM)
 ├─ sidLight->process()       ├─ PHI2 rising-edge sync (tight polling loop)
 └─ poll pbiEventQueue        ├─ Address decode (A0–A10)
                              ├─ EXTSEL assertion
esp_timer @ 22050 Hz          ├─ Data bus drive (read cycle)
 └─ cSIDLight::_tick()        └─ Data bus sample (write cycle)
    └─ dacWrite(GPIO 25)
```

**Core 1** owns the entire bus timing path. It never calls a FreeRTOS primitive inside the hot loop; the only synchronization point with Core 0 is a 4-element `QueueHandle_t` that carries PBI enable/disable events (written with `xQueueSend(..., 0)` — non-blocking).

**Cross-core shared state** (`_phaseInc[]`, `_voiceVol[]`) is declared `volatile uint32_t / int32_t`. On Xtensa LX6, DRAM is not cached per-core, so 32-bit aligned volatile stores are immediately visible across cores without explicit barriers or spinlocks.

### MonitorTask Flow (Core 1)

```
for (;;) {
    wait PHI2 rising edge          ← tight polling, no interrupt latency
    snapshot GPIO.in / GPIO.in1
    decode A0–A10
    assert EXTSEL if $D1xx or ROM

    if R/W == Read:
        drive data bus from SID / ROM / internal regs
        wait PHI2 falling edge
        release data bus (High-Z)
        release EXTSEL

    if R/W == Write:
        wait PHI2 falling edge     ← data bus stable on falling edge
        sample data bus
        dispatch: $D1FF → PBI enable queue
                  $D100–$D11F → sidLight->write()
                  $D120–$D1FE → esp32_internal_regs[]
        release EXTSEL
}
```

### Data Bus LUT

`data_set_lut[256]` is precomputed at boot: each entry holds the GPIO bitmask for the 8 data pins corresponding to a given byte value. This converts a byte write to the bus into an O(1) indexed bitmask lookup, avoiding bit-scatter at runtime.

---

## SID Register Map

Registers are mapped at `$D100` (offset 0x00) through `$D11F` (offset 0x1F).

| Offset | Register | Description |
|--------|----------|-------------|
| `$00/$01` | V1 Freq Lo/Hi | Voice 1 frequency (16-bit) |
| `$02/$03` | V1 PW Lo/Hi | Voice 1 pulse width (not emulated) |
| `$04` | V1 CTRL | Voice 1: bit 0 = GATE, bits 4–7 = waveform select |
| `$05/$06` | V1 AD/SR | Voice 1 ADSR (not emulated) |
| `$07–$0D` | V2 | Voice 2 (same layout as V1) |
| `$0E–$14` | V3 | Voice 3 (same layout as V1) |
| `$15/$16` | FC Lo/Hi | Filter cutoff (not emulated) |
| `$17` | Res/Filt | Filter resonance / routing (not emulated) |
| `$18` | MODE/VOL | Bits 3:0 = master volume (0–15) |

### Control Register (`$D1FF`)

| Bit | Function |
|-----|----------|
| 0 | `1` = Enable PBI device and ROM. `0` = Disable + silence all SID voices. |

---

## cSIDLight Audio Engine

Defined in `include/csidlight.h`. Self-contained — no external audio library.

### Parameters

| Parameter | Value |
|-----------|-------|
| Sample rate | 22050 Hz |
| Timer period | ~45 µs (via `esp_timer`) |
| Output pin | GPIO 25 (ESP32 DAC1) |
| Voices | 3 independent square-wave oscillators |
| Emulated | Frequency, GATE, master volume |
| Not emulated | ADSR envelope, waveform selection, filter, ring-mod, sync |

### Frequency Formula (PAL)

```
freq_hz   = (reg16 × 985248) / 2^24
phaseInc  = (freq_hz × 2^32) / 22050
```

At each 22050 Hz tick:

```
phase[i] += phaseInc[i]
square    = (phase[i] & 0x80000000) ? +voiceVol[i] : -voiceVol[i]
mix       = clamp(128 + Σ square[i], 0, 255)
dacWrite(25, mix)
```

### Master Volume

```
voiceVol = (reg[0x18] & 0x0F) × (128 / 48)   // ≤ 42 per voice; 3 × 42 = 126 ≤ 128
```

### API Surface

| Method | Caller | Notes |
|--------|--------|-------|
| `write(reg, val)` | MonitorTask (Core 1, IRAM) | Single `volatile` byte store — no locking |
| `read(reg)` | MonitorTask (Core 1, IRAM) | Single `volatile` byte load |
| `process()` | `loop()` (Core 0) | Compares register snapshot, updates synthesis params |
| `reset()` | `loop()` on PBI disable | Zeroes all registers, silences voices |
| `_tick()` | `esp_timer` (Core 0) | Generates one audio sample, writes DAC |

---

## PBI ROM Driver (6502)

Source: `6502/src/pbi-driver.s`  
Binary included in firmware as: `include/pbi-driver.h` → `pbi_driver[]`

The driver registers an Atari CIO device named **`I:`** (device number `I`). It is exposed at `$D800` and the Atari OS loads it automatically when PBI is enabled.

### CIO Routines

| Routine | AUX registers | Behavior |
|---------|---------------|----------|
| OPEN | `AUX1` = mode | Registers device `I:` with Atari OS |
| CLOSE | — | Writes 0 to all 32 SID registers (`$D100–$D11F`) — silences SID |
| GETBYT (GET) | `ICAX1` = register offset | Reads `$D100 + offset` → `A` |
| PUTBYT (PUT) | `A` = value, `ICAX1` = offset | Writes `A` → `$D100 + offset` |

### Build the 6502 Driver

```bash
cd 6502
make
```

Requires `ca65` / `ld65` (cc65 toolchain). The build produces `pbi-driver.h` with the binary as a C array.

---

## Audio Circuit

GPIO 25 is the ESP32 DAC1 output (0–3.3 V, 8-bit resolution).

**PCB modification required:** the default BOboard PCB routes GPIO 25 through the TXS0108 level translator, which is incompatible with DAC use. Cut the PCB trace between GPIO 25 and pin A of the TXS0108, and add a 10 kΩ pull-down on the now-floating pin A side.

**Signal path to Atari POKEY ECI:**

```
GPIO 25 (DAC)
    │
   10 µF  ──── DC block capacitor
    │
  100 kΩ  ──── into ECI AUDIO IN (POKEY pin 3)
                │
               10 nF ─── GND      (low-pass, fc ≈ 1.75 kHz)
```

The SID signal mixes directly into the POKEY internal audio mixer; the Atari speaker output carries both POKEY and SID audio.

---

## Build and Flash

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or IDE plugin)
- ESP32 platform: `espressif32 @ 6.6.0`

### Build

```bash
pio run -e nodemcu-32s
```

### Flash

```bash
pio run -e nodemcu-32s -t upload
```

Default upload speed: 921600 baud. Adjust `upload_port` in `platformio.ini` if needed.

### Monitor

```bash
pio device monitor -b 115200
```

Serial output is TX-only (GPIO 1). Do not connect an external serial RX to GPIO 3 (it is used as data bus D5).

### Build Flags

| Flag | Purpose |
|------|---------|
| `-O3` | Maximum optimization |
| `-Wno-shift-count-overflow` | Suppress benign GPIO bitmask shift warnings |
| `-D USE_ESP32_REGISTER_ACCESS` | Enable direct `GPIO.out_w1ts` / `GPIO.in` register access |

---

## Usage from Atari Side

### Enable the Device

Write `$01` to `$D1FF`:

```
POKE 53759, 1   ; $D1FF — enable PBI device and ROM
```

After this, `OPEN #n, 8, 0, "I:"` will open the SID device handler.

### Disable the Device

```
POKE 53759, 0   ; $D1FF — disable, silence SID
```

### Write a SID Register (BASIC)

```basic
OPEN #1, 8, 0, "I:"       : REM open device
POKE 842, 0               : REM ICAX1 = register 0 (V1 Freq Lo)
PUT #1, 100               : REM write 100 to reg 0
CLOSE #1
```

### Register Access (cc65 C)

```c
#include <atari.h>

#define SID_BASE 0xD100

OS.iocb[1].buffer  = "I:";
OS.iocb[1].command = IOCB_OPEN;
OS.iocb[1].aux1    = 8;
ciov();

/* write one register */
OS.iocb[1].aux1    = 0x04;           /* V1 CTRL register */
OS.iocb[1].command = IOCB_PUTCHR;
OS.iocb[1].buffer  = (void*)0x11;   /* GATE on, square wave */
ciov();
```

---

## Examples

### `examples/cc65/sid_dump_player.c`

Plays a raw SID register dump (25 bytes per frame, PAL 50 Hz) using the `I:` device via CIO. Each frame writes registers 0–24 in order and then waits for vertical blank.

Build:

```bash
cd examples/cc65
make
```

Requires cc65 and the Atari target library.

### `examples/atari-basic/sid_dump_player.bas`

Atari BASIC version of the same SID dump player. Load from disk and run.

---

## File Structure

```
sidboard/
├── platformio.ini              Build configuration (ESP32, 240 MHz, -O3)
├── Logs.md                     Architecture decisions and change log
├── src/
│   └── main.cpp               MonitorTask, setup(), loop(), LUT, GPIO config
├── include/
│   ├── csidlight.h            cSIDLight class — audio engine (header-only)
│   └── pbi-driver.h           pbi_driver[] — 6502 ROM binary as C array
├── 6502/
│   ├── src/pbi-driver.s       6502 assembly — CIO device handler for I:
│   ├── pbi-driver.ld          Linker script (origin $D800)
│   └── Makefile               Builds pbi-driver.h from assembly
└── examples/
    ├── cc65/
    │   ├── sid_dump_player.c   cc65 SID dump player (CIO I: device)
    │   └── Makefile
    └── atari-basic/
        └── sid_dump_player.bas Atari BASIC SID dump player
```
