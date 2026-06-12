# Sidboard — ESP32 CCTL/SID Firmware for Atari 8-bit

Firmware for an **ESP32 (NodeMCU-32S)** expansion board connected to an Atari 8-bit computer via the **CCTL (Cartridge Control Line)**. It provides:

- **SID 6581/8580 sound emulation** (3 square-wave voices, 22050 Hz, via ESP32 DAC1 on GPIO 25)
- **Real-time bus monitoring** synchronized to the 6502 PHI2 clock
- **Timestamped serial log** of every register access and latch event
- **Zero FreeRTOS overhead** in the hot bus-sampling path

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Pin Mapping](#pin-mapping)
3. [Memory Map](#memory-map)
4. [Architecture](#architecture)
5. [SID Register Map](#sid-register-map)
6. [cSIDLight Audio Engine](#csidlight-audio-engine)
7. [Audio Circuit](#audio-circuit)
8. [Build and Flash](#build-and-flash)
9. [Usage from Atari Side](#usage-from-atari-side)
10. [File Structure](#file-structure)

---

## Hardware Requirements

| Component | Detail |
|-----------|--------|
| MCU | ESP32 (NodeMCU-32S) @ 240 MHz |
| Atari interface | Cartridge slot — CCTL line ($D5xx), 5-bit address bus, 8-bit data bus |
| Level translator | TXS0108 — 3.3 V ↔ 5 V for data and address lines |
| Audio output | GPIO 25 (DAC1) → audio mixer via RC filter |

---

## Pin Mapping

> Full detail including free GPIOs and physical layout: see **[PIN_MAPPING.md](PIN_MAPPING.md)**.

### Control Signals

| Signal | GPIO | Direction | Description |
|--------|------|-----------|-------------|
| PHI2 | 2 | Input | 6502 phase-2 clock (1.79 MHz PAL); rising edge triggers bus sample |
| R/W | 15 | Input | High = Read, Low = Write |
| SEL_N | 22 | Input, Active Low | CCTL — asserted by Atari for $D5xx page accesses |

### Data Bus (bidirectional)

| Bit | GPIO |
|-----|------|
| D0 | 4 |
| D1 | 5 |
| D2 | 13 |
| D3 | 14 |
| D4 | 16 |
| D5 | 17 |
| D6 | 18 |
| D7 | 19 |

### Address Bus (input only, A0–A4)

| Bit | GPIO | GPIO_IN1 bit |
|-----|------|-------------|
| A0 | 34 | bit 2 |
| A1 | 35 | bit 3 |
| A2 | 36 | bit 4 |
| A3 | 39 | bit 7 |
| A4 | 32 | bit 0 |

Only 5 address bits are decoded (32 locations). A5–A10 are not wired.

### Audio and Debug

| Signal | GPIO | Direction | Description |
|--------|------|-----------|-------------|
| DAC1 | 25 | Output | SID audio — 8-bit, 22050 Hz, mono |
| TX | 1 | Output | Serial debug, 115200 baud, TX-only |

---

## Memory Map

```
CCTL active ($D5xx, SEL_N low) — A0-A4 decoded to 5-bit offset:

Offset   Full address   Description
──────   ────────────   ──────────────────────────────────────────────────
0x00     $D500          SID register 0  (V1 Freq Lo)
 …        …              …
0x1E     $D51E          SID register 30 (unused)
0x1F     $D5FF *        Latch control — bit 0: 1=enable, 0=disable+reset
```

\* $D5FF is decoded as offset 0x1F because only A0–A4 are wired (all five bits high).  
SID registers are accessible for reads and writes only when the latch is enabled.

---

## Architecture

### Dual-Core Split

```
Core 0 (system)                  Core 1 (dedicated)
────────────────                 ─────────────────────────────────────────
loop()                           MonitorTask  (priority 10, IRAM)
 ├─ sidLight->process()           ├─ PHI2 rising-edge sync (tight polling)
 ├─ drain latchEventQueue         ├─ SEL_N (CCTL) detection
 └─ drain log_queue → Serial      ├─ A0-A4 address decode
                                  ├─ Data bus drive (read cycle)
esp_timer @ 22050 Hz             └─ Data bus sample (write cycle)
 └─ cSIDLight::_tick()
    └─ dacWrite(GPIO 25)
```

**Core 1** owns the entire bus timing path. No FreeRTOS primitives are called inside the hot loop — the only inter-core communication is two non-blocking `xQueueSend(..., 0)` calls: one for latch state changes, one for the log queue.

**Cross-core shared state** (`_regs[]`) is declared `volatile uint8_t`. On Xtensa LX6, DRAM is not cached per-core, so volatile byte stores from Core 1 are immediately visible to Core 0 without explicit barriers.

**Log drain** dequeues at most **one entry per `loop()` iteration**. `Serial.printf()` at 115200 baud blocks ~3.5 ms per line; draining all 64 entries in a tight loop would stall `sidLight->process()` for ~220 ms, delaying SID register propagation to the audio engine. The 64-entry queue absorbs bursts while `process()` runs at full loop rate.

### MonitorTask Flow (Core 1)

```
for (;;) {
    wait PHI2 rising edge          ← tight polling loop, no interrupt latency
    if SEL_N not asserted → continue

    decode A0-A4 from GPIO_IN1     ← 5-bit offset (0x00-0x1F)

    if R/W == Read:
        if latch enabled and offset < 0x1F:
            drive data bus from sidLight->read(offset)
        wait PHI2 falling edge
        release data bus (High-Z)

    if R/W == Write:
        wait PHI2 falling edge     ← data stable on falling edge
        sample data bus
        if offset == 0x1F:         → update latch, notify Core 0
        else if latch enabled:     → sidLight->write(offset, data)
}
```

### Data Bus LUT

`data_set_lut[256]` is precomputed at boot in IRAM: each entry holds the GPIO bitmask for the 8 data pins corresponding to a given byte value. Driving the bus on a read is an O(1) indexed bitmask lookup with no bit-scatter at runtime.

---

## SID Register Map

Registers are accessible at `$D500` (offset `0x00`) through `$D51E` (offset `0x1E`).  
Offset `0x1F` ($D5FF) is the latch control register, not a SID register.

| Offset | Address | Register | Description |
|--------|---------|----------|-------------|
| `$00/$01` | `$D500/$D501` | V1 Freq Lo/Hi | Voice 1 frequency (16-bit) |
| `$02/$03` | `$D502/$D503` | V1 PW Lo/Hi | Voice 1 pulse width (not emulated) |
| `$04` | `$D504` | V1 CTRL | bit 0 = GATE, bit 6 = SQR wave |
| `$05/$06` | `$D505/$D506` | V1 AD/SR | Voice 1 ADSR (not emulated) |
| `$07–$0D` | `$D507–$D50D` | V2 | Voice 2 (same layout as V1) |
| `$0E–$14` | `$D50E–$D514` | V3 | Voice 3 (same layout as V1) |
| `$15/$16` | `$D515/$D516` | FC Lo/Hi | Filter cutoff (not emulated) |
| `$17` | `$D517` | Res/Filt | Filter resonance / routing (not emulated) |
| `$18` | `$D518` | MODE/VOL | Bits 3:0 = master volume (0–15) |
| `$19/$1A` | `$D519/$D51A` | OSC3/ENV3 | Voice 3 readouts |
| `$1B–$1E` | `$D51B–$D51E` | — | Unused |
| **`$1F`** | **`$D5FF`** | **LATCH** | **bit 0: 1=enable SID, 0=disable+reset** |

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
| `write(reg, val)` | MonitorTask (Core 1, IRAM) | Single `volatile` byte store — no locking |
| `read(reg)` | MonitorTask (Core 1, IRAM) | Single `volatile` byte load |
| `process()` | `loop()` (Core 0) | Snapshot diff → update synthesis params |
| `reset()` | `loop()` on latch disable | Zero all registers, silence voices |
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

Serial output is TX-only on GPIO 1. Example log output:

```
[sidboard] CCTL/SID Firmware Ready (TX-Only)
[sidboard] cSIDLight initialized, DAC1 on GPIO 25
[sidboard] MonitorTask running on Core 1.
>>> Latch ENABLED
[    0.123456] [LATCH] $D5FF ENABLED  ($01)
[    0.123500] [D500 - V1_FREQ_LO  ] W $3C
[    0.123501] [D504 - V1_CTRL     ] W $11
```

### Build Flags

| Flag | Purpose |
|------|---------|
| `-O3` | Maximum optimization |
| `-Wno-shift-count-overflow` | Suppress benign GPIO bitmask shift warnings |
| `-D USE_ESP32_REGISTER_ACCESS` | Enable direct `GPIO.out_w1ts` / `GPIO.in` register access |

---

## Usage from Atari Side

SID registers are directly accessible via POKE/PEEK in the $D5xx range. The latch must be enabled first.

### Enable the SID

Write `$01` to `$D5FF` ($54783):

```
POKE 54783, 1   ; $D5FF — enable SID
```

### Disable the SID

```
POKE 54783, 0   ; $D5FF — disable and silence SID
```

### Write a SID Register (Atari BASIC)

```basic
POKE 54783, 1        : REM enable SID
POKE 54528, 100      : REM $D500 — V1 Freq Lo = 100
POKE 54529, 10       : REM $D501 — V1 Freq Hi = 10
POKE 54532, 17       : REM $D504 — V1 CTRL: GATE on + SQR wave ($11)
POKE 54552, 15       : REM $D518 — MODE/VOL: max volume
```

### Read a SID Register (Atari BASIC)

```basic
POKE 54783, 1
X = PEEK(54532)      : REM read V1 CTRL ($D504)
```

### Write a SID Register (cc65 C)

```c
#define SID_BASE  0xD500
#define SID_LATCH 0xD5FF

/* enable */
*(volatile uint8_t *)SID_LATCH = 0x01;

/* set voice 1 frequency and gate */
*(volatile uint8_t *)(SID_BASE + 0x00) = 0x3C;  /* Freq Lo */
*(volatile uint8_t *)(SID_BASE + 0x01) = 0x0A;  /* Freq Hi */
*(volatile uint8_t *)(SID_BASE + 0x04) = 0x11;  /* CTRL: GATE + SQR */
*(volatile uint8_t *)(SID_BASE + 0x18) = 0x0F;  /* MODE/VOL: max */
```

---

## File Structure

```
sidboard/
├── platformio.ini              Build configuration (ESP32, 240 MHz, -O3)
├── PIN_MAPPING.md              Full ESP32 ↔ Atari bus pin assignment table
├── Logs.md                     Architecture decisions and change log
├── README.md                   This file
├── src/
│   └── main.cpp               MonitorTask, setup(), loop(), LUT, GPIO config
├── include/
│   └── csidlight.h            cSIDLight class — audio engine (header-only)
├── 6502/
│   ├── src/pbi-driver.s       Legacy 6502 CIO device handler (not loaded)
│   ├── pbi-driver.ld          Linker script
│   └── Makefile               Builds pbi-driver.h from assembly
└── examples/
    ├── cc65/
    │   ├── sid_dump_player.c   SID dump player (direct POKE, adapt for $D5xx)
    │   └── Makefile
    └── atari-basic/
        └── sid_dump_player.bas BASIC SID dump player (adapt for $D5xx)
```
