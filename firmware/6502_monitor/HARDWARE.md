# 6502 Monitor — Hardware Reference

## 1. Overview

The ESP32 behaves as a ROM-less peripheral on the Atari 800XL/130XE bus:

- register windows **$D100–$D1FF** (`~D1XX`) and **$D500–$D5FF** (`~CCTL`), both
  decoded by the Atari MMU / PLA; only **A0–A7** are sampled
- select latch per window at **$D1FF** / **$D5FF** (bit 7, device ID `$80`);
  while selected the device stores writes and answers reads of `$xx00–$xxFE`
- never drives the bus when not selected; no ROM, no **MPD**, no **EXTSEL**
- logs every access to both windows with microsecond timestamps via a
  lock-free ring buffer (Core 1 → Core 0)

All 5 V Atari signals pass through **TXS0108E** bidirectional level translators
before reaching the 3.3 V ESP32 GPIOs; D0–D7 are driven back through them on
reads.  The board has no address decoder: `~D1XX` (GPIO 22) and `~CCTL`
(GPIO 23) arrive on J3 pins 11 and 12 through U3.  `~D1XX` is not
PHI2-qualified: the firmware decodes during PHI1.

---

## 2. Atari 6502 Bus Timing

### Clock source

| Parameter | Value |
|---|---|
| NTSC colour crystal | 14.31818 MHz |
| CPU (SALLY) clock | 14.31818 ÷ 8 = **1.7897 MHz** |
| Full bus cycle | ≈ **558.7 ns** |
| PHI2 HIGH period | ≈ **279 ns** |
| PHI2 LOW period | ≈ **279 ns** |

### Key 6502 timing parameters (65C02 / SALLY, worst-case)

| Parameter | Symbol | Value | Notes |
|---|---|---|---|
| Address valid after PHI2↓ | tADS | ≤ 300 ns | Address bus stable early in PHI2-LOW |
| Address hold after PHI2↓ | tAH | ≥ 10 ns | |
| **Read data setup before PHI2↓** | **tDSR** | **≥ 100 ns** | **Critical — the device must drive the bus in time** |
| Read data hold after PHI2↓ | tDHR | ≥ 10 ns | |
| Write data valid after PHI2↑ | tMDS | ≤ 225 ns (typ. 80–100 ns) | CPU drives data ~80 ns after PHI2↑ |
| Write data hold after PHI2↓ | tDHW | ≥ 10 ns | |

### Read cycle window

```
         PHI2
          ___________
_________|           |___________
         ^           ^
         PHI2↑       PHI2↓
         |           |
         |<- 279 ns ->|
```

---

## 3. ESP32 Strapping Pins

The ESP32 ROM samples the strapping pins at every chip reset (power-on, EN,
brown-out, watchdog), before any firmware runs.  On this board the TXS0108E
translators are always enabled (`TXS_EN` pulled up by R1/R3/R4, SW1 only
disables them by hand), so at reset the strapping pins see the Atari bus.
The module is always programmed on its own, connected only to the PC.

| GPIO | Signal | Strapping function | On the Atari bus |
|---|---|---|---|
| 12 (MTDI) | unused (still wired to U1 / J3 pin 4, former A8) | VDD_SDIO: HIGH = 1.8 V flash | **Fatal** if J3 pin 4 still carries A8 or the TXS0108E pull-up holds it HIGH: the WROOM-32 flash needs 3.3 V, with 1.8 V the ESP32 does not boot.  Fixed by the VDD_SDIO efuses (below). |
| 0 | unused | Boot mode: LOW = download | Not routed on the 30-pin DevKit headers: the NodeMCU pull-up keeps it HIGH, SPI boot. |
| 2 | PHI2 | Download mode needs it LOW | Ignored while GPIO0 is HIGH. |
| 15 (MTDO) | R/W | LOW = no ROM boot messages | Harmless. |
| 5 | D1 | SDIO slave timing | Harmless (SDIO unused). |

All bus pins are inputs until the bus loop drives D0–D7 for a selected read,
so nothing is driven during the start-up.

**VDD_SDIO efuses.**  `XPD_SDIO_FORCE = 1` (ignore GPIO12),
`XPD_SDIO_REG = 1` (regulator on), `XPD_SDIO_TIEH = 1` (3.3 V), i.e.
`espefuse.py set_flash_voltage 3.3V`.  Irreversible, and the right setting for
the WROOM-32.  `efuse_vdd_sdio.py` checks them before every upload and burns
them when they are blank.

**Serial monitor.**  On the NodeMCU, RTS and DTR drive EN and GPIO0 through
the auto-reset transistors: `monitor_rts = 0` / `monitor_dtr = 0` keep the
monitor from resetting the ESP32 while it sits on the Atari bus.
