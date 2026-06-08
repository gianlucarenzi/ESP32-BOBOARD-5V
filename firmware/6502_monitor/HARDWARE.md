# VeraX16 PBI — Hardware Reference

## 1. Overview

The ESP32 interfaces the Atari 800XL PBI (Parallel Bus Interface) to:

- serve a 2 KB ROM image at **$D800–$DFFF** (PBI device ROM)
- respond to register reads/writes at **$D100–$D1FF** (D1xx page)
- forward VERA chip accesses ($D100–$D11F) via **DEV_SEL_N**
- control the Atari floating-point ROM via **EXTSEL_N**
- emulate **512 bytes of RAM** at **$D600–$D7FF** (Mirroring 256 bytes)

All 5 V Atari signals pass through **TXS0108E** bidirectional level translators
before reaching the 3.3 V ESP32 GPIOs.  The $D800–$DFFF chip-select is
generated in hardware by a **74HC138** 3-to-8 decoder.

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
| **Read data setup before PHI2↓** | **tDSR** | **≥ 100 ns** | **Critical — ROM must drive bus in time** |
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
