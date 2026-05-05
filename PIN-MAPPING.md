# ESP32-BOBOARD-5V — Pin Mapping

**Hardware**: ESP32-BOBOARD-5V (NodeMCU-32S + 3× TXS0108EPW level shifters)  
**Atari target**: 800XL PBI (Parallel Bus Interface, 32-pin 2×16 edge connector)  
**MCU**: ESP32 (3.3 V) — GPIOs 6–11 reserved for internal SPI flash, NOT available.

---

## J3 — Atari PBI Connector (5 V, 32-pin 2×16)

J3 is the PCB edge connector that plugs directly into the Atari PBI slot.  
All signals cross through a TXS0108EPW bidirectional level shifter (5 V Atari ↔ 3.3 V ESP32).

### Signal directions

| Direction         | Signals                            |
| ----------------- | ---------------------------------- |
| Atari → ESP32     | A0–A15, D0–D7 (read), PHI2, R/~W, CCTL, ~D1XX |
| ESP32 → Atari     | D0–D7 (write), ~MPD, ~EXSEL       |
| ESP32 → Atari     | ~CS (chip select for ext. device)  |

### Atari PBI decoded signals

The Atari address decoder asserts these lines automatically — the ESP32 does **not**
need to software-decode A8–A15 for the standard PBI regions:

| Signal  | Dir       | Active | Meaning                                    |
| ------- | --------- | ------ | ------------------------------------------ |
| `CCTL`  | Atari→ESP32 | LOW  | CPU accessing **$D500–$D5FF** (cartridge control) |
| `~D1XX` | Atari→ESP32 | LOW  | CPU accessing **$D100–$D1FF** (PBI I/O)    |
| `~MPD`  | ESP32→Atari | LOW  | ESP32 disables Atari Math Pack ROM ($D800–$DFFF) |
| `~EXSEL`| ESP32→Atari | LOW  | ESP32 claims external memory ($D600–$DFFF) |

Using these signals the ESP32 **only needs A0–A7** to determine the register offset
within each decoded region. A8–A11 remain useful for $D6xx/$D7xx/$D8xx discrimination
when no decoded signal is active.

---

## Prototype (v1.x) — NodeMCU-32S, `PROTOTYPE=1`

### Current pin assignment

| J3 net | ESP32 GPIO | IN_REG bit | Function         | Status    |
| ------ | ---------- | ---------- | ---------------- | --------- |
| mD2    | GPIO2      | bit 2      | A8               | ✅ connected |
| mD4    | GPIO4      | bit 4      | **D0** (data)    | ✅ connected |
| mD5    | GPIO5      | bit 5      | A9               | ✅ connected |
| mD12   | GPIO12     | bit 12     | A10              | ✅ connected |
| mD13   | GPIO13     | bit 13     | **D1** (data)    | ✅ connected |
| mD14   | GPIO14     | bit 14     | **D2** (data)    | ✅ connected |
| mD15   | GPIO15     | bit 15     | A11              | ✅ connected |
| mRX2   | GPIO16     | bit 16     | **D3** (data)    | ✅ connected |
| mTX2   | GPIO17     | bit 17     | **D4** (data)    | ✅ connected |
| mD18   | GPIO18     | bit 18     | **D5** (data)    | ✅ connected |
| mD19   | GPIO19     | bit 19     | **D6** (data)    | ✅ connected |
| mD21   | GPIO21     | bit 21     | A2               | ✅ connected |
| mD22   | GPIO22     | bit 22     | **D7** (data)    | ✅ connected |
| mD23   | GPIO23     | bit 23     | PHI2             | ✅ connected |
| mD25   | GPIO25     | bit 25     | R/~W             | ✅ connected |
| mD26   | GPIO26     | bit 26     | ~CS (FPGA/VERA)  | ✅ connected |
| mD27   | GPIO27     | bit 27     | A3               | ✅ connected |
| mD32   | GPIO32     | IN1 bit 0  | A5               | ✅ connected |
| mD33   | GPIO33     | IN1 bit 1  | A4               | ✅ connected |
| mD34   | GPIO34 ⁽¹⁾ | IN1 bit 2 | A12              | ✅ connected |
| mD35   | GPIO35 ⁽¹⁾ | IN1 bit 3 | A13              | ✅ connected |
| mVP    | GPIO36 ⁽¹⁾ | IN1 bit 4 | A14              | ✅ connected |
| mVN    | GPIO39 ⁽¹⁾ | IN1 bit 7 | A15              | ✅ connected |
| —      | —          | —          | J3 pin 24 (NC)   | ❌ not wired |
| —      | —          | —          | J3 pin 25 (NC)   | ❌ not wired |

⁽¹⁾ Input-only GPIO on ESP32 — cannot be driven as output.

### Signals missing on prototype (GPIOs 6–11 / 37–38 not available on NodeMCU-32S)

| Signal   | Proto GPIO | Reason unavailable             |
| -------- | ---------- | ------------------------------ |
| A0       | GPIO6      | SPI flash pin                  |
| A1       | GPIO8      | SPI flash pin                  |
| A6       | GPIO38     | Not broken out on NodeMCU-32S  |
| A7       | GPIO37     | Not broken out on NodeMCU-32S  |
| CCTL     | GPIO20     | Not present on standard ESP32  |
| ~D1XX    | GPIO10     | SPI flash pin                  |
| ~MPD     | GPIO7      | SPI flash pin                  |
| ~EXSEL   | GPIO11     | SPI flash pin                  |

### Problems with current data bus layout

The 8 data bits are spread across **five separate bit positions** in `GPIO_IN_REG`
(bits 4, 13, 14, 16, 17, 18, 19, 22) with no contiguous 8-bit window.
Reading and writing the data bus requires a 256-entry LUT and 8 conditional branches.

```c
// Prototype: read requires 8 individual bit extracts
uint8_t data = (((gpio_low >>  4) & 1) << 0) |
               (((gpio_low >> 13) & 1) << 1) |
               ...
               (((gpio_low >> 22) & 1) << 7);

// Prototype: write uses a precomputed 256-entry LUT
uint32_t set_mask = data_set_lut[value];
GPIO.out_w1ts = set_mask;
GPIO.out_w1tc = DATA_FULL_MASK & ~set_mask;
```

---

## Final Board (v2.x) — Optimized Pin Assignment

### Design goals

1. **Data bus D0–D7 on GPIO 12–19** — single contiguous 8-bit window in `GPIO_IN_REG`
   (bits 12–19). One shift + mask to read or write the full byte.
2. **PHI2, R/~W, ~MPD, ~EXSEL adjacent in IN_REG** (bits 23, 25, 26, 27) for fast
   access and logical grouping.
3. **Address bus A0–A5 contiguous** (GPIO 0–5, IN_REG bits 0–5) + A6–A7 adjacent
   (GPIO 21–22, bits 21–22). Two masks to reassemble the full low byte.
4. **CCTL and ~D1XX on GPIO 34–35** (input-only, currently wasted on A12/A13).
   A12–A15 are unnecessary when decoded signals are used; keeping A8–A9 (GPIO32/33)
   is sufficient for $D6xx/$D7xx/$D8xx discrimination.
5. **One TXS0108EPW per functional group** — all 8 data bits in U1, all 8 address
   low-byte bits in U2, all control/decoded signals in U3.

### Proposed pin assignment

| J3 net | ESP32 GPIO | Register      | bit(s) | Function         | Notes |
| ------ | ---------- | ------------- | ------ | ---------------- | ----- |
| —      | GPIO0  ⁽²⁾ | IN_REG bit 0  | 0      | **A0**           | strapping pin, needs 10 kΩ pull-up |
| —      | GPIO1  ⁽³⁾ | IN_REG bit 1  | 1      | **A1**           | UART0 TX — disable console or use USB-CDC |
| —      | GPIO2      | IN_REG bit 2  | 2      | **A2**           | |
| —      | GPIO3  ⁽³⁾ | IN_REG bit 3  | 3      | **A3**           | UART0 RX — disable console or use USB-CDC |
| —      | GPIO4      | IN_REG bit 4  | 4      | **A4**           | freed from D0 |
| —      | GPIO5      | IN_REG bit 5  | 5      | **A5**           | |
| **U1** | **GPIO12** | **IN_REG**    | **12** | **D0**           | data bus — contiguous block |
| **U1** | **GPIO13** | **IN_REG**    | **13** | **D1**           | |
| **U1** | **GPIO14** | **IN_REG**    | **14** | **D2**           | |
| **U1** | **GPIO15** | **IN_REG**    | **15** | **D3**           | freed from A11 |
| **U1** | **GPIO16** | **IN_REG**    | **16** | **D4**           | |
| **U1** | **GPIO17** | **IN_REG**    | **17** | **D5**           | |
| **U1** | **GPIO18** | **IN_REG**    | **18** | **D6**           | |
| **U1** | **GPIO19** | **IN_REG**    | **19** | **D7**           | freed from D6 |
| —      | GPIO21     | IN_REG bit 21 | 21     | **A6**           | freed from A2 |
| —      | GPIO22     | IN_REG bit 22 | 22     | **A7**           | freed from D7 |
| —      | GPIO23     | IN_REG bit 23 | 23     | **PHI2**         | timing-critical |
| —      | GPIO25     | IN_REG bit 25 | 25     | **R/~W**         | timing-critical |
| —      | GPIO26     | IN_REG bit 26 | 26     | **~MPD** (OUT)   | drive LOW → disable Atari Math ROM |
| —      | GPIO27     | IN_REG bit 27 | 27     | **~EXSEL** (OUT) | drive LOW → claim external memory |
| —      | GPIO32     | IN1_REG bit 0 | 0      | **A8**           | needed for D6xx/D7xx/D8xx |
| —      | GPIO33     | IN1_REG bit 1 | 1      | **A9**           | needed for D8xx/D9xx |
| —      | GPIO34 ⁽¹⁾ | IN1_REG bit 2 | 2      | **CCTL** (IN)    | replaces A12 — decoded $D5xx |
| —      | GPIO35 ⁽¹⁾ | IN1_REG bit 3 | 3      | **~D1XX** (IN)   | replaces A13 — decoded $D1xx |
| —      | GPIO36 ⁽¹⁾ | IN1_REG bit 4 | 4      | A10 (optional)   | or free |
| —      | GPIO39 ⁽¹⁾ | IN1_REG bit 7 | 7      | A11 (optional)   | or free |

⁽¹⁾ Input-only.  ⁽²⁾ Strapping pin.  ⁽³⁾ UART0 — see note below.

### Level shifter allocation (v2.x)

| IC | Channels                        | Functional group            |
| -- | ------------------------------- | --------------------------- |
| U1 | GPIO12–19                       | Data bus D0–D7 (bidirectional) |
| U2 | GPIO0–5, GPIO21, GPIO22         | Address bus A0–A7 (input to ESP32) |
| U3 | GPIO23,25,26,27, GPIO32,33,34,35 | Control: PHI2, R/W, ~MPD, ~EXSEL, A8, A9, CCTL, ~D1XX |

All 24 level-shifter channels are used (3 × TXS0108EPW × 8 channels).

---

## Performance Comparison

### Data bus read

```c
// ── PROTOTYPE (v1.x) ─────────────────────────────────────────────
// 8 independent bit extracts from 5 scattered positions in IN_REG
uint8_t data = (((gpio_low >>  4) & 1) << 0) |
               (((gpio_low >> 13) & 1) << 1) |
               (((gpio_low >> 14) & 1) << 2) |
               (((gpio_low >> 16) & 1) << 3) |
               (((gpio_low >> 17) & 1) << 4) |
               (((gpio_low >> 18) & 1) << 5) |
               (((gpio_low >> 19) & 1) << 6) |
               (((gpio_low >> 22) & 1) << 7);

// ── FINAL BOARD (v2.x) ───────────────────────────────────────────
// Single shift + mask — 2 instructions
uint8_t data = (gpio_low >> 12) & 0xFF;
```

### Data bus write

```c
#define DATA_MASK 0x000FF000UL   // bits 12–19

// ── PROTOTYPE (v1.x): 256-entry LUT ──────────────────────────────
GPIO.out_w1ts = data_set_lut[value];
GPIO.out_w1tc = DATA_FULL_MASK & ~data_set_lut[value];

// ── FINAL BOARD (v2.x): shift only, no LUT ───────────────────────
uint32_t set = (uint32_t)value << 12;
GPIO.out_w1ts = set;
GPIO.out_w1tc = DATA_MASK & ~set;
```

### Address low byte read

```c
// ── PROTOTYPE (v1.x) ─────────────────────────────────────────────
// A2–A5 from two registers, A8–A15 from two registers, A0/A1/A6/A7 MISSING
uint16_t address =
    (((gpio_low  >> 21) & 1) << 2) |   // A2
    (((gpio_low  >> 27) & 1) << 3) |   // A3
    (((gpio_high >>  1) & 1) << 4) |   // A4 (GPIO33, IN1_REG)
    (((gpio_high >>  0) & 1) << 5) |   // A5 (GPIO32, IN1_REG)
    (((gpio_low  >>  2) & 1) << 8) |   // A8
    // ... A9-A15 similarly scattered

// ── FINAL BOARD (v2.x) ───────────────────────────────────────────
// A0–A5 contiguous in bits 0–5; A6–A7 adjacent in bits 21–22
uint8_t addr_lo = (gpio_low & 0x3F)                // A0–A5 direct
                | (((gpio_low >> 21) & 0x03) << 6); // A6–A7 → bits 6–7

// A8–A9 from IN1_REG bits 0–1 (contiguous)
uint8_t addr_hi = gpio_high & 0x03;                 // A8–A9 only

// Decoded region (replaces A8–A15 check for D1xx and D5xx):
bool is_d5xx = !((gpio_high >> 2) & 1);  // CCTL low  → GPIO34
bool is_d1xx = !((gpio_high >> 3) & 1);  // ~D1XX low → GPIO35
```

### PHI2 polling (unchanged, already optimal)

```c
// Both v1.x and v2.x — PHI2 stays on GPIO23
#define PHI2_IS_LOW  !((REG_READ(GPIO_IN_REG) >> 23) & 1)
#define PHI2_IS_HIGH   ((REG_READ(GPIO_IN_REG) >> 23) & 1)
```

---

## GPIO0 / GPIO1 / GPIO3 Notes (v2.x)

| GPIO | Issue | Mitigation |
| ---- | ----- | ---------- |
| GPIO0 | Strapping pin — LOW at power-on = download mode | Add 10 kΩ pull-up on BOBOARD PCB. Drive TXS0108EPW `OE` LOW during boot to isolate the Atari A0 line; release `OE` after `setup()` completes. |
| GPIO1 | UART0 TX (serial console) | Use USB-CDC for debug on the final board (ESP32-S3 has native USB). If NodeMCU-32S is retained, leave GPIO1 unused and connect A1 elsewhere. |
| GPIO3 | UART0 RX (serial console) | Same as GPIO1. |

For PROTOTYPE builds (NodeMCU-32S), GPIO0/1/3 can still be used as inputs after
`setup()` initialises them with `gpio_config()`, provided the UART is not initialised
and GPIO0 has a pull-up. Test carefully before relying on them.

---

## High-Speed Bus Mapping (XL/XE Default) — Speed Focused

This layout is the **default** for the ESP32-BOBOARD-5V. It is compatible with both **Atari XL (PBI)** and **Atari XE (ECI + CART)** interfaces. It optimizes for maximum execution speed by grouping all critical signals in the first 32 bits of the GPIO register (`GPIO_IN_REG`).

### Design goals

1. **Fast Decision:** PHI2, R/~W, CCTL, and ~D1XX are all in `GPIO_IN_REG`. A single read and mask determines if the bus cycle is relevant.
2. **Fast Address:** A0–A5 are contiguous (bits 0–5). A6–A7 are adjacent (bits 21–22). Reassembling A0–A7 takes only 2–3 instructions.
3. **Contiguous Data:** D0–D7 remain on GPIO 12–19 for single-byte I/O.
4. **Full Level-Shifter Utilization:** 24 channels used across 3 ICs.

### Pin assignment (Default)

| IC | ESP32 GPIO | Register | bit | Function | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **U1** | **12–19** | **IN_REG** | **12–19** | **D0–D7** | Data Bus (Bidirectional) |
| **U2** | **0, 2–5** | **IN_REG** | **0, 2–5** | **A0, A2–A4** | Address LSB |
| **U2** | **3** | **IN_REG** | **3** | **A1** | Replaces RX0 |
| **U2** | **21–22** | **IN_REG** | **21–22** | **A5–A6** | Address LSB |
| **U3** | **23** | **IN_REG** | **23** | **PHI2** | Clock (Input) |
| **U3** | **25** | **IN_REG** | **25** | **R/~W** | Read/Write (Input) |
| **U3** | **26** | **IN_REG** | **26** | **CCTL** | $D5xx Decoded (Input) |
| **U3** | **27** | **IN_REG** | **27** | **~D1XX** | $D1xx Decoded (Input) |
| **U3** | **32** | **OUT_REG**| **0** | **~MPD** | Math Pack Disable (Output) |
| **U3** | **33** | **OUT_REG**| **1** | **~EXTSEL**| External Select (Output) |
| **U3** | **34–36** | **IN1_REG**| **2–4** | **A7–A9** | Address MSB (Input) |
| **UART**| **1** | — | — | **TX0** | **Free for printf** |

### Performance Optimization

```c
// ONE read to get everything needed for the bus decision
uint32_t reg = REG_READ(GPIO_IN_REG);

// Wait for PHI2 high (bit 23)
while (!(reg & 0x00800000)) reg = REG_READ(GPIO_IN_REG);

// Check CCTL (bit 26) or D1XX (bit 27)
if (!(reg & 0x0C000000)) {
    // Reassemble A0-A6 from reg (A0:0, A2:2, A1:3, A3:4, A4:5, A5:21, A6:22)
    uint8_t addr = (reg & 0x01) | (reg & 0x04) | ((reg & 0x08) >> 2) | ...;
    // (see read_address_bus in main.cpp for full logic)
}
```

---

## J3 Physical Pinout (High-Speed XL/XE)

Pins not listed here carry +5V, +3.3V, GND, or are NC.

| J3 pin | Net | ESP32 GPIO | Atari signal | Notes |
| :--- | :--- | :--- | :--- | :--- |
| 1 | mD0 | GPIO12 | **D0** | Data |
| 2 | mD1 | GPIO13 | **D1** | |
| 3 | mD2 | GPIO14 | **D2** | |
| 4 | mD3 | GPIO15 | **D3** | |
| 5 | mD4 | GPIO16 | **D4** | |
| 6 | mD5 | GPIO17 | **D5** | |
| 7 | mD6 | GPIO18 | **D6** | |
| 8 | mD7 | GPIO19 | **D7** | |
| 9 | mA0 | GPIO0 | **A0** | Strapping pin (A0) |
| 10 | mA1 | GPIO3 | **A1** | RX0 pin used for A1 |
| 11 | mA2 | GPIO2 | **A2** | |
| 12 | mA3 | GPIO4 | **A3** | |
| 13 | mA4 | GPIO5 | **A4** | |
| 14 | mA5 | GPIO21 | **A5** | |
| 15 | mA6 | GPIO22 | **A6** | |
| 16 | mA7 | GPIO34 | **A7** | IN-only |
| 17 | mA8 | GPIO35 | **A8** | IN-only |
| 18 | mA9 | GPIO36 | **A9** | IN-only |
| 19 | mPHI | GPIO23 | **PHI2** | Clock |
| 20 | mRW | GPIO25 | **R/~W** | |
| 21 | mCCT | GPIO26 | **CCTL** | $D5xx |
| 22 | mD1X | GPIO27 | **~D1XX** | $D1xx |
| 23 | mMPD | GPIO32 | **~MPD** | Output |
| 24 | mEXT | GPIO33 | **~EXTSEL**| Output |
| 25 | mTX0 | GPIO1 | **TX0** | **Serial Debug Out** |
| 26 | mCS | GPIO26 | **~CS** | shared or optional |

> **Note**: This physical mapping requires the specific PCB layout of the ESP32-BOBOARD-5V v2.x.
