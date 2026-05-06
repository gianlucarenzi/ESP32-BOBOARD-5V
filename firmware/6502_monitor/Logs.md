# Project Logs & Architectural Decisions

## 2026-05-06: High-Performance PBI Driver Implementation

### Overview
Successfully implemented a high-performance ESP32 firmware for the Atari Parallel Bus Interface (PBI). The final architecture prioritizes low latency and thread safety using a combination of external hardware decoding and FreeRTOS synchronization.

### Final Hardware Mapping
The system uses an external **74HC138** decoder for the `$D800-$DFFF` range to reduce ESP32 pin usage and latency.

#### 74HC138 Wiring (External Decoder)
| Pin 74HC138 | Signal / Connection | Function |
| :--- | :--- | :--- |
| 1 (A) | **A11** (Atari) | Address selector bit 0 |
| 2 (B) | **A12** (Atari) | Address selector bit 1 |
| 3 (C) | **A14** (Atari) | Address selector bit 2 |
| 4 (G2A) | **A13** (Atari) | Enable (Active LOW) - Must be 0 |
| 5 (G2B) | **GND** | Enable (Active LOW) - Always GND |
| 6 (G1) | **A15** (Atari) | Enable (Active HIGH) - Must be 1 |
| 7 (Y7) | **ROM_SEL / MPD** | **Output**: Active LOW for $D800-$DFFF |
| 8 (GND) | **GND** | Common Ground |
| 16 (VCC) | **+5V** (Atari) | Power Supply |

**Connection Notes:**
- **MPD:** Connect Pin 7 (Y7) directly to Atari Bus MPD line.
- **ROM_SEL:** Connect Pin 7 (Y7) to ESP32 **GPIO 4** (via 5V->3.3V Level Shifter).

#### ESP32 Pinout
| Signal | ESP32 GPIO | Direction | Description |
| :--- | :--- | :--- | :--- |
| **Data Bus (D0-D7)** | 18, 19, 21-23, 25-27 | Bidirectional | Direct register access for latency < 50ns |
| **Address (A0-A5)** | 32-36, 39 | Input | Lower 6 bits for ROM indexing |
| **Address (A6-A10)** | 16, 17, 14, 12, 13 | Input | Higher 5 bits for 2KB ROM indexing |
| **PHI2** | 2 | Input | 6502 Clock (Sync trigger) |
| **R/W** | 15 | Input | High = Read, Low = Write |
| **D1XX** | 5 | Input | Device Selection Page (Active Low) |
| **ROM_SEL** | 4 | Input | Active Low from 74HC138 ($D800-$DFFF) |
| **EXTSEL** | 0 | Output | External Select (Active Low) to Atari |
| **MPD** | - | Hardware | Managed directly by 74HC138 Y7 output |
| **Debug Serial TX** | 1 (TX) | Output | Standard Serial Debug enabled |
| **DEV_SEL / CS** | 3 | Output | Chip Select hardware ($D100-$D11F, A0-A4) |
| **Internal Regs** | - | Software | Registri ESP32 ($D120-$D1FE, A5-A7 coinvolti) |



### Key Architectural Decisions

1.  **Direct GPIO Register Access:**
    - Used `GPIO.in`, `GPIO.out_w1ts`, and `GPIO.out_w1tc` instead of `digitalRead/Write` or `gpio_config`.
    - Reduced I/O latency from microseconds to ~20ns, fitting within the 279ns Atari timing window.

2.  **Multicore Optimization (FreeRTOS):**
    - **Core 1 (MonitorTask):** Dedicated to the high-speed bus monitor loop. Uses a local state for `pbi_enabled` to avoid cache coherence overhead.
    - **Core 0 (System):** Handles Serial Debugging and FreeRTOS system tasks.
    - **Queue Communication:** Used `xQueueSend` and `xQueueReceive` for thread-safe state synchronization between cores. This avoids the "bruttissima" shared global variable approach.

3.  **Hardware Decoding ($D800-$DFFF):**
    - Offloading address decoding to a **74HC138** freed up 5 GPIOs and removed critical timing bottlenecks from the firmware.
    - `MPD` is now handled at hardware speed (~10ns), preventing bus contention before the ESP32 can even react.

4.  **Serial Debug Recovery:**
    - By moving `EXTSEL` and `MPD` away from GPIO 1 and 3, standard hardware serial debugging was restored, facilitating further development.

### Performance Verification
- **PHI2 sync:** Confirmed stable triggering on rising edge.
- **Data Injection:** 2KB ROM emulation serving `pbi_driver` array correctly within the PHI2 HIGH window.
- **Device Selection:** $D1FF write logic correctly enables/disables the PBI driver response.

## 2026-05-07: VERA Text Mode Boot Screen

### Overview
The 6502 PBI ROM now initializes the external VERA chip mapped at `$D100-$D11F` during driver startup. The boot sequence configures a text screen in a Commander X16-inspired style for Atari PBI/ECI usage.

### Video Initialization
- **VERA register model:** The driver now uses CX16-compatible VERA register naming, remapped to the Atari PBI window at `$D100`.
- **Screen mode:** Layer 1 text mode configured for **80x25** cells with **8x16** character geometry.
- **Display style:** Blue background, black border, white text.
- **Banner text:** The ROM writes:
  - `**** COMMANDER X16 VERA ****`
  - `PBI VIDEO INTERFACE`
  - `READY.`

### Font Handling
- The VERA bitstream does **not** provide a guaranteed ready-to-use system font after reset.
- For this reason, the ROM now loads a **minimal 8x16 font subset** into VERA VRAM at startup.
- The embedded glyph set is intentionally compact and currently covers only the characters required by the boot banner.

### ROM Size Constraint
- The 6502 PBI ROM linker configuration is now limited to **2KB** (`$0800` bytes).
- This matches the ESP32-side ROM fetch logic, which serves the driver with `offset & 0x7FF`.
- Current generated ROM size: **2048 bytes**.
