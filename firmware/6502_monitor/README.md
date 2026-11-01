# Atari ROM-less PBI / CCTL Peripheral & Bus Monitor

ESP32 firmware for the ESP32-BOBOARD-5V that behaves as a ROM-less Atari 8-bit
peripheral in two register windows, `$D100-$D1FF` (PBI) and `$D500-$D5FF`
(CCTL), and logs every access to both windows over a lock-free ring buffer.

## Features
- **Two register windows**, selected by the Atari MMU / PLA:
    - `~D1XX` → `$D100-$D1FF`, select latch at `$D1FF`.
    - `~CCTL` → `$D500-$D5FF`, select latch at `$D5FF`.

  Only A0-A7 are sampled: the page comes from the select line.  Each window
  has its own latch and its own 256-byte register file.
- **PBI rules:**
    - A write to `$D1FF` / `$D5FF` selects the device when bit 7 (device ID
      `$80`) is 1 and deselects it when bit 7 is 0.
    - While selected, writes to `$xx00-$xxFE` are stored and reads of
      `$xx00-$xxFE` are answered with the stored byte.
    - Not selected, the device never drives the bus.  `$D1FF` / `$D5FF` are
      never driven on reads (`$D1FF` is the PBI interrupt status read by the OS).
    - The registers are written only by the Atari; power-up value `$00`.
- **No ROM, no MPD, no EXTSEL:** the OS PBI scan finds no signature at
  `$D803` / `$D80B` and skips the device; programs select it directly by
  writing `$80` to `$D1FF` (and `$00` to release it).  EXTSEL is not needed:
  there is no internal RAM to replace in the I/O area.
- **Bus monitor:** every access to both windows is logged, selected or not;
  reads of other devices are sampled just before PHI2 falls.  Registers
  `$00-$1A` are shown with their VERA names; `$09-$0C` are resolved through
  DCSEL (bits `[2:1]` of the last write to offset `$05`, per window).
- **Serial output** (`LogTask` on Core 0, the only writer on Serial, 115200
  baud by default, `-D LOG_BAUD=...`; each line is one `Serial.write()`, so
  lines never interleave):
    ```
    [    1.234567] [D1FF - PBI_SELECT          ] W $80  device SELECTED
    [    1.234601] [D105 - VERA_CTRL           ] W $04  SEL
    [    1.234640] [D105 - VERA_CTRL           ] R $04  SEL DRV
    [    1.300000] [D500 - VERA_ADDR_L         ] R $FF  -
    ```
  `SEL` = device selected in that window, `DRV` = byte driven by the device,
  `-` = not selected (the logged byte is whatever was on the bus).
  Timestamps come from the Core 1 cycle counter (CCOUNT extended to 64 bit),
  in µs since the bus loop started.  Events are dropped and counted when the
  1024-entry ring is full.
- **Bus loop:** IRAM-resident `MonitorTask` owns Core 1 (interrupts masked,
  interrupt watchdog disabled, no FreeRTOS calls).  `~D1XX` is not
  PHI2-qualified, so A0-A7, R/W and the window selects are decoded during PHI1
  (`BUS_ADDR_SETTLE_NS`, default 200 ns after PHI2 falls) and the read byte
  is already in `GPIO.out` when PHI2 rises: one store turns D0-D7 on.  A
  select that appears only on the rising edge is handled from that sample
  (late drive).  Write data is sampled `BUS_WR_SAMPLE_NS` (default 120 ns)
  after PHI2 rises.  Optimizations ported from Vera-Module-RBL-XE.
- **Hardware Target:** NodeMCU DevKit V1 (ESP32-WROOM, 30 pins).

## Hardware Setup
Refer to the following documents for detailed pinout:
- [General Pin Mapping](PIN_MAPPING.md) (NodeMCU DevKit V1)
- [Hardware Architecture](HARDWARE.md)

## Development
This project uses **PlatformIO**.
- Style: **Allman** (braces on new line).
- Column Limit: **80**.
- Tool: `clang-format`.

### Configuration
One build environment, `nodemcu-32s`, serves both windows.  Bus timing can be
tuned with `-D BUS_ADDR_SETTLE_NS=...`, `-D BUS_WR_SAMPLE_NS=...` and
`-D BUS_PHI2_HIGH_MAX_NS=...` after measuring the real bus.

Build and flash:
```sh
pio run -e nodemcu-32s --target upload
```

**Programming rule:** the ESP32 module is always programmed on its own,
disconnected from the Atari and connected only to the PC (serial bootloader).

**VDD_SDIO efuse:** GPIO12 is not used by the firmware but is still wired to
U1 / J3 pin 4 (former A8), and at every chip reset the ESP32 ROM samples it to
select the flash voltage (HIGH = 1.8 V: the WROOM-32 flash does not boot).
Before every upload, `efuse_vdd_sdio.py` reads the efuses and, if they are
still blank, burns VDD_SDIO to a fixed 3.3 V
(`espefuse.py set_flash_voltage 3.3V`, **irreversible**), so that GPIO12 is
ignored from then on.  Already set: nothing to do.  Any other combination:
the upload stops without burning anything.  Controlled by
`custom_efuse_vdd_sdio = 3.3V` in `platformio.ini` (empty value = no check).
On first use the script creates `.pio/espefuse-venv` with the espefuse
dependencies (needs network access once).  See [HARDWARE.md](HARDWARE.md#3-esp32-strapping-pins).

**Serial monitor:** `monitor_rts = 0` and `monitor_dtr = 0`, so opening the
monitor does not reset the ESP32 through the NodeMCU auto-reset circuit.  The
debug UART is also on J4: read-only use needs GND (pin 1) and TX0 (pin 3,
3.3 V); leave RX0 (pin 2) unconnected.
