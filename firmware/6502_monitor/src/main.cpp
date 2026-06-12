/**
 * 6502_monitor -- ESP32 PBI ROM Emulator ($D800-$DFFF)
 *
 * FIRMWARE: Targets NodeMCU DevKit V1 (ESP32-WROOM).
 *
 * BEHAVIOUR:
 * - ROMSEL ($D800-$DFFF): asserts MPD low; drives pbi_rom[A0-A10] on reads.
 *   Full 2 KB addressing via A0-A10 (no mirroring).
 * - EXTSEL ($D1XX / CCTL): asserted low when the internal latch is active.
 * - Latch (PBI mode): set by writing $80 to $D1FF, cleared by writing $00.
 * - Latch (CCTL mode): always active.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// Bus Protocol Mode
// ---------------------------------------------------------------------------
#define BUS_MODE_PBI  0  // Parallel Bus Interface (Atari XL/XE)
#define BUS_MODE_CCTL 1  // Cartridge Control (Cartridge Slot)

#ifndef BUS_MODE
#define BUS_MODE BUS_MODE_PBI
#endif

// ---------------------------------------------------------------------------
// Pin Definitions (NodeMCU DevKit V1 / ESP32-WROOM)
// ---------------------------------------------------------------------------
// Data Bus (D0-D7)
#define PIN_D0 4
#define PIN_D1 5
#define PIN_D2 13
#define PIN_D3 14
#define PIN_D4 16
#define PIN_D5 17
#define PIN_D6 18
#define PIN_D7 19
// Address Bus (A6-A10)
#define PIN_A6  21
#define PIN_A7  27
#define PIN_A8  12  // formerly RESET
#define PIN_A9  25  // formerly VCS
#define PIN_A10 26  // formerly RAMSEL
// PBI / Control
#define PIN_SEL_N  22  // $D1XX / CCTL selection (Active LOW, Input)
#define PIN_ROMSEL 23  // $D800-$DFFF range select (Active LOW, Input)
// Outputs
#define PIN_EXTSEL 3   // Disable Atari internal memory (Active LOW, Output)
#define PIN_MPD    0   // Math Pack Disable (Active LOW, Output)
// Common Bus Signals
#define PIN_PHI2   2   // 6502 Phase 2 Clock (1.79 MHz)
#define PIN_RW     15  // Read/Write (High = Read, Low = Write)
// Address Pins (A0-A5)
#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 39
#define PIN_A4 32
#define PIN_A5 33

static const uint8_t DBUS_PINS[8] = {4, 5, 13, 14, 16, 17, 18, 19};

#define DBUS_MASK                                                          \
    ((1UL << 4) | (1UL << 5) | (1UL << 13) | (1UL << 14) | (1UL << 16) | \
     (1UL << 17) | (1UL << 18) | (1UL << 19))

// ---------------------------------------------------------------------------
// PBI ROM & Drive LUT
// ---------------------------------------------------------------------------
// Full 2 KB ROM image in IRAM — A0-A10 decoded, no aliasing.
static IRAM_ATTR uint8_t pbi_rom[2048];

// Drive LUT: byte value → 32-bit GPIO bitmask for <50 ns data bus writes.
static IRAM_ATTR uint32_t drive_lut[256];

// Internal device-active latch.
volatile bool latch_active = (BUS_MODE == BUS_MODE_CCTL);

// ---------------------------------------------------------------------------
// Decoding Helpers (IRAM)
// ---------------------------------------------------------------------------
static inline uint8_t IRAM_ATTR decode_data(uint32_t lo)
{
    return (uint8_t)(((lo >> 4) & 1) | ((lo >> 5) & 1) << 1 |
                     ((lo >> 13) & 1) << 2 | ((lo >> 14) & 1) << 3 |
                     ((lo >> 16) & 1) << 4 | ((lo >> 17) & 1) << 5 |
                     ((lo >> 18) & 1) << 6 | ((lo >> 19) & 1) << 7);
}

// Returns an 11-bit address (A0-A10) for indexing pbi_rom[].
static inline uint16_t IRAM_ATTR decode_addr(uint32_t lo, uint32_t hi)
{
    // A0-A5: GPIO_IN1_REG (GPIO 32-39 map to bits 0-7 of hi)
    uint16_t a = (uint16_t)(((hi >> 2) & 1) | ((hi >> 3) & 1) << 1 |
                             ((hi >> 4) & 1) << 2 | ((hi >> 7) & 1) << 3 |
                             ((hi >> 0) & 1) << 4 | ((hi >> 1) & 1) << 5);
    // A6-A7: GPIO_IN_REG
    a |= ((lo >> 21) & 1) << 6 | ((lo >> 27) & 1) << 7;
    // A8-A10: GPIO_IN_REG (GPIO 12, 25, 26)
    a |= ((lo >> 12) & 1) << 8 | ((lo >> 25) & 1) << 9 | ((lo >> 26) & 1) << 10;
    return a;
}

static inline void IRAM_ATTR bus_drive(uint8_t val)
{
    uint32_t m       = drive_lut[val];
    GPIO.out_w1tc    = DBUS_MASK & ~m;
    GPIO.out_w1ts    = m;
    GPIO.enable_w1ts = DBUS_MASK;
}

static inline void IRAM_ATTR bus_release()
{
    GPIO.enable_w1tc = DBUS_MASK;
}

// ============================================================================
// MonitorTask -- Core 1 High-Speed Bus Handler
// ============================================================================
void IRAM_ATTR MonitorTask(void *pvParameters)
{
    uint32_t lo, hi;
    uint16_t addr;

    const uint32_t m_phi2   = (1UL << PIN_PHI2);
    const uint32_t m_rw     = (1UL << PIN_RW);
    const uint32_t m_sel    = (1UL << PIN_SEL_N);
    const uint32_t m_romsel = (1UL << PIN_ROMSEL);
    const uint32_t m_mpd    = (1UL << PIN_MPD);
    const uint32_t m_extsel = (1UL << PIN_EXTSEL);

    // Initial state: outputs inactive (HIGH)
    GPIO.out_w1ts = m_mpd | m_extsel;

    while (true)
    {
        // 1. Sync on PHI2 rising edge
        while (!(GPIO.in & m_phi2))
            ;

        // 2. Sample address and control lines
        lo   = GPIO.in;
        hi   = GPIO.in1.val;
        addr = decode_addr(lo, hi);

        // 3. ROMSEL active ($D800-$DFFF): assert MPD, serve ROM on reads
        if (!(lo & m_romsel))
        {
            GPIO.out_w1tc = m_mpd;
            if (lo & m_rw)
                bus_drive(pbi_rom[addr & 0x7FF]);
        }
        else
        {
            GPIO.out_w1ts = m_mpd;
        }

        // 4. SEL_N active ($D1XX / CCTL): assert EXTSEL if latch is set
        if (!(lo & m_sel))
        {
            if (latch_active)
                GPIO.out_w1tc = m_extsel;

#if BUS_MODE == BUS_MODE_PBI
            // Latch control via $D1FF: write $80 = enable, $00 = disable
            if (!(lo & m_rw) && (addr & 0xFF) == 0xFF)
            {
                uint8_t data = decode_data(GPIO.in);
                latch_active = (data == 0x80);
                if (!latch_active)
                    GPIO.out_w1ts = m_extsel;
            }
#endif
        }
        else
        {
            GPIO.out_w1ts = m_extsel;
        }

        // 5. PHI2 falling edge: release data bus
        while (GPIO.in & m_phi2)
            ;
        bus_release();
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup()
{
    Serial.begin(115200, SERIAL_8N1, -1, 1);
    Serial.println("\n[6502_monitor] PBI ROM Emulator booting...");

    // Build drive LUT
    for (int i = 0; i < 256; i++)
    {
        uint32_t m = 0;
        for (int b = 0; b < 8; b++)
            if ((i >> b) & 1) m |= (1UL << DBUS_PINS[b]);
        drive_lut[i] = m;
    }

    // ROM placeholder: all NOP ($EA)
    memset(pbi_rom, 0xEA, sizeof(pbi_rom));

    // Control outputs: inactive (HIGH)
    pinMode(PIN_MPD,    OUTPUT);
    digitalWrite(PIN_MPD,    HIGH);
    pinMode(PIN_EXTSEL, OUTPUT);
    digitalWrite(PIN_EXTSEL, HIGH);

    // Data bus: high-impedance inputs initially
    for (int i = 0; i < 8; i++) pinMode(DBUS_PINS[i], INPUT);

    // Address and bus control inputs
    pinMode(PIN_PHI2,   INPUT);
    pinMode(PIN_RW,     INPUT);
    pinMode(PIN_SEL_N,  INPUT);
    pinMode(PIN_ROMSEL, INPUT);
    pinMode(PIN_A0,  INPUT);
    pinMode(PIN_A1,  INPUT);
    pinMode(PIN_A2,  INPUT);
    pinMode(PIN_A3,  INPUT);
    pinMode(PIN_A4,  INPUT);
    pinMode(PIN_A5,  INPUT);
    pinMode(PIN_A6,  INPUT);
    pinMode(PIN_A7,  INPUT);
    pinMode(PIN_A8,  INPUT);
    pinMode(PIN_A9,  INPUT);
    pinMode(PIN_A10, INPUT);

    xTaskCreatePinnedToCore(MonitorTask, "Monitor", 4096, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);

    Serial.println("[6502_monitor] Running.");
}

void loop()
{
    delay(1000);
}
