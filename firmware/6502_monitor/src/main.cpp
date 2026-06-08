/**
 * 6502_monitor -- Optimized ESP32 Monitor & RAM Emulator ($D600-$D7FF)
 *
 * UNIVERSAL FIRMWARE: Supports NodeMCU DevKit V1 and ESP32-PICO-D4 SiP.
 *
 * CORE ARCHITECTURE:
 * - Real-time Bus Monitoring: PHI2-synchronized sampling on Core 1 (IRAM).
 * - Fast Data Injection: Pre-computed GPIO LUT (Look-Up Table) for < 50ns
 * latency.
 * - Memory Emulation: 512-byte block for $D600-$D7FF range.
 * - Hardware Management: Controls Atari RESET, EXTSEL, and MPD signals.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// Hardware & Mode Selection
// ---------------------------------------------------------------------------
#define TARGET_NODEMCU 0
#define TARGET_PICO_D4 1

// Default target if not specified via build flags
#ifndef HARDWARE_TARGET
#define HARDWARE_TARGET TARGET_NODEMCU
#endif

// Bus Protocol Mode
#define BUS_MODE_PBI  0  // Parallel Bus Interface (Atari XL/XE)
#define BUS_MODE_CCTL 1  // Cartridge Control (Cartridge Slot)

#ifndef BUS_MODE
#define BUS_MODE BUS_MODE_PBI
#endif

// ---------------------------------------------------------------------------
// Pin Definitions (Target Specific)
// ---------------------------------------------------------------------------
#if HARDWARE_TARGET == TARGET_NODEMCU
// Data Bus (D0-D7)
#define PIN_D0 4
#define PIN_D1 5
#define PIN_D2 13
#define PIN_D3 14
#define PIN_D4 16
#define PIN_D5 17
#define PIN_D6 18
#define PIN_D7 19
// Address Bus (High Bits)
#define PIN_A6 21
#define PIN_A7 27
// PBI / Control
#define PIN_SEL_N  22  // $D1XX page selection (Active LOW)
#define PIN_VCS    25  // Virtual Chip Select (Active LOW Output)
#define PIN_ROMSEL 23  // $D800-$DFFF range select (Active LOW)

static const uint8_t DBUS_PINS[8] = {4, 5, 13, 14, 16, 17, 18, 19};

#define DBUS_MASK                                                        \
    ((1UL << 4) | (1UL << 5) | (1UL << 13) | (1UL << 14) | (1UL << 16) | \
     (1UL << 17) | (1UL << 18) | (1UL << 19))
#else  // PICO_D4 target (Optimized for SiP pinout, avoiding Flash pins)
// Data Bus (D0-D7) - GPIO 16/17 avoided (internal Flash)
#define PIN_D0     4
#define PIN_D1     5
#define PIN_D2     13
#define PIN_D3     14
#define PIN_D4     18
#define PIN_D5     19
#define PIN_D6     21
#define PIN_D7     22
// Address Bus (High Bits) - Using Input-Only pins 37/38
#define PIN_A6     37
#define PIN_A7     38
// PBI / Control
#define PIN_SEL_N  23
#define PIN_VCS    27
#define PIN_ROMSEL 25

static const uint8_t DBUS_PINS[8] = {4, 5, 13, 14, 18, 19, 21, 22};

#define DBUS_MASK                                                        \
    ((1UL << 4) | (1UL << 5) | (1UL << 13) | (1UL << 14) | (1UL << 18) | \
     (1UL << 19) | (1UL << 21) | (1UL << 22))
#endif

// Common Bus Signals
#define PIN_PHI2   2   // 6502 Phase 2 Clock (1.79 MHz)
#define PIN_RW     15  // Read/Write (High = Read, Low = Write)
#define PIN_RAMSEL 26  // $D600-$D7FF range select (Active LOW)
#define PIN_EXTSEL 3   // External Select / MPD (Active LOW Output)
#define PIN_MPD    0   // Math Pack Disable (Active LOW Output)
#define PIN_RESET  12  // Atari System RESET (Active LOW Output)

// Common Address Pins (A0-A5)
#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 39
#define PIN_A4 32
#define PIN_A5 33

// ---------------------------------------------------------------------------
// Emulated Memory & Drive LUT
// ---------------------------------------------------------------------------
// 512-byte RAM emulated in internal IRAM for zero-wait-state access.
// Note: Mirroring occurs between $D6xx and $D7xx due to 8-bit addressing.
static IRAM_ATTR uint8_t emulated_ram[512];

// Drive LUT: Maps a byte value (0-255) to a 32-bit GPIO bitmask for fast
// output.
static IRAM_ATTR uint32_t drive_lut[256];

// Global state for device activation (controlled via $D1FF in PBI mode).
volatile bool vcs_enabled = (BUS_MODE == BUS_MODE_CCTL);

// ---------------------------------------------------------------------------
// Decoding Helpers (IRAM resident for speed)
// ---------------------------------------------------------------------------

/**
 * Decodes the data bus bits from the 32-bit GPIO_IN_REG value.
 */
static inline uint8_t IRAM_ATTR decode_data(uint32_t lo)
{
#if HARDWARE_TARGET == TARGET_NODEMCU
    return (uint8_t)(((lo >> 4) & 1) | ((lo >> 5) & 1) << 1 |
                     ((lo >> 13) & 1) << 2 | ((lo >> 14) & 1) << 3 |
                     ((lo >> 16) & 1) << 4 | ((lo >> 17) & 1) << 5 |
                     ((lo >> 18) & 1) << 6 | ((lo >> 19) & 1) << 7);
#else
    return (uint8_t)(((lo >> 4) & 1) | ((lo >> 5) & 1) << 1 |
                     ((lo >> 13) & 1) << 2 | ((lo >> 14) & 1) << 3 |
                     ((lo >> 18) & 1) << 4 | ((lo >> 19) & 1) << 5 |
                     ((lo >> 21) & 1) << 6 | ((lo >> 22) & 1) << 7);
#endif
}

/**
 * Decodes the address bus bits (A0-A7) from GPIO_IN_REG and GPIO_IN1_REG.
 */
static inline uint8_t IRAM_ATTR decode_addr_low(uint32_t lo, uint32_t hi)
{
    // A0-A5 (GPIO 34, 35, 36, 39, 32, 33)
    uint8_t a = (uint8_t)(((hi >> 2) & 1) | ((hi >> 3) & 1) << 1 |
                          ((hi >> 4) & 1) << 2 | ((hi >> 7) & 1) << 3 |
                          ((hi >> 0) & 1) << 4 | ((hi >> 1) & 1) << 5);
#if HARDWARE_TARGET == TARGET_NODEMCU
    a |= ((lo >> 21) & 1) << 6 | ((lo >> 27) & 1) << 7;  // A6-A7 on GPIO 21, 27
#else
    a |= ((hi >> 5) & 1) << 6 | ((hi >> 6) & 1)
                                    << 7;  // A6-A7 on GPIO 37, 38 (Input1 bank)
#endif
    return a;
}

/**
 * Drives a byte value onto the ESP32 GPIOs configured as data bus.
 */
static inline void IRAM_ATTR bus_drive(uint8_t val)
{
    uint32_t m       = drive_lut[val];
    GPIO.out_w1tc    = DBUS_MASK & ~m;  // Clear bits that are 0 in mask
    GPIO.out_w1ts    = m;               // Set bits that are 1 in mask
    GPIO.enable_w1ts = DBUS_MASK;       // Switch GPIOs to Output mode
}

/**
 * Releases the data bus (sets GPIOs to High-Z / Input mode).
 */
static inline void IRAM_ATTR bus_release()
{
    GPIO.enable_w1tc = DBUS_MASK;  // Switch GPIOs to Input mode
}

// ============================================================================
// MonitorTask -- Core 1 High-Speed Bus Handler
// ============================================================================
void IRAM_ATTR MonitorTask(void *pvParameters)
{
    uint32_t lo, hi;
    uint8_t  addr, data;

    // Pre-calculate bitmasks for frequently used signals
    const uint32_t m_phi2 = (1UL << PIN_PHI2);
    const uint32_t m_rw   = (1UL << PIN_RW);
    const uint32_t m_sel  = (1UL << PIN_SEL_N);
    const uint32_t m_vcs  = (1UL << PIN_VCS);

#if BUS_MODE == BUS_MODE_PBI
    const uint32_t m_romsel = (1UL << PIN_ROMSEL);
    const uint32_t m_ramsel = (1UL << PIN_RAMSEL);
    const uint32_t m_mpd    = (1UL << PIN_MPD);
    const uint32_t m_extsel = (1UL << PIN_EXTSEL);

    // Initial State: Device IDLE, Signals HIGH
    GPIO.out_w1ts = m_vcs | m_mpd | m_extsel;
#else
    GPIO.out_w1ts = m_vcs;
#endif

    while (true)
    {
        // 1. SYNC: Wait for PHI2 Rising Edge (Start of 6502 cycle)
        while (!(GPIO.in & m_phi2))
            ;

        // 2. SAMPLE: Capture Address Bus and Control Signals
        lo   = GPIO.in;
        hi   = GPIO.in1.val;
        addr = decode_addr_low(lo, hi);

#if BUS_MODE == BUS_MODE_PBI
        // --- PBI PROTOCOL HANDLER ---

        // Logic for Device Activation via $D1FF write
        if (!(lo & m_sel))
        {
            if (!(lo & m_rw) && (addr == 0xFF))
            {
                // Sampling data for device config
                data = decode_data(GPIO.in);
                if (data == 0x80)
                {
                    vcs_enabled   = true;
                    GPIO.out_w1tc = m_vcs;
                }  // Enable Device
                else if (data == 0x00)
                {
                    vcs_enabled   = false;
                    GPIO.out_w1ts = m_vcs;
                }  // Disable Device
            }
        }

        if (vcs_enabled)
        {
            bool is_ram = !(lo & m_ramsel);  // $D600-$D7FF range
            bool is_rom = !(lo & m_romsel);  // $D800-$DFFF range
            bool is_sel = !(lo & m_sel);     // $D1XX page

            // Assert EXTSEL to disable internal Atari memory if we are
            // responding
            if (is_ram || is_rom || is_sel)
                GPIO.out_w1tc = m_extsel;
            else
                GPIO.out_w1ts = m_extsel;

            // Assert MPD if accessing ROM range (prevents Math Pack conflict)
            if (is_rom)
                GPIO.out_w1tc = m_mpd;
            else
                GPIO.out_w1ts = m_mpd;

            // Handle Emulated RAM ($D600-$D7FF)
            if (is_ram)
            {
                // Mirroring Note: Without A8 pin, D6xx and D7xx share the same
                // 256 bytes
                if (lo & m_rw)
                {
                    bus_drive(emulated_ram[addr]);
                }
                else
                {
                    // Wait for data to become valid on the bus (6502 write
                    // timing)
                    delayMicroseconds(0);
                    emulated_ram[addr] = decode_data(GPIO.in);
                }
            }
        }
        else
        {
            // Device Disabled: Ensure all output signals are High (Inactive)
            GPIO.out_w1ts = m_extsel | m_mpd;
        }
#else
        // --- CCTL (CART) MODE HANDLER ---
        if (!(lo & m_sel))
            GPIO.out_w1tc = m_vcs;
        else
            GPIO.out_w1ts = m_vcs;
#endif

        // 3. RELEASE: Wait for PHI2 Falling Edge (End of cycle)
        while (GPIO.in & m_phi2)
            ;
        bus_release();  // Tri-state the bus immediately
    }
}

// ============================================================================
// System Setup
// ============================================================================
void setup()
{
    // Hold Atari in RESET during ESP32 boot
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW);

    // Serial Debug (TX Only to free GPIO 3 if needed)
    Serial.begin(115200, SERIAL_8N1, -1, 1);
    Serial.println("\n[6502_monitor] System Booting...");

    // Build Drive LUT for fast GPIO manipulation
    for (int i = 0; i < 256; i++)
    {
        uint32_t m = 0;
        for (int b = 0; b < 8; b++)
        {
            if ((i >> b) & 1) m |= (1UL << DBUS_PINS[b]);
        }
        drive_lut[i] = m;
    }

    // Initialize Control Outputs
    pinMode(PIN_EXTSEL, OUTPUT);
    digitalWrite(PIN_EXTSEL, HIGH);
    pinMode(PIN_MPD, OUTPUT);
    digitalWrite(PIN_MPD, HIGH);
    pinMode(PIN_VCS, OUTPUT);
    digitalWrite(PIN_VCS, HIGH);

    // Initialize Data Bus as Inputs
    for (int i = 0; i < 8; i++) pinMode(DBUS_PINS[i], INPUT);

    // Initialize Address and Clock Inputs
    pinMode(PIN_PHI2, INPUT);
    pinMode(PIN_RW, INPUT);
    pinMode(PIN_SEL_N, INPUT);
    pinMode(PIN_A0, INPUT);
    pinMode(PIN_A1, INPUT);
    pinMode(PIN_A2, INPUT);
    pinMode(PIN_A3, INPUT);
    pinMode(PIN_A4, INPUT);
    pinMode(PIN_A5, INPUT);
    pinMode(PIN_A6, INPUT);
    pinMode(PIN_A7, INPUT);

#if BUS_MODE == BUS_MODE_PBI
    pinMode(PIN_ROMSEL, INPUT);
    pinMode(PIN_RAMSEL, INPUT);
#endif

    // Start high-priority MonitorTask on Core 1
    xTaskCreatePinnedToCore(MonitorTask, "Monitor", 4096, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);

    // Allow system to stabilize before releasing Atari
    delay(100);
    digitalWrite(PIN_RESET, HIGH);
    Serial.println("[6502_monitor] Atari RESET Released.");
}

void loop()
{
    // System housekeeping loop
    delay(1000);
}
