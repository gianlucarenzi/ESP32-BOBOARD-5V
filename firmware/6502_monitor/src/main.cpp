/**
 * 6502_monitor -- ESP32 PBI ROM Emulator ($D800-$DFFF)
 *
 * FIRMWARE: Targets NodeMCU DevKit V1 (ESP32-WROOM).
 *
 * BEHAVIOUR:
 * - ROMSEL ($D800-$DFFF): asserts MPD low; drives pbi_rom[A0-A10] on reads.
 *   Full 2 KB addressing via A0-A10 (no aliasing).
 * - EXTSEL ($D1XX): asserted low when the internal latch is active.
 * - Latch (PBI mode): set by writing $80 to $D1FF, cleared by writing $00.
 * - Latch (CCTL mode): always active.
 * - LOG: FreeRTOS queue from Core 1 to Core 0; prints VCS latch changes
 *   and every $D100-$D1FE read/write access.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp_timer.h>

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
// Log Queue (Core 1 → Core 0)
// ---------------------------------------------------------------------------
#define LOG_QUEUE_SIZE 64

#define EVT_LATCH 0   // VCS latch state change
#define EVT_REG   1   // $D100-$D1FE register access

// flags: bit 0 = 1→write / 0→read   bit 1 = latch state (EVT_LATCH only)
typedef struct {
    uint64_t ts_us;   // timestamp: microseconds since boot (esp_timer_get_time())
    uint8_t  type;
    uint8_t  offset;  // register offset within $D1XX (0x00-0xFE)
    uint8_t  data;
    uint8_t  flags;
} LogEvt;

static QueueHandle_t log_queue;

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
    // A0-A5: GPIO_IN1_REG (GPIO 32-39 → bits 0-7 of hi)
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

// Helper: enqueue a log event without blocking (drops if queue full).
static inline void IRAM_ATTR log_send(uint8_t type, uint8_t offset,
                                      uint8_t data, uint8_t flags)
{
    LogEvt evt = {(uint64_t)esp_timer_get_time(), type, offset, data, flags};
    xQueueSend(log_queue, &evt, 0);
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

            uint8_t offset = addr & 0xFF;

#if BUS_MODE == BUS_MODE_PBI
            if (offset == 0xFF)
            {
                // $D1FF: latch control — only on writes
                if (!(lo & m_rw))
                {
                    uint8_t data     = decode_data(GPIO.in);
                    bool    prev     = latch_active;
                    latch_active     = (data == 0x80);
                    if (!latch_active)
                        GPIO.out_w1ts = m_extsel;
                    // Log only when state changes
                    if (latch_active != prev)
                        log_send(EVT_LATCH, 0xFF, data,
                                 latch_active ? 0x02 : 0x00);
                }
            }
            else
#endif
            {
                // $D100-$D1FE: VERA register access — log R and W
                // Re-read GPIO.in so VERA has had time to put read data on bus.
                uint8_t data  = decode_data(GPIO.in);
                uint8_t flags = (lo & m_rw) ? 0x00 : 0x01;  // 0=read, 1=write
                log_send(EVT_REG, offset, data, flags);
            }
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

    // Create log queue before starting MonitorTask
    log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogEvt));

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
    Serial.println("[6502_monitor] VCS=OFF  Latch=DISABLED");
}

// ---------------------------------------------------------------------------
// VERA register name lookup — Core 0 only, not IRAM-resident.
// Registers $09-$0C are muxed by DCSEL (bits [2:1] of VERA_CTRL, offset $05).
// ---------------------------------------------------------------------------
static const char *vera_reg_name(uint8_t offset, uint8_t dcsel)
{
    static const char *base[8] = {
        "VERA_ADDR_L",  // $00
        "VERA_ADDR_M",  // $01
        "VERA_ADDR_H",  // $02
        "VERA_DATA0",   // $03
        "VERA_DATA1",   // $04
        "VERA_CTRL",    // $05
        "VERA_IEN",     // $06
        "VERA_ISR",     // $07
    };
    if (offset < 8)
        return base[offset];

    if (offset >= 0x09 && offset <= 0x0C)
    {
        static const char *mux[7][4] = {
            // DCSEL=0
            { "VERA_DC_VIDEO",    "VERA_DC_HSCALE",       "VERA_DC_VSCALE",       "VERA_DC_BORDER"      },
            // DCSEL=1
            { "VERA_DC_HSTART",   "VERA_DC_HSTOP",        "VERA_DC_VSTART",       "VERA_DC_VSTOP"       },
            // DCSEL=2
            { "VERA_FX_CTRL",     "VERA_FX_TILEBASE",     "VERA_FX_MAPBASE",      "VERA_FX_MULT"        },
            // DCSEL=3
            { "VERA_FX_X_INCR_L", "VERA_FX_X_INCR_H",    "VERA_FX_Y_INCR_L",    "VERA_FX_Y_INCR_H"   },
            // DCSEL=4
            { "VERA_FX_X_POS_L",  "VERA_FX_X_POS_H",     "VERA_FX_Y_POS_L",     "VERA_FX_Y_POS_H"    },
            // DCSEL=5
            { "VERA_FX_X_POS_S",  "VERA_FX_Y_POS_S",     "VERA_FX_POLY_FILL_L", "VERA_FX_POLY_FILL_H" },
            // DCSEL=6
            { "VERA_FX_CACHE_L",  "VERA_FX_CACHE_M",     "VERA_FX_CACHE_H",     "VERA_FX_CACHE_U"     },
        };
        return mux[(dcsel < 7) ? dcsel : 0][offset - 0x09];
    }

    if (offset >= 0x14 && offset <= 0x1A)
    {
        static const char *l1[7] = {
            "VERA_L1_CONFIG",    // $14
            "VERA_L1_MAPBASE",   // $15
            "VERA_L1_TILEBASE",  // $16
            "VERA_L1_HSCR_L",   // $17
            "VERA_L1_HSCR_H",   // $18
            "VERA_L1_VSCR_L",   // $19
            "VERA_L1_VSCR_H",   // $1A
        };
        return l1[offset - 0x14];
    }

    return "?";
}

// ============================================================================
// Loop -- Core 0: drain log queue and print
// ============================================================================
void loop()
{
    // Shadow of VERA_CTRL DCSEL bits [2:1]; updated on every write to $D105.
    static uint8_t dcsel = 0;

    LogEvt evt;
    while (xQueueReceive(log_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        uint32_t sec = (uint32_t)(evt.ts_us / 1000000ULL);
        uint32_t us  = (uint32_t)(evt.ts_us % 1000000ULL);
        if (evt.type == EVT_LATCH)
        {
            bool enabled = (evt.flags & 0x02) != 0;
            Serial.printf("[%5lu.%06lu] [VCS ] Latch %s ($%02X written to $D1FF)\n",
                          sec, us,
                          enabled ? "ENABLED " : "DISABLED", evt.data);
        }
        else  // EVT_REG
        {
            // Track DCSEL so muxed registers ($09-$0C) resolve correctly.
            if (evt.offset == 0x05 && (evt.flags & 0x01))
                dcsel = (evt.data >> 1) & 0x07;

            Serial.printf("[%5lu.%06lu] [D1%02X - %-20s] %c $%02X\n",
                          sec, us,
                          evt.offset,
                          vera_reg_name(evt.offset, dcsel),
                          (evt.flags & 0x01) ? 'W' : 'R',
                          evt.data);
        }
    }
}
