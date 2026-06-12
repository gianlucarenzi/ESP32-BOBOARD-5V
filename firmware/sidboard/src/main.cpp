/**
 * ESP32 Atari CCTL/SID Firmware - Sidboard
 *
 * ARCHITECTURE:
 * - SEL_N (GPIO 22) connected to CCTL (Cartridge Control Line, $D5xx).
 * - Core 1: MonitorTask — High-speed bus monitor, no FreeRTOS in hot path.
 * - Core 0: loop() + esp_timer (cSIDLight audio at 22050 Hz on GPIO 25).
 * - Communication: FreeRTOS queues — latchEventQueue (bool) + log_queue.
 *
 * MEMORY MAP ($D5xx via CCTL, A0-A4 decoded):
 * - $D500-$D51E  SID Registers (offsets 0x00-0x1E, 31 registers)
 * - $D5FF        Latch control: write bit 0 = 1 → enable, 0 → disable+reset
 *                (decoded as offset 0x1F — A0-A4 all high)
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>

#include "csidlight.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

cSIDLight *sidLight = nullptr;

// ---------------------------------------------------------------------------
// Pin Definitions
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

static const uint8_t DBUS_PINS[8] = {4, 5, 13, 14, 16, 17, 18, 19};

#define DBUS_MASK \
    ((1UL << 4) | (1UL << 5) | (1UL << 13) | (1UL << 14) | \
     (1UL << 16) | (1UL << 17) | (1UL << 18) | (1UL << 19))

// Address Bus A0-A4 (GPIO_IN1_REG, GPIOs 32-39)
// A0=GPIO34, A1=GPIO35, A2=GPIO36, A3=GPIO39, A4=GPIO32
#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 39
#define PIN_A4 32

// CCTL and common bus signals
#define PIN_SEL_N 22   // CCTL — $D5xx range (Active LOW, Input)
#define PIN_PHI2  2    // 6502 Phase 2 Clock (1.79 MHz PAL)
#define PIN_RW    15   // Read/Write (High = Read, Low = Write)

// GPIO 25 (formerly A9) → ESP32 DAC1, used by cSIDLight audio output.
// GPIO 26 (formerly A10), GPIO 33 (formerly A5), GPIO 21 (formerly A6),
// GPIO 27 (formerly A7), GPIO 12 (formerly A8) are available for future use.
// GPIO 0 (formerly MPD) and GPIO 3 (formerly EXTSEL) are also free.
// GPIO 23 (formerly ROMSEL) is free.

// ---------------------------------------------------------------------------
// Log Queue (Core 1 → Core 0)
// ---------------------------------------------------------------------------
#define LOG_QUEUE_SIZE 64

#define EVT_LATCH 0   // $D5FF latch state change
#define EVT_REG   1   // $D500-$D51E register access

// flags: bit 0 = 1→write / 0→read
//        bit 1 = new latch state (EVT_LATCH only: 1=enabled, 0=disabled)
typedef struct {
    uint64_t ts_us;
    uint8_t  type;
    uint8_t  offset;  // 5-bit decoded address (0x00-0x1F)
    uint8_t  data;
    uint8_t  flags;
} LogEvt;

static QueueHandle_t log_queue;
static QueueHandle_t latchEventQueue;

static IRAM_ATTR uint32_t data_set_lut[256];

static inline void IRAM_ATTR log_send(uint8_t type, uint8_t offset,
                                      uint8_t data, uint8_t flags)
{
    LogEvt evt = {(uint64_t)esp_timer_get_time(), type, offset, data, flags};
    xQueueSend(log_queue, &evt, 0);
}

// ---------------------------------------------------------------------------
// Precompute GPIO bitmask LUT for data bus writes
// ---------------------------------------------------------------------------
static void precompute_data_lut()
{
    for (int i = 0; i < 256; i++)
    {
        uint32_t mask = 0;
        for (int b = 0; b < 8; b++)
            if ((i >> b) & 1) mask |= (1UL << DBUS_PINS[b]);
        data_set_lut[i] = mask;
    }
}

// ---------------------------------------------------------------------------
// Address bus decode: A0-A4 → 5-bit offset (0x00-0x1F)
// GPIO_IN1 bit layout: GPIO32=bit0 … GPIO36=bit4, GPIO39=bit7
// ---------------------------------------------------------------------------
static inline uint8_t IRAM_ATTR decode_addr(uint32_t hi)
{
    return (uint8_t)(((hi >> 2) & 1)      |   // GPIO34 = A0
                     ((hi >> 3) & 1) << 1 |   // GPIO35 = A1
                     ((hi >> 4) & 1) << 2 |   // GPIO36 = A2
                     ((hi >> 7) & 1) << 3 |   // GPIO39 = A3
                     ((hi >> 0) & 1) << 4);   // GPIO32 = A4
}

// ---------------------------------------------------------------------------
// Data bus decode from GPIO_IN_REG
// ---------------------------------------------------------------------------
static inline uint8_t IRAM_ATTR decode_data(uint32_t lo)
{
    return (uint8_t)(((lo >> 4)  & 1)      |   // GPIO4  = D0
                     ((lo >> 5)  & 1) << 1 |   // GPIO5  = D1
                     ((lo >> 13) & 1) << 2 |   // GPIO13 = D2
                     ((lo >> 14) & 1) << 3 |   // GPIO14 = D3
                     ((lo >> 16) & 1) << 4 |   // GPIO16 = D4
                     ((lo >> 17) & 1) << 5 |   // GPIO17 = D5
                     ((lo >> 18) & 1) << 6 |   // GPIO18 = D6
                     ((lo >> 19) & 1) << 7);   // GPIO19 = D7
}

// ---------------------------------------------------------------------------
// Data bus drive helpers
// ---------------------------------------------------------------------------
static inline void IRAM_ATTR bus_drive(uint8_t val)
{
    uint32_t m       = data_set_lut[val];
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
    bool latch_enabled = false;

    const uint32_t m_phi2 = (1UL << PIN_PHI2);
    const uint32_t m_rw   = (1UL << PIN_RW);
    const uint32_t m_sel  = (1UL << PIN_SEL_N);

    for (;;)
    {
        uint32_t lo;

        // 1. SYNC: Wait for PHI2 Rising Edge
        while (!((lo = GPIO.in) & m_phi2))
            ;

        bool is_cctl = !(lo & m_sel);
        if (!is_cctl)
            continue;

        bool     is_read = (lo & m_rw);
        uint8_t  offset  = decode_addr(GPIO.in1.val);  // 0x00-0x1F

        // 2. READ RESPONSE
        if (is_read)
        {
            if (latch_enabled && offset < SID_NUM_REGS)
                bus_drive(sidLight->read(offset));

            log_send(EVT_REG, offset, 0, 0x00);

            // Release on PHI2 falling edge
            while (GPIO.in & m_phi2)
                ;
            bus_release();
        }
        else
        {
            // 3. WRITE: Wait for PHI2 Falling Edge (data stable)
            while (GPIO.in & m_phi2)
                ;

            uint8_t data = decode_data(GPIO.in);

            if (offset == 0x1F)  // Latch control ($D5FF decoded as A0-A4 all 1)
            {
                bool new_state = (data & 0x01);
                if (new_state != latch_enabled)
                {
                    latch_enabled = new_state;
                    xQueueSend(latchEventQueue, &latch_enabled, 0);
                }
                uint8_t fl = 0x01 | (new_state ? 0x02 : 0x00);
                log_send(EVT_LATCH, 0x1F, data, fl);
            }
            else if (latch_enabled && offset < SID_NUM_REGS)
            {
                sidLight->write(offset, data);
                log_send(EVT_REG, offset, data, 0x01);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// SID register name lookup — Core 0, not IRAM-resident
// ---------------------------------------------------------------------------
static const char *sid_reg_name(uint8_t offset)
{
    static const char *names[31] = {
        "V1_FREQ_LO", "V1_FREQ_HI", "V1_PW_LO",  "V1_PW_HI",
        "V1_CTRL",    "V1_AD",      "V1_SR",      "V2_FREQ_LO",
        "V2_FREQ_HI", "V2_PW_LO",  "V2_PW_HI",   "V2_CTRL",
        "V2_AD",      "V2_SR",      "V3_FREQ_LO", "V3_FREQ_HI",
        "V3_PW_LO",   "V3_PW_HI",  "V3_CTRL",    "V3_AD",
        "V3_SR",      "FC_LO",      "FC_HI",      "RES_FILT",
        "MODE_VOL",   "OSC3",       "ENV3",        "unused27",
        "unused28",   "unused29",   "unused30",
    };
    if (offset < 31) return names[offset];
    return "?";
}

// ============================================================================
// System Setup
// ============================================================================
void setup()
{
    Serial.begin(115200, SERIAL_8N1, -1, 1);
    Serial.println("\n[sidboard] CCTL/SID Firmware Ready (TX-Only)");

    latchEventQueue = xQueueCreate(4, sizeof(bool));
    log_queue       = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogEvt));

    precompute_data_lut();

    // Data bus: high-impedance inputs initially
    for (int i = 0; i < 8; i++) pinMode(DBUS_PINS[i], INPUT);

    // Address A0-A4 and bus control inputs
    pinMode(PIN_A0,    INPUT);
    pinMode(PIN_A1,    INPUT);
    pinMode(PIN_A2,    INPUT);
    pinMode(PIN_A3,    INPUT);
    pinMode(PIN_A4,    INPUT);
    pinMode(PIN_PHI2,  INPUT);
    pinMode(PIN_RW,    INPUT);
    pinMode(PIN_SEL_N, INPUT);

    // cSIDLight: 3-voice square-wave synthesis, DAC1 output on GPIO 25
    sidLight = new cSIDLight();
    Serial.println("[sidboard] cSIDLight initialized, DAC1 on GPIO 25");

    xTaskCreatePinnedToCore(MonitorTask, "CCTL", 4096, NULL, 10, NULL, 1);
    Serial.println("[sidboard] MonitorTask running on Core 1.");
}

// ============================================================================
// Loop -- Core 0: audio processing + log drain
// ============================================================================
void loop()
{
    sidLight->process();

    bool latch_state;
    if (xQueueReceive(latchEventQueue, &latch_state, 0) == pdTRUE)
    {
        Serial.printf(">>> Latch %s\n", latch_state ? "ENABLED" : "DISABLED");
        if (!latch_state) sidLight->reset();
    }

    LogEvt evt;
    while (xQueueReceive(log_queue, &evt, 0) == pdTRUE)
    {
        uint32_t sec = (uint32_t)(evt.ts_us / 1000000ULL);
        uint32_t us  = (uint32_t)(evt.ts_us % 1000000ULL);

        if (evt.type == EVT_LATCH)
        {
            bool enabled = (evt.flags & 0x02) != 0;
            Serial.printf("[%5lu.%06lu] [LATCH] $D5FF %s ($%02X)\n",
                          sec, us,
                          enabled ? "ENABLED " : "DISABLED",
                          evt.data);
        }
        else
        {
            uint8_t     off  = evt.offset;
            const char *name = (off < 31) ? sid_reg_name(off) : "LATCH_CTL";
            Serial.printf("[%5lu.%06lu] [D5%02X - %-12s] %c $%02X\n",
                          sec, us,
                          off,
                          name,
                          (evt.flags & 0x01) ? 'W' : 'R',
                          evt.data);
        }
    }
}
