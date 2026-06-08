#include <Arduino.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>

#include "csidlight.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pbi-driver.h"  // Contains pbi_driver[] array

/**
 * ESP32 Atari PBI/ECI Firmware - Sidboard
 *
 * ARCHITECTURE:
 * - External Address Decoding: $D800-$DFFF via 74HC138 (Y7 -> ROM_SEL).
 * - Core 1: MonitorTask — High-speed bus monitor, no shared variables.
 * - Core 0: loop() + esp_timer (cSIDLight audio) + system tasks.
 * - Communication: FreeRTOS Queue (PBI enable/disable events only).
 *
 * MEMORY MAP ($D1xx):
 * - $D100-$D11F  SID Registers emulated by cSIDLight (enabled by $D1FF bit 0)
 * - $D120-$D1FE  Internal ESP32 Registers
 * - $D1FF        Bit 0 = 1: Enable device / Bit 0 = 0: Disable + reset SID
 */

cSIDLight *sidLight = nullptr;

// --- PIN DEFINITIONS ---
#define PIN_PHI2    2   // Phase 2 Clock (Input)
#define PIN_RW      15  // Read/Write (Input, High=Read)
#define PIN_D1XX    5   // $D1XX page selection (Active LOW Input)
#define PIN_ROM_SEL 4   // $D800-$DFFF range (Active LOW Input from 74HC138)
#define PIN_EXTSEL  0   // External Select (Active LOW Output)

// Data Bus Pins: GPIO 18, 19, 21, 22, 23, 3, 26, 27
#define MASK_DATA_BUS                                            \
    ((1ULL << 18) | (1ULL << 19) | (1ULL << 21) | (1ULL << 22) | \
     (1ULL << 23) | (1ULL << 3) | (1ULL << 26) | (1ULL << 27))

// --- FREE RTOS ---
QueueHandle_t   pbiEventQueue;
static uint32_t data_set_lut[256];
static uint8_t  esp32_internal_regs[256] = {0};  // $D100-$D1FF (offsets 0-255)

/**
 * Precomputes the GPIO bitmask LUT for the data bus.
 */
void precompute_data_lut()
{
    const uint8_t pins[8] = {18, 19, 21, 22, 23, 3, 26, 27};
    for (int i = 0; i < 256; i++)
    {
        uint32_t mask = 0;
        for (int b = 0; b < 8; b++)
        {
            if (i & (1 << b))
            {
                mask |= (1UL << pins[b]);
            }
        }
        data_set_lut[i] = mask;
    }
}

/**
 * Optimized read for Address Bus A0-A10.
 */
static inline uint16_t IRAM_ATTR read_pbi_offset(uint32_t g_low,
                                                 uint32_t g_high)
{
    uint16_t addr = 0;
    // A0-A5 (GPIO High: 32, 33, 34, 35, 36, 39)
    if (g_high & (1UL << 0)) addr |= (1 << 0);
    if (g_high & (1UL << 1)) addr |= (1 << 1);
    if (g_high & (1UL << 2)) addr |= (1 << 2);
    if (g_high & (1UL << 3)) addr |= (1 << 3);
    if (g_high & (1UL << 4)) addr |= (1 << 4);
    if (g_high & (1UL << 7))
        addr |= (1 << 5);  // GPIO 39 is bit 7 in GPIO_IN1_REG

    // A6-A10 (GPIO Low: 16, 17, 14, 12, 13)
    if (g_low & (1UL << 16)) addr |= (1 << 6);
    if (g_low & (1UL << 17)) addr |= (1 << 7);
    if (g_low & (1UL << 14)) addr |= (1 << 8);
    if (g_low & (1UL << 12)) addr |= (1 << 9);
    if (g_low & (1UL << 13)) addr |= (1 << 10);

    return addr;
}

/**
 * Reads the data bus bits from GPIO_IN_REG.
 */
static inline uint8_t IRAM_ATTR read_data_bus(uint32_t g_low)
{
    uint8_t res = 0;
    if (g_low & (1UL << 18)) res |= (1 << 0);
    if (g_low & (1UL << 19)) res |= (1 << 1);
    if (g_low & (1UL << 21)) res |= (1 << 2);
    if (g_low & (1UL << 22)) res |= (1 << 3);
    if (g_low & (1UL << 23)) res |= (1 << 4);
    if (g_low & (1UL << 3)) res |= (1 << 5);
    if (g_low & (1UL << 26)) res |= (1 << 6);
    if (g_low & (1UL << 27)) res |= (1 << 7);
    return res;
}

// ============================================================================
// MonitorTask -- Core 1 High-Speed Bus Handler
// ============================================================================
void IRAM_ATTR MonitorTask(void *pvParameters)
{
    bool local_pbi_enabled = false;

    for (;;)
    {
        uint32_t g_low;
        // 1. SYNC: Wait for PHI2 Rising Edge
        while (!((g_low = GPIO.in) & (1UL << PIN_PHI2)))
            ;

        bool is_read = (g_low & (1UL << PIN_RW));
        bool is_d1xx = !(g_low & (1UL << PIN_D1XX));
        bool is_rom  = !(g_low & (1UL << PIN_ROM_SEL));

        uint32_t g_high = GPIO.in1.val;
        uint16_t offset = read_pbi_offset(g_low, g_high);  // A0-A10

        // --- DECODE D1xx SUB-RANGES ---
        bool is_sid_range = is_d1xx && (offset < 0x20);  // $D100-$D11F SID
        bool is_esp32_range =
            is_d1xx &&
            (offset >= 0x20 && offset < 0xFF);  // $D120-$D1FE internal

        // 2. BUS SIGNALING
        if (is_d1xx || (local_pbi_enabled && is_rom))
        {
            GPIO.out_w1tc =
                (1UL << PIN_EXTSEL);  // Signal Atari that PBI responds
        }

        // 3. READ RESPONSE
        if (is_read)
        {
            if (local_pbi_enabled)
            {
                if (is_rom)
                {
                    uint32_t mask    = data_set_lut[pbi_driver[offset & 0x7FF]];
                    GPIO.out_w1ts    = mask;
                    GPIO.out_w1tc    = (uint32_t)MASK_DATA_BUS & ~mask;
                    GPIO.enable_w1ts = (uint32_t)MASK_DATA_BUS;
                }
                else if (is_sid_range)
                {
                    uint32_t mask = data_set_lut[sidLight->read(offset & 0x1F)];
                    GPIO.out_w1ts = mask;
                    GPIO.out_w1tc = (uint32_t)MASK_DATA_BUS & ~mask;
                    GPIO.enable_w1ts = (uint32_t)MASK_DATA_BUS;
                }
                else if (is_esp32_range)
                {
                    uint32_t mask =
                        data_set_lut[esp32_internal_regs[offset & 0xFF]];
                    GPIO.out_w1ts    = mask;
                    GPIO.out_w1tc    = (uint32_t)MASK_DATA_BUS & ~mask;
                    GPIO.enable_w1ts = (uint32_t)MASK_DATA_BUS;
                }
            }
        }
        else
        {
            // 4. WRITE SAMPLING: Wait for PHI2 Falling Edge
            while (GPIO.in & (1UL << PIN_PHI2))
                ;

            if (is_d1xx)
            {
                uint8_t  data = read_data_bus(GPIO.in);
                uint16_t off8 = offset & 0xFF;

                if (off8 == 0xFF)  // Config Register $D1FF
                {
                    bool new_state = (data & 0x01);
                    if (new_state != local_pbi_enabled)
                    {
                        local_pbi_enabled = new_state;
                        xQueueSend(pbiEventQueue, &local_pbi_enabled, 0);
                    }
                }
                else if (local_pbi_enabled &&
                         off8 < 0x20)  // SID Registers $D100-$D11F
                {
                    sidLight->write(off8, data);
                }
                else if (local_pbi_enabled && off8 >= 0x20)
                {
                    esp32_internal_regs[off8] =
                        data;  // Internal register write
                }
            }
            GPIO.out_w1ts = (1UL << PIN_EXTSEL);
            continue;
        }

        // 5. RELEASE: Wait for PHI2 Falling Edge
        while (GPIO.in & (1UL << PIN_PHI2))
            ;
        GPIO.enable_w1tc = (uint32_t)MASK_DATA_BUS;  // ESP32 High-Z
        GPIO.out_w1ts    = (1UL << PIN_EXTSEL);      // Release signals
    }
}

// ============================================================================
// System Setup
// ============================================================================
void setup()
{
    // SERIAL DEBUG: TX-Only to free GPIO 3
    Serial.begin(115200, SERIAL_8N1, -1, 1);
    Serial.println("\n[sidboard] PBI Firmware Ready (TX-Only)");

    pbiEventQueue = xQueueCreate(4, sizeof(bool));
    precompute_data_lut();

    gpio_config_t io_conf = {};
    io_conf.mode          = GPIO_MODE_INPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;

    // Inputs (Address A6-A10 and control)
    io_conf.pin_bit_mask = MASK_DATA_BUS | (1ULL << PIN_PHI2) |
                           (1ULL << PIN_RW) | (1ULL << PIN_D1XX) |
                           (1ULL << PIN_ROM_SEL) | (1ULL << 16) | (1ULL << 17) |
                           (1ULL << 14) | (1ULL << 12) | (1ULL << 13);
    gpio_config(&io_conf);

    // Inputs (Address A0-A5)
    uint64_t high_mask = (1ULL << 32) | (1ULL << 33) | (1ULL << 34) |
                         (1ULL << 35) | (1ULL << 36) | (1ULL << 39);
    io_conf.pin_bit_mask = high_mask;
    gpio_config(&io_conf);

    // Outputs: EXTSEL
    pinMode(PIN_EXTSEL, OUTPUT);
    digitalWrite(PIN_EXTSEL, HIGH);

    // Initialize cSIDLight Engine (DAC on GPIO 25)
    sidLight = new cSIDLight();
    Serial.println(
        "[sidboard] cSIDLight initialized on $D100-$D11F, DAC GPIO25");

    // Start Monitor on Core 1
    xTaskCreatePinnedToCore(MonitorTask, "PBI", 4096, NULL, 10, NULL, 1);
}

void loop()
{
    // Audio engine periodic processing
    sidLight->process();

    bool pbi_state;
    if (xQueueReceive(pbiEventQueue, &pbi_state, 10 / portTICK_PERIOD_MS))
    {
        Serial.printf(">>> PBI Device %s\n",
                      pbi_state ? "ENABLED" : "DISABLED");
        if (!pbi_state) sidLight->reset();  // Silence all voices when disabled
    }
}
