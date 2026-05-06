#include <Arduino.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pbi-driver.h" // Contains pbi_driver[] array

/**
 * ESP32 Atari PBI/ECI Firmware - Final Architecture
 * 
 * Logic: 
 * - External Address Decoding ($D800-$DFFF) via 74HC138 (Y7 -> ROM_SEL).
 * - MPD handled in hardware by 74HC138.
 * - Core 1: High-speed Bus Monitor (No shared variables).
 * - Core 0: Serial Debug & System Tasks.
 * - Communication: FreeRTOS Queues.
 * 
 * Pin Mapping:
 * - Data Bus (D0-D7): 18, 19, 21, 22, 23, 25, 26, 27
 * - Address Bus (A0-A10): 32, 33, 34, 35, 36, 39 (A0-A5), 16, 17, 14, 12, 13 (A6-A10)
 * - Control: PHI2 (2), R/W (15), D1XX (5), ROM_SEL (4)
 * - Output: EXTSEL (0), DEV_SEL Status (3)
 * - Serial: TX0 (1), RX0 (3 - Disabled)
 */

// --- PIN DEFINITIONS ---
#define PIN_PHI2    2
#define PIN_RW      15
#define PIN_D1XX    5   // Active LOW from Atari
#define PIN_ROM_SEL 4   // Active LOW from 74HC138
#define PIN_EXTSEL  0   // Active LOW Output to Atari
#define PIN_DEV_SEL 3   // Active LOW Status (High = Disabled, Low = Enabled)

#define MASK_DATA_BUS ( (1ULL << 18) | (1ULL << 19) | (1ULL << 21) | (1ULL << 22) | \
                        (1ULL << 23) | (1ULL << 25) | (1ULL << 26) | (1ULL << 27) )

// --- FREE RTOS ---
QueueHandle_t pbiEventQueue;
static uint32_t data_set_lut[256];
static uint8_t esp32_internal_regs[256] = {0}; // Mapping for $D100-$D1FF (offsets 0-255)

void precompute_data_lut()
{
    const uint8_t pins[8] = {18, 19, 21, 22, 23, 25, 26, 27};
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

// Optimized read for A0-A10
static inline uint16_t IRAM_ATTR read_pbi_offset(uint32_t g_low, uint32_t g_high)
{
    uint16_t addr = 0;
    // A0-A5 (GPIO High: 32, 33, 34, 35, 36, 39)
    if (g_high & (1UL << 0)) addr |= (1 << 0);
    if (g_high & (1UL << 1)) addr |= (1 << 1);
    if (g_high & (1UL << 2)) addr |= (1 << 2);
    if (g_high & (1UL << 3)) addr |= (1 << 3);
    if (g_high & (1UL << 4)) addr |= (1 << 4);
    if (g_high & (1UL << 7)) addr |= (1 << 5); // GPIO 39 is bit 7 in GPIO_IN1_REG
    
    // A6-A10 (GPIO Low: 16, 17, 14, 12, 13)
    if (g_low & (1UL << 16)) addr |= (1 << 6);
    if (g_low & (1UL << 17)) addr |= (1 << 7);
    if (g_low & (1UL << 14)) addr |= (1 << 8);
    if (g_low & (1UL << 12)) addr |= (1 << 9);
    if (g_low & (1UL << 13)) addr |= (1 << 10);
    
    return addr;
}

static inline uint8_t IRAM_ATTR read_data_bus(uint32_t g_low)
{
    uint8_t res = 0;
    if (g_low & (1UL << 18)) res |= (1 << 0);
    if (g_low & (1UL << 19)) res |= (1 << 1);
    if (g_low & (1UL << 21)) res |= (1 << 2);
    if (g_low & (1UL << 22)) res |= (1 << 3);
    if (g_low & (1UL << 23)) res |= (1 << 4);
    if (g_low & (1UL << 25)) res |= (1 << 5);
    if (g_low & (1UL << 26)) res |= (1 << 6);
    if (g_low & (1UL << 27)) res |= (1 << 7);
    return res;
}

void IRAM_ATTR MonitorTask(void *pvParameters)
{
    bool local_pbi_enabled = false;

    for (;;)
    {
        uint32_t g_low;
        // 1. Wait for PHI2 Rising Edge
        while (!((g_low = GPIO.in) & (1UL << PIN_PHI2)));

        bool is_read = (g_low & (1UL << PIN_RW));
        bool is_d1xx = !(g_low & (1UL << PIN_D1XX));
        bool is_rom  = !(g_low & (1UL << PIN_ROM_SEL));

        uint32_t g_high = GPIO.in1.val;
        uint16_t offset = read_pbi_offset(g_low, g_high); // A0-A10
        
        // --- LOGICA CHIP SELECT DISPOSITIVO ($D100 - $D11F) ---
        bool is_device_range = is_d1xx && (offset < 0x20);
        // --- LOGICA REGISTRI INTERNI ESP32 ($D120 - $D1FE) ---
        bool is_esp32_range = is_d1xx && (offset >= 0x20 && offset < 0xFF);

        // 2. Hardware Signal Management
        if (is_d1xx || (local_pbi_enabled && is_rom))
        {
            GPIO.out_w1tc = (1UL << PIN_EXTSEL); // Segnala all'Atari che il PBI risponde
        }

        if (local_pbi_enabled && is_device_range)
        {
            GPIO.out_w1tc = (1UL << PIN_DEV_SEL); // Attiva CS per il chip esterno (A0-A4)
        }

        // 3. Risposta al Bus (READ)
        if (is_read)
        {
            if (local_pbi_enabled)
            {
                if (is_rom)
                {
                    uint32_t mask = data_set_lut[pbi_driver[offset & 0x7FF]];
                    GPIO.out_w1ts = mask;
                    GPIO.out_w1tc = (uint32_t)MASK_DATA_BUS & ~mask;
                    GPIO.enable_w1ts = (uint32_t)MASK_DATA_BUS;
                }
                else if (is_esp32_range)
                {
                    uint32_t mask = data_set_lut[esp32_internal_regs[offset & 0xFF]];
                    GPIO.out_w1ts = mask;
                    GPIO.out_w1tc = (uint32_t)MASK_DATA_BUS & ~mask;
                    GPIO.enable_w1ts = (uint32_t)MASK_DATA_BUS;
                }
            }
        }
        else
        {
            // Scrittura: Campionamento a fine ciclo
            while (GPIO.in & (1UL << PIN_PHI2));
            
            if (is_d1xx)
            {
                uint8_t data = read_data_bus(GPIO.in);
                uint16_t off8 = offset & 0xFF;

                if (off8 == 0xFF) // Configurazione $D1FF
                {
                    bool new_state = (data & 0x01);
                    if (new_state != local_pbi_enabled)
                    {
                        local_pbi_enabled = new_state;
                        xQueueSend(pbiEventQueue, &local_pbi_enabled, 0);
                    }
                }
                else if (local_pbi_enabled && off8 >= 0x20)
                {
                    esp32_internal_regs[off8] = data; // Scrittura registri interni (A5-A7 coinvolti)
                }
            }
            GPIO.out_w1ts = (1UL << PIN_EXTSEL) | (1UL << PIN_DEV_SEL);
            continue; 
        }

        // 4. Fine ciclo PHI2
        while (GPIO.in & (1UL << PIN_PHI2));
        GPIO.enable_w1tc = (uint32_t)MASK_DATA_BUS; // ESP32 High-Z
        GPIO.out_w1ts = (1UL << PIN_EXTSEL) | (1UL << PIN_DEV_SEL); // Rilascio Segnali
    }
}

void setup()
{
    // ABILITA DEBUG SERIALE SOLO TX (Libera GPIO 3)
    Serial.begin(115200, SERIAL_8N1, -1, 1); 
    Serial.println("\n[6502 Monitor] PBI Firmware Ready (TX-Only)");
    
    pbiEventQueue = xQueueCreate(4, sizeof(bool));
    precompute_data_lut();

    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    // Inputs Low
    io_conf.pin_bit_mask = MASK_DATA_BUS | (1ULL << PIN_PHI2) | (1ULL << PIN_RW) | 
                           (1ULL << PIN_D1XX) | (1ULL << PIN_ROM_SEL) |
                           (1ULL << 16) | (1ULL << 17) | (1ULL << 14) | (1ULL << 12) | (1ULL << 13);
    gpio_config(&io_conf);

    // Inputs High (A0-A5)
    uint64_t high_mask = (1ULL << 32) | (1ULL << 33) | (1ULL << 34) | 
                         (1ULL << 35) | (1ULL << 36) | (1ULL << 39);
    io_conf.pin_bit_mask = high_mask;
    gpio_config(&io_conf);

    // Outputs: EXTSEL e Status LED/Signal
    pinMode(PIN_EXTSEL, OUTPUT);
    digitalWrite(PIN_EXTSEL, HIGH);
    pinMode(PIN_DEV_SEL, OUTPUT);
    digitalWrite(PIN_DEV_SEL, HIGH);

    // Start Monitor on Core 1
    xTaskCreatePinnedToCore(MonitorTask, "PBI", 4096, NULL, 10, NULL, 1);
}

void loop()
{
    bool pbi_state;
    if (xQueueReceive(pbiEventQueue, &pbi_state, portMAX_DELAY))
    {
        Serial.printf(">>> PBI Device %s\n", pbi_state ? "ENABLED" : "DISABLED");
    }
}
