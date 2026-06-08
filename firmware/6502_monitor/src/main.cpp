/**
 * 6502_monitor -- Optimized ESP32 Monitor for 6502 CPU @ 1.79 MHz
 * 
 * FINAL VERSION: 
 * - RX0 (GPIO 3) used as GPIO for EXTSEL.
 * - TX0 (GPIO 1) kept for Serial Debug.
 * - GPIO 12 used for MPD (Requires external Pull-Down for safe boot).
 * - Avoids GPIO 0.
 * 
 * Signals Mapped (24 total):
 * - D0-D7, A0-A7, PHI2, RW, SEL_N, ROMSEL, RAMSEL, VCS, MPD, EXTSEL.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define BUS_MODE_PBI  0
#define BUS_MODE_CCTL 1

#ifndef BUS_MODE
#define BUS_MODE BUS_MODE_PBI
#endif

// ---------------------------------------------------------------------------
// Pin Assignments (Optimized to use 24 pins + TX0 for Debug)
// ---------------------------------------------------------------------------

// Data Bus (D0-D7) - Bank 0
#define PIN_D0 4
#define PIN_D1 5
#define PIN_D2 13
#define PIN_D3 14
#define PIN_D4 16
#define PIN_D5 17
#define PIN_D6 18
#define PIN_D7 19
static const uint8_t DBUS_PINS[8] = {4, 5, 13, 14, 16, 17, 18, 19};
#define DBUS_MASK ((1UL<<4)|(1UL<<5)|(1UL<<13)|(1UL<<14)|(1UL<<16)|(1UL<<17)|(1UL<<18)|(1UL<<19))

// Address Bus (A0-A7)
#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 39
#define PIN_A4 32
#define PIN_A5 33
#define PIN_A6 21
#define PIN_A7 27

// Control Signals
#define PIN_PHI2    2
#define PIN_RW      15
#define PIN_SEL_N   22    // D1XX_N or CCTL_N
#define PIN_ROMSEL  23    // $D800-$DFFF (PBI)
#define PIN_RAMSEL  26    // $D600-$D7FF (PBI)

// Outputs
#define PIN_EXTSEL  3     // RX0 Pin (Initialized as GPIO)
#define PIN_VCS     25    // Device Select
#define PIN_MPD     12    // Math Pack Disable (WARNING: Must be LOW at boot!)

// ---------------------------------------------------------------------------
// Global State
// ---------------------------------------------------------------------------
volatile bool vcs_enabled = (BUS_MODE == BUS_MODE_CCTL);

// ---------------------------------------------------------------------------
// Decoding Helpers (IRAM optimized)
// ---------------------------------------------------------------------------
static inline uint8_t IRAM_ATTR decode_data(uint32_t lo) {
    return (uint8_t)(
        ((lo >> 4)  & 0x01)       | // D0
        ((lo >> 5)  & 0x01) << 1  | // D1
        ((lo >> 13) & 0x01) << 2  | // D2
        ((lo >> 14) & 0x01) << 3  | // D3
        ((lo >> 16) & 0x01) << 4  | // D4
        ((lo >> 17) & 0x01) << 5  | // D5
        ((lo >> 18) & 0x01) << 6  | // D6
        ((lo >> 19) & 0x01) << 7    // D7
    );
}

static inline uint8_t IRAM_ATTR decode_addr_low(uint32_t lo, uint32_t hi) {
    return (uint8_t)(
        ((hi >> 2)  & 0x01)       | // A0 (34)
        ((hi >> 3)  & 0x01) << 1  | // A1 (35)
        ((hi >> 4)  & 0x01) << 2  | // A2 (36)
        ((hi >> 7)  & 0x01) << 3  | // A3 (39)
        ((hi >> 0)  & 0x01) << 4  | // A4 (32)
        ((hi >> 1)  & 0x01) << 5  | // A5 (33)
        ((lo >> 21) & 0x01) << 6  | // A6 (21)
        ((lo >> 27) & 0x01) << 7    // A7 (27)
    );
}

// ============================================================================
// MonitorTask -- Core 1, IRAM
// ============================================================================
void IRAM_ATTR MonitorTask(void *pvParameters) {
    uint32_t lo, hi;
    uint8_t addr, data;
    
    const uint32_t m_phi2   = (1UL << PIN_PHI2);
    const uint32_t m_rw     = (1UL << PIN_RW);
    const uint32_t m_sel    = (1UL << PIN_SEL_N);
    const uint32_t m_vcs    = (1UL << PIN_VCS);

#if BUS_MODE == BUS_MODE_PBI
    const uint32_t m_romsel = (1UL << PIN_ROMSEL);
    const uint32_t m_ramsel = (1UL << PIN_RAMSEL);
    const uint32_t m_mpd    = (1UL << PIN_MPD);
    const uint32_t m_extsel = (1UL << PIN_EXTSEL);
    GPIO.out_w1ts = m_vcs | m_mpd | m_extsel; 
#else
    GPIO.out_w1ts = m_vcs;
#endif

    while (true) {
        // 1. Wait for PHI2 High
        while (!(GPIO.in & m_phi2));

        lo = GPIO.in;
        hi = GPIO.in1.val;
        addr = decode_addr_low(lo, hi);

#if BUS_MODE == BUS_MODE_PBI
        // --- PBI LOGIC ---
        if (!(lo & m_sel)) {
            // Write to $D1FF handles VCS toggle
            if (!(lo & m_rw) && (addr == 0xFF)) {
                data = decode_data(GPIO.in);
                if (data == 0x80) {
                    vcs_enabled = true;
                    GPIO.out_w1tc = m_vcs;
                } else if (data == 0x00) {
                    vcs_enabled = false;
                    GPIO.out_w1ts = m_vcs;
                }
            }
        }

        // EXTSEL and MPD logic
        if (vcs_enabled) {
            // EXTSEL active if accessing D1xx, D6xx or D8xx
            if (!(lo & (m_sel | m_ramsel | m_romsel))) {
                GPIO.out_w1tc = m_extsel;
            } else {
                GPIO.out_w1ts = m_extsel;
            }
            // MPD active if accessing D8xx range
            if (!(lo & m_romsel)) {
                GPIO.out_w1tc = m_mpd;
            } else {
                GPIO.out_w1ts = m_mpd;
            }
        } else {
            GPIO.out_w1ts = m_extsel | m_mpd;
        }
#else
        // --- CCTL LOGIC ---
        if (!(lo & m_sel)) {
            GPIO.out_w1tc = m_vcs; 
        } else {
            GPIO.out_w1ts = m_vcs;
        }
#endif

        // 3. Wait for PHI2 Low
        while (GPIO.in & m_phi2);
    }
}

void setup() {
    // Initialize UART0 TX only on GPIO 1, disable RX on GPIO 3
    Serial.begin(115200, SERIAL_8N1, -1, 1);
    Serial.println("\n6502 Monitor Started.");
    Serial.println("Note: RX0 is now used as GPIO for EXTSEL.");

    // Explicitly re-configure RX0 (GPIO 3) as Output for EXTSEL
    pinMode(PIN_EXTSEL, OUTPUT);
    digitalWrite(PIN_EXTSEL, HIGH); // Start Inactive (High)

    // Setup Data Bus
    for(int i=0; i<8; i++) pinMode(DBUS_PINS[i], INPUT);
    
    // Setup Control Inputs
    pinMode(PIN_PHI2, INPUT);
    pinMode(PIN_RW, INPUT);
    pinMode(PIN_SEL_N, INPUT);
    
    // Setup Address Bus
    pinMode(PIN_A0, INPUT); pinMode(PIN_A1, INPUT);
    pinMode(PIN_A2, INPUT); pinMode(PIN_A3, INPUT);
    pinMode(PIN_A4, INPUT); pinMode(PIN_A5, INPUT);
    pinMode(PIN_A6, INPUT); pinMode(PIN_A7, INPUT);

    // Setup Outputs
    pinMode(PIN_VCS, OUTPUT);
    digitalWrite(PIN_VCS, HIGH);

#if BUS_MODE == BUS_MODE_PBI
    pinMode(PIN_ROMSEL, INPUT);
    pinMode(PIN_RAMSEL, INPUT);
    pinMode(PIN_MPD, OUTPUT);
    digitalWrite(PIN_MPD, HIGH);
#endif

    // Start Monitor Task on Core 1
    xTaskCreatePinnedToCore(MonitorTask, "Monitor", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}

void loop() {
    // Core 0 handles debug heartbeats
    static bool last_vcs = false;
    if (vcs_enabled != last_vcs) {
        Serial.printf("VCS Status Changed: %s\n", vcs_enabled ? "ACTIVE" : "INACTIVE");
        last_vcs = vcs_enabled;
    }
    delay(500);
}
