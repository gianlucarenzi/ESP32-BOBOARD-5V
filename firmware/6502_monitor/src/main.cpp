/**
 * 6502_monitor -- ESP32 PBI ROM Emulator ($D800-$DFFF)
 *
 * FIRMWARE: Targets NodeMCU DevKit V1 (ESP32-WROOM, Xtensa LX6).
 *
 * BEHAVIOUR:
 * - ROMSEL ($D800-$DFFF): only when the latch is active (PBI rule: a device
 *   not selected through $D1FF must not answer), asserts MPD low and drives
 *   pbi_rom[A0-A10] on reads.
 *   Full 2 KB addressing via A0-A10 (no aliasing).
 * - EXTSEL ($D1XX): asserted low when the internal latch is active.
 * - Latch (PBI mode):  set by writing $80 to $D1FF, cleared by any other value.
 * - Latch (CCTL mode): same mechanism, latch address is $D5FF instead of $D1FF;
 *   ROMSEL / MPD / EXTSEL follow the latch exactly as in PBI mode.
 * - LOG: lock-free ring from the Core 1 bus loop to LogTask on Core 0; prints
 *   VCS latch changes and every $D100-$D1FE read/write access.
 *
 * Bus loop optimizations ported from Vera-Module-RBL-XE (ESP32-S3), adapted to
 * the ESP32 classic, which has no dedicated GPIO and no ee.* instructions:
 * - Core 1 belongs to the bus loop: interrupts masked, interrupt watchdog off,
 *   no FreeRTOS call; Serial and the log printer run on Core 0.
 * - Hot-loop data in DRAM (IRAM is not byte-addressable on the ESP32).
 * - Table-driven address decode and precomputed GPIO.out words for the ROM.
 * - One GPIO.out store per cycle for D0-D7, MPD and EXTSEL; GPIO accesses in
 *   inline assembly, without the memw the compiler puts around volatiles.
 * - Write data sampled at a fixed time after PHI2 rises (CCOUNT), read data
 *   of the VERA registers sampled just before PHI2 falls.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <soc/timer_group_struct.h>
#include <hal/wdt_hal.h>
#include <hal/cpu_ll.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pbi-driver.h"

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

// UART0 TX for the log (RX unused: GPIO3 is EXTSEL)
#define PIN_UART_TX 1

static const uint8_t DBUS_PINS[8] = {PIN_D0, PIN_D1, PIN_D2, PIN_D3,
                                     PIN_D4, PIN_D5, PIN_D6, PIN_D7};

#define DBUS_MASK                                                          \
    ((1UL << PIN_D0) | (1UL << PIN_D1) | (1UL << PIN_D2) | (1UL << PIN_D3) | \
     (1UL << PIN_D4) | (1UL << PIN_D5) | (1UL << PIN_D6) | (1UL << PIN_D7))

/*
 * GPIO.out is written as a whole once per bus cycle: D0-D7, MPD and EXTSEL
 * are the only GPIO outputs of Bank 0 (GPIO 0-31).  The other Bank 0 pins
 * are inputs, or GPIO1 = U0TXD, which the IO_MUX routes to the UART and
 * ignores GPIO.out.  A new GPIO output on Bank 0 must be added to the word.
 */
static_assert(PIN_MPD < 32 && PIN_EXTSEL < 32 && PIN_PHI2 < 32 && PIN_RW < 32 &&
              PIN_SEL_N < 32 && PIN_ROMSEL < 32, "bus loop expects these pins in Bank 0");
static_assert(PIN_MPD == 0 && PIN_EXTSEL == 3,
              "the bus loop assembly builds MPD (bit 0) and EXTSEL (bit 3) with extui/slli");
static_assert(PIN_A0 >= 32 && PIN_A1 >= 32 && PIN_A2 >= 32 && PIN_A3 >= 32 &&
              PIN_A4 >= 32 && PIN_A5 >= 32, "A0-A5 are decoded from GPIO.in1");

// ---------------------------------------------------------------------------
// Bus Timing
//
// The loop detects the PHI2 rising edge, samples address, R/W, SEL_N and
// ROMSEL, drives MPD / EXTSEL and the ROM byte.  Write data is valid t_MDS
// after PHI2 rises (typ. 80-100 ns, see HARDWARE.md): it is sampled
// BUS_WR_SAMPLE_NS after the detected rising edge, not right after the
// decode.  Tune with -D after measuring the real bus.
// ---------------------------------------------------------------------------
#ifndef BUS_WR_SAMPLE_NS
#define BUS_WR_SAMPLE_NS 120u
#endif

#define CPU_MHZ          (F_CPU / 1000000UL)
#define NS_TO_CYCLES(ns) (((ns) * CPU_MHZ + 999u) / 1000u)

// ---------------------------------------------------------------------------
// Log Ring (Core 1 bus loop → Core 0 LogTask)
//
// Single producer / single consumer, lock-free: the bus loop only stores the
// event and advances log_head (no FreeRTOS call, interrupts are masked on
// Core 1).  When the ring is full new events are dropped and counted.
// ---------------------------------------------------------------------------
#define LOG_RING_SIZE 1024u   // power of two

#define EVT_LATCH 0   // VCS latch state change
#define EVT_REG   1   // $D100-$D1FE register access

// flags: bit 0 = 1→write / 0→read   bit 1 = latch state (EVT_LATCH only)   bit 2 = MPD asserted (LO)   bit 3 = EXTSEL asserted (LO)
typedef struct {
    uint32_t cyc_lo;  // timestamp: 64-bit Core 1 CCOUNT since the bus loop started
    uint32_t cyc_hi;
    uint8_t  type;
    uint8_t  offset;  // register offset within $D1XX (0x00-0xFE)
    uint8_t  data;
    uint8_t  flags;
} LogEvt;

static DRAM_ATTR LogEvt   log_ring[LOG_RING_SIZE];
static DRAM_ATTR uint32_t log_head    = 0;  // written by the bus loop only
static DRAM_ATTR uint32_t log_tail    = 0;  // written by LogTask only
static DRAM_ATTR uint32_t log_dropped = 0;  // written by the bus loop only

static volatile bool log_ready = false;     // Serial started by LogTask

// ---------------------------------------------------------------------------
// PBI ROM & Decode Tables
//
// Everything the bus loop reads lives in DRAM: on the ESP32 IRAM only
// supports 32-bit accesses, an 8-bit load from it raises LoadStoreError.
// ---------------------------------------------------------------------------
// Full 2 KB ROM image — A0-A10 decoded, no aliasing.
static DRAM_ATTR uint8_t pbi_rom[2048];

// GPIO.out data bits (D0-D7) for each ROM byte, indexed by A0-A10.
static DRAM_ATTR uint32_t rom_out[2048];

// A0-A5 from GPIO.in1 bits 0-7 (GPIO 32-39).
static DRAM_ATTR uint16_t addr_lut_hi[256];
// A6, A7, A9, A10 from GPIO.in bits 21-27 (GPIO 21, 27, 25, 26).
static DRAM_ATTR uint16_t addr_lut_lo[128];

static_assert(PIN_A6 >= 21 && PIN_A6 <= 27 && PIN_A7 >= 21 && PIN_A7 <= 27 &&
              PIN_A9 >= 21 && PIN_A9 <= 27 && PIN_A10 >= 21 && PIN_A10 <= 27 &&
              PIN_A8 == 12, "addr_lut_lo covers GPIO 21-27, A8 is GPIO 12");

// ---------------------------------------------------------------------------
// Decoding Helpers
// ---------------------------------------------------------------------------
#define BUS_INLINE static inline __attribute__((always_inline))

// D0-D7 on GPIO 4,5 | 13,14 | 16-19.
static_assert(PIN_D0 == 4 && PIN_D1 == 5 && PIN_D2 == 13 && PIN_D3 == 14 &&
              PIN_D4 == 16 && PIN_D5 == 17 && PIN_D6 == 18 && PIN_D7 == 19,
              "decode_data() hard-codes the data bus pins");

BUS_INLINE uint8_t decode_data(uint32_t lo)
{
    return (uint8_t)(((lo >> 4) & 0x03u) | ((lo >> 11) & 0x0Cu) | ((lo >> 12) & 0xF0u));
}

// 11-bit address (A0-A10) for indexing pbi_rom[].
BUS_INLINE uint32_t decode_addr(uint32_t lo, uint32_t hi)
{
    return addr_lut_hi[hi & 0xFFu] | addr_lut_lo[(lo >> 21) & 0x7Fu] |
           ((lo >> (PIN_A8 - 8)) & 0x100u);
}

static uint16_t addr_bits(uint32_t word, const uint8_t *pins, const uint8_t *abits,
                          int n, int base)
{
    uint16_t a = 0;
    for (int i = 0; i < n; i++)
        if ((word >> (pins[i] - base)) & 1u)
            a |= (uint16_t)(1u << abits[i]);
    return a;
}

static void build_tables(void)
{
    static const uint8_t hi_pins[6]  = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A4, PIN_A5};
    static const uint8_t hi_abits[6] = {0, 1, 2, 3, 4, 5};
    static const uint8_t lo_pins[4]  = {PIN_A6, PIN_A7, PIN_A9, PIN_A10};
    static const uint8_t lo_abits[4] = {6, 7, 9, 10};

    for (uint32_t i = 0; i < 256; i++)
        addr_lut_hi[i] = addr_bits(i, hi_pins, hi_abits, 6, 32);
    for (uint32_t i = 0; i < 128; i++)
        addr_lut_lo[i] = addr_bits(i, lo_pins, lo_abits, 4, 21);

    for (uint32_t a = 0; a < 2048; a++)
    {
        uint32_t m = 0;
        for (int b = 0; b < 8; b++)
            if ((pbi_rom[a] >> b) & 1u)
                m |= (1UL << DBUS_PINS[b]);
        rom_out[a] = m;
    }
}

// ---------------------------------------------------------------------------
// GPIO access without memw (the compiler emits one before every volatile
// access).  Xtensa keeps loads and stores in program order, so GPIO.out
// still reaches the pins before GPIO.enable_w1ts turns the outputs on; the
// CPU just does not wait for each APB access to complete.
// ---------------------------------------------------------------------------
#define GPIO_OFF(f) offsetof(gpio_dev_t, f)

BUS_INLINE uint32_t gpio_in(void)
{
    uint32_t v;
    asm volatile("l32i %0, %1, %2" : "=r"(v) : "r"(&GPIO), "n"(GPIO_OFF(in)));
    return v;
}

BUS_INLINE uint32_t gpio_in1(void)
{
    uint32_t v;
    asm volatile("l32i %0, %1, %2" : "=r"(v) : "r"(&GPIO), "n"(GPIO_OFF(in1)));
    return v;
}

BUS_INLINE void gpio_out(uint32_t v)
{
    asm volatile("s32i %0, %1, %2" : : "r"(v), "r"(&GPIO), "n"(GPIO_OFF(out)) : "memory");
}

BUS_INLINE void bus_drive_enable(void)
{
    asm volatile("s32i %0, %1, %2" : : "r"(DBUS_MASK), "r"(&GPIO), "n"(GPIO_OFF(enable_w1ts))
                 : "memory");
}

BUS_INLINE void bus_release(void)
{
    asm volatile("s32i %0, %1, %2" : : "r"(DBUS_MASK), "r"(&GPIO), "n"(GPIO_OFF(enable_w1tc))
                 : "memory");
}

// PHI2 edge waits: one GPIO.in load and one bit branch per sample.
// They return the sample that saw the edge.
BUS_INLINE uint32_t wait_phi2_high(void)
{
    uint32_t v;
    asm volatile("1: l32i %0, %1, %2 \n\t"
                 "   bbci %0, %3, 1b"
                 : "=&r"(v) : "r"(&GPIO), "n"(GPIO_OFF(in)), "n"(PIN_PHI2));
    return v;
}

BUS_INLINE uint32_t wait_phi2_low(void)
{
    uint32_t v;
    asm volatile("1: l32i %0, %1, %2 \n\t"
                 "   bbsi %0, %3, 1b"
                 : "=&r"(v) : "r"(&GPIO), "n"(GPIO_OFF(in)), "n"(PIN_PHI2));
    return v;
}

// Push a log event into the ring without blocking (drops if full).
BUS_INLINE void log_send(uint32_t cyc_lo, uint32_t cyc_hi, uint8_t type,
                         uint8_t offset, uint8_t data, uint8_t flags)
{
    uint32_t h = log_head;
    if (h - __atomic_load_n(&log_tail, __ATOMIC_ACQUIRE) >= LOG_RING_SIZE)
    {
        log_dropped++;
        return;
    }
    LogEvt *e = &log_ring[h & (LOG_RING_SIZE - 1u)];
    e->cyc_lo = cyc_lo;
    e->cyc_hi = cyc_hi;
    e->type   = type;
    e->offset = offset;
    e->data   = data;
    e->flags  = flags;
    __atomic_store_n(&log_head, h + 1u, __ATOMIC_RELEASE);
}

/*
 * The bus loop runs with interrupts masked, so Core 1 no longer runs the
 * FreeRTOS tick hook that feeds the interrupt watchdog (MWDT1, which checks
 * CPU1 too: CONFIG_ESP_INT_WDT_CHECK_CPU1).  Stop it, or it resets the chip
 * after 300 ms.
 */
static void bus_int_wdt_disable(void)
{
    wdt_hal_context_t ctx = {};
    ctx.inst     = WDT_MWDT1;
    ctx.mwdt_dev = &TIMERG1;
    wdt_hal_write_protect_disable(&ctx);
    wdt_hal_disable(&ctx);
    wdt_hal_write_protect_enable(&ctx);
}

// ============================================================================
// MonitorTask -- Core 1, IRAM-resident, interrupts masked
//
// Core 1 runs only this task: it never yields and nothing else runs there
// (no tick, no interrupts), so no bus cycle is missed.  Core 0 runs LogTask,
// Serial and its UART interrupt.
// ============================================================================
static void IRAM_ATTR MonitorTask(void *pvParameters)
{
    const uint32_t m_phi2   = (1UL << PIN_PHI2);
    const uint32_t m_rw     = (1UL << PIN_RW);
    const uint32_t m_sel    = (1UL << PIN_SEL_N);
    const uint32_t m_romsel = (1UL << PIN_ROMSEL);
    const uint32_t m_mpd    = (1UL << PIN_MPD);
    const uint32_t m_extsel = (1UL << PIN_EXTSEL);

    const uint32_t wr_sample = NS_TO_CYCLES(BUS_WR_SAMPLE_NS);

    bool     latch  = false;   // internal device-active latch
    uint32_t cyc_hi = 0;       // CCOUNT wraps every 2^32 / 240 MHz = 17.9 s
    uint32_t t_prev = cpu_ll_get_cycle_count();

    // Initial state: outputs inactive (HIGH), data bus released.
    bus_release();
    gpio_out(m_mpd | m_extsel);

    // From here on Core 1 belongs to the bus: no tick, no interrupts.
    bus_int_wdt_disable();
    portDISABLE_INTERRUPTS();

    for (;;)
    {
        /*
         * 1-4. Critical path, in assembly so that every operand stays in a
         * register and the constants are loaded before the wait:
         *   1. sync on the PHI2 rising edge;
         *   2. sample address and control lines (GPIO.in, GPIO.in1);
         *   3. control outputs: with the latch set MPD = ROMSEL level and
         *      EXTSEL = SEL_N level (asserted in their windows); with the
         *      latch clear both stay HIGH (ctrl_or);
         *   4. ROM read with the latch set: A0-A10 from the tables, D0-D7
         *      from rom_out[], one GPIO.out store with the control bits,
         *      then the outputs on.
         */
        uint32_t lo, hi, t_rise, out, t, a;
        const uint32_t ctrl_or = latch ? 0u : (m_mpd | m_extsel);
        asm volatile(
            "1: l32i   %[lo], %[gpio], %[o_in]    \n\t"
            "   bbci   %[lo], %[b_phi2], 1b       \n\t"
            "   rsr.ccount %[tr]                  \n\t"
            "   l32i   %[lo], %[gpio], %[o_in]    \n\t"   /* GPIO.in  */
            "   l32i   %[hi], %[gpio], %[o_in1]   \n\t"   /* GPIO.in1 */
            "   extui  %[out], %[lo], %[b_rom], 1 \n\t"   /* MPD level  -> bit 0 */
            "   extui  %[t], %[lo], %[b_sel], 1   \n\t"
            "   slli   %[t], %[t], %[b_ext]       \n\t"   /* EXTSEL level -> bit 3 */
            "   or     %[out], %[out], %[t]       \n\t"
            "   or     %[out], %[out], %[ctrl_or] \n\t"
            "   bnez   %[ctrl_or], 2f            \n\t"   /* latch clear: no ROM */
            "   bbsi   %[lo], %[b_rom], 2f        \n\t"   /* ROMSEL not active */
            "   bbci   %[lo], %[b_rw], 2f         \n\t"   /* write cycle */
            "   extui  %[t], %[hi], 0, 8          \n\t"   /* GPIO 32-39 */
            "   addx2  %[t], %[t], %[luthi]       \n\t"
            "   l16ui  %[a], %[t], 0              \n\t"   /* A0-A5 */
            "   extui  %[t], %[lo], 21, 7         \n\t"   /* GPIO 21-27 */
            "   addx2  %[t], %[t], %[lutlo]       \n\t"
            "   l16ui  %[t], %[t], 0              \n\t"   /* A6, A7, A9, A10 */
            "   or     %[a], %[a], %[t]           \n\t"
            "   extui  %[t], %[lo], %[b_a8], 1    \n\t"
            "   slli   %[t], %[t], 8              \n\t"   /* A8 */
            "   or     %[a], %[a], %[t]           \n\t"
            "   addx4  %[t], %[a], %[romout]      \n\t"
            "   l32i   %[t], %[t], 0              \n\t"   /* rom_out[A0-A10] */
            "   or     %[t], %[t], %[out]         \n\t"
            "   s32i   %[t], %[gpio], %[o_out]    \n\t"   /* GPIO.out */
            "   s32i   %[dmask], %[gpio], %[o_en] \n\t"   /* GPIO.enable_w1ts */
            "   j      3f                         \n\t"
            "2: s32i   %[out], %[gpio], %[o_out]  \n\t"   /* GPIO.out, controls only */
            "3:                                   \n\t"
            : [lo] "=&r"(lo), [hi] "=&r"(hi), [tr] "=&r"(t_rise),
              [out] "=&r"(out), [t] "=&r"(t), [a] "=&r"(a)
            : [gpio] "r"(&GPIO), [ctrl_or] "r"(ctrl_or),
              [luthi] "r"(addr_lut_hi), [lutlo] "r"(addr_lut_lo),
              [romout] "r"(rom_out), [dmask] "r"(DBUS_MASK),
              [o_in] "n"(GPIO_OFF(in)), [o_in1] "n"(GPIO_OFF(in1)),
              [o_out] "n"(GPIO_OFF(out)), [o_en] "n"(GPIO_OFF(enable_w1ts)),
              [b_phi2] "n"(PIN_PHI2), [b_rw] "n"(PIN_RW), [b_rom] "n"(PIN_ROMSEL),
              [b_sel] "n"(PIN_SEL_N), [b_ext] "n"(PIN_EXTSEL), [b_a8] "n"(PIN_A8)
            : "memory");

        if (t_rise < t_prev)
            cyc_hi++;
        t_prev = t_rise;

        const bool sel = !(lo & m_sel);
        const bool rom = !(lo & m_romsel) && latch;   // ROM window answered
        const bool rd  = (lo & m_rw) != 0;
        (void)a;

        if (!sel)
        {
            // 5. PHI2 falling edge: release data bus
            wait_phi2_low();
            bus_release();
            continue;
        }

        const uint8_t offset = (uint8_t)decode_addr(lo, hi);
        uint8_t fl = 0;
        if (rom)              fl |= 0x04;  // MPD asserted
        if (!(out & m_extsel)) fl |= 0x08;  // EXTSEL asserted

        if (!rd)
        {
            // Write: sample the data at a fixed time after PHI2 rises
            // (valid from t_MDS to 30 ns after PHI2 falls).
            const uint32_t t_wr = t_rise + wr_sample;
            while ((int32_t)(cpu_ll_get_cycle_count() - t_wr) < 0)
                ;
            const uint8_t data = decode_data(gpio_in());

            if (offset == 0xFF)
            {
                // $D1FF (PBI) / $D5FF (CCTL): latch control
                const bool prev = latch;
                latch = (data == 0x80);
                if (!latch)
                    GPIO.out_w1ts = m_extsel;
                // Log only when state changes
                if (latch != prev)
                {
                    uint8_t lf = latch ? 0x02 : 0x00;
                    if (rom)   lf |= 0x04;  // MPD asserted
                    if (latch) lf |= 0x08;  // EXTSEL now asserted
                    log_send(t_rise, cyc_hi, EVT_LATCH, 0xFF, data, lf);
                }
            }
            else
            {
                // $D100-$D1FE (PBI) / $D500-$D5FE (CCTL): register write
                log_send(t_rise, cyc_hi, EVT_REG, offset, data, fl | 0x01);
            }
            wait_phi2_low();
            bus_release();
        }
        else
        {
            // Read: the VERA drives the bus; keep the last sample taken
            // while PHI2 is still high.
            uint32_t last = lo, now;
            while ((now = gpio_in()) & m_phi2)
                last = now;
            bus_release();
            if (offset != 0xFF)
                log_send(t_rise, cyc_hi, EVT_REG, offset, decode_data(last), fl);
        }
    }
}

// ---------------------------------------------------------------------------
// VERA register name lookup — Core 0 only (LogTask), not IRAM-resident.
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

    if (offset == 0x08)
        return "VERA_IRQLINE_L";

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

    if (offset >= 0x0D && offset <= 0x13)
    {
        static const char *l0[7] = {
            "VERA_L0_CONFIG",    // $0D
            "VERA_L0_MAPBASE",   // $0E
            "VERA_L0_TILEBASE",  // $0F
            "VERA_L0_HSCR_L",   // $10
            "VERA_L0_HSCR_H",   // $11
            "VERA_L0_VSCR_L",   // $12
            "VERA_L0_VSCR_H",   // $13
        };
        return l0[offset - 0x0D];
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
// LogTask -- Core 0: owns Serial (its UART interrupt is allocated on the core
// that calls Serial.begin) and drains log_ring filled by the bus loop.
// ============================================================================
static void log_drain(void)
{
    // Shadow of VERA_CTRL DCSEL bits [2:1]; updated on every write to $D105.
    static uint8_t  dcsel        = 0;
    static uint32_t dropped_seen = 0;

    uint32_t t = log_tail;
    while (t != __atomic_load_n(&log_head, __ATOMIC_ACQUIRE))
    {
        LogEvt evt = log_ring[t & (LOG_RING_SIZE - 1u)];
        __atomic_store_n(&log_tail, ++t, __ATOMIC_RELEASE);

        const uint64_t ts_us = ((((uint64_t)evt.cyc_hi) << 32) | evt.cyc_lo) / CPU_MHZ;
        const uint32_t sec   = (uint32_t)(ts_us / 1000000ULL);
        const uint32_t us    = (uint32_t)(ts_us % 1000000ULL);
        if (evt.type == EVT_LATCH)
        {
            bool enabled = (evt.flags & 0x02) != 0;
            Serial.printf("[%5lu.%06lu] [VCS ] Latch %s ($%02X written to %s) MPD=%s EXTSEL=%s\n",
                          sec, us,
                          enabled ? "ENABLED " : "DISABLED", evt.data,
                          (BUS_MODE == BUS_MODE_PBI) ? "$D1FF" : "$D5FF",
                          (evt.flags & 0x04) ? "LO" : "HI",
                          (evt.flags & 0x08) ? "LO" : "HI");
        }
        else  // EVT_REG
        {
            // Track DCSEL so muxed registers ($09-$0C) resolve correctly.
            if (evt.offset == 0x05 && (evt.flags & 0x01))
                dcsel = (evt.data >> 1) & 0x07;

            Serial.printf("[%5lu.%06lu] [D1%02X - %-20s] %c $%02X  MPD=%s EXTSEL=%s\n",
                          sec, us,
                          evt.offset,
                          vera_reg_name(evt.offset, dcsel),
                          (evt.flags & 0x01) ? 'W' : 'R',
                          evt.data,
                          (evt.flags & 0x04) ? "LO" : "HI",
                          (evt.flags & 0x08) ? "LO" : "HI");
        }
    }

    const uint32_t dropped = log_dropped;
    if (dropped != dropped_seen)
    {
        Serial.printf("[6502_monitor] log: %lu events dropped (ring full)\n",
                      (unsigned long)(dropped - dropped_seen));
        dropped_seen = dropped;
    }
}

static void LogTask(void *arg)
{
    Serial.begin(115200, SERIAL_8N1, -1, PIN_UART_TX);
    log_ready = true;
    for (;;)
    {
        log_drain();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// Setup -- runs on Core 1 (ARDUINO_RUNNING_CORE) until MonitorTask takes the
// core over.
// ============================================================================
void setup()
{
    // Serial and the log printer live on Core 0.
    xTaskCreatePinnedToCore(LogTask, "BusLog", 4096, NULL, 1, NULL, 0);
    while (!log_ready)
        delay(1);

    Serial.println("\n[6502_monitor] PBI ROM Emulator booting...");

    // Load PBI driver ROM image (generated from 6502/pbi-driver.bin via pre_build.py)
    static_assert(sizeof(pbi_driver) >= sizeof(pbi_rom), "PBI ROM image must be 2 KB");
    memcpy(pbi_rom, pbi_driver, sizeof(pbi_rom));
    build_tables();

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

    Serial.printf("[6502_monitor] Bus loop on Core 1 (write sample %u ns)\n",
                  BUS_WR_SAMPLE_NS);
    Serial.println("[6502_monitor] VCS=OFF  Latch=DISABLED");
    Serial.flush();

    // MonitorTask takes Core 1 over (highest priority, never yields): this
    // call does not return.
    xTaskCreatePinnedToCore(MonitorTask, "Monitor", 4096, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);
    Serial.println("[6502_monitor] FATAL: cannot start the bus loop");
}

void loop()
{
    // Not reached: MonitorTask owns Core 1.
    vTaskDelay(portMAX_DELAY);
}
