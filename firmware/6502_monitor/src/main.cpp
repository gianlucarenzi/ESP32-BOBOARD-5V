/**
 * 6502_monitor -- ROM-less Atari PBI / CCTL peripheral and bus monitor
 *
 * FIRMWARE: Targets NodeMCU DevKit V1 (ESP32-WROOM, Xtensa LX6).
 *
 * BEHAVIOUR:
 * - Two register windows, decoded by the Atari MMU / PLA:
 *     ~D1XX  $D100-$D1FF  (PBI device area)
 *     ~CCTL  $D500-$D5FF  (cartridge control area)
 *   Only A0-A7 are sampled: the page comes from the select line.
 * - Each window has its own select latch and its own 256-byte register file:
 *     $D1FF / $D5FF write: bit 7 (PBI device ID $80) selects (1) or
 *     deselects (0) the device in that window.
 *     $xx00-$xxFE write while selected: stored in the register file.
 *     $xx00-$xxFE read  while selected: the device drives the stored byte.
 *   The latch address itself is never driven on reads ($D1FF is the PBI
 *   interrupt status, read by the OS).  Not selected: the device never drives
 *   the bus, as every PBI device.
 * - The registers are written only by the Atari (power-up value $00).
 * - No ROM: the OS PBI scan does not find a signature at $D803 / $D80B and
 *   skips the device; programs select it directly through $D1FF.
 * - No MPD / EXTSEL: no ROM at $D800, no RAM to replace in the I/O area.
 * - LOG: lock-free ring from the Core 1 bus loop to LogTask on Core 0; prints
 *   latch changes and every access to both windows, selected or not.
 *
 * Bus loop (optimizations ported from Vera-Module-RBL-XE, adapted to the
 * ESP32 classic: no dedicated GPIO, no ee.* instructions):
 * - Core 1 belongs to the bus loop: interrupts masked, interrupt watchdog off,
 *   no FreeRTOS call; Serial and the log printer run on Core 0.
 * - The address is decoded during PHI1: ~D1XX is not PHI2-qualified, so A0-A7,
 *   R/W, ~D1XX and ~CCTL are sampled BUS_ADDR_SETTLE_NS after PHI2 falls and
 *   the read byte is already in GPIO.out (outputs still off) when PHI2 rises:
 *   on the rising edge one store turns D0-D7 on.  If a select line shows up
 *   only on the rising edge (a PHI2-qualified decode), the byte is driven
 *   late from that sample.
 * - Hot-loop data in DRAM (IRAM is not byte-addressable on the ESP32); GPIO
 *   accesses in inline assembly, without the memw the compiler puts around
 *   volatiles.
 * - Write data sampled at a fixed time after PHI2 rises (CCOUNT).
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <soc/timer_group_struct.h>
#include <hal/wdt_hal.h>
#include <hal/cpu_ll.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// Pin Definitions (NodeMCU DevKit V1 / ESP32-WROOM)
// ---------------------------------------------------------------------------
// Data Bus (D0-D7), bidirectional
#define PIN_D0 4
#define PIN_D1 5
#define PIN_D2 13
#define PIN_D3 14
#define PIN_D4 16
#define PIN_D5 17
#define PIN_D6 18
#define PIN_D7 19
// Address Bus (A0-A7)
#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 39
#define PIN_A4 32
#define PIN_A5 33
#define PIN_A6 21
#define PIN_A7 27
// Window selects from the Atari MMU / PLA (Active LOW)
#define PIN_D1XX_N 22  // $D100-$D1FF
#define PIN_CCTL_N 23  // $D500-$D5FF
// Common Bus Signals
#define PIN_PHI2   2   // 6502 Phase 2 Clock (1.79 MHz)
#define PIN_RW     15  // Read/Write (High = Read, Low = Write)

// UART0 TX for the log (RX unused)
#define PIN_UART_TX 1

/*
 * Not used by the firmware, left as inputs:
 *   GPIO 12 — still wired to U1 / J3 pin 4 (former A8).  Strapping pin
 *             (MTDI): the VDD_SDIO efuses make the ESP32 ignore it.
 *   GPIO 25, 26 — free TXS0108E channels (former A9, A10).
 *   GPIO 0, 3 — not routed to J3 on this board.
 */

static const uint8_t DBUS_PINS[8] = {PIN_D0, PIN_D1, PIN_D2, PIN_D3,
                                     PIN_D4, PIN_D5, PIN_D6, PIN_D7};

#define DBUS_MASK                                                          \
    ((1UL << PIN_D0) | (1UL << PIN_D1) | (1UL << PIN_D2) | (1UL << PIN_D3) | \
     (1UL << PIN_D4) | (1UL << PIN_D5) | (1UL << PIN_D6) | (1UL << PIN_D7))

/*
 * GPIO.out is written as a whole: D0-D7 are the only GPIO outputs of Bank 0
 * (GPIO 0-31).  The other Bank 0 pins are inputs, or GPIO1 = U0TXD, which the
 * IO_MUX routes to the UART and ignores GPIO.out.  A new GPIO output on Bank 0
 * must be added to the word.
 */
static_assert(PIN_PHI2 < 32 && PIN_RW < 32 && PIN_D1XX_N < 32 && PIN_CCTL_N < 32,
              "bus loop expects these pins in Bank 0");
static_assert(PIN_A0 >= 32 && PIN_A1 >= 32 && PIN_A2 >= 32 && PIN_A3 >= 32 &&
              PIN_A4 >= 32 && PIN_A5 >= 32, "A0-A5 are decoded from GPIO.in1");
static_assert(PIN_A6 >= 21 && PIN_A6 <= 27 && PIN_A7 >= 21 && PIN_A7 <= 27,
              "addr_lut_lo covers GPIO 21-27");

// ---------------------------------------------------------------------------
// Bus Timing
//
// PHI1: the 6502 drives address and R/W t_ADS after PHI2 falls (~140 ns for
// a 2 MHz part); the MMU / PLA then decodes ~D1XX / ~CCTL and the TXS0108E
// adds its delay.  The loop samples BUS_ADDR_SETTLE_NS after it sees PHI2
// fall.  Read data must be valid ~100 ns before PHI2 falls (HARDWARE.md); it
// is on the bus a few cycles after the detected PHI2 rising edge.
//
// PHI2: write data is valid t_MDS after PHI2 rises (typ. 80-100 ns): sampled
// BUS_WR_SAMPLE_NS after the detected rising edge.
//
// The settle time counts from the earlier of the detected PHI2 falling edge
// and the predicted one, PHI2 rise + BUS_PHI2_HIGH_MAX_NS (longest PHI2 high
// phase, NTSC 279 ns / PAL 282 ns plus duty-cycle error): work done in PHI2
// high that ends after the real edge does not delay the next sample.
//
// Tune with -D after measuring the real bus.
// ---------------------------------------------------------------------------
#ifndef BUS_ADDR_SETTLE_NS
#define BUS_ADDR_SETTLE_NS 200u
#endif
#ifndef BUS_WR_SAMPLE_NS
#define BUS_WR_SAMPLE_NS 120u
#endif
#ifndef BUS_PHI2_HIGH_MAX_NS
#define BUS_PHI2_HIGH_MAX_NS 292u
#endif

#define CPU_MHZ          (F_CPU / 1000000UL)
#define NS_TO_CYCLES(ns) (((ns) * CPU_MHZ + 999u) / 1000u)

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------
#define PBI_DEV_ID   0x80u  // PBI device ID bit, also the CCTL select bit
#define LATCH_OFFSET 0xFFu  // $D1FF / $D5FF

#define WIN_D1XX 0
#define WIN_CCTL 1
#define WINDOWS  2

static const uint8_t WIN_PAGE[WINDOWS] = {0xD1, 0xD5};

// Register files, written only by the Atari.
static DRAM_ATTR uint8_t  regs[WINDOWS][256];
// GPIO.out words (D0-D7) of regs[][], kept in step on every write.
static DRAM_ATTR uint32_t regs_out[WINDOWS][256];

// ---------------------------------------------------------------------------
// Log Ring (Core 1 bus loop → Core 0 LogTask)
//
// Single producer / single consumer, lock-free: the bus loop only stores the
// event and advances log_head (no FreeRTOS call, interrupts are masked on
// Core 1).  When the ring is full new events are dropped and counted.
// ---------------------------------------------------------------------------
#define LOG_RING_SIZE 1024u   // power of two

#define EVT_LATCH 0   // select latch change ($D1FF / $D5FF write)
#define EVT_REG   1   // register access

// flags: bit 0 = write   bit 1 = device selected (EVT_REG) / new latch state
//        (EVT_LATCH)     bit 2 = byte driven by this device (reads)
#define FL_WRITE  0x01u
#define FL_SEL    0x02u
#define FL_DRIVEN 0x04u

typedef struct {
    uint32_t cyc_lo;  // timestamp: 64-bit Core 1 CCOUNT since the bus loop started
    uint32_t cyc_hi;
    uint8_t  type;
    uint8_t  win;     // WIN_D1XX / WIN_CCTL
    uint8_t  offset;  // A0-A7
    uint8_t  data;
    uint8_t  flags;
} LogEvt;

static DRAM_ATTR LogEvt   log_ring[LOG_RING_SIZE];
static DRAM_ATTR uint32_t log_head    = 0;  // written by the bus loop only
static DRAM_ATTR uint32_t log_tail    = 0;  // written by LogTask only
static DRAM_ATTR uint32_t log_dropped = 0;  // written by the bus loop only

// ---------------------------------------------------------------------------
// Decoding Tables & Helpers
// ---------------------------------------------------------------------------
#define BUS_INLINE static inline __attribute__((always_inline))

// A0-A5 from GPIO.in1 bits 0-7 (GPIO 32-39).
static DRAM_ATTR uint8_t addr_lut_hi[256];
// A6, A7 from GPIO.in bits 21-27 (GPIO 21, 27); ~D1XX / ~CCTL ignored.
static DRAM_ATTR uint8_t addr_lut_lo[128];

// D0-D7 on GPIO 4,5 | 13,14 | 16-19.
static_assert(PIN_D0 == 4 && PIN_D1 == 5 && PIN_D2 == 13 && PIN_D3 == 14 &&
              PIN_D4 == 16 && PIN_D5 == 17 && PIN_D6 == 18 && PIN_D7 == 19,
              "decode_data() / encode_data() hard-code the data bus pins");

BUS_INLINE uint8_t decode_data(uint32_t lo)
{
    return (uint8_t)(((lo >> 4) & 0x03u) | ((lo >> 11) & 0x0Cu) | ((lo >> 12) & 0xF0u));
}

BUS_INLINE uint32_t encode_data(uint32_t d)
{
    return ((d & 0x03u) << 4) | ((d & 0x0Cu) << 11) | ((d & 0xF0u) << 12);
}

BUS_INLINE uint32_t decode_addr(uint32_t lo, uint32_t hi)
{
    return addr_lut_hi[hi & 0xFFu] | addr_lut_lo[(lo >> 21) & 0x7Fu];
}

static uint8_t addr_bits(uint32_t word, const uint8_t *pins, const uint8_t *abits,
                         int n, int base)
{
    uint8_t a = 0;
    for (int i = 0; i < n; i++)
        if ((word >> (pins[i] - base)) & 1u)
            a |= (uint8_t)(1u << abits[i]);
    return a;
}

static void build_tables(void)
{
    static const uint8_t hi_pins[6]  = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A4, PIN_A5};
    static const uint8_t hi_abits[6] = {0, 1, 2, 3, 4, 5};
    static const uint8_t lo_pins[2]  = {PIN_A6, PIN_A7};
    static const uint8_t lo_abits[2] = {6, 7};

    for (uint32_t i = 0; i < 256; i++)
        addr_lut_hi[i] = addr_bits(i, hi_pins, hi_abits, 6, 32);
    for (uint32_t i = 0; i < 128; i++)
        addr_lut_lo[i] = addr_bits(i, lo_pins, lo_abits, 2, 21);

    for (uint32_t w = 0; w < WINDOWS; w++)
        for (uint32_t a = 0; a < 256; a++)
        {
            regs[w][a]     = 0x00;
            regs_out[w][a] = encode_data(0x00);
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

/*
 * Wait for PHI2 to rise and, if `drive` is not zero, turn D0-D7 on right
 * after the sample that saw the edge (GPIO.out already holds the byte).
 * Returns that sample.
 */
BUS_INLINE uint32_t wait_phi2_high_drive(uint32_t drive)
{
    uint32_t v;
    asm volatile("1: l32i %[v], %[gpio], %[o_in]     \n\t"
                 "   bbci %[v], %[b_phi2], 1b        \n\t"
                 "   beqz %[drv], 2f                 \n\t"
                 "   s32i %[dmask], %[gpio], %[o_en] \n\t"
                 "2:                                 \n\t"
                 : [v] "=&r"(v)
                 : [gpio] "r"(&GPIO), [drv] "r"(drive), [dmask] "r"(DBUS_MASK),
                   [o_in] "n"(GPIO_OFF(in)), [o_en] "n"(GPIO_OFF(enable_w1ts)),
                   [b_phi2] "n"(PIN_PHI2)
                 : "memory");
    return v;
}

BUS_INLINE void wait_until(uint32_t t)
{
    while ((int32_t)(cpu_ll_get_cycle_count() - t) < 0)
        ;
}

// Push a log event into the ring without blocking (drops if full).
BUS_INLINE void log_push(const LogEvt *src)
{
    uint32_t h = log_head;
    if (h - __atomic_load_n(&log_tail, __ATOMIC_ACQUIRE) >= LOG_RING_SIZE)
    {
        log_dropped++;
        return;
    }
    log_ring[h & (LOG_RING_SIZE - 1u)] = *src;
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
    const uint32_t m_rw   = (1UL << PIN_RW);
    const uint32_t m_d1xx = (1UL << PIN_D1XX_N);
    const uint32_t m_cctl = (1UL << PIN_CCTL_N);
    const uint32_t m_phi2 = (1UL << PIN_PHI2);

    const uint32_t settle    = NS_TO_CYCLES(BUS_ADDR_SETTLE_NS);
    const uint32_t wr_sample = NS_TO_CYCLES(BUS_WR_SAMPLE_NS);
    const uint32_t phi2_max  = NS_TO_CYCLES(BUS_PHI2_HIGH_MAX_NS);

    uint8_t  selected[WINDOWS] = {0, 0};  // select latches
    uint32_t cyc_hi = 0;                  // CCOUNT wraps every 2^32 / 240 MHz = 17.9 s
    uint32_t t_prev = cpu_ll_get_cycle_count();

    LogEvt   pend;                        // event of the previous cycle
    bool     pend_valid = false;

    bus_release();
    gpio_out(0);

    // From here on Core 1 belongs to the bus: no tick, no interrupts.
    bus_int_wdt_disable();
    portDISABLE_INTERRUPTS();

    // Align to the end of a PHI2-high phase.
    wait_phi2_high();
    uint32_t t_fall_max = cpu_ll_get_cycle_count() + phi2_max;
    wait_phi2_low();

    for (;;)
    {
        // ---- PHI1: PHI2 has just fallen ------------------------------------
        uint32_t t_fall = cpu_ll_get_cycle_count();
        if ((int32_t)(t_fall - t_fall_max) > 0)
            t_fall = t_fall_max;            // edge seen late: use the prediction

        // 1. Close the previous cycle (releasing an idle bus is harmless).
        bus_release();

        // 2. Log of the previous cycle: short, it fits in the settle time.
        if (pend_valid)
        {
            log_push(&pend);
            pend_valid = false;
        }

        // 3. Address, R/W and window selects, once stable.
        wait_until(t_fall + settle);
        uint32_t lo  = gpio_in();
        uint32_t hi  = gpio_in1();
        uint32_t off = decode_addr(lo, hi);
        bool     rd  = (lo & m_rw) != 0;
        int      win = !(lo & m_d1xx) ? WIN_D1XX : (!(lo & m_cctl) ? WIN_CCTL : -1);

        // 4. Read of a selected register: byte into GPIO.out now (outputs
        //    off), outputs on at the PHI2 rising edge.
        uint32_t drive = (win >= 0 && rd && selected[win] && off != LATCH_OFFSET);
        if (drive)
            gpio_out(regs_out[win][off]);

        const uint32_t s_rise = wait_phi2_high_drive(drive);
        const uint32_t t_rise = cpu_ll_get_cycle_count();
        t_fall_max = t_rise + phi2_max;
        if (t_rise < t_prev)
            cyc_hi++;
        t_prev = t_rise;

        // 5. Select seen only on the rising edge (PHI2-qualified decode):
        //    take it from that sample and drive late.
        if (win < 0)
        {
            win = !(s_rise & m_d1xx) ? WIN_D1XX : (!(s_rise & m_cctl) ? WIN_CCTL : -1);
            if (win < 0)
            {
                wait_phi2_low();            // not our windows
                continue;
            }
            drive = (rd && selected[win] && off != LATCH_OFFSET);
            if (drive)
            {
                gpio_out(regs_out[win][off]);
                bus_drive_enable();
            }
        }

        pend.cyc_lo = t_rise;
        pend.cyc_hi = cyc_hi;
        pend.win    = (uint8_t)win;
        pend.offset = (uint8_t)off;
        pend_valid  = true;

        if (rd)
        {
            pend.type  = EVT_REG;
            pend.flags = (selected[win] ? FL_SEL : 0u) | (drive ? FL_DRIVEN : 0u);
            if (drive)
            {
                pend.data = regs[win][off];
                wait_phi2_low();
            }
            else
            {
                // Someone else (or nobody) drives the bus: keep the last
                // sample taken while PHI2 is still high.
                uint32_t last = s_rise, now;
                while ((now = gpio_in()) & m_phi2)
                    last = now;
                pend.data = decode_data(last);
            }
        }
        else
        {
            // 6. Write: sample the data at a fixed time after PHI2 rises
            //    (valid from t_MDS to 30 ns after PHI2 falls).
            wait_until(t_rise + wr_sample);
            const uint8_t data = decode_data(gpio_in());
            pend.data = data;

            if (off == LATCH_OFFSET)
            {
                // $D1FF / $D5FF: each device latches only its own ID bit.
                const uint8_t sel = (data & PBI_DEV_ID) ? 1u : 0u;
                if (sel != selected[win])
                {
                    selected[win] = sel;
                    pend.type  = EVT_LATCH;
                    pend.flags = FL_WRITE | (sel ? FL_SEL : 0u);
                }
                else
                {
                    pend.type  = EVT_REG;
                    pend.flags = FL_WRITE | (sel ? FL_SEL : 0u);
                }
            }
            else
            {
                pend.type  = EVT_REG;
                pend.flags = FL_WRITE | (selected[win] ? FL_SEL : 0u);
                if (selected[win])
                {
                    regs[win][off]     = data;
                    regs_out[win][off] = encode_data(data);
                }
            }
            wait_phi2_low();
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
// LogTask -- Core 0: the only writer on Serial.
//
// Nothing else calls Serial after boot: setup() configures the board
// silently and LogTask prints the banner, the bus events and the errors in
// one sequence.  Each line is formatted into a local buffer and sent with a
// single Serial.write(): uartWriteBuf() holds the UART mutex for the whole
// call, so even an unexpected writer could not split a line.  (Print::printf
// would also be one write, but it mallocs every line longer than 64 bytes;
// Print::println is two writes.)
//
// TX path (arduino-esp32 2.0.x): no TX ring buffer (setTxBufferSize 0), so
// the write copies into the 128-byte hardware FIFO and blocks LogTask until
// the last byte is in the FIFO.  The blocking happens on Core 0 only; the
// bus loop just keeps filling log_ring and counts what does not fit.
// ============================================================================
#ifndef LOG_BAUD
#define LOG_BAUD 115200u
#endif

#define LOG_LINE_MAX 128u

static volatile bool bus_start_failed = false;   // set by setup()

static void log_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void log_line(const char *fmt, ...)
{
    char    buf[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if ((size_t)n >= sizeof(buf))
    {
        // Truncated: keep the line terminated.
        n = sizeof(buf) - 1;
        buf[n - 1] = '\n';
    }
    Serial.write((const uint8_t *)buf, (size_t)n);
}

static void log_drain(void)
{
    // Shadow of VERA_CTRL DCSEL bits [2:1] per window; updated on every
    // write to offset $05.
    static uint8_t  dcsel[WINDOWS] = {0, 0};
    static uint32_t dropped_seen   = 0;

    uint32_t t = log_tail;
    while (t != __atomic_load_n(&log_head, __ATOMIC_ACQUIRE))
    {
        LogEvt evt = log_ring[t & (LOG_RING_SIZE - 1u)];
        __atomic_store_n(&log_tail, ++t, __ATOMIC_RELEASE);

        const uint64_t ts_us = ((((uint64_t)evt.cyc_hi) << 32) | evt.cyc_lo) / CPU_MHZ;
        const uint32_t sec   = (uint32_t)(ts_us / 1000000ULL);
        const uint32_t us    = (uint32_t)(ts_us % 1000000ULL);
        const uint8_t  w     = evt.win < WINDOWS ? evt.win : 0;
        const uint8_t  page  = WIN_PAGE[w];

        if (evt.type == EVT_LATCH)
        {
            log_line("[%5lu.%06lu] [%02X%02X - %-20s] W $%02X  device %s\n",
                     (unsigned long)sec, (unsigned long)us, page, evt.offset,
                     (w == WIN_D1XX) ? "PBI_SELECT" : "CCTL_SELECT", evt.data,
                     (evt.flags & FL_SEL) ? "SELECTED" : "DESELECTED");
        }
        else  // EVT_REG
        {
            const bool wr = (evt.flags & FL_WRITE) != 0;
            // Track DCSEL so muxed registers ($09-$0C) resolve correctly.
            if (evt.offset == 0x05 && wr)
                dcsel[w] = (evt.data >> 1) & 0x07;

            const char *name = (evt.offset == LATCH_OFFSET)
                                   ? ((w == WIN_D1XX) ? "PBI_SELECT" : "CCTL_SELECT")
                                   : vera_reg_name(evt.offset, dcsel[w]);
            const char *tag  = !(evt.flags & FL_SEL)     ? "-"
                               : (evt.flags & FL_DRIVEN) ? "SEL DRV"
                                                         : "SEL";
            log_line("[%5lu.%06lu] [%02X%02X - %-20s] %c $%02X  %s\n",
                     (unsigned long)sec, (unsigned long)us, page, evt.offset,
                     name, wr ? 'W' : 'R', evt.data, tag);
        }
    }

    const uint32_t dropped = log_dropped;
    if (dropped != dropped_seen)
    {
        log_line("[6502_monitor] log: %lu events dropped (ring full)\n",
                 (unsigned long)(dropped - dropped_seen));
        dropped_seen = dropped;
    }
}

static void LogTask(void *arg)
{
    Serial.begin(LOG_BAUD, SERIAL_8N1, -1, PIN_UART_TX);

    log_line("\n[6502_monitor] ROM-less PBI / CCTL peripheral, device ID $%02X\n",
             (unsigned)PBI_DEV_ID);
    log_line("[6502_monitor] Windows $D100-$D1FE (select $D1FF), $D500-$D5FE (select $D5FF)\n");
    log_line("[6502_monitor] Bus loop on Core 1 (settle %u ns, write sample %u ns), "
             "log %u baud\n",
             (unsigned)BUS_ADDR_SETTLE_NS, (unsigned)BUS_WR_SAMPLE_NS, (unsigned)LOG_BAUD);

    bool fail_seen = false;
    for (;;)
    {
        log_drain();
        if (bus_start_failed && !fail_seen)
        {
            log_line("[6502_monitor] FATAL: cannot start the bus loop\n");
            fail_seen = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// Setup -- runs on Core 1 (ARDUINO_RUNNING_CORE) until MonitorTask takes the
// core over.  It never touches Serial (see LogTask).
// ============================================================================
void setup()
{
    build_tables();

    // Every bus pin is an input; D0-D7 become outputs only while the device
    // answers a read, under the control of the bus loop.  GPIO.out is 0 and
    // no Bank 0 pin is an output, so nothing is driven at start-up.
    for (int i = 0; i < 8; i++) pinMode(DBUS_PINS[i], INPUT);

    pinMode(PIN_PHI2,   INPUT);
    pinMode(PIN_RW,     INPUT);
    pinMode(PIN_D1XX_N, INPUT);
    pinMode(PIN_CCTL_N, INPUT);
    pinMode(PIN_A0, INPUT);
    pinMode(PIN_A1, INPUT);
    pinMode(PIN_A2, INPUT);
    pinMode(PIN_A3, INPUT);
    pinMode(PIN_A4, INPUT);
    pinMode(PIN_A5, INPUT);
    pinMode(PIN_A6, INPUT);
    pinMode(PIN_A7, INPUT);

    // Serial and the log printer live on Core 0: banner first, then the
    // bus events, all from the same task.
    xTaskCreatePinnedToCore(LogTask, "BusLog", 4096, NULL, 1, NULL, 0);

    // MonitorTask takes Core 1 over (highest priority, never yields): on
    // success this call does not return.
    if (xTaskCreatePinnedToCore(MonitorTask, "Monitor", 4096, NULL,
                                configMAX_PRIORITIES - 1, NULL, 1) != pdPASS)
        bus_start_failed = true;
}

void loop()
{
    // Not reached: MonitorTask owns Core 1.
    vTaskDelay(portMAX_DELAY);
}
