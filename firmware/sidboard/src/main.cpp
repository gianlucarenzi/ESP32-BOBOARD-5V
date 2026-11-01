/**
 * sidboard -- SID 6581/8580 sound for the Atari 8-bit, ESP32-BOBOARD-5V
 *
 * FIRMWARE: NodeMCU-32S (ESP32-WROOM, Xtensa LX6).
 *
 * Two builds (PlatformIO environments), same board and wiring:
 * - sid-cctl (SID_MODE_PBI = 0): the SID sits in the cartridge control area,
 *   always on, no latch, no configuration:
 *     $D500-$D51E  SID registers (~CCTL, J3 pin 12)
 * - sid-pbi (SID_MODE_PBI = 1): the SID is a PBI device, ID $80:
 *     $D100-$D11E  SID registers, while selected (~D1XX, J3 pin 11)
 *     $D1FF        PBI select latch (W): bit 7 = 1 selects, 0 deselects;
 *                  never driven ($D1FF is the PBI interrupt status)
 *   Deselecting only removes the SID from the bus: the registers stay and
 *   the voices keep playing (the OS PBI scan at RESET writes $80 then $00
 *   to $D1FF).  Selected or not, the device never drives other addresses.
 * Only A0-A7 are sampled: the page comes from the select line.  Offsets
 * $1F-$FE of the page are not decoded (never driven, writes ignored).
 * Reads of $00-$1E return the stored register byte.
 *
 * AUDIO: cSIDLight (include/csidlight.h), 3 square-wave voices, frequency,
 * gate and master volume; DAC1 on GPIO 25 at 22.2 kHz from an esp_timer on
 * Core 0; SidTask (Core 0) turns register changes into synthesis parameters.
 *
 * BUS LOOP (optimizations of the 6502_monitor firmware, from
 * Vera-Module-RBL-XE):
 * - Core 1 belongs to the bus loop: interrupts masked, interrupt watchdog
 *   off, no FreeRTOS call; Serial, the log printer and the audio run on
 *   Core 0.
 * - The address is decoded during PHI1: A0-A7, R/W and the select line are
 *   sampled BUS_ADDR_SETTLE_NS after PHI2 falls and a read byte is already
 *   in GPIO.out (outputs off) when PHI2 rises: one store turns D0-D7 on.  A
 *   select seen only on the rising edge is handled from that sample.
 * - GPIO accesses in inline assembly, without memw; hot data in DRAM.
 * - Write data sampled BUS_WR_SAMPLE_NS after PHI2 rises (CCOUNT).
 * - LOG: lock-free ring from Core 1 to LogTask on Core 0, the only writer on
 *   Serial, one Serial.write() per line.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <soc/timer_group_struct.h>
#include <hal/wdt_hal.h>
#include <hal/cpu_ll.h>
#include <esp_timer.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>

#include "csidlight.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef SID_MODE_PBI
#define SID_MODE_PBI 0
#endif

// ---------------------------------------------------------------------------
// Pin Definitions (ESP32-BOBOARD-5V, same as the 6502_monitor firmware)
// ---------------------------------------------------------------------------
#define PIN_D0 4
#define PIN_D1 5
#define PIN_D2 13
#define PIN_D3 14
#define PIN_D4 16
#define PIN_D5 17
#define PIN_D6 18
#define PIN_D7 19
#define PIN_A0 34
#define PIN_A1 35
#define PIN_A2 36
#define PIN_A3 39
#define PIN_A4 32
#define PIN_A5 33
#define PIN_A6 21
#define PIN_A7 27
#define PIN_D1XX_N 22  // $D100-$D1FF (J3 pin 11)
#define PIN_CCTL_N 23  // $D500-$D5FF (J3 pin 12)
#define PIN_PHI2   2
#define PIN_RW     15
#define PIN_UART_TX 1

#if SID_MODE_PBI
#define PIN_SEL_N  PIN_D1XX_N
#define SID_PAGE   0xD1
#else
#define PIN_SEL_N  PIN_CCTL_N
#define SID_PAGE   0xD5
#endif

#define PBI_DEV_ID   0x80u
#define LATCH_OFFSET 0xFFu

static const uint8_t DBUS_PINS[8] = {PIN_D0, PIN_D1, PIN_D2, PIN_D3,
                                     PIN_D4, PIN_D5, PIN_D6, PIN_D7};

#define DBUS_MASK                                                          \
    ((1UL << PIN_D0) | (1UL << PIN_D1) | (1UL << PIN_D2) | (1UL << PIN_D3) | \
     (1UL << PIN_D4) | (1UL << PIN_D5) | (1UL << PIN_D6) | (1UL << PIN_D7))

// GPIO.out is written as a whole: D0-D7 are the only GPIO outputs of Bank 0
// (GPIO 25 is the DAC: its output does not come from GPIO.out).
static_assert(PIN_PHI2 < 32 && PIN_RW < 32 && PIN_SEL_N < 32, "bus pins in Bank 0");
static_assert(PIN_A6 >= 21 && PIN_A6 <= 27 && PIN_A7 >= 21 && PIN_A7 <= 27,
              "addr_lut_lo covers GPIO 21-27");
static_assert(SID_NUM_REGS == 31, "movi 31 in the bus loop assembly");

// ---------------------------------------------------------------------------
// Bus Timing (see 6502_monitor; tune with -D after measuring the bus)
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
// SID state seen by the bus loop assembly (fixed offsets)
// ---------------------------------------------------------------------------
typedef struct {
    volatile uint8_t *regs;  // +0  cSIDLight register array
    uint8_t           sel;   // +4  1: answers (always 1 in CCTL mode)
    uint8_t           pad[3];
} SidBus;

static_assert(offsetof(SidBus, regs) == 0 && offsetof(SidBus, sel) == 4,
              "SidBus layout used by the bus loop assembly");

static SidBus     sid_bus;
static cSIDLight *sidLight = nullptr;

#ifdef AUDIO_DEMO
#include "Commando_dmp.h"
static int                demo_frame   = 0;
static const int          demo_total   = sidLength / 25;
static esp_timer_handle_t demo_timer_h = nullptr;

static void demo_tick(void *)
{
    if (!sidLight) return;
    const uint8_t *f = sidData + (size_t)demo_frame * 25;
    for (uint8_t r = 0; r < 25; r++)
        sidLight->write(r, f[r]);
    if (++demo_frame >= demo_total) demo_frame = 0;
}
#endif

// ---------------------------------------------------------------------------
// Log Ring (Core 1 bus loop -> Core 0 LogTask), lock-free SPSC
// ---------------------------------------------------------------------------
#define LOG_RING_SIZE 1024u

#define EVT_LATCH 0   // $D1FF select change (sid-pbi)
#define EVT_REG   1   // access to the SID page

#define FL_WRITE  0x01u
#define FL_SEL    0x02u   // SID answering / new latch state
#define FL_DRIVEN 0x04u

typedef struct {
    uint32_t cyc_lo;
    uint32_t cyc_hi;
    uint8_t  type;
    uint8_t  offset;
    uint8_t  data;
    uint8_t  flags;
} LogEvt;

static LogEvt   log_ring[LOG_RING_SIZE];
static uint32_t log_head    = 0;
static uint32_t log_tail    = 0;
static uint32_t log_dropped = 0;

// ---------------------------------------------------------------------------
// Decoding Tables & Helpers
// ---------------------------------------------------------------------------
#define BUS_INLINE static inline __attribute__((always_inline))

static uint8_t  addr_lut_hi[256];   // A0-A5 from GPIO.in1 bits 0-7
static uint8_t  addr_lut_lo[128];   // A6, A7 from GPIO.in bits 21-27
static uint32_t drive_lut[256];     // GPIO.out word for each byte

static_assert(PIN_D0 == 4 && PIN_D1 == 5 && PIN_D2 == 13 && PIN_D3 == 14 &&
              PIN_D4 == 16 && PIN_D5 == 17 && PIN_D6 == 18 && PIN_D7 == 19,
              "decode_data() / encode_data() hard-code the data bus pins");

BUS_INLINE uint8_t decode_data(uint32_t lo)
{
    return (uint8_t)(((lo >> 4) & 0x03u) | ((lo >> 11) & 0x0Cu) | ((lo >> 12) & 0xF0u));
}

static uint32_t encode_data(uint32_t d)
{
    return ((d & 0x03u) << 4) | ((d & 0x0Cu) << 11) | ((d & 0xF0u) << 12);
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
    for (uint32_t i = 0; i < 256; i++)
        drive_lut[i] = encode_data(i);
}

#define GPIO_OFF(f) offsetof(gpio_dev_t, f)

BUS_INLINE uint32_t gpio_in(void)
{
    uint32_t v;
    asm volatile("l32i %0, %1, %2" : "=r"(v) : "r"(&GPIO), "n"(GPIO_OFF(in)));
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

// Wait for PHI2 to rise; if `drive`, turn D0-D7 on right after the sample
// that saw the edge (GPIO.out already holds the byte).
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

// The bus loop runs with interrupts masked: stop the interrupt watchdog
// (MWDT1 checks CPU1 too), or it resets the chip.
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
// ============================================================================
static void IRAM_ATTR MonitorTask(void *pvParameters)
{
    const uint32_t m_rw   = (1UL << PIN_RW);
    const uint32_t m_sel  = (1UL << PIN_SEL_N);
    const uint32_t m_phi2 = (1UL << PIN_PHI2);

    const uint32_t settle    = NS_TO_CYCLES(BUS_ADDR_SETTLE_NS);
    const uint32_t wr_sample = NS_TO_CYCLES(BUS_WR_SAMPLE_NS);
    const uint32_t phi2_max  = NS_TO_CYCLES(BUS_PHI2_HIGH_MAX_NS);

    uint32_t cyc_hi = 0;
    uint32_t t_prev = cpu_ll_get_cycle_count();
    LogEvt   pend;
    bool     pend_valid = false;

    bus_release();
    gpio_out(0);

    bus_int_wdt_disable();
    portDISABLE_INTERRUPTS();

    wait_phi2_high();
    uint32_t t_fall_max = cpu_ll_get_cycle_count() + phi2_max;
    wait_phi2_low();

    for (;;)
    {
        // ---- PHI1: PHI2 has just fallen ------------------------------------
        uint32_t t_fall = cpu_ll_get_cycle_count();
        if ((int32_t)(t_fall - t_fall_max) > 0)
            t_fall = t_fall_max;

        bus_release();
        if (pend_valid)
        {
            log_push(&pend);
            pend_valid = false;
        }

        wait_until(t_fall + settle);

        /*
         * PHI1 decode: A0-A7 from the tables; with the SID page selected, a
         * read of $00-$1E (and the SID answering: always in CCTL mode,
         * latched in PBI mode) puts the register byte into GPIO.out now.
         * hit = 1: the SID page was selected in this sample.
         */
        uint32_t lo, off, t, val, drive, hit;
        asm volatile(
            "   l32i   %[lo], %[gpio], %[o_in]    \n\t"
            "   l32i   %[t], %[gpio], %[o_in1]    \n\t"
            "   extui  %[t], %[t], 0, 8           \n\t"
            "   add    %[t], %[t], %[luthi]       \n\t"
            "   l8ui   %[off], %[t], 0            \n\t"   /* A0-A5 */
            "   extui  %[t], %[lo], 21, 7         \n\t"
            "   add    %[t], %[t], %[lutlo]       \n\t"
            "   l8ui   %[t], %[t], 0              \n\t"   /* A6, A7 */
            "   or     %[off], %[off], %[t]       \n\t"
            "   movi   %[drv], 0                  \n\t"
            "   movi   %[val], 0                  \n\t"
            "   movi   %[hit], 0                  \n\t"
            "   bbsi   %[lo], %[b_sel], 9f        \n\t"   /* page not selected */
            "   movi   %[hit], 1                  \n\t"
            "   bbci   %[lo], %[b_rw], 9f         \n\t"   /* write cycle */
            "   l8ui   %[t], %[st], 4             \n\t"   /* sel */
            "   beqz   %[t], 9f                   \n\t"
            "   movi   %[t], 31                   \n\t"   /* SID_NUM_REGS */
            "   bgeu   %[off], %[t], 9f           \n\t"
            "   l32i   %[t], %[st], 0             \n\t"   /* regs */
            "   add    %[t], %[t], %[off]         \n\t"
            "   l8ui   %[val], %[t], 0            \n\t"
            "   addx4  %[t], %[val], %[dlut]      \n\t"
            "   l32i   %[t], %[t], 0              \n\t"
            "   s32i   %[t], %[gpio], %[o_out]    \n\t"   /* GPIO.out, outputs off */
            "   movi   %[drv], 1                  \n\t"
            "9:                                   \n\t"
            : [lo] "=&r"(lo), [off] "=&r"(off), [t] "=&r"(t), [val] "=&r"(val),
              [drv] "=&r"(drive), [hit] "=&r"(hit)
            : [gpio] "r"(&GPIO), [luthi] "r"(addr_lut_hi), [lutlo] "r"(addr_lut_lo),
              [dlut] "r"(drive_lut), [st] "r"(&sid_bus),
              [o_in] "n"(GPIO_OFF(in)), [o_in1] "n"(GPIO_OFF(in1)),
              [o_out] "n"(GPIO_OFF(out)),
              [b_sel] "n"(PIN_SEL_N), [b_rw] "n"(PIN_RW)
            : "memory");
        const bool rd = (lo & m_rw) != 0;

        const uint32_t s_rise = wait_phi2_high_drive(drive);
        const uint32_t t_rise = cpu_ll_get_cycle_count();
        t_fall_max = t_rise + phi2_max;
        if (t_rise < t_prev)
            cyc_hi++;
        t_prev = t_rise;

        // Select seen only on the rising edge: take it from that sample.
        if (!hit)
        {
            if (s_rise & m_sel)
            {
                wait_phi2_low();            // not the SID page
                continue;
            }
            if (rd && sid_bus.sel && off < SID_NUM_REGS)
            {
                val   = sid_bus.regs[off];
                drive = 1;
                gpio_out(drive_lut[val]);
                bus_drive_enable();
            }
        }

        pend.cyc_lo = t_rise;
        pend.cyc_hi = cyc_hi;
        pend.offset = (uint8_t)off;
        pend.type   = EVT_REG;
        pend_valid  = true;

        if (rd)
        {
            pend.flags = (sid_bus.sel ? FL_SEL : 0u) | (drive ? FL_DRIVEN : 0u);
            if (drive)
            {
                pend.data = (uint8_t)val;
                wait_phi2_low();
            }
            else
            {
                uint32_t last = s_rise, now;
                while ((now = gpio_in()) & m_phi2)
                    last = now;
                pend.data = decode_data(last);
            }
        }
        else
        {
            wait_until(t_rise + wr_sample);
            const uint8_t data = decode_data(gpio_in());
            pend.data  = data;
            pend.flags = FL_WRITE | (sid_bus.sel ? FL_SEL : 0u);
#if SID_MODE_PBI
            if (off == LATCH_OFFSET)
            {
                // $D1FF: each PBI device latches only its own ID bit.
                const uint8_t sel = (data & PBI_DEV_ID) ? 1u : 0u;
                if (sel != sid_bus.sel)
                {
                    sid_bus.sel = sel;
                    pend.type   = EVT_LATCH;
                    pend.flags  = FL_WRITE | (sel ? FL_SEL : 0u);
                }
            }
            else
#endif
            if (sid_bus.sel && off < SID_NUM_REGS)
                sid_bus.regs[off] = data;   // cSIDLight::write(), inlined
            wait_phi2_low();
        }
    }
}

// ============================================================================
// SidTask -- Core 0: register changes -> synthesis parameters (the samples
// come from the cSIDLight esp_timer, also on Core 0).
// ============================================================================
static void SidTask(void *arg)
{
    for (;;)
    {
        sidLight->process();
        vTaskDelay(1);
    }
}

// ============================================================================
// LogTask -- Core 0: the only writer on Serial, one write per line.
// ============================================================================
#ifndef LOG_BAUD
#define LOG_BAUD 115200u
#endif
#define LOG_LINE_MAX 128u

static volatile bool bus_start_failed = false;

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
        n = sizeof(buf) - 1;
        buf[n - 1] = '\n';
    }
    Serial.write((const uint8_t *)buf, (size_t)n);
}

static const char *sid_reg_name(uint8_t off)
{
    static const char *names[31] = {
        "V1_FREQ_LO", "V1_FREQ_HI", "V1_PW_LO",   "V1_PW_HI",   "V1_CTRL",
        "V1_AD",      "V1_SR",      "V2_FREQ_LO", "V2_FREQ_HI", "V2_PW_LO",
        "V2_PW_HI",   "V2_CTRL",    "V2_AD",      "V2_SR",      "V3_FREQ_LO",
        "V3_FREQ_HI", "V3_PW_LO",   "V3_PW_HI",   "V3_CTRL",    "V3_AD",
        "V3_SR",      "FC_LO",      "FC_HI",      "RES_FILT",   "MODE_VOL",
        "POTX",       "POTY",       "OSC3",       "ENV3",       "UNUSED",
        "UNUSED",
    };
    if (off < SID_NUM_REGS)
        return names[off];
#if SID_MODE_PBI
    if (off == LATCH_OFFSET)
        return "PBI_SELECT";
#endif
    return "NOT_DECODED";
}

static void log_drain(void)
{
    static uint32_t dropped_seen = 0;

    uint32_t t = log_tail;
    while (t != __atomic_load_n(&log_head, __ATOMIC_ACQUIRE))
    {
        LogEvt evt = log_ring[t & (LOG_RING_SIZE - 1u)];
        __atomic_store_n(&log_tail, ++t, __ATOMIC_RELEASE);

        const uint64_t ts_us = ((((uint64_t)evt.cyc_hi) << 32) | evt.cyc_lo) / CPU_MHZ;
        const uint32_t sec   = (uint32_t)(ts_us / 1000000ULL);
        const uint32_t us    = (uint32_t)(ts_us % 1000000ULL);
        const char    *name  = sid_reg_name(evt.offset);

        if (evt.type == EVT_LATCH)
            log_line("[%5lu.%06lu] [%02X%02X - %-12s] W $%02X  %s\n",
                     (unsigned long)sec, (unsigned long)us, SID_PAGE, evt.offset, name,
                     evt.data, (evt.flags & FL_SEL) ? "SELECTED" : "DESELECTED");
        else
        {
            const char *tag = !(evt.flags & FL_SEL)     ? "-"
                              : (evt.flags & FL_DRIVEN) ? "SEL DRV"
                                                        : "SEL";
            log_line("[%5lu.%06lu] [%02X%02X - %-12s] %c $%02X  %s\n",
                     (unsigned long)sec, (unsigned long)us, SID_PAGE, evt.offset, name,
                     (evt.flags & FL_WRITE) ? 'W' : 'R', evt.data, tag);
        }
    }

    const uint32_t dropped = log_dropped;
    if (dropped != dropped_seen)
    {
        log_line("[sidboard] log: %lu events dropped (ring full)\n",
                 (unsigned long)(dropped - dropped_seen));
        dropped_seen = dropped;
    }
}

static void LogTask(void *arg)
{
    Serial.begin(LOG_BAUD, SERIAL_8N1, -1, PIN_UART_TX);
#if SID_MODE_PBI
    log_line("\n[sidboard] SID PBI device, ID $%02X: $D100-$D11E, select $D1FF "
             "(~D1XX, J3 pin 11)\n", (unsigned)PBI_DEV_ID);
#else
    log_line("\n[sidboard] SID in CCTL, always on: $D500-$D51E (~CCTL, J3 pin 12)\n");
#endif
    log_line("[sidboard] cSIDLight: 3 voices, DAC1 on GPIO %u, %u Hz\n",
             (unsigned)SID_DAC_PIN, (unsigned)SID_ACTUAL_RATE);
#ifdef AUDIO_DEMO
    log_line("[sidboard] AUDIO_DEMO: Commando, %d frames @ 50 Hz (looping)\n", demo_total);
#endif
    log_line("[sidboard] Bus loop on Core 1 (settle %u ns, write sample %u ns), log %u baud\n",
             (unsigned)BUS_ADDR_SETTLE_NS, (unsigned)BUS_WR_SAMPLE_NS, (unsigned)LOG_BAUD);

    bool fail_seen = false;
    for (;;)
    {
        log_drain();
        if (bus_start_failed && !fail_seen)
        {
            log_line("[sidboard] FATAL: cannot start the bus loop\n");
            fail_seen = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// Setup -- Core 1 until MonitorTask takes it over; never touches Serial.
// ============================================================================
void setup()
{
    build_tables();

    // Every bus pin is an input; D0-D7 become outputs only while the SID
    // answers a read.  GPIO.out is 0: nothing is driven at start-up.
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

    // cSIDLight starts its esp_timer here (Core 0 timer task).
    sidLight     = new cSIDLight();
    sid_bus.regs = sidLight->regs();
    sid_bus.sel  = SID_MODE_PBI ? 0 : 1;

#ifdef AUDIO_DEMO
    {
        esp_timer_create_args_t da = {};
        da.callback        = &demo_tick;
        da.dispatch_method = ESP_TIMER_TASK;
        da.name            = "sid_demo";
        esp_timer_create(&da, &demo_timer_h);
        esp_timer_start_periodic(demo_timer_h, 20000);  // 50 Hz
    }
#endif

    xTaskCreatePinnedToCore(LogTask, "SidLog", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(SidTask, "SidSynth", 4096, NULL, 5, NULL, 0);

    // MonitorTask takes Core 1 over: on success this call does not return.
    if (xTaskCreatePinnedToCore(MonitorTask, "Monitor", 4096, NULL,
                                configMAX_PRIORITIES - 1, NULL, 1) != pdPASS)
        bus_start_failed = true;
}

void loop()
{
    // Not reached: MonitorTask owns Core 1.
    vTaskDelay(portMAX_DELAY);
}
