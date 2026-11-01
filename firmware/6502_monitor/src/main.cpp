/**
 * 6502_monitor -- ROM-less Atari PBI / CCTL peripheral and bus monitor
 *
 * FIRMWARE: Targets NodeMCU DevKit V1 (ESP32-WROOM, Xtensa LX6).
 *
 * BEHAVIOUR: a banked RAM expansion in the Atari I/O area, GeoRAM style:
 * one page holds the registers, the other one is a 256-byte window on the
 * RAM.
 * - Two pages, decoded by the Atari MMU / PLA (on a C64 the same two select
 *   pins would be IO1 / IO2):
 *     ~D1XX  $D100-$D1FF  (PBI device area)
 *     ~CCTL  $D500-$D5FF  (cartridge control area)
 *   Only A0-A7 are sampled: the page comes from the select line.
 * - Deselected: $D1FF and $D5FF are both select latches (W, bit 7 = PBI
 *   device ID $80); nothing else is decoded, nothing is driven.
 * - Only one latch at a time.  Writing bit 7 = 1 to the latch of a page
 *   selects the device with the registers in that page and the RAM in the
 *   other one:
 *     $D1FF <- $80   registers $D1xx, RAM $D500-$D5FF (256 bytes)
 *     $D5FF <- $80   registers $D5xx, RAM $D100-$D1FE (255 bytes: $D1FF is
 *                    the PBI interrupt / select register of the Atari and is
 *                    never driven, writes ignored; -D D1FF_RAM=1 makes it
 *                    RAM too, only without other PBI devices)
 *   The register page:
 *     $xxFB-$xxFC  number of banks (R, 16 bit, little-endian)
 *     $xxFD-$xxFE  bank register (R/W, 16 bit, little-endian)
 *     $xxFF        select latch (W): bit 7 = 0 deselects; never driven
 *     $xx00-$xxFA  reserved: never driven, writes ignored
 *   The latch of the RAM page is RAM: to move the registers to the other
 *   page, deselect first.
 * - One RAM, one bank register for both modes: bank n is the same 256 bytes
 *   whichever page shows it.  The banks are allocated at boot from the free
 *   ESP32 DRAM heap (every byte-addressable block, minus a reserve for the
 *   system); the count depends on the heap.  A bank number beyond the count
 *   selects no RAM: the RAM page is not driven and writes are ignored.
 * - Power-up: deselected, bank 0, RAM $00.  The device does not see the
 *   Atari RESET: its state is kept across it.  RAM is not kept across an
 *   ESP32 reset.
 * - No ROM: the OS PBI scan does not find a signature at $D803 / $D80B and
 *   skips the device; programs select it directly through $D1FF / $D5FF.
 *   The scan writes $80 then $00 to $D1FF: deselected afterwards, unless
 *   the registers are in $D5xx (then $D1FF is protected, or RAM with
 *   D1FF_RAM).
 * - No MPD / EXTSEL: no ROM at $D800, no internal RAM to replace in the I/O
 *   area.
 * - LOG: lock-free ring from the Core 1 bus loop to LogTask on Core 0; prints
 *   latch changes and every access to both windows, selected or not.
 *
 * BUS_C64 = 1 (PlatformIO env c64): GeoRAM on the Commodore 64 expansion
 * port instead, see "C64 GeoRAM" below.
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
 * - Hot-loop data in DRAM, as plain globals (IRAM is not byte-addressable on
 *   the ESP32, so nothing the loop reads may be IRAM_ATTR); GPIO
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
#include <esp_heap_caps.h>
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
#ifndef BUS_C64
#define BUS_C64 0
#endif

#if BUS_C64
/*
 * C64 (same scheme as Vera-Module-RBL-C64): the VIC-II owns the bus during
 * PHI1 (AEC low), so the 6510 address, R/W and /IO1 / /IO2 are valid only
 * while PHI2 is high (~490 ns at 0.985 / 1.023 MHz).  After PHI2 rises the
 * loop polls /IO1 / /IO2 for at most IO_QUALIFY_NS; the PLA asserts them
 * (a few tens of ns after the rise) only for CPU accesses, with the address
 * already stable, so A0-A7 are sampled and a read byte driven at once.
 * Read data must be valid t_DSU (100 ns) before PHI2 falls; write data is
 * valid t_MDS (<= 200 ns) after PHI2 rises.
 */
#ifndef IO_QUALIFY_NS
#define IO_QUALIFY_NS 200u
#endif
#define BUS_ADDR_SETTLE_NS IO_QUALIFY_NS   // not used by the C64 loop
#ifndef BUS_WR_SAMPLE_NS
#define BUS_WR_SAMPLE_NS 300u
#endif
#else
#ifndef BUS_ADDR_SETTLE_NS
#define BUS_ADDR_SETTLE_NS 200u
#endif
#ifndef BUS_WR_SAMPLE_NS
#define BUS_WR_SAMPLE_NS 120u
#endif
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
#define BANK_SIZE    256u   // one bank = one RAM page
#define COUNT_OFFSET 0xFBu  // $xxFB-$xxFC  number of banks (R)
#define BANK_OFFSET  0xFDu  // $xxFD-$xxFE  bank register (R/W)
#define LATCH_OFFSET 0xFFu  // $D1FF / $D5FF

// 1: $D1FF is RAM when the RAM is in $D1xx (only without other PBI devices:
// the OS reads $D1FF on every IRQ, other PBI devices drive it).
#ifndef D1FF_RAM
#define D1FF_RAM 0
#endif

#define WIN_D1XX 0
#define WIN_CCTL 1
#define WINDOWS  2
#define CTL_NONE 0xFFu      // deselected

static const uint8_t WIN_PAGE[WINDOWS] = {0xD1, 0xD5};

// Bytes of the RAM page: $D1FF stays the PBI register unless D1FF_RAM.
#define WIN_RAM_SIZE(w) (((w) == WIN_D1XX && !D1FF_RAM) ? BANK_SIZE - 1u : BANK_SIZE)

/*
 * Banked RAM: 256-byte banks carved at boot from the free DRAM heap, in as
 * many blocks as the heap has (they are not contiguous), and listed in
 * bank_tab[] (bank number -> RAM).  Only byte-addressable internal DRAM
 * (MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL) is used: the bus loop reads it
 * with 8-bit loads.
 */
#ifndef BANK_HEAP_RESERVE
#define BANK_HEAP_RESERVE (24u * 1024u)   // left to the system (stacks, UART, ...)
#endif
#define BANK_MIN_CHUNK    (4u * 1024u)    // smaller free blocks are left alone

static uint8_t **bank_tab;            // bank -> 256 bytes
static uint32_t  bank_heap_left;      // free heap after the allocation (log)
static uint32_t  bank_chunks;         // heap blocks used (log)

// Device state, read by the bus loop assembly: field offsets are fixed.
typedef struct {
    uint8_t *ram[WINDOWS];      // +0 / +4  RAM of the current bank, set for the
                                //          RAM page only, NULL beyond the count
    uint32_t ram_size[WINDOWS]; // +8 / +12 bytes of the page as RAM page
    uint8_t  regs[4];           // +16  count lo, count hi, bank lo, bank hi
    uint8_t  ctl;               // +20  page holding registers and latch,
                                //      CTL_NONE: deselected
    uint8_t  pad[3];
} DevState;

static_assert(sizeof(DevState) == 24 && offsetof(DevState, ram) == 0 &&
              offsetof(DevState, ram_size) == 8 && offsetof(DevState, regs) == 16 &&
              offsetof(DevState, ctl) == 20, "DevState layout used by the bus loop assembly");
static_assert(WIN_D1XX == 0 && WIN_CCTL == 1, "window index used by the bus loop assembly");
static_assert(COUNT_OFFSET == 0xFBu && BANK_OFFSET == 0xFDu,
              "movi -0xFB in the bus loop assembly");

static DevState dev;

static inline __attribute__((always_inline)) uint32_t dev_count(void)
{
    return dev.regs[0] | (uint32_t)dev.regs[1] << 8;
}

static inline __attribute__((always_inline)) uint32_t dev_bank(void)
{
    return dev.regs[2] | (uint32_t)dev.regs[3] << 8;
}

// Show the current bank in the RAM page (none while deselected, NULL when
// no RAM is behind that number).
static inline __attribute__((always_inline)) void dev_map(void)
{
    dev.ram[0] = NULL;
    dev.ram[1] = NULL;
    if (dev.ctl != CTL_NONE)
    {
        const uint32_t b = dev_bank();
        dev.ram[dev.ctl ^ 1u] = (b < dev_count()) ? bank_tab[b] : NULL;
    }
}

/*
 * C64 GeoRAM (BUS_C64): always enabled, no latch.
 *   /IO1  $DE00-$DEFF  256-byte window on the RAM (R/W)
 *   /IO2  $DFFE        page within the 16 KB block (W, bits 0-5)
 *         $DFFF        block (W)
 *   Both registers are write-only, as on the original GeoRAM (software keeps
 *   a copy); the rest of IO2 is not decoded.  RAM address = block * 16384 +
 *   page * 256 + offset.
 * Size: GEORAM_KB (default 256 KB = 16 blocks), a power of two as on the
 *   real GeoRAMs: the block register is masked to the size, so block 16
 *   aliases block 0 (what size detection relies on).  The DRAM heap alone
 *   holds ~210 KB, so the missing pages come from the free IRAM (~70 KB,
 *   32-bit access only): every RAM access of the C64 loop is a 32-bit load
 *   (and a read-modify-write for stores), which also works on DRAM.  If the
 *   heap is short, the size drops to the next power of two (banner).
 * Wiring: /IO1 -> J3 pin 12 (GPIO23, the ~CCTL input), /IO2 -> J3 pin 11
 * (GPIO22, the ~D1XX input).
 */
#define PIN_IO1_N PIN_CCTL_N
#define PIN_IO2_N PIN_D1XX_N
#define GEO_PAGES_PER_BLOCK 64u
#define GEO_PAGE_REG  0xFEu
#define GEO_BLOCK_REG 0xFFu
#define WIN_IO1 0           // log: page index of IO1 / IO2
#define WIN_IO2 1

#ifndef GEORAM_KB
#define GEORAM_KB 256u
#endif
#define GEO_IRAM_RESERVE (2u * 1024u)    // IRAM left to the system
static_assert((GEORAM_KB & (GEORAM_KB - 1u)) == 0 && GEORAM_KB >= 16u && GEORAM_KB <= 4096u,
              "GEORAM_KB: power of two, 16 KB - 4 MB");

static uint32_t geo_blocks;     // 16 KB blocks, power of two (0: no RAM)
static uint32_t geo_dram_pages; // pages carved from DRAM / IRAM (log)
static uint32_t geo_iram_pages;
static uint32_t geo_iram_left;  // free IRAM after the allocation (log)
static uint8_t  geo_page;       // $DFFE
static uint8_t  geo_block;      // $DFFF
static uint8_t *geo_ram;        // RAM of the current page, NULL: no RAM

// Page index, with the block masked to the size as on a real GeoRAM.
static inline __attribute__((always_inline)) uint32_t geo_bank(void)
{
    return (geo_block & (geo_blocks - 1u)) * GEO_PAGES_PER_BLOCK +
           (geo_page & (GEO_PAGES_PER_BLOCK - 1u));
}

static inline __attribute__((always_inline)) void geo_map(void)
{
    geo_ram = geo_blocks ? bank_tab[geo_bank()] : NULL;
}

// GPIO.out word (D0-D7) for each byte value.
static uint32_t drive_lut[256];

// ---------------------------------------------------------------------------
// Log Ring (Core 1 bus loop → Core 0 LogTask)
//
// Single producer / single consumer, lock-free: the bus loop only stores the
// event and advances log_head (no FreeRTOS call, interrupts are masked on
// Core 1).  When the ring is full new events are dropped and counted.
// ---------------------------------------------------------------------------
#define LOG_RING_SIZE 1024u   // power of two

#define EVT_LATCH 0   // select latch change ($D1FF / $D5FF write)
#define EVT_REG   1   // access to RAM, registers or reserved offsets

// flags: bit 0 = write   bit 1 = device selected (EVT_REG) / new latch state
//        (EVT_LATCH)     bit 2 = byte driven by this device (reads)
//        bit 3 = the page was the RAM page
#define FL_WRITE  0x01u
#define FL_SEL    0x02u
#define FL_DRIVEN 0x04u
#define FL_RAM    0x08u

typedef struct {
    uint32_t cyc_lo;  // timestamp: 64-bit Core 1 CCOUNT since the bus loop started
    uint32_t cyc_hi;
    uint8_t  type;
    uint8_t  win;     // page: WIN_D1XX / WIN_CCTL
    uint8_t  offset;  // A0-A7
    uint8_t  data;
    uint8_t  flags;
    uint16_t bank;    // bank register at the time of the access
} LogEvt;

static LogEvt   log_ring[LOG_RING_SIZE];
static uint32_t log_head    = 0;  // written by the bus loop only
static uint32_t log_tail    = 0;  // written by LogTask only
static uint32_t log_dropped = 0;  // written by the bus loop only

// ---------------------------------------------------------------------------
// Decoding Tables & Helpers
// ---------------------------------------------------------------------------
#define BUS_INLINE static inline __attribute__((always_inline))

// A0-A5 from GPIO.in1 bits 0-7 (GPIO 32-39).
static uint8_t addr_lut_hi[256];
// A6, A7 from GPIO.in bits 21-27 (GPIO 21, 27); ~D1XX / ~CCTL ignored.
static uint8_t addr_lut_lo[128];

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
    for (uint32_t i = 0; i < 256; i++)
        drive_lut[i] = encode_data(i);

}

/*
 * Carve 256-byte banks from the heap regions with `caps` into tab[n..max),
 * leaving `reserve` bytes free.  Zeroed with 32-bit stores (IRAM takes no
 * byte stores).  Returns the new number of banks.
 */
static uint32_t carve_banks(uint8_t **tab, uint32_t n, uint32_t max, uint32_t caps,
                            size_t reserve)
{
    while (n < max)
    {
        const size_t free    = heap_caps_get_free_size(caps);
        const size_t largest = heap_caps_get_largest_free_block(caps);
        if (free <= reserve)
            break;
        size_t want = free - reserve;
        if (want > largest)
            want = largest;
        if (want > (max - n) * BANK_SIZE)
            want = (max - n) * BANK_SIZE;
        want &= ~(size_t)(BANK_SIZE - 1u);
        if (want < BANK_MIN_CHUNK && want < (max - n) * BANK_SIZE)
            break;
        if (want == 0)
            break;
        uint8_t *chunk = (uint8_t *)heap_caps_malloc(want, caps);
        if (!chunk)
            break;
        volatile uint32_t *w = (volatile uint32_t *)chunk;
        for (size_t i = 0; i < want / 4u; i++)
            w[i] = 0;
        bank_chunks++;
        for (size_t o = 0; o < want; o += BANK_SIZE)
            tab[n++] = chunk + o;
    }
    return n;
}

/*
 * Allocate the banks from the free heap.  Runs in setup(), before the bus
 * loop starts.  Atari: every byte-addressable DRAM block.  C64: GEORAM_KB,
 * DRAM first, then IRAM.
 */
static void alloc_banks(void)
{
    const uint32_t caps = MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL;

#if BUS_C64
    const uint32_t max_banks = GEORAM_KB * 1024u / BANK_SIZE;
#else
    // Upper bound for the table: every free byte in banks, 4 bytes of table
    // per bank; the bank register is 16 bit.
    const size_t free0     = heap_caps_get_free_size(caps);
    uint32_t     max_banks = (free0 > BANK_HEAP_RESERVE)
                                 ? (uint32_t)((free0 - BANK_HEAP_RESERVE) / (BANK_SIZE + 4u))
                                 : 0u;
    if (max_banks > 0xFFFFu)
        max_banks = 0xFFFFu;
#endif
    uint8_t **tab = max_banks ? (uint8_t **)heap_caps_malloc(max_banks * sizeof(uint8_t *), caps)
                              : NULL;
    uint32_t n = tab ? carve_banks(tab, 0, max_banks, caps, BANK_HEAP_RESERVE) : 0u;

    bank_tab = tab;

#if BUS_C64
    geo_dram_pages = n;
    if (tab)
        n = carve_banks(tab, n, max_banks, MALLOC_CAP_EXEC, GEO_IRAM_RESERVE);
    geo_iram_pages = n - geo_dram_pages;
    geo_iram_left  = heap_caps_get_free_size(MALLOC_CAP_EXEC);

    // Largest power of two of whole blocks that fits.
    const uint32_t blocks = n / GEO_PAGES_PER_BLOCK;
    geo_blocks = 0;
    while ((geo_blocks ? geo_blocks * 2u : 1u) <= blocks)
        geo_blocks = geo_blocks ? geo_blocks * 2u : 1u;
    geo_page  = 0;
    geo_block = 0;
    geo_map();
    n = geo_blocks * GEO_PAGES_PER_BLOCK;
#endif
    bank_heap_left = heap_caps_get_free_size(caps);

    dev.ctl         = CTL_NONE;
    dev.ram_size[0] = WIN_RAM_SIZE(WIN_D1XX);
    dev.ram_size[1] = WIN_RAM_SIZE(WIN_CCTL);
    dev.regs[0]     = (uint8_t)n;
    dev.regs[1]     = (uint8_t)(n >> 8);
    dev.regs[2]     = 0;
    dev.regs[3]     = 0;
    dev_map();
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

/*
 * Byte the device answers with at offset `off` of page `win`: the RAM of the
 * current bank or a register.  Returns 0 when the device must not drive the
 * bus (deselected, latch, reserved offsets, protected $D1FF, bank beyond the
 * count).  C copy of the PHI1 assembly, used on the late-select path only.
 */
BUS_INLINE uint32_t dev_read(int win, uint32_t off, uint32_t *val)
{
    const uint8_t *ram = dev.ram[win];
    if (ram)
    {
        if (off >= dev.ram_size[win])
            return 0;
        *val = ram[off];
        return 1;
    }
    if (dev.ctl == (uint32_t)win && off - COUNT_OFFSET < 4u)
    {
        *val = dev.regs[off - COUNT_OFFSET];
        return 1;
    }
    return 0;
}

// Log flags for an access to page `win` with the current state.
BUS_INLINE uint8_t dev_flags(int win)
{
    if (dev.ctl == CTL_NONE)
        return 0;
    return (dev.ctl == (uint32_t)win) ? FL_SEL : (FL_SEL | FL_RAM);
}

// ============================================================================
// MonitorTask -- Core 1, IRAM-resident, interrupts masked
//
// Core 1 runs only this task: it never yields and nothing else runs there
// (no tick, no interrupts), so no bus cycle is missed.  Core 0 runs LogTask,
// Serial and its UART interrupt.
// ============================================================================
#if BUS_C64
/*
 * Byte access to a bank in DRAM or IRAM with 32-bit loads / stores only
 * (IRAM raises LoadStoreError on 8-bit accesses).  The load is in assembly so
 * that the compiler cannot narrow it to an l8ui.
 */
BUS_INLINE uint32_t ram_load32(const uint8_t *p)
{
    uint32_t v;
    asm volatile("l32i %0, %1, 0" : "=r"(v) : "r"(p) : "memory");
    return v;
}

BUS_INLINE uint8_t ram_read32(const uint8_t *ram, uint32_t off)
{
    return (uint8_t)(ram_load32(ram + (off & ~3u)) >> ((off & 3u) * 8u));
}

BUS_INLINE void ram_write32(uint8_t *ram, uint32_t off, uint8_t data)
{
    uint8_t *const p  = ram + (off & ~3u);
    const uint32_t sh = (off & 3u) * 8u;
    const uint32_t w  = (ram_load32(p) & ~(0xFFu << sh)) | ((uint32_t)data << sh);
    asm volatile("s32i %0, %1, 0" : : "r"(w), "r"(p) : "memory");
}

/*
 * C64 bus loop: everything happens while PHI2 is high.  PHI2 rises, poll
 * /IO1 / /IO2 for at most IO_QUALIFY_NS (neither: not ours, or a VIC-II
 * cycle); as soon as one is LOW sample A0-A7; a read of IO1 drives the byte
 * at once and until PHI2 falls; a write is sampled BUS_WR_SAMPLE_NS after
 * the rise.  Plenty of time at 1 MHz: the decode is plain C.
 */
static void IRAM_ATTR MonitorTask(void *pvParameters)
{
    const uint32_t m_rw  = (1UL << PIN_RW);
    const uint32_t m_io1 = (1UL << PIN_IO1_N);
    const uint32_t m_io2 = (1UL << PIN_IO2_N);

    const uint32_t qualify   = NS_TO_CYCLES(IO_QUALIFY_NS);
    const uint32_t wr_sample = NS_TO_CYCLES(BUS_WR_SAMPLE_NS);

    uint32_t cyc_hi = 0;
    uint32_t t_prev = cpu_ll_get_cycle_count();
    LogEvt   pend;
    bool     pend_valid = false;

    bus_release();
    gpio_out(0);

    bus_int_wdt_disable();
    portDISABLE_INTERRUPTS();

    wait_phi2_low();

    for (;;)
    {
        // ---- PHI1: the VIC-II's half; log the previous cycle ---------------
        if (pend_valid)
        {
            log_push(&pend);
            pend_valid = false;
        }

        // ---- PHI2 high: the 6510's half -------------------------------------
        wait_phi2_high();
        const uint32_t t_rise = cpu_ll_get_cycle_count();
        if (t_rise < t_prev)
            cyc_hi++;
        t_prev = t_rise;

        // /IO1 or /IO2 within IO_QUALIFY_NS, else not ours (or VIC-II DMA).
        const uint32_t t_limit = t_rise + qualify;
        uint32_t lo;
        while (((lo = gpio_in()) & (m_io1 | m_io2)) == (m_io1 | m_io2))
            if ((int32_t)(cpu_ll_get_cycle_count() - t_limit) >= 0)
                break;
        int win;
        if (!(lo & m_io1))
            win = WIN_IO1;
        else if (!(lo & m_io2))
            win = WIN_IO2;
        else
        {
            wait_phi2_low();
            continue;
        }
        const uint32_t off = decode_addr(lo, gpio_in1());

        pend.cyc_lo = t_rise;
        pend.cyc_hi = cyc_hi;
        pend.win    = (uint8_t)win;
        pend.offset = (uint8_t)off;
        pend.bank   = (uint16_t)geo_bank();
        pend.type   = EVT_REG;
        pend_valid  = true;

        if (lo & m_rw)
        {
            const uint8_t *ram = geo_ram;
            if (win == WIN_IO1 && ram)
            {
                const uint8_t v = ram_read32(ram, off);
                gpio_out(drive_lut[v]);
                bus_drive_enable();
                pend.data  = v;
                pend.flags = FL_SEL | FL_RAM | FL_DRIVEN;
                wait_phi2_low();
                bus_release();
            }
            else
            {
                // IO2 is write-only; no RAM behind the block: not driven.
                uint32_t last = lo, now;
                while ((now = gpio_in()) & (1UL << PIN_PHI2))
                    last = now;
                pend.data  = decode_data(last);
                pend.flags = (win == WIN_IO1) ? (FL_SEL | FL_RAM) : FL_SEL;
            }
        }
        else
        {
            wait_until(t_rise + wr_sample);
            const uint8_t data = decode_data(gpio_in());
            pend.data  = data;
            pend.flags = FL_WRITE | FL_SEL | (win == WIN_IO1 ? FL_RAM : 0u);
            if (win == WIN_IO1)
            {
                if (geo_ram)
                    ram_write32(geo_ram, off, data);
            }
            else if (off == GEO_PAGE_REG)
            {
                geo_page = data;
                geo_map();
            }
            else if (off == GEO_BLOCK_REG)
            {
                geo_block = data;
                geo_map();
            }
            wait_phi2_low();
        }
    }
}
#else
static void IRAM_ATTR MonitorTask(void *pvParameters)
{
    const uint32_t m_rw   = (1UL << PIN_RW);
    const uint32_t m_d1xx = (1UL << PIN_D1XX_N);
    const uint32_t m_cctl = (1UL << PIN_CCTL_N);
    const uint32_t m_phi2 = (1UL << PIN_PHI2);

    const uint32_t settle    = NS_TO_CYCLES(BUS_ADDR_SETTLE_NS);
    const uint32_t wr_sample = NS_TO_CYCLES(BUS_WR_SAMPLE_NS);
    const uint32_t phi2_max  = NS_TO_CYCLES(BUS_PHI2_HIGH_MAX_NS);

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

        /*
         * 4. PHI1 decode, in assembly so that every operand stays in a
         *    register: A0-A7 from the tables, page from ~D1XX / ~CCTL; on a
         *    read of the RAM page (bank within the count, not the protected
         *    $D1FF) or of a register ($FB-$FE) of the register page, the
         *    byte goes into GPIO.out now (outputs off) and the outputs are
         *    turned on at the PHI2 rising edge.
         *    win = 0 ($D1xx), 1 ($D5xx), -1 (neither).
         */
        uint32_t lo, off, t, val, ws, drive;
        int32_t  win;
        asm volatile(
            "   l32i   %[lo], %[gpio], %[o_in]    \n\t"   /* GPIO.in  */
            "   l32i   %[t], %[gpio], %[o_in1]    \n\t"   /* GPIO.in1 */
            "   extui  %[t], %[t], 0, 8           \n\t"   /* GPIO 32-39 */
            "   add    %[t], %[t], %[luthi]       \n\t"
            "   l8ui   %[off], %[t], 0            \n\t"   /* A0-A5 */
            "   extui  %[t], %[lo], 21, 7         \n\t"   /* GPIO 21-27 */
            "   add    %[t], %[t], %[lutlo]       \n\t"
            "   l8ui   %[t], %[t], 0              \n\t"   /* A6, A7 */
            "   or     %[off], %[off], %[t]       \n\t"
            "   movi   %[drv], 0                  \n\t"
            "   movi   %[val], 0                  \n\t"
            "   movi   %[win], 0                  \n\t"
            "   bbci   %[lo], %[b_d1], 1f         \n\t"   /* ~D1XX low: page 0 */
            "   movi   %[win], 1                  \n\t"
            "   bbci   %[lo], %[b_cc], 1f         \n\t"   /* ~CCTL low: page 1 */
            "   movi   %[win], -1                 \n\t"
            "   j      9f                         \n\t"
            "1: bbci   %[lo], %[b_rw], 9f         \n\t"   /* write cycle */
            "   addx4  %[ws], %[win], %[st]       \n\t"   /* &dev.ram[win] */
            "   l32i   %[t], %[ws], 0             \n\t"   /* ram */
            "   beqz   %[t], 2f                   \n\t"   /* not the RAM page, or no RAM */
            "   l32i   %[ws], %[ws], 8            \n\t"   /* ram_size[win] */
            "   bgeu   %[off], %[ws], 9f          \n\t"   /* protected $D1FF */
            "   add    %[t], %[t], %[off]         \n\t"
            "   l8ui   %[val], %[t], 0            \n\t"
            "   j      3f                         \n\t"
            "2: l8ui   %[t], %[st], 20            \n\t"   /* ctl */
            "   bne    %[t], %[win], 9f           \n\t"   /* not the register page */
            "   movi   %[t], -0xFB                \n\t"   /* COUNT_OFFSET */
            "   add    %[t], %[off], %[t]         \n\t"
            "   bgeui  %[t], 4, 9f                \n\t"   /* not $FB-$FE */
            "   add    %[t], %[t], %[st]          \n\t"
            "   l8ui   %[val], %[t], 16           \n\t"   /* regs[off - $FB] */
            "3: addx4  %[t], %[val], %[dlut]      \n\t"
            "   l32i   %[t], %[t], 0              \n\t"
            "   s32i   %[t], %[gpio], %[o_out]    \n\t"   /* GPIO.out, outputs off */
            "   movi   %[drv], 1                  \n\t"
            "9:                                   \n\t"
            : [lo] "=&r"(lo), [off] "=&r"(off), [t] "=&r"(t), [val] "=&r"(val),
              [ws] "=&r"(ws), [drv] "=&r"(drive), [win] "=&r"(win)
            : [gpio] "r"(&GPIO), [luthi] "r"(addr_lut_hi), [lutlo] "r"(addr_lut_lo),
              [dlut] "r"(drive_lut), [st] "r"(&dev),
              [o_in] "n"(GPIO_OFF(in)), [o_in1] "n"(GPIO_OFF(in1)),
              [o_out] "n"(GPIO_OFF(out)),
              [b_d1] "n"(PIN_D1XX_N), [b_cc] "n"(PIN_CCTL_N), [b_rw] "n"(PIN_RW)
            : "memory");
        (void)ws;
        const bool rd = (lo & m_rw) != 0;

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
            drive = rd ? dev_read(win, off, &val) : 0;
            if (drive)
            {
                gpio_out(drive_lut[val]);
                bus_drive_enable();
            }
        }

        pend.cyc_lo = t_rise;
        pend.cyc_hi = cyc_hi;
        pend.win    = (uint8_t)win;
        pend.offset = (uint8_t)off;
        pend.bank   = (uint16_t)dev_bank();
        pend_valid  = true;

        if (rd)
        {
            pend.type  = EVT_REG;
            pend.flags = dev_flags(win) | (drive ? FL_DRIVEN : 0u);
            if (drive)
            {
                pend.data = (uint8_t)val;
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

            pend.type  = EVT_REG;
            pend.flags = FL_WRITE | dev_flags(win);

            if (off == LATCH_OFFSET && (dev.ctl == CTL_NONE || dev.ctl == (uint32_t)win))
            {
                // Latch of the register page, or of either page while
                // deselected: bit 7 (device ID) selects with the registers
                // here, 0 deselects.
                const uint8_t ctl = (data & PBI_DEV_ID) ? (uint8_t)win : (uint8_t)CTL_NONE;
                if (ctl != dev.ctl)
                {
                    dev.ctl = ctl;
                    dev_map();
                    pend.type  = EVT_LATCH;
                    pend.flags = FL_WRITE | (ctl != CTL_NONE ? FL_SEL : 0u);
                }
            }
            else if (dev.ram[win])
            {
                // RAM page ($D1FF protected unless D1FF_RAM).
                if (off < dev.ram_size[win])
                    dev.ram[win][off] = data;
            }
            else if (dev.ctl == (uint32_t)win &&
                     (off == BANK_OFFSET || off == BANK_OFFSET + 1u))
            {
                dev.regs[off - COUNT_OFFSET] = data;
                dev_map();
            }
            // Anything else: deselected, reserved, read-only or a bank
            // beyond the count: ignored.
            wait_phi2_low();
        }
    }
}
#endif  // BUS_C64

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
static volatile bool banks_ready      = false;   // set by setup()

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

// Name of an offset within a page, for the log.
static const char *offset_name(uint8_t w, uint8_t off, uint8_t flags, uint16_t bank,
                               char *buf, size_t len)
{
#if BUS_C64
    (void)flags;
    if (w == WIN_IO1)
    {
        snprintf(buf, len, "RAM blk $%02X pg $%02X +$%02X",
                 (unsigned)(bank / GEO_PAGES_PER_BLOCK) & 0xFFu,
                 (unsigned)(bank % GEO_PAGES_PER_BLOCK), off);
        return buf;
    }
    if (off == GEO_PAGE_REG)
        return "GEORAM_PAGE";
    if (off == GEO_BLOCK_REG)
        return "GEORAM_BLOCK";
    return "UNUSED";
#endif
    if ((flags & FL_RAM) && off < WIN_RAM_SIZE(w))
    {
        snprintf(buf, len, "RAM bank $%04X +$%02X", bank, off);
        return buf;
    }
    if (off == LATCH_OFFSET)
        return (w == WIN_D1XX) ? "PBI_SELECT" : "CCTL_SELECT";
    if (flags & FL_RAM)
        return "RESERVED";
    if (off == COUNT_OFFSET)
        return "BANK_COUNT_L";
    if (off == COUNT_OFFSET + 1u)
        return "BANK_COUNT_H";
    if (off == BANK_OFFSET)
        return "BANK_SELECT_L";
    if (off == BANK_OFFSET + 1u)
        return "BANK_SELECT_H";
    return "RESERVED";
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
        const uint8_t  w     = evt.win < WINDOWS ? evt.win : 0;
        const uint8_t  page  = BUS_C64 ? (uint8_t)(0xDEu + w) : WIN_PAGE[w];
        char           nbuf[32];
        const char    *name  = offset_name(w, evt.offset, evt.flags, evt.bank,
                                           nbuf, sizeof(nbuf));

        if (evt.type == EVT_LATCH)
        {
            if (evt.flags & FL_SEL)
                log_line("[%5lu.%06lu] [%02X%02X - %-20s] W $%02X  SELECTED: "
                         "registers $%02Xxx, RAM $%02Xxx\n",
                         (unsigned long)sec, (unsigned long)us, page, evt.offset,
                         name, evt.data, page, WIN_PAGE[w ^ 1u]);
            else
                log_line("[%5lu.%06lu] [%02X%02X - %-20s] W $%02X  DESELECTED\n",
                         (unsigned long)sec, (unsigned long)us, page, evt.offset,
                         name, evt.data);
        }
        else  // EVT_REG
        {
            const bool  wr  = (evt.flags & FL_WRITE) != 0;
            const char *tag = !(evt.flags & FL_SEL)     ? "-"
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

    while (!banks_ready)
        vTaskDelay(pdMS_TO_TICKS(1));

#if BUS_C64
    log_line("\n[6502_monitor] GeoRAM on the C64 expansion port: %lu KB = %lu blocks x 16 KB%s\n",
             (unsigned long)(geo_blocks * 16u), (unsigned long)geo_blocks,
             (geo_blocks * 16u < GEORAM_KB) ? " (heap too small for GEORAM_KB)" : "");
    log_line("[6502_monitor] Pages: %lu in DRAM, %lu in IRAM, %lu heap blocks; "
             "left %lu bytes DRAM, %lu bytes IRAM\n",
             (unsigned long)geo_dram_pages, (unsigned long)geo_iram_pages,
             (unsigned long)bank_chunks, (unsigned long)bank_heap_left,
             (unsigned long)geo_iram_left);
    log_line("[6502_monitor] IO1 $DE00-$DEFF RAM, IO2 $DFFE page, $DFFF block (write-only)\n");
    log_line("[6502_monitor] /IO1 on J3 pin 12 (GPIO%u), /IO2 on J3 pin 11 (GPIO%u)\n",
             (unsigned)PIN_IO1_N, (unsigned)PIN_IO2_N);
    log_line("[6502_monitor] Bus loop on Core 1 (/IOx within %u ns, write sample %u ns after "
             "PHI2 rises), log %u baud\n",
             (unsigned)IO_QUALIFY_NS, (unsigned)BUS_WR_SAMPLE_NS, (unsigned)LOG_BAUD);
#else
    const uint32_t banks = dev_count();
    log_line("\n[6502_monitor] Banked RAM PBI / CCTL peripheral, device ID $%02X\n",
             (unsigned)PBI_DEV_ID);
    log_line("[6502_monitor] %lu banks x %u bytes = %lu KB, %lu heap blocks, "
             "%lu bytes of heap left\n",
             (unsigned long)banks, (unsigned)BANK_SIZE,
             (unsigned long)(banks * BANK_SIZE / 1024u),
             (unsigned long)bank_chunks, (unsigned long)bank_heap_left);
    log_line("[6502_monitor] $D1FF <- $80: RAM $D500-$D5FF; $D5FF <- $80: RAM $D100-$D1%s\n",
             D1FF_RAM ? "FF" : "FE ($D1FF protected)");
    log_line("[6502_monitor] Register page: count $xxFB-$xxFC, bank $xxFD-$xxFE, "
             "latch $xxFF\n");
    log_line("[6502_monitor] Bus loop on Core 1 (settle %u ns, write sample %u ns), "
             "log %u baud\n",
             (unsigned)BUS_ADDR_SETTLE_NS, (unsigned)BUS_WR_SAMPLE_NS, (unsigned)LOG_BAUD);
#endif

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
    // bus events, all from the same task.  The banner waits for the banks.
    xTaskCreatePinnedToCore(LogTask, "BusLog", 4096, NULL, 1, NULL, 0);

    // Banks from the free heap, leaving BANK_HEAP_RESERVE to the system
    // (the MonitorTask stack below comes out of it).
    alloc_banks();
    banks_ready = true;

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
