/*
 * sid_dump_player.c -- plays a SID register dump on the sidboard
 *
 * The dump (D:SIDDUMP.BIN, made by ../mkdump.py from Commando_dmp.h) is a
 * flat list of frames of 25 bytes: SID registers $00-$18, one frame per
 * PAL VBlank (50 Hz).  Each frame is written straight into the SID
 * registers, then the player waits for the next VBlank (RTCLOK).
 *
 *   sid-atari-cctl firmware: registers at $D500, always on      (SID_PBI = 0)
 *   sid-atari-pbi  firmware: registers at $D100, select $D1FF   (SID_PBI = 1)
 *
 * At the end (or on a key) the voices are gated off and the volume set to 0;
 * on sid-atari-pbi the device is deselected.
 *
 * Build: make (both variants).
 */
#include <atari.h>
#include <stdio.h>

#ifndef SID_PBI
#define SID_PBI 0
#endif

#if SID_PBI
#define SID_BASE   0xD100u
#define PBI_SELECT (*(volatile unsigned char *)0xD1FFu)
#define PBI_ID     0x80u
#else
#define SID_BASE   0xD500u
#endif

#define SID           ((volatile unsigned char *)SID_BASE)
#define SID_REG_COUNT 25u
#define MAX_DUMP_SIZE 28000u   /* Commando: 27500 bytes, 1100 frames */

static unsigned char dump_buffer[MAX_DUMP_SIZE];

static void wait_vbl_tick(void)
{
    unsigned char tick = OS.rtclok[2];
    while (OS.rtclok[2] == tick) {
    }
}

static void sid_silence(void)
{
    SID[0x04] = 0;            /* gates off */
    SID[0x0B] = 0;
    SID[0x12] = 0;
    SID[0x18] = 0;            /* volume 0 */
}

int main(void)
{
    static const char dump_name[] = "D:SIDDUMP.BIN";
    FILE *dump_file;
    unsigned int length;
    unsigned int frame;
    unsigned char reg;
    const unsigned char *f;

    dump_file = fopen(dump_name, "rb");
    if (dump_file == 0) {
        printf("Cannot open %s\n", dump_name);
        return 1;
    }
    length = fread(dump_buffer, 1, MAX_DUMP_SIZE, dump_file);
    fclose(dump_file);

    if (length < SID_REG_COUNT) {
        printf("Dump too short (%u bytes)\n", length);
        return 1;
    }
    printf("%u frames, SID at $%04X\n", length / SID_REG_COUNT, SID_BASE);

#if SID_PBI
    PBI_SELECT = PBI_ID;
#endif
    OS.ch = 0xFF;             /* no key pressed */
    for (frame = 0; frame + SID_REG_COUNT <= length; frame += SID_REG_COUNT) {
        f = dump_buffer + frame;
        for (reg = 0; reg < SID_REG_COUNT; ++reg) {
            SID[reg] = f[reg];
        }
        wait_vbl_tick();
        if (OS.ch != 0xFF) {
            break;            /* any key stops */
        }
    }
    sid_silence();
#if SID_PBI
    PBI_SELECT = 0;
#endif

    printf("Played %u frames\n", frame / SID_REG_COUNT);
    return 0;
}
