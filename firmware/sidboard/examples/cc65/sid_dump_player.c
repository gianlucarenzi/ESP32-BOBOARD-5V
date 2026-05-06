#include <atari.h>
#include <stdio.h>

#define SID_CHANNEL     1
#define SID_REG_COUNT   25u
#define SID_MODE_WRITE  8u
#define MAX_DUMP_SIZE   4096u

static unsigned char dump_buffer[MAX_DUMP_SIZE];
static unsigned char io_byte;

static void ciov_sid_channel(void)
{
    __asm__("ldx #$10");
    __asm__("jsr $E456");
}

static unsigned char sid_open(void)
{
    static const char device_name[] = "I:";

    OS.iocb[SID_CHANNEL].buffer = (void*) device_name;
    OS.iocb[SID_CHANNEL].buflen = sizeof(device_name) - 1u;
    OS.iocb[SID_CHANNEL].command = IOCB_OPEN;
    OS.iocb[SID_CHANNEL].aux1 = SID_MODE_WRITE;
    OS.iocb[SID_CHANNEL].aux2 = 0;
    ciov_sid_channel();

    return OS.iocb[SID_CHANNEL].status;
}

static unsigned char sid_close(void)
{
    OS.iocb[SID_CHANNEL].command = IOCB_CLOSE;
    ciov_sid_channel();

    return OS.iocb[SID_CHANNEL].status;
}

static unsigned char sid_put(unsigned char reg, unsigned char value)
{
    io_byte = value;
    OS.iocb[SID_CHANNEL].buffer = &io_byte;
    OS.iocb[SID_CHANNEL].buflen = 1u;
    OS.iocb[SID_CHANNEL].command = IOCB_PUTCHR;
    OS.iocb[SID_CHANNEL].aux1 = reg;
    ciov_sid_channel();

    return OS.iocb[SID_CHANNEL].status;
}

static void wait_vbl_tick(void)
{
    unsigned char tick = OS.rtclok[2];
    while (OS.rtclok[2] == tick) {
    }
}

int main(void)
{
    static const char dump_name[] = "D:SIDDUMP.BIN";
    FILE* dump_file;
    unsigned int length = 0;
    unsigned int frame;
    unsigned char reg;
    int ch;
    unsigned char status;

    dump_file = fopen(dump_name, "rb");
    if (dump_file == 0) {
        printf("Cannot open %s\n", dump_name);
        return 1;
    }

    while (length < MAX_DUMP_SIZE) {
        ch = fgetc(dump_file);
        if (ch == EOF) {
            break;
        }
        dump_buffer[length++] = (unsigned char) ch;
    }

    fclose(dump_file);

    if (length == 0) {
        printf("Empty dump file\n");
        return 1;
    }

    if (length < SID_REG_COUNT) {
        printf("Dump too short\n");
        return 1;
    }

    status = sid_open();
    if (status >= 128u) {
        printf("I: open error %u\n", status);
        return 1;
    }

    for (frame = 0; frame + SID_REG_COUNT <= length; frame += SID_REG_COUNT) {
        for (reg = 0; reg < SID_REG_COUNT; ++reg) {
            status = sid_put(reg, dump_buffer[frame + reg]);
            if (status >= 128u) {
                printf("I: write error %u at byte %u\n", status, frame + reg);
                sid_close();
                return 1;
            }
        }
        wait_vbl_tick();
    }

    status = sid_close();
    if (status >= 128u) {
        printf("I: close error %u\n", status);
        return 1;
    }

    printf("Sent %u frames to I:\n", length / SID_REG_COUNT);
    return 0;
}
