/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

//#include "stm32h523xx.h"
#include "gamesquirrel/core.h"
#include "gamesquirrel/usb.h"
#include "gamesquirrel/audio.h"
#include "gamesquirrel/sd_card.h"
#include "gamesquirrel/display.h"
#include "gamesquirrel/system.h"
#include "gamesquirrel/fatfs.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

uint8_t xdram_buffer[512] __attribute__((section (".xdram"))) __attribute__((used));

const SystemLib system_table __attribute__((section (".syslib"))) __attribute__((used)) =
{
    { LIB_SYSTEM, 0, 1, 0 },
    NULL, //void *(*FindLibrary)(uint8_t id, uint8_t major_version);

    NULL, //void *(*FastAlloc)(int bytes);
    NULL, //void (*FastFree)(void *ptr);

    NULL, //bool (*SDCardIsInserted)(void);
    NULL, //int (*SDCardInitialize)(void);
    NULL, //int (*SDCardReadBlock)(int32_t block, uint8_t data_out[512]);
    NULL, //int (*SDCardWriteBlock)(int32_t block, uint8_t data_in[512]);
};
const SystemLib *system_lib = &system_table;


uint32_t ADCRead(int n); // FIXME fake

static const uint32_t blink_interval_us = 250000;

void hw_init(void);

// FIXME figure out what to do about newlib and the newlib OS interface

// FIXME why does this get optimized out if not marked used?
int _write(int fhdl, const char *buf, size_t count) __attribute__ ((used));
int _read(int fhdl, char *buf, size_t count) __attribute__ ((used));

// Default logging with on-board UART
int _write (int fhdl, const char *buf, size_t count) {
    // FIXME do some smart delay algorithm so we don't drop characters
    // when a terminal is connected, but don't block when it isn't.
    (void) fhdl;
    return UsbSend(buf, count);
}

int _read (int fhdl, char *buf, size_t count) {
    (void) fhdl;
    int rd = UsbReceive(buf, count);
    return (rd > 0) ? rd : -1;
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}

void hw_init(void)
{
    CoreInit();
    UsbInit();
    AudioInit();
    DisplayInit();
}

void MemTest(void)
{
    volatile uint32_t *buffer = (void *)0x90000000;

    static int base = 0;

    uint32_t t0 = TimeMicroseconds();
    for (int i=0; i<4096; i++)
    {
        buffer[base+i] = ((uint32_t)i)*0x10203041;
    }
    uint32_t t1 = TimeMicroseconds();
    printf("write time = %lu  ", t1-t0);

    int errs = 0;
    t0 = TimeMicroseconds();
    for (int i=0; i<4096; i++)
    {
        if (buffer[base+i] != ((uint32_t)i)*0x10203041)
            errs ++;
    }
    t1 = TimeMicroseconds();
    printf("read time = %lu  errs = %d\r\n", t1-t0, errs);

    base = (base + 4096) & (1024*1024-1);
}

int main(void)
{
    hw_init();
    UsbStart();

    printf("Hello, world\r\n");

    xdram_buffer[0] = 17;

    uint32_t last_us = TimeMicroseconds();
    while (true)
    {
        static char buff[16];
        int length = UsbReceive(buff, 16);
        if (length > 0)
        {
            for (int i=0; i<length; i++)
                buff[i] = buff[i] ^ 0x20;
            UsbSend(buff, length);
        }

        uint32_t us = TimeMicroseconds();
        if (us - last_us < blink_interval_us)
            continue;
        last_us += blink_interval_us;

        LEDWrite(0, (us / 1000) % 1000);
        LEDWrite(1, (us / 5000) % 1000);

        AudioStart(NULL, 0); // FIXME needs real data
        //MemTest();

        if (ButtonRead(0))
        {
            if (SDCardInit())
                printf("SD Card Initialized\r\n");
            else
                printf("SD Card Init FAILED\r\n");

            static FatFs fs;
            static DiskCache cache;
            static FatDir dir;
            static FatDirEntry dirent;
            if (FatInit(&fs, &cache) != SD_OK)
                printf("FatInit FAILED\r\n");

            if (!FatOpenRoot(&fs, &dir))
                printf("FatOpenRootFAILED\r\n");

            while (FatGetNextEntry(&dir, &dirent) == SD_OK)
            {
                DelayClocks(250000);
                printf("%.8s %.3s  size = %lu\r\n",
                        dirent.name, dirent.ext, dirent.size);
            }
            DelayClocks(250000);
            printf("FAT bits = %ld\r\n", fs.fat_bits);
        }

        printf("Tick %lu %.8lX %.8lX %d %d %d %d\r\n",
                us,
                ADCRead(1),
                ADCRead(2),
                ButtonRead(0),
                ButtonRead(1),
                ButtonRead(2),
                ButtonRead(3));
    }
}

