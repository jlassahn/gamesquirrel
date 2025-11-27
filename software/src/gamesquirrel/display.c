/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#pragma GCC optimize ("-fno-strict-aliasing")

#include "gamesquirrel/display.h"
#include "gamesquirrel/core.h"
#include "stm32h523xx.h"

static PixelFormat pix_format = PIX_RGB565;
static ImageLayout img_layout = IMG_LANDSCAPE;

// FIXME design and implement display API
// FIXME add VSync interrupt

static void Delay(uint32_t microseconds)
{
    uint32_t t0 = TimeMicroseconds();
    while ((TimeMicroseconds()-t0) < microseconds)
    {}
}

static void Command(int cmd, int n, uint32_t d)
{
    // violates strict aliasing rules
    volatile uint8_t *tx_reg = (volatile uint8_t *)&SPI2->TXDR;

    GPIOB->BSRR = 0x50000000; // CS low on PB12 CMD low on PB14
    *tx_reg = cmd;
    while ((SPI2->SR & 0x1000) == 0) // wait for TXC
    {}
    GPIOB->BSRR = 0x00004000; // CMD high on PB14

    uint32_t shift = d;
    for (int i=0; i<n; i++)
    {
        *tx_reg = shift&0xFF;
        while ((SPI2->SR & 0x1000) == 0) // wait for TXC
        {}
        shift = shift >> 8;
    }
    GPIOB->BSRR = 0x00001000; // CS high on PB14
}

void DisplayInit(void)
{
    /*
        SWRESET (0x01) // power on reset
        SLPOUT (0x11)  // exit sleep mode (sleep mode is default)
        // PTLON (0x12) // partial mode off by default
        // NORON (0x13) // normal (not partial) display (normal mode is default)
        //INVONOFF (0x20) // inversion off is default
        //INVON (0x21)
        DISPON (0x29) // display off is default
        CASET (0x2A) (4 args) (column range, needs to be set after MADCTL?)
        RASET (0x2B) (4 args) (row range needs to be set)
            // start high  0x00
            // start low   0x00
            // end high   0x01  0x00
            // end low    0x3F  0xEF
        // PTLAR (0x30) partial address
        // TEOFF (0x34)
        TEON (0x35) 0x00 // enable tearing mode VSYNC output 0 arg means vsync 1 arg means hsync and vsync
        MADCTL (0x36) 0x00 // set address scanning order 
            // 0x20 MV = 1 for landscape mode, MV = 0 for portait mode
            // SWRESET doesn't change MADCTL
        // IDMOFF (0x38) // Idle mode is off by default
        COLMOD (0x3A) 0x55  // 0x55 is 16 bits per pixel
        TESCAN (0x44) // 16 bit line

        RAMWR (0x2C) ...
        RAMWRC (0x3C) ...
    */
    Command(0x01, 0, 0); // SWRESET
    Delay(150000);
    Command(0x11, 0, 0); // SLPOUT
    Delay(5000);
    // Command 2 set may only be used when a HW pin is configured?
    // Command(0xDF, 4, 0x0102695A); // Command set 2 enable
    // Command(0xB0, 2, 0xF800); //RAM Control
    Command(0x36, 1, 0x20); // landscape mode
    Command(0x3A, 1, 0x55); // 0x55 is 16 bits per pixel
    Command(0x29, 0, 0); // DISPON
    Command(0x35, 1, 0); // TEON  0 -> vsync 1 -> hsync and vsync
    Command(0x44, 2, 0x40); // TESCAN 16 bit line FIXME what does this do?
    // 0x10 -> 1.12ms

    // FIXME do something real
    (void)img_layout;
    (void)pix_format;

    /*
    Command(0x2C, 2, 0x00FF);
    for (int i=0; i< 1000; i++)
        Command(0x3C, 2, 0x00F8);
    */
}

void DisplayFill(void *framebuffer, int x0, int y0, int width, int height)
{
    int size;

    switch (pix_format)
    {
        case PIX_RGB888:
            size = width*height*3;
            break;

        case PIX_RGB565:
            size = width*height*2;
            break;

        case PIX_RGB444:
            size = width*height;
            size += (size + 1)  >> 1; // FIXME does this work for odd sizes?
            break;

        default:
            // FIXME handle error
            return;
    }

    uint32_t arg;
    arg = (((x0 >> 8) & 0xFF) << 0) |
          (((x0 >> 0) & 0xFF) << 8) |
          ((((x0+width-1) >> 8) & 0xFF) << 16) |
          ((((x0+width-1) >> 0) & 0xFF) << 24);
    Command(0x2A, 4, arg); // column range
    arg = (((y0 >> 8) & 0xFF) << 0) |
          (((y0 >> 0) & 0xFF) << 8) |
          ((((y0+height-1) >> 8) & 0xFF) << 16) |
          ((((y0+height-1) >> 0) & 0xFF) << 24);
    Command(0x2B, 4,arg); // row range

    // violates strict aliasing rules
    volatile uint8_t *tx_reg = (volatile uint8_t *)&SPI2->TXDR;
    uint8_t *data = (uint8_t *)framebuffer;

    GPIOB->BSRR = 0x50000000; // CS low on PB12 CMD low on PB14
    *tx_reg = 0x2C; // RAMWR command
    while ((SPI2->SR & 0x1000) == 0) // wait for TXC
    {}
    GPIOB->BSRR = 0x00004000; // CMD high on PB14

    for (int i=0; i<size; i++)
    {
        *tx_reg = *data;
        while ((SPI2->SR & 0x0002) == 0) // wait for TXP
        {}
        data ++;
    }
    while ((SPI2->SR & 0x1000) == 0) // wait for TXC
    {}
    GPIOB->BSRR = 0x00001000; // CS high on PB14
}

