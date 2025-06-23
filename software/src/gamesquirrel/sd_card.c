/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#pragma GCC optimize ("-fno-strict-aliasing")

#include "gamesquirrel/sd_card.h"
#include "stm32h523xx.h"
#include "gamesquirrel/core.h"
#include <stdint.h>
#include <stdbool.h>

// #include <stdio.h> // FIXME testing

typedef struct SdState SdState;
struct SdState
{
    bool initialized;
    bool high_speed;
};

static SdState sd_state;

static uint8_t SendReceive(uint8_t x)
{
    // violates strict aliasing rules
    volatile uint8_t *tx_reg = (volatile uint8_t *)&SPI1->TXDR;
    volatile uint8_t *rx_reg = (volatile uint8_t *)&SPI1->RXDR;

    // FIXME how fast is this?  Does it need to be optimized?
    *tx_reg = x;
    while ((SPI1->SR & 0x1001) != 0x1001) // wait for TXC and RXP
    {}
    uint8_t b = *rx_reg;
    return b;
}

static void SetCS(void)
{
    GPIOA->BSRR = 0x80000000; // CS low on PA15
}

static void ClearCS(void)
{
    GPIOA->BSRR = 0x00008000; // CS high on PA15
}

static bool SendCommand(uint8_t cmd, uint32_t data, uint8_t crc)
{
    SetCS();
    SendReceive(cmd | 0x40); // Command
    SendReceive((data >> 24) & 0xFF); // Param MSB
    SendReceive((data >> 16) & 0xFF);
    SendReceive((data >> 8) & 0xFF);
    SendReceive(data & 0xFF); // Param LSB
    SendReceive(crc); // CRC

    return true; // FIXME maybe check that we're getting 0xFF back on sends
}

static bool ReceiveResp(uint8_t *stat_out, uint32_t *data_out)
{
    uint8_t r;
    for (int i=0; i<10; i++)
    {
        r = SendReceive(0xFF);
        if (r != 0xFF)
            break;
    }

    uint32_t data = SendReceive(0xFF);
    data = (data << 8) | SendReceive(0xFF);
    data = (data << 8) | SendReceive(0xFF);
    data = (data << 8) | SendReceive(0xFF);

    ClearCS();
    *stat_out = r;
    *data_out = data;

    return (r != 0xFF);
}

SDCardError SDCardReadBlock(uint32_t addr, uint8_t buffer[512])
{
    // Addr is blocks if CCS is 1, bytes of CCS is 0
    if (!sd_state.initialized)
        return SD_HWFAIL;
    if (!sd_state.high_speed)
        addr = addr*512;

    SendCommand(17, addr, 0);

    // Wait for response, and continue reading
    uint8_t r;
    for (int i=0; i<10; i++)
    {
        r = SendReceive(0xFF);
        if (r != 0xFF)
            break;
    }
    if (r != 0x00)
    {
        ClearCS();
        sd_state.initialized = false;
        return SD_HWFAIL;
    }

    for (int i=0; i<100; i++) // longer timeout here?
    {
        r = SendReceive(0xFF);
        if (r != 0xFF)
            break;
    }
    if (r != 0xFE)
    {
        ClearCS();
        sd_state.initialized = false;
        return SD_HWFAIL;
    }

    for (int i=0; i<512; i++)
    {
        buffer[i] = SendReceive(0xFF);
    }

    // and two bytes of checksum which we ignore
    SendReceive(0xFF);
    SendReceive(0xFF);
    ClearCS();
    return SD_OK;
}

bool SDCardInit(void)
{
    uint8_t r;
    uint32_t data;

    if (sd_state.initialized)
        return true;

    sd_state.high_speed = false;

    // send at least 74 clocks with CS incative to allow the card to initialize
    ClearCS();
    for (int i=0; i<10; i++)
    {
        if (SendReceive(0xFF) != 0xFF)
            return false;
    }

    DelayClocks(250000);

    SendCommand(0, 0x00000000, 0x94);
    ReceiveResp(&r, &data);
    if (r != 0x01)
        return false;

    bool v2_card = false;
    SendCommand(8, 0x000001AA, 0x86);
    ReceiveResp(&r, &data);
    if (r == 0x01)
        v2_card = true;

    // Read OCR, only voltage bits are meaningful at this point
    SendCommand(58, 0x0, 0);
    ReceiveResp(&r, &data);

    // 14 clocks minimum, at 25MHz, 2_000_000 maximum wait time
    for (int timeout=0; timeout<2000000; timeout++)
    {
        SendCommand(55,0,0); // APP_CMD, next command is an ACMD
        ReceiveResp(&r, &data);

        uint32_t arg = 0;
        if (v2_card)
            arg = 0x40000000; // sets HCS bit

        SendCommand(41,arg, 0);
        ReceiveResp(&r, &data);
        if (r == 0x00)
            break;
    }
    if (r != 0)
        return false;


    if (v2_card)
    {
        // Send CMD58 again check CCS for card type
        SendCommand(58, 0x0, 0);
        ReceiveResp(&r, &data);
        if ((data & 0x80000000) == 0) // not powered up
            return false;
        if (data & 0x40000000) //high speed mode
            sd_state.high_speed = true;
    }
    // FIXME maybe force block length to 512 using CMD16

    sd_state. initialized = true;
    return true;
}

bool SDCardIsPresent(void)
{
    return (GPIOB->IDR & 0x0040) == 0;
}

