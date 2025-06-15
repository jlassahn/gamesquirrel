/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_SD_CARD_H
#define INCLUDED_SD_CARD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum SDCardError SDCardError;
enum SDCardError
{
    SD_OK,
    SD_HWFAIL,
    SD_OUT_OF_RANGE
};

bool SDCardIsPresent(void);
bool SDCardInit(void);

SDCardError SDCardReadBlock(uint32_t block, uint8_t data_out[512]);
SDCardError SDCardWriteBlock(uint32_t block, uint8_t data_in[512]);

#endif

