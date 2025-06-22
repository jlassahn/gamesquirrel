/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_DISK_CACHE_H
#define INCLUDED_DISK_CACHE_H

#include "gamesquirrel/sd_card.h"

#define DISK_CACHE_BLOCKS 3

typedef struct DiskCacheTag DiskCacheTag;
struct DiskCacheTag
{
    uint32_t block;
    uint8_t *data;
};

typedef struct DiskCache DiskCache;
struct DiskCache
{
    DiskCacheTag tags[DISK_CACHE_BLOCKS];
    uint8_t data[512*DISK_CACHE_BLOCKS];
};

// FIXME make flush and init different operations,
// so we can support writes later.

void DiskCacheFlush(DiskCache *cache);
SDCardError DiskCacheRead(DiskCache *cache, uint32_t block, uint8_t **ptr_out);


#endif

