/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include "gamesquirrel/disk_cache.h"
#include <string.h>

void DiskCacheFlush(DiskCache *cache)
{
    for (int i=0; i<DISK_CACHE_BLOCKS; i++)
    {
        cache->tags[i].block = 0xFFFFFFFF;
        cache->tags[i].data = cache->data + 512*i;
    }
}

SDCardError DiskCacheRead(DiskCache *cache, uint32_t block, uint8_t **ptr_out)
{
    if (block == 0xFFFFFFFF)
        return SD_OUT_OF_RANGE;

    int slot = -1;
    DiskCacheTag *tag = NULL;
    for (int i=0; i<DISK_CACHE_BLOCKS; i++)
    {
        if (cache->tags[i].block == block)
        {
            slot = i;
            tag = &cache->tags[i];
            break;
        }
    }

    if (tag == NULL)
    {
        slot = DISK_CACHE_BLOCKS-1;
        tag = &cache->tags[slot];
        SDCardError err = SDCardReadBlock(block, tag->data);
        if (err != SD_OK)
            return err;
        tag->block = block;
    }

    // reorder slots in LRU order
    if (slot > 0)
    {
        DiskCacheTag tmp_tag = *tag;
        for (int i=slot; i>0; i--)
            cache->tags[i] = cache->tags[i-1];
        cache->tags[0] = tmp_tag;
        tag = &cache->tags[0];
    }

    *ptr_out = tag->data;
    return SD_OK;
}


