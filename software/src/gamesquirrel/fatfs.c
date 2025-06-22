
/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include "gamesquirrel/fatfs.h"
#include "gamesquirrel/sd_card.h"

#include <stdio.h> // FIXME fake

bool FatParseMBR(const uint8_t block[512], FatFs *fs)
{
    if (block[510] != 0x55)
        return false;
    if (block[511] != 0xAA)
        return false;

    // first partition table entry
    int pe = 0x01BE;

    uint32_t lba =
        (uint32_t)block[pe+8]
        + ((uint32_t)block[pe+9] << 8)
        + ((uint32_t)block[pe+10] << 16)
        + ((uint32_t)block[pe+11] << 24);
    fs->partition_start_lba = lba;

    lba =
        (uint32_t)block[pe+12]
        + ((uint32_t)block[pe+13] << 8)
        + ((uint32_t)block[pe+14] << 16)
        + ((uint32_t)block[pe+15] << 24);
    fs->partition_length_lba = lba;

    return true;
}

bool FatParseBPB(const uint8_t block[512], FatFs *fs)
{
    if (block[510] != 0x55)
        return false;
    if (block[511] != 0xAA)
        return false;

    // bytes per sector
    uint32_t val =
        (uint32_t)block[11] +
        ((uint32_t)block[12] << 8);
    if (val != 512)
        return false;

    fs->lba_per_cluster = block[13];

    val =
        (uint32_t)block[14] +
        ((uint32_t)block[15] << 8);
    fs->fat_lba_offset = val;

    val =
        (uint32_t)block[17] +
        ((uint32_t)block[18] << 8);
    fs->root_entries = val;

    int fat_table_count = block[16];
    int bits = 16;
    uint32_t root_cluster = 0;
    uint32_t fat_size =
        (uint32_t)block[22] +
        ((uint32_t)block[23] << 8);
    if (fat_size == 0)
    {
        fat_size =
            (uint32_t)block[36] +
            ((uint32_t)block[37] << 8) +
            ((uint32_t)block[38] << 16) +
            ((uint32_t)block[39] << 24);
        root_cluster =
            (uint32_t)block[44] +
            ((uint32_t)block[45] << 8) +
            ((uint32_t)block[46] << 16) +
            ((uint32_t)block[47] << 24);
        bits = 32;
    }

    // heuristic for detecting FAT12.
    if (bits == 16)
    {
        uint32_t clusters = fs->partition_length_lba/fs->lba_per_cluster;
        if (fat_size*(512/2) < clusters)
            bits = 12;
    }

    int32_t root_lba = (fs->root_entries*32 + 511)/512;
    fs->data_lba_offset = fs->fat_lba_offset + fat_size*fat_table_count + root_lba;
    fs->root_cluster = root_cluster;
    fs->fat_bits = bits;

    return true;
}

uint32_t FatGetEntry(FatFs *fs, int32_t cluster)
{
    int32_t fat_base = fs->partition_start_lba + fs->fat_lba_offset;
    if (fs->fat_bits == 12)
    {
        int32_t idx = cluster + (cluster>>1);
        int32_t lba = fat_base + idx/512;

        uint8_t *block;
        if (DiskCacheRead(fs->cache, lba, &block) != SD_OK)
            return 0xFF7;
        uint32_t entry = block[idx & 511];

        idx ++;
        lba = fat_base + idx/512;
        if (DiskCacheRead(fs->cache, lba, &block) != SD_OK)
            return 0xFF7;
        entry |= ((uint32_t)block[idx & 511]) << 8;

        if (cluster & 1)
            entry = entry >> 4;
        entry = entry & 0x0FFF;
        return entry;
    }

    if (fs->fat_bits == 16)
    {
        int32_t idx = cluster*2;
        int32_t lba = fat_base + idx/512;
        uint8_t *block;
        if (DiskCacheRead(fs->cache, lba, &block) != SD_OK)
            return 0xFFF7;
        uint32_t entry = block[idx & 511];
        entry |= ((uint32_t)block[(idx+1) & 511]) << 8;
        return entry;
    }

    if (fs->fat_bits == 32)
    {
        int32_t idx = cluster*4;
        int32_t lba = fat_base + idx/512;
        uint8_t *block;
        if (DiskCacheRead(fs->cache, lba, &block) != SD_OK)
            return 0x0FFFFFF7;
        uint32_t entry = block[idx & 511];
        entry |= ((uint32_t)block[(idx+1) & 511]) << 8;
        entry |= ((uint32_t)block[(idx+2) & 511]) << 16;
        entry |= ((uint32_t)block[(idx+3) & 511]) << 24;
        entry &= 0x0FFFFFFF;
        return entry;
    }
    return 0xFFFFFFF7;
}

