/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_FATFS_H
#define INCLUDED_FATFS_H

#include "gamesquirrel/sd_card.h"
#include "gamesquirrel/disk_cache.h"
#include <stdint.h>
#include <stdbool.h>

/*
    We support SDSC and SDHC cards, with
    An MBR partition table
    Using LBA addressing for partitions
    Only access data on the first partition
    Only 512 byte sectors
    FAT12, FAT16, or FAT32 filesystems
    No long filename support
*/

/* Error codes:
    // Device is unplugged or not responding -- EIO
    // Device has bad or incompatible filesystem -- ENOTBLK
    // Attempt to read past the end of the device -- EDOM
    // Attempt to write when not supported -- EROFS
    // Can't find the file we want, e.g. MAIN.ELF -- ENOENT
*/

typedef struct FatFs FatFs;
struct FatFs
{
    uint32_t partition_start_lba;
    uint32_t partition_length_lba;
    int32_t lba_per_cluster;
    int32_t fat_lba_offset;
    int32_t data_lba_offset;
    int32_t root_entries;
    int32_t root_cluster;
    int32_t fat_bits;

    DiskCache *cache;
};

// Physical layout
struct DirectoryEntry
{
    char name[8];
    char ext[3];
    uint8_t attr;
    uint8_t reserved;
    uint8_t hundredths;
    uint16_t create_time;
    uint16_t create_date;
    uint16_t access_date;
    uint16_t cluster_hi;
    uint16_t write_time;
    uint16_t write_date;
    uint16_t cluster_lo;
    uint32_t size;
};

bool FatParseMBR(const uint8_t block[512], FatFs *fs);
bool FatParseBPB(const uint8_t block[512], FatFs *fs);
uint32_t FatGetEntry(FatFs *fs, int32_t cluster);

#endif

