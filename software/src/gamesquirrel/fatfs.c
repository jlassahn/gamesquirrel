
/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include "gamesquirrel/fatfs.h"
#include "gamesquirrel/sd_card.h"
#include <string.h>

#include <stdio.h> // FIXME fake

typedef struct DirData DirData;
struct DirData
{
    uint8_t name[8];
    uint8_t ext[3];
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

bool FatOpenRoot(FatFs *fs, FatDir *dir_out)
{
    dir_out->fs = fs;
    dir_out->start_cluster = fs->root_cluster;
    dir_out->current_cluster = fs->root_cluster;
    dir_out->current_offset = 0;
    return true;
}

static SDCardError FatGetDirData(FatDir *dir, DirData **dd_out)
{
    if (dir->start_cluster == 0) // this is an old-style root directory
    {
        if (dir->current_offset >= dir->fs->root_entries*32)
            return SD_OUT_OF_RANGE;

        int32_t root_size = dir->fs->root_entries*32/512;
        int32_t root_lba = dir->fs->partition_start_lba
            + dir->fs->data_lba_offset
            - root_size;
        int32_t lba = root_lba + dir->current_offset/512;
        int32_t offset = dir->current_offset & 511;

        uint8_t *block;
        SDCardError err =  DiskCacheRead(dir->fs->cache, lba, &block);
        if (err != SD_OK)
            return err;

        *dd_out = (DirData *)(block + offset);
        return SD_OK;
    }
    else
    {
        if (dir->current_offset < 0)
            return SD_OUT_OF_RANGE;

        int32_t cluster = dir->current_cluster;
        int32_t offset = dir->current_offset;

        int32_t lba = dir->fs->partition_start_lba
            +  dir->fs->data_lba_offset
            + (cluster - 2)*dir->fs->lba_per_cluster;
        lba += offset/512;
        offset = offset & 511;

        uint8_t *block;
        SDCardError err =  DiskCacheRead(dir->fs->cache, lba, &block);
        if (err != SD_OK)
            return err;

        *dd_out = (DirData *)(block + offset);
        return SD_OK;
    }
}

static void FatNextDirData(FatDir *dir)
{
    if (dir->start_cluster == 0) // this is an old-style root directory
    {
        dir->current_offset += 32;
    }
    else
    {
        dir->current_offset += 32;

        int32_t cluster_size = dir->fs->lba_per_cluster * 512;
        if (dir->current_offset >= cluster_size)
        {
            int32_t cluster = dir->current_cluster;
            uint32_t fat_entry = FatGetEntry(dir->fs, cluster);

            // FIXME more efficient lookup here?
            uint32_t max_entry;
            if (dir->fs->fat_bits == 12)
                max_entry = 0xFF5;
            else if (dir->fs->fat_bits == 16)
                max_entry = 0xFFF5;
            else
                max_entry = 0x0FFFFFF5;

            if (fat_entry > max_entry)
            {
                // End of file
                dir->current_offset = -1;
                return;
            }
            else
            {
                dir->current_offset = 0;
                dir->current_cluster = fat_entry;
            }
        }
    }
}

SDCardError FatGetNextEntry(FatDir *dir, FatDirEntry *entry_out)
{
    while (true)
    {
        DirData *dd;
        SDCardError err = FatGetDirData(dir, &dd);
        if (err != SD_OK)
            return err;

        if (dd->name[0] ==  0x00) // 0x00 means end of entry list
            return SD_OUT_OF_RANGE;

        FatNextDirData(dir);

        if (dd->name[0] ==  0xE5) // 0xE5 means empty entry but more to follow
            continue;
        // ATTR_VOLUME_ID == 0x08 -- not a real file, maybe LFS
        if (dd->attr & 0x08)
            continue;
        memcpy(entry_out->name, dd->name, 8);
        memcpy(entry_out->ext, dd->ext, 3);
        entry_out->size = dd->size;
        entry_out->cluster = (uint32_t)dd->cluster_lo
            | ((uint32_t)dd->cluster_hi << 16);
        entry_out->attributes = dd->attr & 0x3F;

        return SD_OK;
    }
}

bool FatOpenDir(FatFs *fs, FatDirEntry *entry, FatDir *dir_out)
{
    if ((entry->attributes & 0x18) != 0x10)
        return false;

    dir_out->fs = fs;
    dir_out->start_cluster = entry->cluster;
    dir_out->current_cluster = entry->cluster;
    dir_out->current_offset = 0;

    return true;
}

bool FatOpenFile(FatFs *fs, FatDirEntry *entry, FatFile *file_out)
{
    if ((entry->attributes & 0x18) != 0x00)
        return false;

    file_out->fs = fs;
    file_out->start_cluster = entry->cluster;
    file_out->size = entry->size;
    file_out->position = 0;
    file_out->current_cluster = entry->cluster;
    file_out->current_offset = 0;

    return true;
}

SDCardError FatFileRead(FatFile *file, void *buffer, int count)
{
    if (file->size < file->position + count)
        return SD_OUT_OF_RANGE;

    while (count > 0)
    {
        int len = count;
        int32_t cluster_size = file->fs->lba_per_cluster*512;
        int32_t cluster = file->current_cluster;
        int32_t offset = file->current_offset;

        int32_t lba = file->fs->partition_start_lba
                +  file->fs->data_lba_offset
                + (cluster - 2)*file->fs->lba_per_cluster;
        lba += offset/512;
        offset = offset & 511;

        if (offset + len > 512)
            len = 512 - offset;

        uint8_t *block;
        SDCardError err =  DiskCacheRead(file->fs->cache, lba, &block);
        if (err != SD_OK)
            return err;
        memcpy(buffer, block + offset, len);
        buffer += len;
        count -= len;
        file->current_offset += len;
        file->position += len;

        if (file->current_offset >= cluster_size)
        {
            uint32_t fat_entry = FatGetEntry(file->fs, cluster);

            // FIXME more efficient lookup here?
            uint32_t max_entry;
            if (file->fs->fat_bits == 12)
                max_entry = 0xFF5;
            else if (file->fs->fat_bits == 16)
                max_entry = 0xFFF5;
            else
                max_entry = 0x0FFFFFF5;

            if (fat_entry > max_entry)
            {
                // Unexpected End of file -- corrupt filesystem
                file->current_offset = 0;
                file->current_cluster = file->start_cluster;
                return SD_HWFAIL; // FIXME better error message
            }
            else
            {
                file->current_offset = 0;
                file->current_cluster = fat_entry;
            }
        }
    }
    return SD_OK;
}

SDCardError FatInit(FatFs *fs, DiskCache *cache)
{
    SDCardError err;
    uint8_t *block;

    DiskCacheFlush(cache);
    err = DiskCacheRead(cache, 0, &block);
    if (err != SD_OK)
        return err;
    if (!FatParseMBR(block, fs))
        return SD_HWFAIL; //FIXME better error

    err = DiskCacheRead(cache, fs->partition_start_lba, &block);
    if (err != SD_OK)
        return err;
    if (!FatParseBPB(block, fs))
        return SD_HWFAIL; //FIXME better error

    fs->cache = cache;
    return SD_OK;
}
