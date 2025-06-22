
#include "unit_tests.h"
#include "mock_sd_card.h"
#include "gamesquirrel/sd_card.h"
#include "gamesquirrel/disk_cache.h"
#include "gamesquirrel/fatfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

void TestDiskCache(void)
{
    static DiskCache cache;

    PatternFs(1024*1024);
    DiskCacheFlush(&cache);
    for (int i=0; i<DISK_CACHE_BLOCKS; i++)
    {
        CHECK(cache.tags[i].block == 0xFFFFFFFF);
        CHECK(cache.tags[i].data != NULL);
    }

    uint8_t *ptr = NULL;

    CHECK(DiskCacheRead(&cache, 0xFFFFFFFF, &ptr) == SD_OUT_OF_RANGE);

    ForceReadError(SD_HWFAIL);
    CHECK(DiskCacheRead(&cache, 0, &ptr) == SD_HWFAIL);
    CHECK(ptr == NULL);
    ForceReadError(SD_OK);

    ClearReadCount();
    CHECK(DiskCacheRead(&cache, 0, &ptr) == SD_OK);
    CHECK(ptr != NULL);
    CHECK(CheckBlock(0, ptr));
    CHECK(GetReadCount() == 1);

    ClearReadCount();
    CHECK(DiskCacheRead(&cache, 0, &ptr) == SD_OK);
    CHECK(ptr != NULL);
    CHECK(CheckBlock(0, ptr));
    CHECK(GetReadCount() == 0);

    DiskCacheFlush(&cache);
    ClearReadCount();
    CHECK(DiskCacheRead(&cache, 1, &ptr) == SD_OK);
    CHECK(CheckBlock(1, ptr));
    CHECK(DiskCacheRead(&cache, 2, &ptr) == SD_OK);
    CHECK(CheckBlock(2, ptr));
    CHECK(DiskCacheRead(&cache, 3, &ptr) == SD_OK);
    CHECK(CheckBlock(3, ptr));
    CHECK(DiskCacheRead(&cache, 1, &ptr) == SD_OK);
    CHECK(CheckBlock(1, ptr));
    CHECK(DiskCacheRead(&cache, 2, &ptr) == SD_OK);
    CHECK(CheckBlock(2, ptr));
    CHECK(DiskCacheRead(&cache, 3, &ptr) == SD_OK);
    CHECK(CheckBlock(3, ptr));
    CHECK(GetReadCount() == 3);

    DiskCacheFlush(&cache);
    ClearReadCount();
    for (int i=0; i<2048; i++)
    {
        CHECK(DiskCacheRead(&cache, i, &ptr) == SD_OK);
        CHECK(CheckBlock(i, ptr));
    }
    CHECK(GetReadCount() == 2048);

    CHECK(DiskCacheRead(&cache, 2048, &ptr) == SD_OUT_OF_RANGE);

    FreeFs();
}

void TestFatFs(void)
{
    static FatFs fatfs;
    static DiskCache cache;

    uint8_t *block;

    // Not a real Filesystem
    PatternFs(1024*1024);
    DiskCacheFlush(&cache);

    memset(&fatfs, 0, sizeof(fatfs));
    CHECK(DiskCacheRead(&cache, 0, &block) == SD_OK);
    CHECK(FatParseMBR(block, &fatfs) == false);
    CHECK(FatParseBPB(block, &fatfs) == false);

    FreeFs();

    // FAT16 partitioned filesystem
    LoadFs( TESTPATH "fat16.img");
    DiskCacheFlush(&cache);

    memset(&fatfs, 0, sizeof(fatfs));
    CHECK(DiskCacheRead(&cache, 0, &block) == SD_OK);
    CHECK(FatParseMBR(block, &fatfs));
    CHECK(fatfs.partition_start_lba == 0x0800);
    CHECK(fatfs.partition_length_lba == 0x4800);

    CHECK(DiskCacheRead(&cache, fatfs.partition_start_lba, &block) == SD_OK);
    CHECK(FatParseBPB(block, &fatfs) == true);
    CHECK(fatfs.lba_per_cluster == 4);
    CHECK(fatfs.fat_lba_offset == 4);
    CHECK(fatfs.data_lba_offset == 76);
    CHECK(fatfs.root_entries == 512);
    CHECK(fatfs.root_cluster == 0);
    CHECK(fatfs.fat_bits == 16);
    fatfs.cache = &cache;

    CHECK(FatGetEntry(&fatfs, 0) == 0xFFF8);
    CHECK(FatGetEntry(&fatfs, 1) == 0xFFFF);
    for (int i=0; i<40; i++)
        printf("cluster %d entry %X\n", i, FatGetEntry(&fatfs, i));

    FreeFs();

    // FAT32 partitioned filesystem
    LoadFs( TESTPATH "fat32.img");
    DiskCacheFlush(&cache);

    memset(&fatfs, 0, sizeof(fatfs));
    CHECK(DiskCacheRead(&cache, 0, &block) == SD_OK);
    CHECK(FatParseMBR(block, &fatfs));
    CHECK(fatfs.partition_start_lba == 0x0800);
    CHECK(fatfs.partition_length_lba == 0x4800);

    CHECK(DiskCacheRead(&cache, fatfs.partition_start_lba, &block) == SD_OK);
    CHECK(FatParseBPB(block, &fatfs) == true);
    CHECK(fatfs.lba_per_cluster == 1);
    CHECK(fatfs.fat_lba_offset == 32);
    CHECK(fatfs.data_lba_offset == 316);
    CHECK(fatfs.root_entries == 0);
    CHECK(fatfs.root_cluster == 2);
    CHECK(fatfs.fat_bits == 32);
    fatfs.cache = &cache;

    CHECK(FatGetEntry(&fatfs, 0) == 0x0FFFFFF8);
    CHECK(FatGetEntry(&fatfs, 1) == 0x0FFFFFFF);
    for (int i=0; i<40; i++)
        printf("cluster %d entry %X\n", i, FatGetEntry(&fatfs, i));

    FreeFs();

    // FAT12 no partitions
    LoadFs( TESTPATH "floppy.img");
    DiskCacheFlush(&cache);

    memset(&fatfs, 0, sizeof(fatfs));
    fatfs.partition_start_lba = 0;
    fatfs.partition_length_lba = 2880;

    CHECK(DiskCacheRead(&cache, fatfs.partition_start_lba, &block) == SD_OK);
    CHECK(FatParseBPB(block, &fatfs) == true);
    CHECK(fatfs.lba_per_cluster == 1);
    CHECK(fatfs.fat_lba_offset == 1);
    CHECK(fatfs.data_lba_offset == 33);
    CHECK(fatfs.root_entries == 224);
    CHECK(fatfs.root_cluster == 0);
    CHECK(fatfs.fat_bits == 12);
    fatfs.cache = &cache;

    CHECK(FatGetEntry(&fatfs, 0) == 0x0FF0);
    CHECK(FatGetEntry(&fatfs, 1) == 0x0FFF);
    for (int i=0; i<40; i++)
        printf("cluster %d entry %X\n", i, FatGetEntry(&fatfs, i));

    FreeFs();
}

