
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

static void TestFileContents(FatFile *file, int breakpoint)
{
    int remaining = file->size;
    int count = 0;
    static char buffer[26];

    while (remaining > 0)
    {
        int len = 26;
        if (remaining < len)
            len = remaining;

        // FIXME there's an error in ROOT1.TXT, this patches around it
        if ((count < breakpoint) && (count + len > breakpoint))
            len = breakpoint - count;

        memset(buffer, 0, sizeof(buffer));
        CHECK(FatFileRead(file, buffer, len) == SD_OK);
        CHECK(memcmp(buffer, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", len) == 0);
        if (memcmp(buffer, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", len) != 0)
        {
            printf("compare failed at position %d\n", file->position);
        }

        remaining -= len;
        count += len;
    }
}

static void TestFiles(FatFs *fatfs)
{
    /* expected directory structure:
        ROOT1.TXT
        ROOT2.TXT
        DIR1/
            DIR2/
                SUBFILE1.X
    */

    FatDir dir;
    FatDirEntry dirent;
    FatFile file;

    CHECK(FatOpenRoot(fatfs, &dir) == true);
    memset(&dirent, 0, sizeof(dirent));
    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "ROOT1   ", 8) == 0);
    CHECK(memcmp(dirent.ext, "TXT", 3) == 0);

    memset(&file, 0, sizeof(file));
    CHECK(FatOpenFile(fatfs, &dirent, &file));
    CHECK(file.size == 0x2328);
    TestFileContents(&file, 0xFA0);

    memset(&dirent, 0, sizeof(dirent));
    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "ROOT2   ", 8) == 0);
    CHECK(memcmp(dirent.ext, "TXT", 3) == 0);
    CHECK(FatOpenFile(fatfs, &dirent, &file));
    CHECK(file.size == 0x04D2);
    TestFileContents(&file, -1);

    memset(&dirent, 0, sizeof(dirent));
    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "DIR1    ", 8) == 0);
    CHECK(memcmp(dirent.ext, "   ", 3) == 0);
    // FIXME test whether entry is a directory

    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OUT_OF_RANGE);

    // Recurse into DIR1
    CHECK(FatOpenDir(fatfs, &dirent, &dir) == true);
    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, ".       ", 8) == 0);
    CHECK(memcmp(dirent.ext, "   ", 3) == 0);

    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "..      ", 8) == 0);
    CHECK(memcmp(dirent.ext, "   ", 3) == 0);

    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "DIR2    ", 8) == 0);
    CHECK(memcmp(dirent.ext, "   ", 3) == 0);
    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OUT_OF_RANGE);

    //Recurse into DIR2
    CHECK(FatOpenDir(fatfs, &dirent, &dir) == true);
    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, ".       ", 8) == 0);
    CHECK(memcmp(dirent.ext, "   ", 3) == 0);

    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "..      ", 8) == 0);
    CHECK(memcmp(dirent.ext, "   ", 3) == 0);

    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OK);
    CHECK(memcmp(dirent.name, "SUBFILE1", 8) == 0);
    CHECK(memcmp(dirent.ext, "X  ", 3) == 0);

    memset(&file, 0, sizeof(file));
    CHECK(FatOpenFile(fatfs, &dirent, &file));
    CHECK(file.size == 0x7B);
    TestFileContents(&file, -1);

    CHECK(FatGetNextEntry(&dir, &dirent) == SD_OUT_OF_RANGE);

    //Attempt to recurse into SUBFILE1.X which is not a directory
    CHECK(FatOpenDir(fatfs, &dirent, &dir) == false);

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

    TestFiles(&fatfs);
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

    TestFiles(&fatfs);
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

    TestFiles(&fatfs);
    FreeFs();

    // FIXME need to test with larger directories, to test cluster rollover
}

