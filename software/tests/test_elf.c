/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include "unit_tests.h"
#include "gamesquirrel/elf.h"
#include <stdio.h>
#include <string.h>

static uint8_t *file_data = NULL;
static int32_t file_bytes = 0;

static void FileLoad(const char *filename)
{
    if (file_data != NULL)
    {
        free(file_data);
        file_data = NULL;
        file_bytes = 0;
    }

    FILE *fp = fopen(filename, "rb");
    if (!fp)
    {
        FAIL("Can't open test file");
        return;
    }

    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    file_data = malloc(size);
    file_bytes = size;

    fread(file_data, size, 1, fp);
    fclose(fp);
}

static void FileFree(void)
{
    if (file_data != NULL)
    {
        free(file_data);
        file_data = NULL;
        file_bytes = 0;
    }
}

static bool FileRead(void *ctx, void *buffer, int32_t size, int32_t offset)
{
    CHECK(ctx == (void *)0x12345678);
    CHECK(offset >= 0);
    CHECK(size > 0);
    CHECK(size + offset <= file_bytes);

    memcpy(buffer, file_data + offset, size);
    return true;
}

static bool IsZero(uint8_t *buffer, int32_t size)
{
    for (int32_t i=0; i<size; i++)
    {
        if (buffer[i] != 0)
            return false;
    }
    return true;
}

void TestElf(void)
{
    FileLoad("tests/simple.elf"); // FIXME parameterize root directory

    ElfFile elf;
    memset(&elf, 0, sizeof(elf));
    elf.load = FileRead;
    elf.load_ctx = (void *)0x12345678;

    CHECK(ElfFileHeader(&elf));
    CHECK(elf.eheader.e_ident1 == 0x464C457F);
    CHECK(elf.eheader.e_ident2 == 0x00010101);
    CHECK(elf.eheader.e_type == 0x0002);
    CHECK(elf.eheader.e_machine == 0x0028);

    CHECK(elf.eheader.e_entry == 0x0400832B);
    CHECK(elf.eheader.e_phnum == 2);

    /* Program headers for simple.elf:
  Type           Offset   VirtAddr   PhysAddr   FileSiz MemSiz  Flg Align
  LOAD           0x001000 0x04000000 0x90000000 0x08d58 0x08d58 R E 0x1000
  LOAD           0x009d58 0x90008d58 0x90008d58 0x006b0 0x009cc RW  0x1000
    */

    CHECK(ElfProgramHeader(&elf, 0));
    CHECK(elf.pheader.p_type == 0x01);
    CHECK(elf.pheader.p_offset == 0x001000);
    CHECK(elf.pheader.p_vaddr == 0x04000000);
    CHECK(elf.pheader.p_paddr == 0x90000000);
    CHECK(elf.pheader.p_filesz == 0x08D58);
    CHECK(elf.pheader.p_memsz == 0x08D58);

    CHECK(ElfProgramHeader(&elf, 1));
    CHECK(elf.pheader.p_type == 0x01);
    CHECK(elf.pheader.p_offset == 0x009D58);
    CHECK(elf.pheader.p_vaddr == 0x90008D58);
    CHECK(elf.pheader.p_paddr == 0x90008D58);
    CHECK(elf.pheader.p_filesz == 0x006B0);
    CHECK(elf.pheader.p_memsz == 0x009CC);

    uint8_t *buffer = malloc(0x003FC000);
    memset(buffer, 0x55, 0x003FC000);

    CHECK(ElfProgramHeader(&elf, 0));
    CHECK(ElfLoadSection(&elf, buffer, 0x90000000, 0x003FC000));
    CHECK(ElfProgramHeader(&elf, 1));
    CHECK(ElfLoadSection(&elf, buffer, 0x90000000, 0x003FC000));
    CHECK(memcmp(buffer, file_data + 0x01000, 0x08D58 + 0x06B0) == 0);
    CHECK(IsZero(buffer+0x08D58 + 0x06B0, 0x09CC - 0x06B0));

    CHECK(elf.high_address == 0x90000000 + 0x08D58 + 0x09CC);

    free(buffer);

    FileFree();
}
