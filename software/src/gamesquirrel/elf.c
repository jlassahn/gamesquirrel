/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include "gamesquirrel/elf.h"
#include <string.h>

bool ElfLoad(ElfFile *elf)
{
    // FIXME implement
    // ElfFileHeader(elf);
    // for each program header
    // ElfProgramHeader(elf, i);
    // if p_type == 1
    //  ElfLoadSection(elf, (void *)0x90000000, 0x90000000, 0x003FC000);
    (void)elf;
    return false;
}

bool ElfFileHeader(ElfFile *elf)
{
    if (!elf->load(elf->load_ctx, &elf->eheader, sizeof(ElfHeader), 0))
        return false;
    // FIXME check values
    return true;
}

bool ElfProgramHeader(ElfFile *elf, int n)
{
    if (n < 0)
        return false;
    if (n >= elf->eheader.e_phnum)
        return false;

    int32_t offset = elf->eheader.e_phoff + n*sizeof(ProgramHeader);
    if (!elf->load(elf->load_ctx, &elf->pheader, sizeof(ProgramHeader), offset))
        return false;
    return true;
}

bool ElfLoadSection(ElfFile *elf, void *buffer, uint32_t addr, int32_t size)
{
    if (elf->pheader.p_type != 1)
        return false;
    if (elf->pheader.p_paddr < addr)
        return false;
    if (elf->pheader.p_paddr + elf->pheader.p_memsz > addr + size)
        return false;
    if (elf->pheader.p_memsz < elf->pheader.p_filesz)
        return false;

    uint8_t *ptr = buffer;
    ptr += elf->pheader.p_paddr - addr;
    uint32_t bytes = elf->pheader.p_filesz;
    uint32_t in_off = elf->pheader.p_offset;
    if (!elf->load(elf->load_ctx, ptr, bytes, in_off))
        return false;

    if (elf->pheader.p_memsz > elf->pheader.p_filesz)
    {
        int32_t zsize = elf->pheader.p_memsz - elf->pheader.p_filesz;
        memset(ptr + bytes, 0, zsize);
    }

    uint32_t high_addr = elf->pheader.p_paddr + elf->pheader.p_memsz;
    if (elf->high_address < high_addr)
        elf->high_address = high_addr;

    return true;
}

