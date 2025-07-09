/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_ELF_H
#define INCLUDED_ELF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct ElfHeader ElfHeader;
struct ElfHeader
{
    uint32_t e_ident1;
    uint32_t e_ident2;
    uint32_t e_ident3;
    uint32_t e_ident4;
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry; // entry point address
    uint32_t e_phoff; // program header offset
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum; // program header count
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
};

typedef struct ProgramHeader ProgramHeader;
struct ProgramHeader
{
    uint32_t p_type;
    uint32_t p_offset;
    uint32_t p_vaddr;
    uint32_t p_paddr;
    uint32_t p_filesz;
    uint32_t p_memsz;
    uint32_t p_flags;
    uint32_t p_align;
};

typedef struct ElfFile ElfFile;
struct ElfFile
{
    bool (*load)(void *ctx, void *buffer, int32_t size, int32_t offset);
    void *load_ctx;
    uint32_t high_address;

    ElfHeader eheader;
    ProgramHeader pheader;
};

bool ElfFileHeader(ElfFile *elf);
bool ElfProgramHeader(ElfFile *elf, int n);
bool ElfLoadSection(ElfFile *elf, void *buffer, uint32_t addr, int32_t size);

bool ElfLoad(ElfFile *elf);

#endif

