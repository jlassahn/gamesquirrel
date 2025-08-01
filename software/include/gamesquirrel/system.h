/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn

  System calls into the GameSquirrel system software for use by applications.
*/

#include <stdint.h>
#include <stdbool.h>

typedef enum LibraryID
{
    LIB_INVALID = 0,
    LIB_SYSTEM,
    LIB_FILE,
}
LibraryID;

typedef struct LibraryInfo LibraryInfo;
struct LibraryInfo
{
    uint8_t id;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t patch_version;
};

typedef struct SystemLib SystemLib;
struct SystemLib
{
    LibraryInfo info;
    void *(*FindLibrary)(uint8_t id, uint8_t major_version);

    void *(*FastAlloc)(int bytes);
    void (*FastFree)(void *ptr);

    // Errors for SD Card I/O
    // EIO 5  I/O Error (disk removed or failing)
    // ENODEV 19 No Device
    // EROFS 30 Read only filesystem
    // EDOM 33 Input argument out of domain
    // ENOSYS 38 not implemented

    bool (*SDCardIsInserted)(void);
    int (*SDCardInitialize)(void);
    int (*SDCardReadBlock)(int32_t block, uint8_t data_out[512]);
    int (*SDCardWriteBlock)(int32_t block, uint8_t data_in[512]);

};

extern const SystemLib *system_lib;

