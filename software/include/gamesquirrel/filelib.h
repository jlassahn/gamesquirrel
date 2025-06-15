/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn

  System calls into the GameSquirrel system software for use by applications.
*/

#include "gamesquirrel/systemlib.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct FileLib1 FileLib1;
struct FileLib1
{
    LibraryInfo info;

    int32_t sizeof_filesystem;
    int32_t sizeof_file;

    int OpenFilesystem(Filesystem *fs);
    void CloseFilesystem(Filesystem *fs);
    int OpenFile(Filesystem *fs, const char *name, File *file);
    void Close(File *file);
    int Read(File *file, uint8_t *data_out, int32_t length);
};

