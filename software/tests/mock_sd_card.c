
#include "unit_tests.h"
#include "mock_sd_card.h"
#include "gamesquirrel/sd_card.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static int32_t fs_bytes = 0;
static uint8_t *fs_data = NULL;
static SDCardError force_error = SD_OK;
static int read_count = 0;

void LoadFs(const char *filename)
{
    if (fs_data != NULL)
    {
        free(fs_data);
        fs_data = NULL;
        fs_bytes = 0;
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
    fs_data = malloc(size);
    fs_bytes = size;

    fread(fs_data, size, 1, fp);
    fclose(fp);
}

void PatternFs(int32_t bytes)
{
    fs_data = malloc(bytes);
    fs_bytes = bytes;
    uint32_t *ptr = (uint32_t *)fs_data;
    int32_t words = bytes/4;

    for (int32_t i=0; i<words; i++)
        ptr[i] = i;
}

void FreeFs(void)
{
    if (fs_data != NULL)
    {
        free(fs_data);
        fs_data = NULL;
        fs_bytes = 0;
    }
}

bool CheckBlock(uint32_t block, uint8_t data[512])
{
    if ((block + 1)*512 > (uint32_t)fs_bytes)
        return false;
    return (memcmp(data, fs_data + block*512, 512) == 0);
}

void ClearReadCount(void)
{
    read_count = 0;
}

int GetReadCount(void)
{
    return read_count;
}

void ForceReadError(SDCardError err)
{
    force_error = err;
}

// Mock of system function
SDCardError SDCardReadBlock(uint32_t block, uint8_t data_out[512])
{
    read_count ++;

    if (force_error != SD_OK)
        return force_error;

    if ((block + 1)*512 > (uint32_t)fs_bytes)
        return SD_OUT_OF_RANGE;

    memcpy(data_out, fs_data + block*512, 512);
    return SD_OK;
}

