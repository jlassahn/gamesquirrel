
#ifndef INCLUDED_MOCK_SD_CARD_H
#define INCLUDED_MOCK_SD_CARD_H

#include <stdint.h>
#include <stdbool.h>
#include "gamesquirrel/sd_card.h"

// These tests rely on external files in the tests directory
// and will only work when run from the root of the software tree.

#define TESTPATH "tests/"

void LoadFs(const char *filename);
void PatternFs(int32_t bytes);
void FreeFs(void);
bool CheckBlock(uint32_t block, uint8_t data[512]);
void ForceReadError(SDCardError err);
void ClearReadCount(void);
int GetReadCount(void);

#endif

