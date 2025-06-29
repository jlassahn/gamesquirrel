/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include <stdint.h>
#include <stdbool.h>

void CoreInit(void);

void DelayClocks(uint32_t n);
uint32_t TimeMicroseconds(void);

bool ButtonRead(int n); // n in 0 ... 3
void LEDWrite(int n, int val);  // n in 0, 1, val in 0 ... 1000

