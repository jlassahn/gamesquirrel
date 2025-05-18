
#include <stdint.h>
#include <stdbool.h>

void CoreInit(void);

uint32_t TimeMicroseconds(void);

bool ButtonRead(int n); // n in 0 ... 3
void LEDWrite(int n, int val);  // n in 0, 1, val in 0 ... 1000

