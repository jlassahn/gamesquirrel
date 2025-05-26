
// #include "stm32h523xx.h"
#include "gamesquirrel/core.h"
#include "gamesquirrel/usb.h"
#include "gamesquirrel/audio.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

static const uint32_t blink_interval_us = 250000;

void hw_init(void);

volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

// FIXME why does this get optimized out if not marked used?
int _write(int fhdl, const char *buf, size_t count) __attribute__ ((used));
int _read(int fhdl, char *buf, size_t count) __attribute__ ((used));

// Default logging with on-board UART
int _write (int fhdl, const char *buf, size_t count) {
	(void) fhdl;
	return UsbSend(buf, count);
}

int _read (int fhdl, char *buf, size_t count) {
	(void) fhdl;
	int rd = UsbReceive(buf, count);
	return (rd > 0) ? rd : -1;
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}

void hw_init(void)
{
	CoreInit();
	UsbInit();
	AudioInit();
}

void MemTest(void)
{
	volatile uint32_t *buffer = (void *)0x90000000;
	buffer[0] = 0x12345678;
	buffer[1] = 0xABCDEF0;

	printf("octospi %.8lX %.8lX\r\n", buffer[0], buffer[1]);
}

int main(void)
{
	hw_init();
	UsbStart();

	printf("Hello, world\r\n");

	uint32_t last_us = TimeMicroseconds();
	while (true)
	{
		static char buff[16];
		int length = UsbReceive(buff, 16);
		if (length > 0)
		{
			for (int i=0; i<length; i++)
				buff[i] = buff[i] ^ 0x20;
			UsbSend(buff, length);
		}

		uint32_t us = TimeMicroseconds();
		if (us - last_us < blink_interval_us)
			continue;
		last_us += blink_interval_us;

		LEDWrite(0, (us / 1000) % 1000);
		LEDWrite(1, (us / 5000) % 1000);

		AudioStart(NULL, 0); // FIXME needs real data
		MemTest();

		printf("Tick %lu %d %d %d %d\r\n",
				us,
				ButtonRead(0),
				ButtonRead(1),
				ButtonRead(2),
				ButtonRead(3));
	}
}

