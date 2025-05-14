
#include "stm32h523xx.h"
#include "gamesquirrel/core.h"
#include "gamesquirrel/usb.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

static const uint32_t blink_interval_ms = 250;

// LED on GPIOA Pin5, Pin9, and Pin10
#define LED_PORT              GPIOA
#define LED_PIN               0x0620

// Button on GPIOC Pin13
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            0x2000

void board_led_write(bool state);
uint32_t board_button_read(void);
int board_uart_read(uint8_t *buf, int len);
int board_uart_write(void const *buf, int len);
uint32_t board_millis(void);

void hw_init(void);
void board_init(void);

void board_led_write(bool state) {
	if (state)
		LED_PORT->BSRR = LED_PIN;
	else
		LED_PORT->BRR = LED_PIN;
}

uint32_t board_button_read(void) {
	return (BUTTON_PORT->IDR & BUTTON_PIN) != 0;
}

volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

void HardFault_Handler(void) {
  __asm("BKPT #0\n");
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
	// FIXME rename to CoreInit
	CoreInit();
	UsbInit();
}

int main(void)
{
	hw_init();
	UsbStart();

	/*
	while (!board_button_read())
	{
	}
	*/

	printf("Hello, world\r\n");

	bool led_state = false;
	uint32_t last_ms = board_millis();
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

		uint32_t ms = board_millis();
		if (ms - last_ms < blink_interval_ms)
			continue;
		last_ms = ms;

		board_led_write(led_state);
		led_state = 1 - led_state; // toggle

		printf("Tick %lu  %ld\r\n", ms, TIM2->CNT);
	}
}

