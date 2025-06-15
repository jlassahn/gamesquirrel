
#pragma GCC optimize ("-fno-strict-aliasing")

#include "gamesquirrel/display.h"
#include "stm32h523xx.h"

// FIXME design and implement display API
// FIXME add VSync interrupt

void DisplayInit(void)
{
	// FIXME implement
	// GPIOA-> // FIXME clear CS PB12
	// GPIOA-> // FIXME clear Command PB14
	GPIOB->BSRR = 0x50000000; // CS low on PB12 CMD low on PB14

	// violates strict aliasing rules
	volatile uint8_t *tx_reg = (volatile uint8_t *)&SPI2->TXDR;
	*tx_reg = 0xC5;
	while ((SPI2->SR & 0x1000) == 0) // wait for TXC
	{}
	GPIOB->BSRR = 0x00004000; // CMD high on PB14
	GPIOB->BSRR = 0x00001000; // CS high on PB14

}

