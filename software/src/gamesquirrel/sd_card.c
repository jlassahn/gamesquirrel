
#include "gamesquirrel/sd_card.h"
#include "stm32h523xx.h"

#include "gamesquirrel/core.h"
#include <stdio.h> // FIXME testing

// FIXME card presence detect

// FIXME design and implement SD Card API
bool SDCardInit(void)
{
	// violates strict aliasing rules
	volatile uint8_t *tx_reg = (volatile uint8_t *)&SPI1->TXDR;
	volatile uint8_t *rx_reg = (volatile uint8_t *)&SPI1->RXDR;

	// PB6 is detect
	GPIOA->BSRR = 0x80000000; // CS low on PA15
	uint8_t b = 0xDE;
	uint32_t sr = SPI1->SR;
	if (sr & 0x0001)
		b = *rx_reg;
	(void)b;

	printf("SR %.8lX %.2X\r\n", sr, b);

	// GPIOA-> // FIXME clear CS

	*tx_reg = 0xC5;
	while ((SPI1->SR & 0x1000) == 0) // wait for TXC
	{}

	GPIOA->BSRR = 0x00008000; // CS high on PA15

	return true;
}

