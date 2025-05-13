
#include "gamesquirrel/core.h"
#include "stm32h523xx.h"

static void ClockInit(void)
{
	// set highest voltage on regulator before setting clocks to high speed
	PWR->VOSCR = 0x30; //SCALE0, highest voltage
	while ((PWR->VOSSR & PWR_VOSSR_VOSRDY) == 0)
	{}

	//  FIXME set up RCC_BDCR (RCC Backup Domain) to enable LSE Oscillator

	// Set up PLL1
	RCC->PLL1CFGR =
		0x00030000 | // enable P and Q outputs (R disabled)
		( 8 << 8)  | // M value (16MHz clock / 8 == 2 MHz)
		0x00000010 | // VCO range 2 -4 MHz
		0x00000003;  // Source is HSE

	RCC->PLL1DIVR =
		((250 - 1) << 0) | // N   VCO = 2*250 = 500MHz
		((2 - 1) << 9) |   // P   Core clock 500/2 = 250MHz
		((2 - 1) << 16) |  // Q   SPI2 500/8 = 62.5MHz
		((2 - 1) << 24);   // R   Unused

	// Set up PLL2
	RCC->PLL2CFGR =
		0x00050000 | // enable P and R outputs (Q disabled)
		( 8 << 8)  | // M value (16MHz clock / 8 == 2 MHz)
		0x00000010 | // VCO range 2 -4 MHz
		0x00000003;  // Source is HSE

	RCC->PLL2DIVR =
		((100 - 1) << 0) | // N   VCO = 2*100 = 200MHz
		((8 - 1) << 9) |   // P   SPI1 200/8 = 25MHz
		((2 - 1) << 16) |  // Q   Unused
		((2 - 1) << 24);   // R   OCTOSPI 200/2 = 100MHz

	// Set up PLL3
	RCC->PLL3CFGR =
		0x00060000 | // enable Q and R outputs (P disabled)
		( 8 << 8)  | // M value (16MHz clock / 8 == 2 MHz)
		0x00000010 | // VCO range 2 -4 MHz
		0x00000003;  // Source is HSE

	RCC->PLL3DIVR =
		((96 - 1) << 0) | // N   VCO = 2*96 = 192MHz
		((4 - 1) << 9) |  // P   Unused
		((4 - 1) << 16) | // Q   USB 192/4 = 48MHz
		((4 - 1) << 24);  // R   LPTIM2 192/4 = 48MHz

	RCC->CR =
		RCC_CR_HSION |
		// RCC_CR_HSIKERON
		RCC_CR_HSIDIV_0 |  // HSI divide by 2 (32 MHz)
		// RCC_CR_HSIDIVF
		// RCC_CR_CSION
		// RCC_CR_CSIKERON
		// RCC_CR_HSI48ON |
		RCC_CR_HSEON |
		RCC_CR_HSEBYP |
		RCC_CR_HSEEXT |
		RCC_CR_PLL1ON |
		RCC_CR_PLL2ON |
		RCC_CR_PLL3ON;

	uint32_t rcc_cr_mask =
		RCC_CR_HSIRDY |
		// RCC_CR_CSIRDY
		// RCC_CR_HSI48RDY |
		RCC_CR_HSERDY |
		RCC_CR_PLL1RDY |
		RCC_CR_PLL2RDY |
		RCC_CR_PLL3RDY;

	while ((RCC->CR & rcc_cr_mask) != rcc_cr_mask)
	{}

	// set flash latency before switching CPU clocks
	FLASH->ACR =  0x125; // prefetch enabled, 2 programming delay, 5 wait states
	(void)FLASH->ACR; // readback to confirm write completed

	RCC->CFGR1 = 0x0003; // System clock is PLL1
	RCC->CFGR2 = 0; // All AHB and APB clocks are at System/1
	(void)RCC->CFGR1; // readback to force sync
	(void)RCC->CFGR2; // readback to force sync

	/* Peripheral clock enable */
	// RCC->AHB1ENR = 0xD0000100; // FIXME enable DMA and maybe other stuff

	RCC->AHB2ENR =
		RCC_AHB2ENR_SRAM3EN |
		RCC_AHB2ENR_SRAM2EN |
		RCC_AHB2ENR_RNGEN |
		RCC_AHB2ENR_DAC1EN |
		RCC_AHB2ENR_ADCEN |
		RCC_AHB2ENR_GPIOAEN |
		RCC_AHB2ENR_GPIOBEN |
		RCC_AHB2ENR_GPIOCEN |
		RCC_AHB2ENR_GPIODEN |
		RCC_AHB2ENR_GPIOHEN;

	RCC->AHB4ENR; // FIXME enable stuff

	// FIXME check and enable peripheral clocks below
	RCC->APB1LENR =
		RCC_APB1LENR_CRSEN |
		RCC_APB1LENR_USART3EN;

	RCC->APB1HENR = 0;

	RCC->APB2ENR =
		RCC_APB2ENR_USBEN;

	RCC->APB3ENR; // FIXME

	RCC->CCIPR4 =
		(2 << 0) | // OCTOSPI == pll2_r
		(0 << 2) | // SYSTICKSEL == rcc_hclk/8
		(2 << 4);  // USBSEL == pll3_q
}

static void GPIOInit(void)
{
	// Alternate Function from tables in data sheets
	// 0 is both the unused value and an active value for some pins
	GPIOA->AFR[0] =
		( 0 <<  0) |  // PA0     ADC0
		( 0 <<  4) |  // PA1     ADC1
		( 0 <<  8) |  // PA2     ADC2
		( 0 << 12) |  // PA3     ADC3           USART3_TX
		( 0 << 16) |  // PA4     DAC1           USART3_RX
		( 0 << 20) |  // PA5     DAC2           LED
		( 0 << 24) |  // PA6     QSPI
		( 0 << 28);   // PA7     QSPI
	GPIOA->AFR[1] =
		( 0 <<  0) |  // PA8     VideoTE
		( 0 <<  4) |  // PA9     LED1
		( 0 <<  8) |  // PA10    LED2
		(10 << 12) |  // PA11    USB            USB
		(10 << 16) |  // PA12    USB            USB
		( 0 << 20) |  // PA13    Debug
		( 0 << 24) |  // PA14    Debug
		( 0 << 28);   // PA15    SD_NSS

	GPIOB->AFR[0] =
		( 0 <<  0) |  // PB0     QSPI
		( 0 <<  4) |  // PB1     QSPI
		( 0 <<  8) |  // PB2     QSPI
		( 0 << 12) |  // PB3     SD_SCK
		( 0 << 16) |  // PB4     SD_MISO
		( 0 << 20) |  // PB5     SD_MOSI
		( 0 << 24) |  // PB6     SD_Detect
		( 0 << 28);   // PB7     Button3
	GPIOB->AFR[1] =
		( 0 <<  0) |  // PB8     Button4
		( 0 <<  4) |  // (Unused)
		( 0 <<  8) |  // PB10    QSPI
		( 0 << 12) |  // (Unused)
		( 0 << 16) |  // PB12    VideoNSS
		( 0 << 20) |  // PB13    VideoSCK
		( 0 << 24) |  // PB14    VideoCMD
		( 0 << 28);   // PB15    VideoMOSI

	GPIOC->AFR[0] = 0;
	GPIOC->AFR[1] = 0;
	GPIOD->AFR[0] = 0;
	GPIOD->AFR[1] = 0;
	GPIOH->AFR[0] = 0;
	GPIOH->AFR[1] = 0;

	// All outputs are push/pull not open-drain
	GPIOA->OTYPER = 0;
	GPIOB->OTYPER = 0;
	GPIOC->OTYPER = 0;
	GPIOD->OTYPER = 0;
	GPIOH->OTYPER = 0;

	// Speed 0 slowest to 3 fastest
	GPIOA->OSPEEDR =
		(0 <<  0) | // PA0     ADC0
		(0 <<  2) | // PA1     ADC1
		(0 <<  4) | // PA2     ADC2
		(0 <<  6) | // PA3     ADC3           USART3_TX
		(0 <<  8) | // PA4     DAC1           USART3_RX
		(0 << 10) | // PA5     DAC2           LED
		(0 << 12) | // PA6     QSPI
		(0 << 14) | // PA7     QSPI
		(0 << 16) | // PA8     VideoTE
		(0 << 18) | // PA9     LED1
		(0 << 20) | // PA10    LED2
		(2 << 22) | // PA11    USB            USB
		(2 << 24) | // PA12    USB            USB
		(0 << 26) | // PA13    Debug
		(0 << 28) | // PA14    Debug
		(0 << 30);  // PA15    SD_NSS
	GPIOB->OSPEEDR =
		(0 << 0 ) | // PB0     QSPI
		(0 << 2 ) | // PB1     QSPI
		(0 << 4 ) | // PB2     QSPI
		(0 << 6 ) | // PB3     SD_SCK
		(0 << 8 ) | // PB4     SD_MISO
		(0 << 10) | // PB5     SD_MOSI
		(0 << 12) | // PB6     SD_Detect
		(0 << 14) | // PB7     Button3
		(0 << 16) | // PB8     Button4
		(0 << 18) | // (Unused)
		(0 << 20) | // PB10    QSPI
		(0 << 22) | // (Unused)
		(0 << 24) | // PB12    VideoNSS
		(0 << 26) | // PB13    VideoSCK
		(0 << 28) | // PB14    VideoCMD
		(0 << 30);  // PB15    VideoMOSI
	GPIOC->OSPEEDR = 0;
	GPIOD->OSPEEDR = 0;
	GPIOH->OSPEEDR = 0;

	// Options
	// 0 -no pull
	// 1 -pull up
	// 2 -pull down
	// 3 -reserved
	GPIOA->PUPDR =
		(0 <<  0) | // PA0     ADC0
		(0 <<  2) | // PA1     ADC1
		(0 <<  4) | // PA2     ADC2
		(0 <<  6) | // PA3     ADC3           USART3_TX
		(0 <<  8) | // PA4     DAC1           USART3_RX
		(0 << 10) | // PA5     DAC2           LED
		(0 << 12) | // PA6     QSPI
		(0 << 14) | // PA7     QSPI
		(0 << 16) | // PA8     VideoTE
		(0 << 18) | // PA9     LED1
		(0 << 20) | // PA10    LED2
		(0 << 22) | // PA11    USB            USB
		(0 << 24) | // PA12    USB            USB
		(0 << 26) | // PA13    Debug
		(0 << 28) | // PA14    Debug
		(0 << 30);  // PA15    SD_NSS
	GPIOB->PUPDR =
		(0 << 0 ) | // PB0     QSPI
		(0 << 2 ) | // PB1     QSPI
		(0 << 4 ) | // PB2     QSPI
		(0 << 6 ) | // PB3     SD_SCK
		(0 << 8 ) | // PB4     SD_MISO
		(0 << 10) | // PB5     SD_MOSI
		(0 << 12) | // PB6     SD_Detect
		(0 << 14) | // PB7     Button3
		(0 << 16) | // PB8     Button4
		(0 << 18) | // (Unused)
		(0 << 20) | // PB10    QSPI
		(0 << 22) | // (Unused)
		(0 << 24) | // PB12    VideoNSS
		(0 << 26) | // PB13    VideoSCK
		(0 << 28) | // PB14    VideoCMD
		(0 << 30);  // PB15    VideoMOSI
	GPIOC->PUPDR =
		(2 << 26) | // PC13    Button1        Button
		(0 << 28) | // PC14    OSC32_IN       OSC32_IN
		(0 << 30);  // PC15    OSC32_OUT      OSC32_OUT

	GPIOD->PUPDR = 0;
	GPIOH->PUPDR =
		(0 << 0) | // PH0     OSC_IN         OSC_IN
		(0 << 2);  // PH1     Button2        OSC_OUT

	// initial output data
	GPIOA->ODR = 0;
	GPIOB->ODR = 0;
	GPIOC->ODR = 0;
	GPIOD->ODR = 0;
	GPIOH->ODR = 0;

	// Modes
	// 0 -Input
	// 1 -Output
	// 2 -Alternate Function
	// 3 -Analog Function or disabled
	GPIOA->MODER =
		(3 <<  0) | // PA0     ADC0
		(3 <<  2) | // PA1     ADC1
		(3 <<  4) | // PA2     ADC2
		(3 <<  6) | // PA3     ADC3           USART3_TX
		(3 <<  8) | // PA4     DAC1           USART3_RX
		(1 << 10) | // PA5     DAC2           LED
		(3 << 12) | // PA6     QSPI
		(3 << 14) | // PA7     QSPI
		(3 << 16) | // PA8     VideoTE
		(1 << 18) | // PA9     LED1
		(1 << 20) | // PA10    LED2
		(2 << 22) | // PA11    USB            USB
		(2 << 24) | // PA12    USB            USB
		(2 << 26) | // PA13    Debug
		(2 << 28) | // PA14    Debug
		(2 << 30);  // PA15    SD_NSS
	GPIOB->MODER =
		(3 << 0 ) | // PB0     QSPI
		(3 << 2 ) | // PB1     QSPI
		(3 << 4 ) | // PB2     QSPI
		(2 << 6 ) | // PB3     SD_SCK
		(2 << 8 ) | // PB4     SD_MISO
		(3 << 10) | // PB5     SD_MOSI
		(3 << 12) | // PB6     SD_Detect
		(3 << 14) | // PB7     Button3
		(3 << 16) | // PB8     Button4
		(0 << 18) | // (Unused)
		(3 << 20) | // PB10    QSPI
		(0 << 22) | // (Unused)
		(3 << 24) | // PB12    VideoNSS
		(3 << 26) | // PB13    VideoSCK
		(3 << 28) | // PB14    VideoCMD
		(3 << 30);  // PB15    VideoMOSI
	GPIOC->MODER = 0x03FFFFFF |
		(0 << 26) | // PC13    Button1        Button
		(3 << 28) | // PC14    OSC32_IN       OSC32_IN
		(3 << 30);  // PC15    OSC32_OUT      OSC32_OUT
	GPIOD->MODER = 0x00000030;
	GPIOH->MODER =
		(3 << 0) | // PH0     OSC_IN         OSC_IN
		(3 << 2);  // PH1     Button2        OSC_OUT

}

void CoreInit(void)
{
	ClockInit();

	// This just sets the SystemCoreClock variable,
	// relies on HSE_VALUE being defined
	SystemCoreClockUpdate();

	// 1ms tick timer, in CMSIS, sets up SysTick counter and enables interrupt.
	SysTick_Config(SystemCoreClock / 1000);

	// FIXME is ICACHE enabled?  DCACHE?

	GPIOInit();

	// USB
	/* Enable VDDUSB */
	PWR->USBSCR = PWR_USBSCR_USB33SV;

	// FIXME set up OCTOSPI
	// FIXME set up Display SPI
	// FIXME set up SD card SPI
	// FIXME set up ADCs
	// FIXME set up DACs
	// FIXME set up timers
	// FIXME set up DMA
}

