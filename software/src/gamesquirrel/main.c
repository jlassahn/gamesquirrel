
#define DEMO_BOARD 0
#define ENABLE_UART 0
// #define HSE_VALUE    (16000000UL) // Currently defined in tools/compile_settings.mk

#include "stm32h523xx.h"
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
  board_init();

  UsbInit();
}

static inline void SystemClock_Config(void)
{
	// set highest voltage on regulator before setting clocks to high speed
	PWR->VOSCR = 0x30; //SCALE0, highest voltage
	while ((PWR->VOSSR & PWR_VOSSR_VOSRDY) == 0)
	{}

	// Set up PLL1
	RCC->PLL1CFGR =
		0x00070000 | // enable P Q and R outputs
#if DEMO_BOARD
		(12 << 8)  | // M value (24MHz clock / 12 == 2 MHz)
#else
		( 8 << 8)  | // M value (16MHz clock / 8 == 2 MHz)
#endif
		0x00000010 | // VCO range 2 -4 MHz
		0x00000003;  // Source is HSE

	RCC->PLL1DIVR =
		(250 << 0) | // N  FIXME set to 240 eventually
		(2 << 9) | // P
		(2 << 16) | // Q
		(2 << 24); // R

	RCC->CR =
		RCC_CR_HSION |
		// RCC_CR_HSIKERON
		RCC_CR_HSIDIV_0 |  // HSI divide by 2 (32 MHz)
		// RCC_CR_HSIDIVF
		// RCC_CR_CSION
		// RCC_CR_CSIKERON
		RCC_CR_HSI48ON |  // FIXME currently used by USB, replace with PLL
		RCC_CR_HSEON |
#if DEMO_BOARD
		// RCC_CR_HSEBYP // FIXME actual PCB runs in bypass mode
		// RCC_CR_HSEEXT // FIXME actual PCB ext should be set
#else
		RCC_CR_HSEBYP | // FIXME actual PCB runs in bypass mode
		RCC_CR_HSEEXT | // FIXME actual PCB ext should be set
#endif
		RCC_CR_PLL1ON;
		// RCC_CR_PLL2ON // FIXME actual PCB has more PLLs

	uint32_t rcc_cr_mask =
		RCC_CR_HSIRDY |
		// RCC_CR_CSIRDY
		RCC_CR_HSI48RDY |
		RCC_CR_HSERDY |
		RCC_CR_PLL1RDY;
		// RCC_CR_PLL2RDY

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
	// RCC->AHB1ENR = 0x90000100;

	RCC->AHB2ENR =
		RCC_AHB2ENR_SRAM2EN |
		// RCC_AHB2ENR_DAC1EN |
		// RCC_AHB2ENR_ADCEN |
		RCC_AHB2ENR_GPIOAEN |
		RCC_AHB2ENR_GPIOBEN |
		RCC_AHB2ENR_GPIOCEN |
		RCC_AHB2ENR_GPIODEN |
		RCC_AHB2ENR_GPIOHEN;

	RCC->APB1LENR =
		RCC_APB1LENR_CRSEN |
		RCC_APB1LENR_USART3EN;

	RCC->APB1HENR = 0;

	RCC->APB2ENR =
		RCC_APB2ENR_USBEN;

	// FIXME USB seems to mostly work without CRS, maybe keep it turned off
	// since we'll be using a clock derived from HSE and the PLLs for the
	// final USB config.
	//
	// My H503 only adjusts the trim between 0x1F and 0x20

	CRS->CFGR =
		CRS_CFGR_SYNCSRC_1 | // Sync Source == 2 fo USB
		(34 << 16) |
		(47999);

	CRS->CR =
		CRS_CR_CEN |
		CRS_CR_AUTOTRIMEN |
		(32 << 8);

	/* Select HSI48 as USB clock source */
	RCC->CCIPR4 =
		(0 << 2) | // SYSTICKSEL == rcc_hclk/8
		(3 << 4);  // USBSEL == hsi48_ker_clk
}

void board_init(void)
{
	// FIXME set up SysTick interrupt (seems to be getting done somehow already...)
	SystemClock_Config();
	SystemCoreClockUpdate(); // relies on HSE_VALUE

	// 1ms tick timer
	SysTick_Config(SystemCoreClock / 1000);

	// FIXME is ICACHE enabled?

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

	// USB
	/* Enable VDDUSB */
	PWR->USBSCR = PWR_USBSCR_USB33SV;
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

		// FIXME there's something wrong with the SystemCoreClock value, but SysTick is running at the right speed.
		printf("Tick %lu  %lu\r\n", ms, SystemCoreClock);
	}
}

