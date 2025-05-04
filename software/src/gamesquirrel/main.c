
#include "stm32h503xx.h"
#include "stm32h5xx_hal.h"
#include "gamesquirrel/usb.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

static const uint32_t blink_interval_ms = 250;

// LED
#define LED_PORT              GPIOA
#define LED_PIN               GPIO_PIN_5

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13

// UART Enable for STLink VCOM
#define UART_DEV              USART3
#define UART_CLK_EN           __USART3_CLK_ENABLE
#define UART_GPIO_PORT        GPIOA
#define UART_GPIO_AF          GPIO_AF13_USART3

#define UART_TX_PIN           GPIO_PIN_3
#define UART_RX_PIN           GPIO_PIN_4

#define CFG_BOARD_UART_BAUDRATE 115200   ///< Default baud rate

void board_led_write(bool state);
uint32_t board_button_read(void);
int board_uart_read(uint8_t *buf, int len);
int board_uart_write(void const *buf, int len);
uint32_t board_millis(void);

void hw_init(void);
void board_init(void);

UART_HandleTypeDef UartHandle;

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  HAL_UART_Transmit(&UartHandle, (uint8_t*) (uintptr_t) buf, len, 0xffff);
  return len;
}

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
  HAL_IncTick();
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
  return board_uart_write(buf, (int) count);
}

int _read (int fhdl, char *buf, size_t count) {
  (void) fhdl;
  int rd = board_uart_read((uint8_t*) buf, (int) count);
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

static inline void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  // Configure CRS clock source
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  /* Select HSI48 as USB clock source */
  RCC_PeriphCLKInitTypeDef usb_clk = {0 };
  usb_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  usb_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&usb_clk);

  /* Peripheral clock enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

void board_init(void)
{
	HAL_Init();
	SystemClock_Config();
	SystemCoreClockUpdate();

	// 1ms tick timer
	SysTick_Config(SystemCoreClock / 1000);

	// Alternate Function from tables in data sheets
	// 0 is both the unused value and an active value for some pins
	GPIOA->AFR[0] =
		( 0 <<  0) |  // PA0     ADC0
		( 0 <<  4) |  // PA1     ADC1
		( 0 <<  8) |  // PA2     ADC2
		(13 << 12) |  // PA3     ADC3           USART3_TX
		(13 << 16) |  // PA4     DAC1           USART3_RX
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
		(2 <<  6) | // PA3     ADC3           USART3_TX
		(2 <<  8) | // PA4     DAC1           USART3_RX
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
		(2 <<  6) | // PA3     ADC3           USART3_TX
		(2 <<  8) | // PA4     DAC1           USART3_RX
		(1 << 10) | // PA5     DAC2           LED
		(3 << 12) | // PA6     QSPI
		(3 << 14) | // PA7     QSPI
		(3 << 16) | // PA8     VideoTE
		(3 << 18) | // PA9     LED1
		(3 << 20) | // PA10    LED2
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

	UART_CLK_EN();
	UartHandle = (UART_HandleTypeDef) {
	  .Instance        = UART_DEV,
	  .Init.BaudRate   = CFG_BOARD_UART_BAUDRATE,
	  .Init.WordLength = UART_WORDLENGTH_8B,
	  .Init.StopBits   = UART_STOPBITS_1,
	  .Init.Parity     = UART_PARITY_NONE,
	  .Init.HwFlowCtl  = UART_HWCONTROL_NONE,
	  .Init.Mode       = UART_MODE_TX_RX,
	  .Init.OverSampling = UART_OVERSAMPLING_16,
	  .AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT
	};
	HAL_UART_Init(&UartHandle);

	// USB
	__HAL_RCC_USB_CLK_ENABLE();

	/* Enable VDDUSB */
#if defined (PWR_USBSCR_USB33DEN)
	HAL_PWREx_EnableVddUSB();
#endif
}

int main(void)
{
	hw_init();
	UsbStart();

	while (!board_button_read())
	{
	}

	printf("Hello, world\r\n");
	UsbSend("Hello USB\r\n", 11);

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
			printf("buff %s\r\n", buff);
		}

		uint32_t ms = board_millis();
		if (ms - last_ms < blink_interval_ms)
			continue;
		last_ms = ms;

		board_led_write(led_state);
		led_state = 1 - led_state; // toggle

		printf("interrupts  %d\r\n", UsbInterruptCount());
	}
}

