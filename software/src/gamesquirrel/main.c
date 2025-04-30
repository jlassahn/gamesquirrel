
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
#define LED_STATE_ON          1

// Button
#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13
#define BUTTON_STATE_ACTIVE   1

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
  GPIO_PinState pin_state = (GPIO_PinState) (state ? LED_STATE_ON : (1 - LED_STATE_ON));
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, pin_state);
}

uint32_t board_button_read(void) {
	int val = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
  return val;
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

  usb_init();
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

void board_init(void) {
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();

  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

  GPIO_InitTypeDef GPIO_InitStruct;

  // LED
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  board_led_write(false);

  // Button
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = BUTTON_STATE_ACTIVE ? GPIO_PULLDOWN : GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  // UART
  UART_CLK_EN();
  GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = UART_GPIO_AF;
  HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

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
  // Configure USB DM and DP pins. This is optional, and maintained only for user guidance.
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /* Enable VDDUSB */
  #if defined (PWR_USBSCR_USB33DEN)
  HAL_PWREx_EnableVddUSB();
  #endif
}

int main(void) {

  hw_init();
  printf("Hello, world\r\n");

	extern const uint8_t config_descriptor[];
	for (int i=0; i<75; i++)
	{
		printf(" %.2X,", config_descriptor[i]);
		if ((i&7) == 7)
			printf("\r\n");
	}
	printf("\r\n");

  bool usb_enable = false;
  bool led_state = false;
  uint32_t last_ms = board_millis();
  while (true)
  {
	if (usb_enable)
	{
		usb_tick();
	}
	else
	{
		if (board_button_read())
		{
			usb_start();
			usb_enable = true;
		}
	}

    uint32_t ms = board_millis();
    if (ms - last_ms < blink_interval_ms)
      continue;
	last_ms = ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle

  }
}

