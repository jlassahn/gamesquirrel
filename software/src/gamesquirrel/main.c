
#include "stm32h503xx.h"
#include "stm32h5xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

static const uint32_t blink_interval_ms = 250;

// LED
#define LED_PORT              GPIOA
#define LED_PIN               GPIO_PIN_5
#define LED_STATE_ON          1

// Button
#define BUTTON_PORT           GPIOA
#define BUTTON_PIN            GPIO_PIN_0
#define BUTTON_STATE_ACTIVE   0

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
void led_blinking_task(void);

UART_HandleTypeDef UartHandle;

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
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
  __HAL_RCC_USB_CLK_ENABLE();
}

void board_init(void) {
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();

  // Enable All GPIOs clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

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

  UART_CLK_EN();

  // UART
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

  // Configure USB DM and DP pins. This is optional, and maintained only for user guidance.
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  __HAL_RCC_USB_CLK_ENABLE();

  /* Enable VDDUSB */
  #if defined (PWR_USBSCR_USB33DEN)
  HAL_PWREx_EnableVddUSB();
  #endif
}

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
  return BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
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

//--------------------------------------------------------------------+
// newlib read()/write() retarget
//--------------------------------------------------------------------+
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

  #define FSDEV_REG USB_DRD_FS_NS
  #define USB_CNTR_FRES USB_CNTR_USBRST
  #define FSDEV_EP_COUNT 8

  // Follow the RM mentions to use a special ordering of PDWN and FRES
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  // Perform USB peripheral reset
  FSDEV_REG->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  FSDEV_REG->CNTR &= ~USB_CNTR_PDWN;

  // Wait startup time, for F042 and F070, this is <= 1 us.
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }
  FSDEV_REG->CNTR = 0; // Enable USB

  // FIXME for some reason if I don't read back CNTR the CHEP registers don't
  // work.  WTF?  Is this a hardware quirk or a problem with compiler
  // optimization?  Maybe a compiler optimization issue, since removing the
  // volatile below prevents it from working.
  volatile uint32_t reg = FSDEV_REG->CNTR;
  (void)reg;

  FSDEV_REG->ISTR = 0; // Clear pending interrupts

  // Reset endpoints to disabled
  //for (uint32_t i = 0; i < FSDEV_EP_COUNT; i++) {
    // This doesn't clear all bits since some bits are "toggle", but does set the type to DISABLED.
    //ep_write(i, 0u, false);
    //FSDEV_REG->ep[i].reg = (fsdev_bus_t) 0;
    FSDEV_REG->CHEP0R = 0;
    FSDEV_REG->CHEP1R = 0;
    FSDEV_REG->CHEP2R = 0;
    FSDEV_REG->CHEP3R = 0;
    FSDEV_REG->CHEP4R = 0;
    FSDEV_REG->CHEP5R = 0;
    FSDEV_REG->CHEP6R = 0;
    FSDEV_REG->CHEP7R = 0;
  //}

  FSDEV_REG->CHEP0R = 0x2220;


  //handle_bus_reset(rhport);
    // Set up endpoint registers
    //FSDEV_REG->DADDR = 0u; // disable USB Function
    FSDEV_REG->DADDR = USB_DADDR_EF; // Enable USB Function

  // Enable pull-up if supported
  //dcd_connect(rhport);
    FSDEV_REG->BCDR |= USB_BCDR_DPPU;
}

int main(void) {

  hw_init();
  printf("Hello, world\r\n");
  printf("CHEP0R == %.8lX\r\n", USB_DRD_FS_NS->CHEP0R);
  printf("CHEP1R == %.8lX\r\n", USB_DRD_FS_NS->CHEP1R);
  printf("CHEP2R == %.8lX\r\n", USB_DRD_FS_NS->CHEP2R);
  printf("CHEP3R == %.8lX\r\n", USB_DRD_FS_NS->CHEP3R);
  printf("CHEP4R == %.8lX\r\n", USB_DRD_FS_NS->CHEP4R);
  printf("CHEP5R == %.8lX\r\n", USB_DRD_FS_NS->CHEP5R);
  printf("CHEP6R == %.8lX\r\n", USB_DRD_FS_NS->CHEP6R);
  printf("CHEP7R == %.8lX\r\n", USB_DRD_FS_NS->CHEP7R);

  while (true) {
    led_blinking_task();
  }
}


//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
