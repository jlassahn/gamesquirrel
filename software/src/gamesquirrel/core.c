/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include "gamesquirrel/core.h"
#include "stm32h523xx.h"
#include <string.h>

void HardFault_Handler(void)
{
    LEDWrite(0, 1000);
    LEDWrite(1, 0);

    while (true)
    {
        uint32_t us = TimeMicroseconds();
        if (us & 0x100000)
        {
            LEDWrite(0, 0);
            LEDWrite(1, 1000);
        }
        else
        {
            LEDWrite(0, 1000);
            LEDWrite(1, 0);
        }
    }
}

static void ClockInit(void)
{
    // set highest voltage on regulator before setting clocks to high speed
    PWR->VOSCR = 0x30; //SCALE0, highest voltage
    while ((PWR->VOSSR & PWR_VOSSR_VOSRDY) == 0)
    {}

    RCC->CIER = 0U;

    // Set up PLL1
    RCC->PLL1CFGR =
        0x00030000 | // enable P and Q outputs (R disabled)
        ( 8 << 8)  | // M value (16MHz clock / 8 == 2 MHz)
        0x00000010 | // VCO range 2 -4 MHz
        0x00000003;  // Source is HSE

    RCC->PLL1DIVR =
        ((250 - 1) << 0) | // N   VCO = 2*250 = 500MHz
        ((2 - 1) << 9) |   // P   Core clock 500/2 = 250MHz
        ((8 - 1) << 16) |  // Q   SPI2 500/8 = 62.5MHz
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
        RCC_CR_HSIDIV_0 |  // HSI divide by 2 (32 MHz)
        RCC_CR_HSEON |
        RCC_CR_HSEBYP |
        RCC_CR_HSEEXT |
        RCC_CR_PLL1ON |
        RCC_CR_PLL2ON |
        RCC_CR_PLL3ON;

    uint32_t rcc_cr_mask =
        RCC_CR_HSIRDY |
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
    RCC->AHB1ENR =
        RCC_AHB1ENR_GPDMA1EN |
        RCC_AHB1ENR_GPDMA2EN |
        RCC_AHB1ENR_FLITFEN |
        // RCC_AHB1ENR_CRCEN
        // RCC_AHB1ENR_RAMCFGEN
        // RCC_AHB1ENR_TZSC1EN
        RCC_AHB1ENR_BKPRAMEN |
        RCC_AHB1ENR_DCACHE1EN |
        RCC_AHB1ENR_SRAM1EN;

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

    RCC->AHB4ENR =
        RCC_AHB4ENR_OCTOSPI1EN;

    RCC->APB1LENR =
        RCC_APB1LENR_SPI2EN |
        RCC_APB1LENR_TIM2EN;

    RCC->APB1HENR =
        RCC_APB1HENR_LPTIM2EN;

    RCC->APB2ENR =
        RCC_APB2ENR_TIM1EN |
        RCC_APB2ENR_SPI1EN |
        RCC_APB2ENR_USBEN;

    RCC->APB3ENR =
        RCC_APB3ENR_LPTIM1EN |
        RCC_APB3ENR_RTCAPBEN;

    RCC->CCIPR1;
    RCC->CCIPR2 =
        (0 << 8) | // LPTIM1SEL rcc_pclk3
        (2 << 12); // LPTIM2SEL pll3_r
    RCC->CCIPR3 =
        (1 << 0) | // SPI1SEL pll2_p
        (0 << 3);  // SPI2SEL pll1_q
    RCC->CCIPR4 =
        (2 << 0) | // OCTOSPI == pll2_r
        (0 << 2) | // SYSTICKSEL == rcc_hclk/8
        (2 << 4);  // USBSEL == pll3_q
    RCC->CCIPR5 =
        (0 << 0) | // ADCDACSEL rcc_hclk
        (1 << 4);  // RNGSEL pll1_q
}

static void LowPowerInit(void)
{
    // check if the low power domain is already initialized
    if ((RCC->BDCR & 0x0001) == 0x0001)
        return;

    // Backup domain control register unlock
    PWR->DBPCR = 1;

    // LSE clock
    RCC->BDCR =
        0x00000001 | // LSE ON
        0x00000018 | // LSEDRV high
        0x00000100;  // RTCSEL use LSE

    // FIXME set up realtime clock
    // FIXME functions to access realtime clock
    // FIXME calendar and timezone stuff
    // FIXME set up backup RAM
}

// FIXME do we need a cache control API?
static void CacheInit(void)
{
    // default cache attributes for OctoSPI are write-thru no allocate.
    // make a emory attribute region that makes OctoSPI write-back
    MPU->RNR = 0; // set up region 0
    MPU->RBAR = 0x90000000; // base (not sharable which shouldn't matter)
    MPU->RLAR = 0x9FFFFFE1; // Limit (attr 0, enable)
    MPU->MAIR0 = 0xFF; // inner and outer write-back with allocate
    MPU->MAIR1 = 0;
    MPU->CTRL = 0x05; // enabled, allow defaults

    // FIXME use ICACHE to map external memory at 0x04000000

    ICACHE->CR = 0x0006; // invalidate ICACHE
    while (ICACHE->SR & 1) // wait for busy
    {}
    ICACHE->CR = 0x0005; // enable ICACHE

    DCACHE1->CR = 0x0002; // invalidate full DCACHE
    while (DCACHE1->SR & 9) // wait for busy bits
    {}

    DCACHE1->CR = 0x80000001; // enable DCACHE, increment mode
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
        ( 6 << 24) |  // PA6     QSPI
        (10 << 28);   // PA7     QSPI
    GPIOA->AFR[1] =
        ( 0 <<  0) |  // PA8     VideoTE
        ( 1 <<  4) |  // PA9     LED1
        ( 1 <<  8) |  // PA10    LED2
        (10 << 12) |  // PA11    USB            USB
        (10 << 16) |  // PA12    USB            USB
        ( 0 << 20) |  // PA13    Debug
        ( 0 << 24) |  // PA14    Debug
        ( 0 << 28);   // PA15    SD_NSS

    GPIOB->AFR[0] =
        ( 6 <<  0) |  // PB0     QSPI
        ( 6 <<  4) |  // PB1     QSPI
        ( 9 <<  8) |  // PB2     QSPI
        ( 5 << 12) |  // PB3     SD_SCK
        ( 5 << 16) |  // PB4     SD_MISO
        ( 5 << 20) |  // PB5     SD_MOSI
        ( 0 << 24) |  // PB6     SD_Detect
        ( 0 << 28);   // PB7     Button3
    GPIOB->AFR[1] =
        ( 0 <<  0) |  // PB8     Button4
        ( 0 <<  4) |  // (Unused)
        ( 9 <<  8) |  // PB10    QSPI
        ( 0 << 12) |  // (Unused)
        ( 0 << 16) |  // PB12    VideoNSS
        ( 5 << 20) |  // PB13    VideoSCK
        ( 0 << 24) |  // PB14    VideoCMD
        ( 5 << 28);   // PB15    VideoMOSI

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
        (3 << 12) | // PA6     QSPI
        (3 << 14) | // PA7     QSPI
        (0 << 16) | // PA8     VideoTE
        (0 << 18) | // PA9     LED1
        (0 << 20) | // PA10    LED2
        (2 << 22) | // PA11    USB            USB
        (2 << 24) | // PA12    USB            USB
        (0 << 26) | // PA13    Debug
        (0 << 28) | // PA14    Debug
        (2 << 30);  // PA15    SD_NSS
    GPIOB->OSPEEDR =
        (3 << 0 ) | // PB0     QSPI
        (3 << 2 ) | // PB1     QSPI
        (3 << 4 ) | // PB2     QSPI
        (2 << 6 ) | // PB3     SD_SCK
        (2 << 8 ) | // PB4     SD_MISO
        (2 << 10) | // PB5     SD_MOSI
        (0 << 12) | // PB6     SD_Detect
        (0 << 14) | // PB7     Button3
        (0 << 16) | // PB8     Button4
        (0 << 18) | // (Unused)
        (3 << 20) | // PB10    QSPI
        (0 << 22) | // (Unused)
        (2 << 24) | // PB12    VideoNSS
        (2 << 26) | // PB13    VideoSCK
        (2 << 28) | // PB14    VideoCMD
        (2 << 30);  // PB15    VideoMOSI
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
        (1 << 16) | // PA8     VideoTE
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
        (1 << 8 ) | // PB4     SD_MISO
        (0 << 10) | // PB5     SD_MOSI
        (1 << 12) | // PB6     SD_Detect
        (2 << 14) | // PB7     Button3
        (2 << 16) | // PB8     Button4
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
        (2 << 2);  // PH1     Button2        OSC_OUT

    // initial output data
    GPIOA->ODR = 0x8000; // PA15   SD NSS
    GPIOB->ODR = 0x1000; // PB12   Video NSS
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
        (3 << 10) | // PA5     DAC2           LED
        (2 << 12) | // PA6     QSPI
        (2 << 14) | // PA7     QSPI
        (0 << 16) | // PA8     VideoTE
        (2 << 18) | // PA9     LED1
        (2 << 20) | // PA10    LED2
        (2 << 22) | // PA11    USB            USB
        (2 << 24) | // PA12    USB            USB
        (2 << 26) | // PA13    Debug
        (2 << 28) | // PA14    Debug
        (1 << 30);  // PA15    SD_NSS
    GPIOB->MODER =
        (2 << 0 ) | // PB0     QSPI
        (2 << 2 ) | // PB1     QSPI
        (2 << 4 ) | // PB2     QSPI
        (2 << 6 ) | // PB3     SD_SCK
        (2 << 8 ) | // PB4     SD_MISO
        (2 << 10) | // PB5     SD_MOSI
        (0 << 12) | // PB6     SD_Detect
        (0 << 14) | // PB7     Button3
        (0 << 16) | // PB8     Button4
        (0 << 18) | // (Unused)
        (2 << 20) | // PB10    QSPI
        (0 << 22) | // (Unused)
        (1 << 24) | // PB12    VideoNSS
        (2 << 26) | // PB13    VideoSCK
        (1 << 28) | // PB14    VideoCMD
        (2 << 30);  // PB15    VideoMOSI
    GPIOC->MODER = 0x03FFFFFF |
        (0 << 26) | // PC13    Button1        Button
        (3 << 28) | // PC14    OSC32_IN       OSC32_IN
        (3 << 30);  // PC15    OSC32_OUT      OSC32_OUT
    GPIOD->MODER = 0x00000030;
    GPIOH->MODER =
        (3 << 0) | // PH0     OSC_IN         OSC_IN
        (0 << 2);  // PH1     Button2        OSC_OUT

}

static void TimerInit(void)
{
    // TIM1 generates 1KHz PWM signals for indicator LEDs.
    TIM1->PSC = 250;
    TIM1->CCER = 0x0110;
    TIM1->BDTR = 0x8C00;
    TIM1->CNT = 0;
    TIM1->ARR = 999;
    // Channel 2 PWM mode1, Compare preload
    TIM1->CCMR1 = 0x00006800;
    // Channel 3 PWM mode1, Compare preload
    TIM1->CCMR2 = 0x00000068;

    // FIXME what is the best startup value for LEDs?
    TIM1->CCR2 = 1000; // PWM value for channel 2
    TIM1->CCR3 = 1000; // PWM value for channel 3

    TIM1->EGR = 1;
    TIM1->CR1 = 0x0001;

    // TIM2 is used as a 32 bit free-running 1MHz clock.
    TIM2->PSC = 250; //prescaler
    TIM2->CNT = 0; // initial counter value
    TIM2->ARR = 0xFFFFFFFF; // autoreload value 
    TIM2->EGR = 1; // trigger shadow register update
    TIM2->CR1 = 0x0001; //enable counter

    // LPTIM2 is used to generate a 48KHz pulse for audio DMA
    LPTIM2->CFGR = 0x0000;
    LPTIM2->CR = 0x0001; //enable
    LPTIM2->ARR = 999;
    LPTIM2->CR = 0x0005; //count start
    // enable compare channel 1 to generate events for DAC DMA
    LPTIM2->CCR1 = 0;
    LPTIM2->CCMR1 = 0x00000002
    ;
    // LPTIM1 can be used to measure PLL2/P or PLL3/R
}

static void OctoSPIInit(void)
{
    /*
    OctoSPI memory mapped configuration:
    1. Set up DCR registers for RAM timing
    2. do an indirect write to put the device into QSPI mode
    3. set up read timings in  TCR CCR IR ABR
    4. set up write timings in  WTCR WCCR WIR WABR
    5. enable memory mapped mode in CR

    Access memory at 0x90000000 (In data cache range)
    */

    // memory chip timing and size
    OCTOSPI1->DCR1 =
        0x02000000 | // MTYP is Standard Mode
        0x00150000 | // DEVSIZE is 4MBytes
        0x00000000 | // CSHT min CS high time is 1 clock
        0x00000008 | // DLYBYP Delay bypass enabled
        0x00000000 | // FRC free running clock off
        0x00000000;  // CKMODE 0, clock low while idle

    OCTOSPI1->DCR2 =
        0x00000000 | // WRAPSIZE no wrap
        0x00000000;  // PRESCALER 0

    OCTOSPI1->DCR3 = 0x000A0000; // CSBOUND is 1024
    OCTOSPI1->DCR4 = 0; // REFRESH disabled

    // Enter QSPI mode
    OCTOSPI1->CR = 0x00000001; //enable in indirect write mode
    OCTOSPI1->TCR = 0x40000000; // setd half clock delay and no dummy cycles
    OCTOSPI1->CCR =
        0x00000001 | // instruction 1 byte on one wire
        0x00000000 | // no address
        0x00000000;  // no data

    OCTOSPI1->FCR = 0x1B; // clear flags;

    OCTOSPI1->DLR = 0;
    OCTOSPI1->IR = 0x35; // Enter QPI command
    //OCTOSPI1->AR = 0x002C5555; // address is don't care for ReadID
    while ((OCTOSPI1->SR & 0x22) != 0x02)
    {}
    OCTOSPI1->CR = 0x00000000; // disable

    // set up memory mapped read timing
    OCTOSPI1->TCR =
        0x40000000 | // SSHIFT half-clock delay
        0x00000006;  // DCYC 6 dummy cycles
    OCTOSPI1->CCR =
        0x00000003 | // instruction 1 byte on four wires
        0x00002300 | // address 3 bytes on four wires
        0x03000000;  // data on four wires
    OCTOSPI1->ABR = 0; // no alternate bytes
    OCTOSPI1->IR = 0xEB; // Fast Read command

    // set up memory mapped write timing
    OCTOSPI1->WTCR = 0x00000000;  // DCYC 0 dummy cycles
    OCTOSPI1->WCCR =
        0x20000000 | // Set DQSE for erratum 2.4.1
        0x00000003 | // instruction 1 byte on four wires
        0x00002300 | // address 3 bytes on four wires
        0x03000000;  // data on four wires
    OCTOSPI1->WABR = 0; // no alternate bytes
    OCTOSPI1->WIR = 0x38; // Quad Write command

    OCTOSPI1->CR = 0x30000001; //enable in memory map mode
}

static void SPIInit(void)
{
    // Set up SPI peripherals for the display and SD card.
    // This only configures the SPI ports, initialization of the devices
    // happens later.

    // Neither interface has really consistent transaction sizes, so
    // we can't use TSIZE effectively.  This means we also can't rely on
    // automatic chip select control, so the CS pins are set up as general
    // purpose outputs.

    // set up Display SPI2
    SPI2->CR2 = 0; // TSIZE
    SPI2->CFG1 =
        0x50000000 | // FIXME prescaler divide by 64 for test
        // 0x80000000 | // prescaler disabled
        // 0x00008000 | // FIXME DMA enable
        0x00000007;  // Data size 8 bits
    SPI2->CFG2 =
        0x20000000 | // SSOE enable peripheral control of SS
        0x00020000 | // COMM Tx only
        0x00000000 | // Clock Idles low, samples on rising edge
        0x00400000; // MASTER
    SPI2->IFCR = 0x0BF8; // clear all flags
    SPI2->CR1 = 0x00000001; // SPE Enable
    SPI2->CR1 = 0x00000201; // CSTART

    // set up SD card SPI1
    SPI1->CR1 = 0; // TSIZE
    SPI1->CFG1 =
        0x40000000 | // FIXME prescaler divide by 32 for test
        // 0x80000000 | // prescaler disabled
        0x00000007;  // Data size 8 bits
    SPI1->CFG2 =
        0x20000000 | // SSOE enable peripheral control of SS
        0x00000000 | // Clock Idles low, samples on rising edge
        0x00400000; // MASTER
    SPI1->IFCR = 0x0BF8; // clear all flags
    SPI1->CR1 = 0x00000001;  // SPE Enable
    SPI1->CR1 = 0x00000201;  // CSTART Start
}

static void ADCInit(void)
{
/*
ADC1
    input 0, Joystick X
    input 1, Joystick Y
    input 14, Unused pin PA2
    input 15, Unused pin PA3
    input 17, Vrefint  1.216v
ADC2
    input 0, Joystick X
    input 1, Joystick Y
    input 14, Unused pin PA2
    input 15, Unused pin PA3
    input 16, Vbat
*/
    ADC12_COMMON->CCR =
        0x01000000 | // VBATEN
        0x00400000 | // VREFEN
        0x00030000;  // CKMODE clock == hclk/4 == 62.5MHz

    ADC1->CR = 0x00000000; // disable DEEPPWD
    ADC2->CR = 0x00000000;
    ADC1->CR = 0x10000000; // enable ADVREGEN
    ADC2->CR = 0x10000000;
    // wait for voltage regulator to stabilize 10us max

    uint32_t t0 = TimeMicroseconds();
    while (TimeMicroseconds()-t0 < 11)
    {}

    ADC1->ISR = 0x0001; // clear ADRDY
    ADC2->ISR = 0x0001;
    ADC1->CR = 0x10000001; // set ADEN
    ADC2->CR = 0x10000001;
    // wait for ADRDY
    while (((ADC1->ISR & 0x0001) == 0) || ((ADC2->ISR & 0x0001) == 0))
    {}

    ADC1->CFGR =
        //0x02000000 | // JAUTO
        0x00008000 | // ALIGN left
        0x00002000 | // CONT continuous conversion
        0x00001000;  // OVRMOD overwrite output register
    ADC2->CFGR = 0x0000B000;

    ADC1->CFGR2 = 0;
    ADC2->CFGR2 = 0;
    ADC1->SMPR1 = 5 + 5*8; // sample time for channel 0 (FIXME check)
    ADC2->SMPR1 = 5; // sample time for channel 0 (FIXME check)
    ADC1->SMPR2 = 5 << 21; // sample time for channel 17 (FIXME check)
    ADC1->SQR1 = 0 << 6; // Single sample from channel 0
    ADC2->SQR1 = 1 << 6; // Single sample from channel 1 // FIXME

    ADC1->CR = 0x10000005; // set ADSTART
    ADC2->CR = 0x10000005;

    // FIXME lay out ADC sources for collection
}

static void XRamInit(void)
{
    memset((char *)0x903FC000, 0, 0x4000); // XDRAM
    memset((char *)0x20043000, 0, 0x1000); // XSRAM
}

void CoreInit(void)
{
    SCB->VTOR = FLASH_BASE; // Set up interrupt vector table
    SCB->CPACR |= ((3UL << 20U)|(3UL << 22U)); // Enable Floating Point

    ClockInit();

    LowPowerInit();

    // Set up SysTick as a free-running 24 bit counter
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |  // Use CPU clock as source
                    SysTick_CTRL_ENABLE_Msk;      // Enable SysTick Timer
    CacheInit();

    GPIOInit();
    TimerInit();
    OctoSPIInit();
    SPIInit();
    ADCInit();

    // USB
    PWR->USBSCR = PWR_USBSCR_USB33SV;

    XRamInit();
}

uint32_t TimeMicroseconds(void)
{
    return TIM2->CNT;
}

bool ButtonRead(int n)
{
    // Button1 PC13
    // Button2 PB8
    // Button3 PB7
    // Button4 PH1

    static const GPIO_TypeDef *ports[4] =
    {
        GPIOC,
        GPIOB,
        GPIOB,
        GPIOH
    };

    static const uint32_t pins[4] =
    {
        0x2000,
        0x0100,
        0x0080,
        0x0002
    };

    const GPIO_TypeDef *gpio = ports[n];
    uint32_t pin = pins[n];

    return (gpio->IDR & pin) != 0;
}

void LEDWrite(int n, int val)
{
    if (n == 0)
        TIM1->CCR2 = val;
    if (n == 1)
        TIM1->CCR3 = val;
}

uint32_t ADCRead(int n)
{
    // FIXME read from all ADC sources
    // FIXME put battery state of charge measurement somewhere
    // FIXME fake
    if (n==1)
        return ADC1->DR;
    else
        return ADC2->DR;
}


void DelayClocks(uint32_t n)
{
    uint32_t start = SysTick->VAL;
    uint32_t t;
    do
    {
        t = SysTick->VAL;
    }
    while (((start - t) & 0x00FFFFFF) < n);
}

