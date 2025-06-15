/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#include <stdint.h>
#include <stdbool.h>

// FIXME better handling of unexpected interrupts
void DefaultHandler(void)
{
    while (true)
    {}
}

typedef struct InterruptVectors InterruptVectors;
struct InterruptVectors
{
    void *stack;
    void (*Reset)(void);
    void (*NMI)(void);
    void (*HardFault)(void);
    void (*MemManage)(void);
    void (*BusFault)(void);
    void (*UsageFault)(void);
    void (*SecureFault)(void);
    void *pad00;
    void *pad01;
    void *pad02;
    void (*SVC)(void);
    void (*DebugMon)(void);
    void *pad03;
    void (*PendSV)(void);
    void (*SysTick)(void);
    void (*WWDG)(void);
    void (*PVD_AVD)(void);
    void (*RTC)(void);
    void (*RTC_S)(void);
    void (*TAMP)(void);
    void (*RAMCFG)(void);
    void (*FLASH)(void);
    void (*FLASH_S)(void);
    void (*GTZC)(void);
    void (*RCC)(void);
    void (*RCC_S)(void);
    void (*EXTI0)(void);
    void (*EXTI1)(void);
    void (*EXTI2)(void);
    void (*EXTI3)(void);
    void (*EXTI4)(void);
    void (*EXTI5)(void);
    void (*EXTI6)(void);
    void (*EXTI7)(void);
    void (*EXTI8)(void);
    void (*EXTI9)(void);
    void (*EXTI10)(void);
    void (*EXTI11)(void);
    void (*EXTI12)(void);
    void (*EXTI13)(void);
    void (*EXTI14)(void);
    void (*EXTI15)(void);
    void (*GPDMA1_Channel0)(void);
    void (*GPDMA1_Channel1)(void);
    void (*GPDMA1_Channel2)(void);
    void (*GPDMA1_Channel3)(void);
    void (*GPDMA1_Channel4)(void);
    void (*GPDMA1_Channel5)(void);
    void (*GPDMA1_Channel6)(void);
    void (*GPDMA1_Channel7)(void);
    void (*IWDG)(void);
    void *pad04;
    void (*ADC1)(void);
    void (*DAC1)(void);
    void (*FDCAN1_IT0)(void);
    void (*FDCAN1_IT1)(void);
    void (*TIM1_BRK)(void);
    void (*TIM1_UP)(void);
    void (*TIM1_TRG_COM)(void);
    void (*TIM1_CC)(void);
    void (*TIM2)(void);
    void (*TIM3)(void);
    void (*TIM4)(void);
    void (*TIM5)(void);
    void (*TIM6)(void);
    void (*TIM7)(void);
    void (*I2C1_EV)(void);
    void (*I2C1_ER)(void);
    void (*I2C2_EV)(void);
    void (*I2C2_ER)(void);
    void (*SPI1)(void);
    void (*SPI2)(void);
    void (*SPI3)(void);
    void (*USART1)(void);
    void (*USART2)(void);
    void (*USART3)(void);
    void (*UART4)(void);
    void (*UART5)(void);
    void (*LPUART1)(void);
    void (*LPTIM1)(void);
    void (*TIM8_BRK)(void);
    void (*TIM8_UP)(void);
    void (*TIM8_TRG_COM)(void);
    void (*TIM8_CC)(void);
    void (*ADC2)(void);
    void (*LPTIM2)(void);
    void (*TIM15)(void);
    void *pad05;
    void *pad06;
    void (*USB_DRD_FS)(void);
    void (*CRS)(void);
    void (*UCPD1)(void);
    void (*FMC)(void);
    void (*OCTOSPI1)(void);
    void (*SDMMC1)(void);
    void (*I2C3_EV)(void);
    void (*I2C3_ER)(void);
    void (*SPI4)(void);
    void *pad07;
    void *pad08;
    void (*USART6)(void);
    void *pad09;
    void *pad10;
    void *pad11;
    void *pad12;
    void (*GPDMA2_Channel0)(void);
    void (*GPDMA2_Channel1)(void);
    void (*GPDMA2_Channel2)(void);
    void (*GPDMA2_Channel3)(void);
    void (*GPDMA2_Channel4)(void);
    void (*GPDMA2_Channel5)(void);
    void (*GPDMA2_Channel6)(void);
    void (*GPDMA2_Channel7)(void);
    void *pad13;
    void *pad14;
    void *pad15;
    void *pad16;
    void *pad17;
    void (*FPU)(void);
    void (*ICACHE)(void);
    void (*DCACHE)(void);
    void *pad18;
    void *pad19;
    void (*DCMI_PSSI)(void);
    void (*FDCAN2_IT0)(void);
    void (*FDCAN2_IT1)(void);
    void *pad20;
    void *pad21;
    void (*DTS)(void);
    void (*RNG)(void);
    void *pad22;
    void *pad23;
    void (*HASH)(void);
    void *pad24;
    void (*CEC)(void);
    void (*TIM12)(void);
    void *pad25;
    void *pad26;
    void (*I3C1_EV)(void);
    void (*I3C1_ER)(void);
    void *pad27;
    void *pad28;
    void *pad29;
    void *pad30;
    void *pad31;
    void *pad32;
    void (*I3C2_EV)(void);
    void (*I3C2_ER)(void);
};

extern uint32_t _estack;
extern void Reset_Handler(void);
extern void HardFault_Handler(void);
extern void SysTick_Handler(void); // FIXME remove?
extern void GPDMA1_Channel5_IRQHandler(void);
extern void USB_DRD_FS_IRQHandler(void);

const InterruptVectors interrupt_vectors __attribute__((section (".isr_vector"))) __attribute__((used)) =
{
    .stack = &_estack,
    .Reset = Reset_Handler,
    .NMI = DefaultHandler,
    .HardFault = HardFault_Handler,
    .MemManage = DefaultHandler,
    .BusFault = DefaultHandler,
    .UsageFault = DefaultHandler,
    .SecureFault = DefaultHandler,
    .SVC = DefaultHandler,
    .DebugMon = DefaultHandler,
    .PendSV = DefaultHandler,
    .SysTick = SysTick_Handler,
    .WWDG = DefaultHandler,
    .PVD_AVD = DefaultHandler,
    .RTC = DefaultHandler,
    .RTC_S = DefaultHandler,
    .TAMP = DefaultHandler,
    .RAMCFG = DefaultHandler,
    .FLASH = DefaultHandler,
    .FLASH_S = DefaultHandler,
    .GTZC = DefaultHandler,
    .RCC = DefaultHandler,
    .RCC_S = DefaultHandler,
    .EXTI0 = DefaultHandler,
    .EXTI1 = DefaultHandler,
    .EXTI2 = DefaultHandler,
    .EXTI3 = DefaultHandler,
    .EXTI4 = DefaultHandler,
    .EXTI5 = DefaultHandler,
    .EXTI6 = DefaultHandler,
    .EXTI7 = DefaultHandler,
    .EXTI8 = DefaultHandler,
    .EXTI9 = DefaultHandler,
    .EXTI10 = DefaultHandler,
    .EXTI11 = DefaultHandler,
    .EXTI12 = DefaultHandler,
    .EXTI13 = DefaultHandler,
    .EXTI14 = DefaultHandler,
    .EXTI15 = DefaultHandler,
    .GPDMA1_Channel0 = DefaultHandler,
    .GPDMA1_Channel1 = DefaultHandler,
    .GPDMA1_Channel2 = DefaultHandler,
    .GPDMA1_Channel3 = DefaultHandler,
    .GPDMA1_Channel4 = DefaultHandler,
    .GPDMA1_Channel5 = GPDMA1_Channel5_IRQHandler,
    .GPDMA1_Channel6 = DefaultHandler,
    .GPDMA1_Channel7 = DefaultHandler,
    .IWDG = DefaultHandler,
    .ADC1 = DefaultHandler,
    .DAC1 = DefaultHandler,
    .FDCAN1_IT0 = DefaultHandler,
    .FDCAN1_IT1 = DefaultHandler,
    .TIM1_BRK = DefaultHandler,
    .TIM1_UP = DefaultHandler,
    .TIM1_TRG_COM = DefaultHandler,
    .TIM1_CC = DefaultHandler,
    .TIM2 = DefaultHandler,
    .TIM3 = DefaultHandler,
    .TIM4 = DefaultHandler,
    .TIM5 = DefaultHandler,
    .TIM6 = DefaultHandler,
    .TIM7 = DefaultHandler,
    .I2C1_EV = DefaultHandler,
    .I2C1_ER = DefaultHandler,
    .I2C2_EV = DefaultHandler,
    .I2C2_ER = DefaultHandler,
    .SPI1 = DefaultHandler,
    .SPI2 = DefaultHandler,
    .SPI3 = DefaultHandler,
    .USART1 = DefaultHandler,
    .USART2 = DefaultHandler,
    .USART3 = DefaultHandler,
    .UART4 = DefaultHandler,
    .UART5 = DefaultHandler,
    .LPUART1 = DefaultHandler,
    .LPTIM1 = DefaultHandler,
    .TIM8_BRK = DefaultHandler,
    .TIM8_UP = DefaultHandler,
    .TIM8_TRG_COM = DefaultHandler,
    .TIM8_CC = DefaultHandler,
    .ADC2 = DefaultHandler,
    .LPTIM2 = DefaultHandler,
    .TIM15 = DefaultHandler,
    .USB_DRD_FS = USB_DRD_FS_IRQHandler,
    .CRS = DefaultHandler,
    .UCPD1 = DefaultHandler,
    .FMC = DefaultHandler,
    .OCTOSPI1 = DefaultHandler,
    .SDMMC1 = DefaultHandler,
    .I2C3_EV = DefaultHandler,
    .I2C3_ER = DefaultHandler,
    .SPI4 = DefaultHandler,
    .USART6 = DefaultHandler,
    .GPDMA2_Channel0 = DefaultHandler,
    .GPDMA2_Channel1 = DefaultHandler,
    .GPDMA2_Channel2 = DefaultHandler,
    .GPDMA2_Channel3 = DefaultHandler,
    .GPDMA2_Channel4 = DefaultHandler,
    .GPDMA2_Channel5 = DefaultHandler,
    .GPDMA2_Channel6 = DefaultHandler,
    .GPDMA2_Channel7 = DefaultHandler,
    .FPU = DefaultHandler,
    .ICACHE = DefaultHandler,
    .DCACHE = DefaultHandler,
    .DCMI_PSSI = DefaultHandler,
    .FDCAN2_IT0 = DefaultHandler,
    .FDCAN2_IT1 = DefaultHandler,
    .DTS = DefaultHandler,
    .RNG = DefaultHandler,
    .HASH = DefaultHandler,
    .CEC = DefaultHandler,
    .TIM12 = DefaultHandler,
    .I3C1_EV = DefaultHandler,
    .I3C1_ER = DefaultHandler,
    .I3C2_EV = DefaultHandler,
    .I3C2_ER = DefaultHandler
};

