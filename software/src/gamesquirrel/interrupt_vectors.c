
#include <stdint.h>

typedef struct InterruptVectors InterruptVectors;
struct InterruptVectors
{
	void *stack; //.word	_estack
	void (*Reset)(void); //.word	Reset_Handler
	void (*NMI)(void); //.word	NMI_Handler
	void (*HardFault)(void); //.word	HardFault_Handler
	void (*MemManage)(void); //.word	MemManage_Handler
	void (*BusFault)(void); //.word	BusFault_Handler
	void (*UsageFault)(void); //.word	UsageFault_Handler
	void (*SecureFault)(void); //.word	SecureFault_Handler
	void *pad00; //.word	0
	void *pad01; //.word	0
	void *pad02; //.word	0
	void (*SVC)(void); //.word	SVC_Handler
	void (*DebugMon)(void); //.word	DebugMon_Handler
	void *pad03; //.word	0
	void (*PendSV)(void); //.word	PendSV_Handler
	void (*SysTick)(void); //.word	SysTick_Handler
	void (*WWDG)(void); //.word	WWDG_IRQHandler
	void (*PVD_AVD)(void); //.word	PVD_AVD_IRQHandler
	void (*RTC)(void); //.word	RTC_IRQHandler
	void (*RTC_S)(void); //.word	RTC_S_IRQHandler
	void (*TAMP)(void); //.word	TAMP_IRQHandler
	void (*RAMCFG)(void); //.word	RAMCFG_IRQHandler
	void (*FLASH)(void); //.word	FLASH_IRQHandler
	void (*FLASH_S)(void); //.word	FLASH_S_IRQHandler
	void (*GTZC)(void); //.word	GTZC_IRQHandler
	void (*RCC)(void); //.word	RCC_IRQHandler
	void (*RCC_S)(void); //.word	RCC_S_IRQHandler
	void (*EXTI0)(void); //.word	EXTI0_IRQHandler
	void (*EXTI1)(void); //.word	EXTI1_IRQHandler
	void (*EXTI2)(void); //.word	EXTI2_IRQHandler
	void (*EXTI3)(void); //.word	EXTI3_IRQHandler
	void (*EXTI4)(void); //.word	EXTI4_IRQHandler
	void (*EXTI5)(void); //.word	EXTI5_IRQHandler
	void (*EXTI6)(void); //.word	EXTI6_IRQHandler
	void (*EXTI7)(void); //.word	EXTI7_IRQHandler
	void (*EXTI8)(void); //.word	EXTI8_IRQHandler
	void (*EXTI9)(void); //.word	EXTI9_IRQHandler
	void (*EXTI10)(void); //.word	EXTI10_IRQHandler
	void (*EXTI11)(void); //.word	EXTI11_IRQHandler
	void (*EXTI12)(void); //.word	EXTI12_IRQHandler
	void (*EXTI13)(void); //.word	EXTI13_IRQHandler
	void (*EXTI14)(void); //.word	EXTI14_IRQHandler
	void (*EXTI15)(void); //.word	EXTI15_IRQHandler
	void (*GPDMA1_Channel0)(void); //.word	GPDMA1_Channel0_IRQHandler
	void (*GPDMA1_Channel1)(void); //.word	GPDMA1_Channel1_IRQHandler
	void (*GPDMA1_Channel2)(void); //.word	GPDMA1_Channel2_IRQHandler
	void (*GPDMA1_Channel3)(void); //.word	GPDMA1_Channel3_IRQHandler
	void (*GPDMA1_Channel4)(void); //.word	GPDMA1_Channel4_IRQHandler
	void (*GPDMA1_Channel5)(void); //.word	GPDMA1_Channel5_IRQHandler
	void (*GPDMA1_Channel6)(void); //.word	GPDMA1_Channel6_IRQHandler
	void (*GPDMA1_Channel7)(void); //.word	GPDMA1_Channel7_IRQHandler
	void (*IWDG)(void); //.word	IWDG_IRQHandler
	void *pad04; //.word	0
	void (*ADC1)(void); //.word	ADC1_IRQHandler
	void (*DAC1)(void); //.word	DAC1_IRQHandler
	void (*FDCAN1_IT0)(void); //.word	FDCAN1_IT0_IRQHandler
	void (*FDCAN1_IT1)(void); //.word	FDCAN1_IT1_IRQHandler
	void (*TIM1_BRK)(void); //.word	TIM1_BRK_IRQHandler
	void (*TIM1_UP)(void); //.word	TIM1_UP_IRQHandler
	void (*TIM1_TRG_COM)(void); //.word	TIM1_TRG_COM_IRQHandler
	void (*TIM1_CC)(void); //.word	TIM1_CC_IRQHandler
	void (*TIM2)(void); //.word	TIM2_IRQHandler
	void (*TIM3)(void); //.word	TIM3_IRQHandler
	void (*TIM4)(void); //.word	TIM4_IRQHandler
	void (*TIM5)(void); //.word	TIM5_IRQHandler
	void (*TIM6)(void); //.word	TIM6_IRQHandler
	void (*TIM7)(void); //.word	TIM7_IRQHandler
	void (*I2C1_EV)(void); //.word	I2C1_EV_IRQHandler
	void (*I2C1_ER)(void); //.word	I2C1_ER_IRQHandler
	void (*I2C2_EV)(void); //.word	I2C2_EV_IRQHandler
	void (*I2C2_ER)(void); //.word	I2C2_ER_IRQHandler
	void (*SPI1)(void); //.word	SPI1_IRQHandler
	void (*SPI2)(void); //.word	SPI2_IRQHandler
	void (*SPI3)(void); //.word	SPI3_IRQHandler
	void (*USART1)(void); //.word	USART1_IRQHandler
	void (*USART2)(void); //.word	USART2_IRQHandler
	void (*USART3)(void); //.word	USART3_IRQHandler
	void (*UART4)(void); //.word	UART4_IRQHandler
	void (*UART5)(void); //.word	UART5_IRQHandler
	void (*LPUART1)(void); //.word	LPUART1_IRQHandler
	void (*LPTIM1)(void); //.word	LPTIM1_IRQHandler
	void (*TIM8_BRK)(void); //.word	TIM8_BRK_IRQHandler
	void (*TIM8_UP)(void); //.word	TIM8_UP_IRQHandler
	void (*TIM8_TRG_COM)(void); //.word	TIM8_TRG_COM_IRQHandler
	void (*TIM8_CC)(void); //.word	TIM8_CC_IRQHandler
	void (*ADC2)(void); //.word	ADC2_IRQHandler
	void (*LPTIM2)(void); //.word	LPTIM2_IRQHandler
	void (*TIM15)(void); //.word	TIM15_IRQHandler
	void *pad05; //.word	0
	void *pad06; //.word	0
	void (*USB_DRD_FS)(void); //.word	USB_DRD_FS_IRQHandler
	void (*CRS)(void); //.word	CRS_IRQHandler
	void (*UCPD1)(void); //.word	UCPD1_IRQHandler
	void (*FMC)(void); //.word	FMC_IRQHandler
	void (*OCTOSPI1)(void); //.word	OCTOSPI1_IRQHandler
	void (*SDMMC1)(void); //.word	SDMMC1_IRQHandler
	void (*I2C3_EV)(void); //.word	I2C3_EV_IRQHandler
	void (*I2C3_ER)(void); //.word	I2C3_ER_IRQHandler
	void (*SPI4)(void); //.word	SPI4_IRQHandler
	void *pad07; //.word	0
	void *pad08; //.word	0
	void (*USART6)(void); //.word	USART6_IRQHandler
	void *pad09; //.word	0
	void *pad10; //.word	0
	void *pad11; //.word	0
	void *pad12; //.word	0
	void (*GPDMA2_Channel0)(void); //.word	GPDMA2_Channel0_IRQHandler
	void (*GPDMA2_Channel1)(void); //.word	GPDMA2_Channel1_IRQHandler
	void (*GPDMA2_Channel2)(void); //.word	GPDMA2_Channel2_IRQHandler
	void (*GPDMA2_Channel3)(void); //.word	GPDMA2_Channel3_IRQHandler
	void (*GPDMA2_Channel4)(void); //.word	GPDMA2_Channel4_IRQHandler
	void (*GPDMA2_Channel5)(void); //.word	GPDMA2_Channel5_IRQHandler
	void (*GPDMA2_Channel6)(void); //.word	GPDMA2_Channel6_IRQHandler
	void (*GPDMA2_Channel7)(void); //.word	GPDMA2_Channel7_IRQHandler
	void *pad13; //.word	0
	void *pad14; //.word	0
	void *pad15; //.word	0
	void *pad16; //.word	0
	void *pad17; //.word	0
	void (*FPU)(void); //.word	FPU_IRQHandler
	void (*ICACHE)(void); //.word	ICACHE_IRQHandler
	void (*DCACHE)(void); //.word	DCACHE1_IRQHandler
	void *pad18; //.word	0
	void *pad19; //.word	0
	void (*DCMI_PSSI)(void); //.word	DCMI_PSSI_IRQHandler
	void (*FDCAN2_IT0)(void); //.word	FDCAN2_IT0_IRQHandler
	void (*FDCAN2_IT1)(void); //.word	FDCAN2_IT1_IRQHandler
	void *pad20; //.word	0
	void *pad21; //.word	0
	void (*DTS)(void); //.word	DTS_IRQHandler
	void (*RNG)(void); //.word	RNG_IRQHandler
	void *pad22; //.word	0
	void *pad23; //.word	0
	void (*HASH)(void); //.word	HASH_IRQHandler
	void *pad24; //.word	0
	void (*CEC)(void); //.word	CEC_IRQHandler
	void (*TIM12)(void); //.word	TIM12_IRQHandler
	void *pad25; //.word	0
	void *pad26; //.word	0
	void (*I3C1_EV)(void); //.word	I3C1_EV_IRQHandler
	void (*I3C1_ER)(void); //.word	I3C1_ER_IRQHandler
	void *pad27; //.word	0
	void *pad28; //.word	0
	void *pad29; //.word	0
	void *pad30; //.word	0
	void *pad31; //.word	0
	void *pad32; //.word	0
	void (*I3C2_EV)(void); //.word	I3C2_EV_IRQHandler
	void (*I3C2_ER)(void); //.word	I3C2_ER_IRQHandler
};

extern uint32_t _estack;
extern void Reset_Handler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SecureFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
extern void WWDG_IRQHandler(void);
extern void PVD_AVD_IRQHandler(void);
extern void RTC_IRQHandler(void);
extern void RTC_S_IRQHandler(void);
extern void TAMP_IRQHandler(void);
extern void RAMCFG_IRQHandler(void);
extern void FLASH_IRQHandler(void);
extern void FLASH_S_IRQHandler(void);
extern void GTZC_IRQHandler(void);
extern void RCC_IRQHandler(void);
extern void RCC_S_IRQHandler(void);
extern void EXTI0_IRQHandler(void);
extern void EXTI1_IRQHandler(void);
extern void EXTI2_IRQHandler(void);
extern void EXTI3_IRQHandler(void);
extern void EXTI4_IRQHandler(void);
extern void EXTI5_IRQHandler(void);
extern void EXTI6_IRQHandler(void);
extern void EXTI7_IRQHandler(void);
extern void EXTI8_IRQHandler(void);
extern void EXTI9_IRQHandler(void);
extern void EXTI10_IRQHandler(void);
extern void EXTI11_IRQHandler(void);
extern void EXTI12_IRQHandler(void);
extern void EXTI13_IRQHandler(void);
extern void EXTI14_IRQHandler(void);
extern void EXTI15_IRQHandler(void);
extern void GPDMA1_Channel0_IRQHandler(void);
extern void GPDMA1_Channel1_IRQHandler(void);
extern void GPDMA1_Channel2_IRQHandler(void);
extern void GPDMA1_Channel3_IRQHandler(void);
extern void GPDMA1_Channel4_IRQHandler(void);
extern void GPDMA1_Channel5_IRQHandler(void);
extern void GPDMA1_Channel6_IRQHandler(void);
extern void GPDMA1_Channel7_IRQHandler(void);
extern void IWDG_IRQHandler(void);
extern void ADC1_IRQHandler(void);
extern void DAC1_IRQHandler(void);
extern void FDCAN1_IT0_IRQHandler(void);
extern void FDCAN1_IT1_IRQHandler(void);
extern void TIM1_BRK_IRQHandler(void);
extern void TIM1_UP_IRQHandler(void);
extern void TIM1_TRG_COM_IRQHandler(void);
extern void TIM1_CC_IRQHandler(void);
extern void TIM2_IRQHandler(void);
extern void TIM3_IRQHandler(void);
extern void TIM4_IRQHandler(void);
extern void TIM5_IRQHandler(void);
extern void TIM6_IRQHandler(void);
extern void TIM7_IRQHandler(void);
extern void I2C1_EV_IRQHandler(void);
extern void I2C1_ER_IRQHandler(void);
extern void I2C2_EV_IRQHandler(void);
extern void I2C2_ER_IRQHandler(void);
extern void SPI1_IRQHandler(void);
extern void SPI2_IRQHandler(void);
extern void SPI3_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void USART3_IRQHandler(void);
extern void UART4_IRQHandler(void);
extern void UART5_IRQHandler(void);
extern void LPUART1_IRQHandler(void);
extern void LPTIM1_IRQHandler(void);
extern void TIM8_BRK_IRQHandler(void);
extern void TIM8_UP_IRQHandler(void);
extern void TIM8_TRG_COM_IRQHandler(void);
extern void TIM8_CC_IRQHandler(void);
extern void ADC2_IRQHandler(void);
extern void LPTIM2_IRQHandler(void);
extern void TIM15_IRQHandler(void);
extern void USB_DRD_FS_IRQHandler(void);
extern void CRS_IRQHandler(void);
extern void UCPD1_IRQHandler(void);
extern void FMC_IRQHandler(void);
extern void OCTOSPI1_IRQHandler(void);
extern void SDMMC1_IRQHandler(void);
extern void I2C3_EV_IRQHandler(void);
extern void I2C3_ER_IRQHandler(void);
extern void SPI4_IRQHandler(void);
extern void USART6_IRQHandler(void);
extern void GPDMA2_Channel0_IRQHandler(void);
extern void GPDMA2_Channel1_IRQHandler(void);
extern void GPDMA2_Channel2_IRQHandler(void);
extern void GPDMA2_Channel3_IRQHandler(void);
extern void GPDMA2_Channel4_IRQHandler(void);
extern void GPDMA2_Channel5_IRQHandler(void);
extern void GPDMA2_Channel6_IRQHandler(void);
extern void GPDMA2_Channel7_IRQHandler(void);
extern void FPU_IRQHandler(void);
extern void ICACHE_IRQHandler(void);
extern void DCACHE1_IRQHandler(void);
extern void DCMI_PSSI_IRQHandler(void);
extern void FDCAN2_IT0_IRQHandler(void);
extern void FDCAN2_IT1_IRQHandler(void);
extern void DTS_IRQHandler(void);
extern void RNG_IRQHandler(void);
extern void HASH_IRQHandler(void);
extern void CEC_IRQHandler(void);
extern void TIM12_IRQHandler(void);
extern void I3C1_EV_IRQHandler(void);
extern void I3C1_ER_IRQHandler(void);
extern void I3C2_EV_IRQHandler(void);
extern void I3C2_ER_IRQHandler(void);

const InterruptVectors interrupt_vectors __attribute__((section (".isr_vector"))) __attribute__((used)) =
{
	.stack = &_estack,
	.Reset = Reset_Handler, //keep
	// .NMI = NMI_Handler,
	.HardFault = HardFault_Handler, // keep
	// .MemManage = MemManage_Handler,
	// .BusFault = BusFault_Handler,
	// .UsageFault = UsageFault_Handler,
	// .SecureFault = SecureFault_Handler,
	// .SVC = SVC_Handler,
	// .DebugMon = DebugMon_Handler,
	// .PendSV = PendSV_Handler,
	.SysTick = SysTick_Handler, // keep
	// .WWDG = WWDG_IRQHandler,
	// .PVD_AVD = PVD_AVD_IRQHandler,
	// .RTC = RTC_IRQHandler,
	// .RTC_S = RTC_S_IRQHandler,
	// .TAMP = TAMP_IRQHandler,
	// .RAMCFG = RAMCFG_IRQHandler,
	// .FLASH = FLASH_IRQHandler,
	// .FLASH_S = FLASH_S_IRQHandler,
	// .GTZC = GTZC_IRQHandler,
	// .RCC = RCC_IRQHandler,
	// .RCC_S = RCC_S_IRQHandler,
	// .EXTI0 = EXTI0_IRQHandler,
	// .EXTI1 = EXTI1_IRQHandler,
	// .EXTI2 = EXTI2_IRQHandler,
	// .EXTI3 = EXTI3_IRQHandler,
	// .EXTI4 = EXTI4_IRQHandler,
	// .EXTI5 = EXTI5_IRQHandler,
	// .EXTI6 = EXTI6_IRQHandler,
	// .EXTI7 = EXTI7_IRQHandler,
	// .EXTI8 = EXTI8_IRQHandler,
	// .EXTI9 = EXTI9_IRQHandler,
	// .EXTI10 = EXTI10_IRQHandler,
	// .EXTI11 = EXTI11_IRQHandler,
	// .EXTI12 = EXTI12_IRQHandler,
	// .EXTI13 = EXTI13_IRQHandler,
	// .EXTI14 = EXTI14_IRQHandler,
	// .EXTI15 = EXTI15_IRQHandler,
	// .GPDMA1_Channel0 = GPDMA1_Channel0_IRQHandler,
	// .GPDMA1_Channel1 = GPDMA1_Channel1_IRQHandler,
	// .GPDMA1_Channel2 = GPDMA1_Channel2_IRQHandler,
	// .GPDMA1_Channel3 = GPDMA1_Channel3_IRQHandler,
	// .GPDMA1_Channel4 = GPDMA1_Channel4_IRQHandler,
	.GPDMA1_Channel5 = GPDMA1_Channel5_IRQHandler, //keep
	// .GPDMA1_Channel6 = GPDMA1_Channel6_IRQHandler,
	// .GPDMA1_Channel7 = GPDMA1_Channel7_IRQHandler,
	// .IWDG = IWDG_IRQHandler,
	// .ADC1 = ADC1_IRQHandler,
	// .DAC1 = DAC1_IRQHandler,
	// .FDCAN1_IT0 = FDCAN1_IT0_IRQHandler,
	// .FDCAN1_IT1 = FDCAN1_IT1_IRQHandler,
	// .TIM1_BRK = TIM1_BRK_IRQHandler,
	// .TIM1_UP = TIM1_UP_IRQHandler,
	// .TIM1_TRG_COM = TIM1_TRG_COM_IRQHandler,
	// .TIM1_CC = TIM1_CC_IRQHandler,
	// .TIM2 = TIM2_IRQHandler,
	// .TIM3 = TIM3_IRQHandler,
	// .TIM4 = TIM4_IRQHandler,
	// .TIM5 = TIM5_IRQHandler,
	// .TIM6 = TIM6_IRQHandler,
	// .TIM7 = TIM7_IRQHandler,
	// .I2C1_EV = I2C1_EV_IRQHandler,
	// .I2C1_ER = I2C1_ER_IRQHandler,
	// .I2C2_EV = I2C2_EV_IRQHandler,
	// .I2C2_ER = I2C2_ER_IRQHandler,
	// .SPI1 = SPI1_IRQHandler,
	// .SPI2 = SPI2_IRQHandler,
	// .SPI3 = SPI3_IRQHandler,
	// .USART1 = USART1_IRQHandler,
	// .USART2 = USART2_IRQHandler,
	// .USART3 = USART3_IRQHandler,
	// .UART4 = UART4_IRQHandler,
	// .UART5 = UART5_IRQHandler,
	// .LPUART1 = LPUART1_IRQHandler,
	// .LPTIM1 = LPTIM1_IRQHandler,
	// .TIM8_BRK = TIM8_BRK_IRQHandler,
	// .TIM8_UP = TIM8_UP_IRQHandler,
	// .TIM8_TRG_COM = TIM8_TRG_COM_IRQHandler,
	// .TIM8_CC = TIM8_CC_IRQHandler,
	// .ADC2 = ADC2_IRQHandler,
	// .LPTIM2 = LPTIM2_IRQHandler,
	// .TIM15 = TIM15_IRQHandler,
	.USB_DRD_FS = USB_DRD_FS_IRQHandler,  //keep
	// .CRS = CRS_IRQHandler,
	// .UCPD1 = UCPD1_IRQHandler,
	// .FMC = FMC_IRQHandler,
	// .OCTOSPI1 = OCTOSPI1_IRQHandler,
	// .SDMMC1 = SDMMC1_IRQHandler,
	// .I2C3_EV = I2C3_EV_IRQHandler,
	// .I2C3_ER = I2C3_ER_IRQHandler,
	// .SPI4 = SPI4_IRQHandler,
	// .USART6 = USART6_IRQHandler,
	// .GPDMA2_Channel0 = GPDMA2_Channel0_IRQHandler,
	// .GPDMA2_Channel1 = GPDMA2_Channel1_IRQHandler,
	// .GPDMA2_Channel2 = GPDMA2_Channel2_IRQHandler,
	// .GPDMA2_Channel3 = GPDMA2_Channel3_IRQHandler,
	// .GPDMA2_Channel4 = GPDMA2_Channel4_IRQHandler,
	// .GPDMA2_Channel5 = GPDMA2_Channel5_IRQHandler,
	// .GPDMA2_Channel6 = GPDMA2_Channel6_IRQHandler,
	// .GPDMA2_Channel7 = GPDMA2_Channel7_IRQHandler,
	// .FPU = FPU_IRQHandler,
	// .ICACHE = ICACHE_IRQHandler,
	// .DCACHE = DCACHE1_IRQHandler,
	// .DCMI_PSSI = DCMI_PSSI_IRQHandler,
	// .FDCAN2_IT0 = FDCAN2_IT0_IRQHandler,
	// .FDCAN2_IT1 = FDCAN2_IT1_IRQHandler,
	// .DTS = DTS_IRQHandler,
	// .RNG = RNG_IRQHandler,
	// .HASH = HASH_IRQHandler,
	// .CEC = CEC_IRQHandler,
	// .TIM12 = TIM12_IRQHandler,
	// .I3C1_EV = I3C1_EV_IRQHandler,
	// .I3C1_ER = I3C1_ER_IRQHandler,
	// .I3C2_EV = I3C2_EV_IRQHandler,
	// .I3C2_ER = I3C2_ER_IRQHandler
};

