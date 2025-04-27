
GS_SRCS := \
external/STM32H5xx/Source/Templates/system_stm32h5xx.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart_ex.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c \
external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c \
src/gamesquirrel/main.c


GS_OBJS := $(addprefix build/obj/, $(addsuffix .o, $(basename $(GS_SRCS))))

build/gamesquirrel.elf: $(GS_OBJS)
	$(CC) $(CC_OPTS) $(LD_OPTS) \
	-Wl,-Map=build/gamesquirrel.elf.map \
	-o build/gamesquirrel.elf \
	external/STM32H5xx/Source/Templates/gcc/startup_stm32h503xx.s \
	$(GS_OBJS) \
	$(LIBS)


