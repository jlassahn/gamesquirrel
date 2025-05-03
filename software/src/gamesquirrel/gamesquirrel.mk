
GS_SRCS := \
external/STM32H5xx/Source/Templates/gcc/startup_stm32h503xx.s \
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
src/gamesquirrel/usb.c \
src/gamesquirrel/main.c


GS_OBJS := $(addprefix build/obj/armsys/, $(addsuffix .o, $(basename $(GS_SRCS))))

build/gamesquirrel.elf: $(GS_OBJS)
	$(CC_ARMSYS) $(CC_ARMSYS_OPTS) $(LD_ARMSYS_OPTS) \
	-Wl,-Map=$@.map \
	-o $@ \
	$(GS_OBJS) \
	$(LIBS_ARMSYS)


