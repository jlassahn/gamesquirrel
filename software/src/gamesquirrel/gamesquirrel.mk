
GS_SRCS := \
external/STM32H5xx/Source/Templates/gcc/startup_stm32h523xx.s \
external/STM32H5xx/Source/Templates/system_stm32h5xx.c \
src/gamesquirrel/usb.c \
src/gamesquirrel/main.c


GS_OBJS := $(addprefix build/obj/armsys/, $(addsuffix .o, $(basename $(GS_SRCS))))

build/gamesquirrel.elf: $(GS_OBJS)
	$(CC_ARMSYS) $(CC_ARMSYS_OPTS) $(LD_ARMSYS_OPTS) \
	-Wl,-Map=$@.map \
	-o $@ \
	$(GS_OBJS) \
	$(LIBS_ARMSYS)


