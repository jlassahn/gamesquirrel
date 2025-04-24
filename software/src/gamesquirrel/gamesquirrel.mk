
GS_SRCS := \
external/STM32H5xx/Source/Templates/system_stm32h5xx.c \
src/gamesquirrel/main.c


GS_OBJS := $(addprefix build/obj/, $(addsuffix .o, $(basename $(GS_SRCS))))

build/gamesquirrel.elf: $(GS_OBJS)
	$(CC) $(CC_OPTS) $(LD_OPTS) \
	-Wl,-Map=build/gamesquirrel.elf.map \
	-o build/gamesquirrel.elf \
	external/STM32H5xx/Source/Templates/gcc/startup_stm32h503xx.s \
	$(GS_OBJS) \
	$(LIBS)


