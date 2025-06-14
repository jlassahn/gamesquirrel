
GS_SRCS := \
src/gamesquirrel/boot.s \
src/gamesquirrel/system_stm32h5xx.c \
src/gamesquirrel/interrupt_vectors.c \
src/gamesquirrel/core.c \
src/gamesquirrel/usb.c \
src/gamesquirrel/audio.c \
src/gamesquirrel/display.c \
src/gamesquirrel/sd_card.c \
src/gamesquirrel/main.c

GS_OBJS := $(addprefix build/obj/armsys/, $(addsuffix .o, $(basename $(GS_SRCS))))

build/gamesquirrel.elf: $(GS_OBJS)
	$(CC_ARMSYS) $(CC_ARMSYS_OPTS) $(LD_ARMSYS_OPTS) \
	-Wl,-Map=$@.map \
	-Wl,-T,src/gamesquirrel/linkerscript.ld \
	-o $@ \
	$(GS_OBJS) \
	$(LIBS_ARMSYS)

