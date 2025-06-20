
CC_ARMSYS := arm-none-eabi-gcc

CC_ARMSYS_OPTS := \
-DSTM32H523xx \
-DHSE_VALUE=16000000 \
-DCFG_TUSB_MCU=OPT_MCU_STM32H5 \
-DBOARD_STM32H503NUCLEO \
-mthumb \
-mcpu=cortex-m33 \
-mfloat-abi=hard \
-mfpu=fpv5-sp-d16 \
-fsingle-precision-constant \
-MD \
-ggdb \
-fdata-sections \
-ffunction-sections \
-fno-strict-aliasing \
-Wall \
-Wextra \
-Werror \
-Wfatal-errors \
-Wdouble-promotion \
-Wstrict-prototypes \
-Wstrict-overflow \
-Werror-implicit-function-declaration \
-Wfloat-equal \
-Wundef \
-Wshadow \
-Wwrite-strings \
-Wsign-compare \
-Wmissing-format-attribute \
-Wunreachable-code \
-Wcast-align \
-Wcast-function-type \
-Wcast-qual \
-Wnull-dereference \
-Wuninitialized \
-Wunused \
-Wreturn-type \
-Wredundant-decls \
-Os \
-flto \
-Wno-error=cast-align \
-Wno-error=undef \
-Wno-error=unused-parameter \
-MD \
-Iexternal/tinyusb/src \
-Iexternal/CMSIS/Core/Include \
-Iexternal/STM32H5xx/Include \
-Iexternal/STM32H5xx_HAL_Driver/Inc \
-Isrc \
-Iinclude \
-Isrc/system \
-Isrc/usb_example


LD_ARMSYS_OPTS := \
-Wl,--cref \
-Wl,-gc-sections \
-Wl,--print-memory-usage \
-nostdlib \
-nostartfiles \
--specs=nosys.specs

LIBS_ARMSYS := \
-lgcc \
-lm \
-lc \
-lnosys

CC_HOST := gcc

CC_HOST_OPTS := \
-Wall \
-Wextra \
-Werror \
-O3 \
-g \
-I src \
-I include


LD_HOST_OPTS :=

LIBS_HOST := -lm -lc


build/%.bin: build/%.elf
	arm-none-eabi-objcopy -O binary $< $@

build/obj/armsys/%.o: %.c tools/compile_settings.mk
	mkdir -p $(@D)
	$(CC_ARMSYS) $(CC_ARMSYS_OPTS) -c -o $@ $<

build/obj/armsys/%.o: %.s tools/compile_settings.mk
	mkdir -p $(@D)
	$(CC_ARMSYS) $(CC_ARMSYS_OPTS) -c -o $@ $<

build/obj/host/%.o: %.c tools/compile_settings.mk
	mkdir -p $(@D)
	$(CC_HOST) $(CC_HOST_OPTS) -c -o $@ $<

build/include/%.h: include/%.h
	mkdir -p $(@D)
	cp $< $@

