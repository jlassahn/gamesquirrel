



CC := arm-none-eabi-gcc

CC_OPTS := \
-DSTM32H503xx \
-DHSE_VALUE=24000000 \
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
-Isrc/system \
-Isrc/usb_example


LD_OPTS = \
-Wl,-Map=cdc_msc.elf.map \
-Wl,--cref \
-Wl,-gc-sections \
-Wl,--print-memory-usage \
-Wl,--no-warn-rwx-segment \
-nostdlib \
-nostartfiles \
--specs=nosys.specs \
-Wl,-T,src/system/STM32H503xx_FLASH.ld


SRCS = \
external/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
external/tinyusb/src/tusb.c \
external/tinyusb/src/common/tusb_fifo.c \
external/tinyusb/src/device/usbd.c \
external/tinyusb/src/device/usbd_control.c \
external/tinyusb/src/typec/usbc.c \
external/tinyusb/src/class/audio/audio_device.c \
external/tinyusb/src/class/cdc/cdc_device.c \
external/tinyusb/src/class/dfu/dfu_device.c \
external/tinyusb/src/class/dfu/dfu_rt_device.c \
external/tinyusb/src/class/hid/hid_device.c \
external/tinyusb/src/class/midi/midi_device.c \
external/tinyusb/src/class/msc/msc_device.c \
external/tinyusb/src/class/net/ecm_rndis_device.c \
external/tinyusb/src/class/net/ncm_device.c \
external/tinyusb/src/class/usbtmc/usbtmc_device.c \
external/tinyusb/src/class/video/video_device.c \
external/tinyusb/src/class/vendor/vendor_device.c \
external/tinyusb/src/host/usbh.c \
external/tinyusb/src/host/hub.c \
external/tinyusb/src/class/cdc/cdc_host.c \
external/tinyusb/src/class/hid/hid_host.c \
external/tinyusb/src/class/msc/msc_host.c \
external/tinyusb/src/class/vendor/vendor_host.c \
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
src/usb_example/board.c \
src/usb_example/family.c \
src/usb_example/main.c \
src/usb_example/msc_disk.c \
src/usb_example/usb_descriptors.c

OBJS := $(addprefix build/obj/, $(addsuffix .o, $(basename $(SRCS))))

LIBS := \
-lgcc \
-lm \
-lc \
-lnosys

all: cdc_msc.elf cdc_msc.bin

cdc_msc.elf: $(OBJS)
	$(CC) $(CC_OPTS) $(LD_OPTS) -o cdc_msc.elf \
	external/STM32H5xx/Source/Templates/gcc/startup_stm32h503xx.s \
	$(OBJS) \
	$(LIBS)

cdc_msc.bin: cdc_msc.elf
	arm-none-eabi-objcopy -O binary cdc_msc.elf cdc_msc.bin


build/obj/%.o: %.c | Makefile
	mkdir -p $(@D)
	$(CC) $(CC_OPTS) -c -o $@ $^

