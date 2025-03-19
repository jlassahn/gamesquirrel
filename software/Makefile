
CC = arm-none-eabi-gcc

BASE = ../external/tinyusb

CC_OPTS = \
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
-I$(BASE)/src \
-I../external/CMSIS/Core/Include \
-I../external/STM32H5xx/Include \
-I../external/STM32H5xx_HAL_Driver/Inc \
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
$(BASE)/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
$(BASE)/src/tusb.c \
$(BASE)/src/common/tusb_fifo.c \
$(BASE)/src/device/usbd.c \
$(BASE)/src/device/usbd_control.c \
$(BASE)/src/typec/usbc.c \
$(BASE)/src/class/audio/audio_device.c \
$(BASE)/src/class/cdc/cdc_device.c \
$(BASE)/src/class/dfu/dfu_device.c \
$(BASE)/src/class/dfu/dfu_rt_device.c \
$(BASE)/src/class/hid/hid_device.c \
$(BASE)/src/class/midi/midi_device.c \
$(BASE)/src/class/msc/msc_device.c \
$(BASE)/src/class/net/ecm_rndis_device.c \
$(BASE)/src/class/net/ncm_device.c \
$(BASE)/src/class/usbtmc/usbtmc_device.c \
$(BASE)/src/class/video/video_device.c \
$(BASE)/src/class/vendor/vendor_device.c \
$(BASE)/src/host/usbh.c \
$(BASE)/src/host/hub.c \
$(BASE)/src/class/cdc/cdc_host.c \
$(BASE)/src/class/hid/hid_host.c \
$(BASE)/src/class/msc/msc_host.c \
$(BASE)/src/class/vendor/vendor_host.c \
../external/STM32H5xx/Source/Templates/gcc/startup_stm32h503xx.s \
../external/STM32H5xx/Source/Templates/system_stm32h5xx.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart_ex.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c \
../external/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c \
src/usb_example/board.c \
src/usb_example/family.c \
src/usb_example/main.c \
src/usb_example/msc_disk.c \
src/usb_example/usb_descriptors.c


LIBS = \
-lgcc \
-lm \
-lc \
-lnosys

all: cdc_msc.elf cdc_msc.bin

cdc_msc.elf:
	$(CC) $(CC_OPTS) $(LD_OPTS) -o cdc_msc.elf $(SRCS) $(LIBS)

cdc_msc.bin: cdc_msc.elf
	arm-none-eabi-objcopy -O binary cdc_msc.elf cdc_msc.bin
