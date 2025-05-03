
CDC_SRCS := \
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

CDC_OBJS := $(addprefix build/obj/armsys/, $(addsuffix .o, $(basename $(CDC_SRCS))))

build/cdc_msc.elf: $(CDC_OBJS)
	$(CC_ARMSYS) $(CC_ARMSYS_OPTS) $(LD_ARMSYS_OPTS) \
	-Wl,-Map=build/cdc_msc.elf.map \
	-o build/cdc_msc.elf \
	external/STM32H5xx/Source/Templates/gcc/startup_stm32h503xx.s \
	$(CDC_OBJS) \
	$(LIBS_ARMSYS)

