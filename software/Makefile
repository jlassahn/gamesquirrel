
include tools/compile_settings.mk


all: \
	build/cdc_msc.bin \
	build/gamesquirrel.bin \
	build/devkit.tgz


clean:
	rm -Rf build


include src/usb_example/cdc_msc.mk
include src/gamesquirrel/gamesquirrel.mk
include tools/devkit.mk

