
include tools/compile_settings.mk

.PHONY: all clean test

all: \
	build/tests/unit_tests \
	build/cdc_msc.bin \
	build/gamesquirrel.bin \
	build/devkit.tgz


clean:
	rm -Rf build

test: build/tests/unit_tests
	build/tests/unit_tests

include src/usb_example/cdc_msc.mk
include src/gamesquirrel/gamesquirrel.mk
include tools/devkit.mk
include tests/unit_tests.mk

