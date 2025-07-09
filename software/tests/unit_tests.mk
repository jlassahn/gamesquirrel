


UTEST_SRCS := \
src/gamesquirrel/disk_cache.c \
src/gamesquirrel/fatfs.c \
src/gamesquirrel/elf.c \
tests/unit_tests.c \
tests/mock_sd_card.c \
tests/test_charqueue.c \
tests/test_fatfs.c \
tests/test_elf.c \


UTEST_OBJS := $(addprefix build/obj/host/, $(addsuffix .o, $(basename $(UTEST_SRCS))))

build/tests/unit_tests: $(UTEST_OBJS)
	mkdir -p $(@D)
	$(CC_HOST) $(CC_HOST_OPTS) $(LD_HOST_OPTS) \
	-o $@ \
	$(UTEST_OBJS) \
	$(LIBS_HOST)


