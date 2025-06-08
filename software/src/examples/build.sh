
arm-none-eabi-g++ \
 -mthumb \
 -mcpu=cortex-m33 \
 -mfloat-abi=hard \
 -mfpu=fpv5-sp-d16 \
 -nostartfiles \
 -Wl,-T,linkerscript.ld \
 -fsingle-precision-constant \
 -fdata-sections \
 -ffunction-sections \
 -Wall \
 -Wl,-gc-sections \
 linktest.cpp \
 syslib.cpp

