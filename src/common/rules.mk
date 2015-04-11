# Minimal gcc makefile for LPC810

# default Linux USB device name for upload
TTY ?= /dev/ttyUSB*

# use the arm cross compiler, not std gcc
TRGT = /home/ntuckett/NXP-LPC/gcc-arm-none-eabi-4_9-2014q4/bin/arm-none-eabi-
CC = $(TRGT)gcc
CXX = $(TRGT)g++
CP = $(TRGT)objcopy

# compiler and linker settings
CFLAGS = -mcpu=cortex-m0plus -mthumb -I../common -Os -ggdb
CXXFLAGS = $(CFLAGS) -fno-rtti -fno-exceptions
LDFLAGS = -Wl,--script=../common/LPC810.ld -nostartfiles -Wl,-Map=firmware.map

# permit including this makefile from a sibling directory
vpath %.c ../common
vpath %.cpp ../common

# default target
#upload: firmware.bin
#	lpc21isp -control -donotstart -bin $< $(TTY) 115200 0
all: firmware.bin

firmware.elf: main.o gcc_startup_lpc8xx.o
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $^

%.bin: %.elf
	$(CP) -O binary $< $@

clean:
	rm -f *.o *.elf

# these target names don't represent real files
.PHONY: clean
