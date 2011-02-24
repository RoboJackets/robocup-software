TARGET=robot
OBJS=robot.o fpga.o vic.o timer.o radio.o adc.o
SYS_OBJS=entry.o

CFLAGS=-O2 -Wall -std=gnu99

CROSS=${abspath ${PWD}/../toolchain/install/bin}/arm-elf-
CC=$(CROSS)gcc
LD=$(CROSS)ld
OBJCOPY=$(CROSS)objcopy

.PHONY: all clean prog

all:	robot.elf

clean:
		rm -f $(SYS_OBJS) $(OBJS) $(TARGET).elf $(TARGET).bin $(BINS)

prog:	$(TARGET).elf
		openocd -f openocd.cfg -c "script reflash"

$(TARGET).elf:	$(SYS_OBJS) $(OBJS)
		$(CC) $(SYS_OBJS) $(OBJS) -o $@ -nostdlib -Xlinker -T LPC2103-ROM.ld
		@chmod a-x $@

# For debugging: generate assembly from C
%.S:	%.c
		$(CC) $(CFLAGS) -S $< -o $@

