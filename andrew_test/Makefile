TARGET = blink

# Compiler and Linker Specification
CROSS_COMPILE = arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy

LINKER_SCRIPT = STM32F103C8.ld

# Flags to use when compiling, preprocessing, assembling, and linking
CFLAGS += -g
CFLAGS += -mcpu=cortex-m3 -mthumb -mabi=aapcs
CFLAGS += -Wall -Wextra
CFLAGS += -fno-common -static
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS += -std=c11
LDFLAGS += -march=armv7-m -mabi=aapcs
LDFLAGS += -nostartfiles -nostdlib -lgcc
LDFLAGS += -T$(LINKER_SCRIPT)

#If you have any .h files in another directory, add -I<dir> to this line
CPPFLAGS+= -g

# This generates the list of source files
SRC += $(wildcard *.S) $(wildcard *.c) $(wildcard */*.S) $(wildcard */*.c)

# This generates the list of .o files. The order matters, boot.o must be first
OBJS = init.o
OBJS += $(filter-out init.o,$(patsubst %.S,%.o,$(filter %.S,$(SRC))))
OBJS += $(filter-out init.o,$(patsubst %.c,%.o,$(filter %.c,$(SRC))))

.PHONY: all build dep clean server

all: clean dep build

build: $(TARGET).elf

$(TARGET).elf: $(OBJS)
	rm -f $(TARGET).elf
	$(CC) $(LDFLAGS) $(OBJS) -o $(TARGET).elf

dep: Makefile.dep

Makefile.dep: $(SRC)
	$(CC) -MM $(CPPFLAGS) $(SRC) > $@

clean:
	rm -f *.o */*.o Makefile.dep

server:
	openocd -f debug.cfg

ifneq ($(MAKECMDGOALS),dep)
ifneq ($(MAKECMDGOALS),clean)
include Makefile.dep
endif
endif
