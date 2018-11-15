# project name (generate executable with this name)
TARGET = blink

CROSS_COMPILE = arm-none-eabi-

LINKER_SCRIPT = STM32F103C8.ld

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -g
CFLAGS += -mcpu=cortex-m3 -mthumb -mabi=aapcs
CFLAGS += -Wall -Wextra
CFLAGS += -fno-common -static
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS += -std=c11

LDFLAGS += -march=armv7-m -mabi=aapcs
LDFLAGS += -nostartfiles -nostdlib -lgcc
LDFLAGS += -T$(LINKER_SCRIPT)


# change these to proper directories where each file should be
SRCDIR  = src
STM_SRC = stm_src
OBJDIR  = obj
BINDIR  = bin

SOURCES  := $(wildcard $(SRCDIR)/*.c) $(wildcard $(STM_SRC)/*.c)
INCLUDES := $(wildcard $(SRCDIR)/*.h) $(wildcard $(STM_SRC)/*.h)
OBJECTS  := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(filter $(SRCDIR)%.c, $(SOURCES)))
OBJECTS  += $(patsubst $(STM_SRC)/%.c,$(OBJDIR)/%.o,$(filter $(STM_SRC)%.c, $(SOURCES)))
RM       = rm -f


$(BINDIR)/$(TARGET): $(OBJECTS)
	@$(LD) $(OBJECTS) $(LFLAGS) -o $@

# $(OBJECTS): $(OBJDIR)/%.o : $(SOURCES)%.c
$(OBJECTS): $(SOURCES)
	$(CC) $(CFLAGS) -c $< -o $@

list_source:
	@echo $(SOURCES)

list_obj:
	@echo $(OBJECTS)

clean:
	rm $(OBJDIR)/*.o

remove: clean
#	$(RM) $(BINDIR)/$(TARGET)
