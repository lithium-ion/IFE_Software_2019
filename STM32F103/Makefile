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
# LDFLAGS += -T$(LINKER_SCRIPT)


# change these to proper directories where each file should be
SRCDIR        = src
STANDARD_LIBS = $(SRCDIR)/stm32f10x 
OBJDIR        = obj
BINDIR        = bin

SOURCES  := $(wildcard $(SRCDIR)/*.c)
SOURCES  += $(wildcard $(STANDARD_LIBS)/*.c)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
INCLUDES += $(wildcard $(STANDARD_LIBS)/*.h)
OBJECTS  := $(patsubst $(SRCDIR)/%.c,$(OBJ_DIR)/%.o,$(SOURCES))
OBJECTS  += $(patsubst $(STANDARD_LIBS)/%.c,$(OBJ_DIR)/%.o,$(SOURCES))
RM       = rm -f

$(BINDIR)/$(TARGET): $(OBJECTS)
	@echo "$(OBJECTS)"
	@$(LD) $(OBJECTS) $(LFLAGS) -o $@

# $(OBJECTS): $(OBJDIR)/%.o : $(SOURCES)%.c
$(OBJDIR)/%.o : $(SOURCES)%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJECTS)

remove: clean
	$(RM) $(BINDIR)/$(TARGET)
