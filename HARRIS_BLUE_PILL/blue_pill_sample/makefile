# Makefile for the Blue Pill (STM32F103)
# Author: Harris Mohamed
# Last revision: 11/28/18

# SETUP
COMPILE_OPTS = -mcpu=cortex-m3 -mthumb -Wall -g -O0
INCLUDE_DIRS = -I . -I lib/inc
LIBRARY_DIRS = -L lib

CC = arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

CXX = arm-none-eabi-g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

AS = arm-none-eabi-gcc
ASFLAGS = $(COMPILE_OPTS) -c

LD = arm-none-eabi-gcc
LDFLAGS = -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T stm32.ld

OBJCP = arm-none-eabi-objcopy
OBJCPFLAGS = -O binary

AR = arm-none-eabi-ar
ARFLAGS = cr

# FILE OUTPUTS
MAIN_OUT = main
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_BIN = $(MAIN_OUT).bin

# MAKE 'ALL'
all: $(MAIN_OUT_ELF) $(MAIN_OUT_BIN) 

# DIRECTIVES FOR 'MAIN' ELF AND BIN
$(MAIN_OUT_ELF): main.o stm32f10x_it.o lib/libstm32.a
	$(LD) $(LDFLAGS) main.o stm32f10x_it.o lib/libstm32.a --output $@

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@

# FLASH DIRECTIVES
flash: $(MAIN_OUT)
	@cp $(MAIN_OUT_ELF) jtag/flash
	@cd jtag; openocd -f flash.cfg
	@rm jtag/flash


# STM32 libraries
LIBSTM32_OUT = lib/libstm32.a

LIBSTM32_OBJS = \
 lib/src/stm32f10x_adc.o \
 lib/src/stm32f10x_bkp.o \
 lib/src/stm32f10x_can.o \
 lib/src/stm32f10x_dma.o \
 lib/src/stm32f10x_exti.o \
 lib/src/stm32f10x_flash.o \
 lib/src/stm32f10x_gpio.o \
 lib/src/stm32f10x_i2c.o \
 lib/src/stm32f10x_iwdg.o \
 lib/src/stm32f10x_lib.o \
 lib/src/stm32f10x_nvic.o \
 lib/src/stm32f10x_pwr.o \
 lib/src/stm32f10x_rcc.o \
 lib/src/stm32f10x_rtc.o \
 lib/src/stm32f10x_spi.o \
 lib/src/stm32f10x_systick.o \
 lib/src/stm32f10x_tim.o \
 lib/src/stm32f10x_tim1.o \
 lib/src/stm32f10x_usart.o \
 lib/src/stm32f10x_wwdg.o \
 lib/src/cortexm3_macro.o \
 lib/src/stm32f10x_vector.o

$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(LIBSTM32_OBJS): stm32f10x_conf.h

# MAKE 'CLEAN'
clean:
	-rm *.o lib/src/*.o $(LIBSTM32_OUT) $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)
