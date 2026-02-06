PREFIX   ?= arm-none-eabi
CC       := $(PREFIX)-gcc
OBJCOPY  := $(PREFIX)-objcopy
SIZE     := $(PREFIX)-size

TOP            := ..
LIBOPENCM3_DIR := $(TOP)/lib/libopencm3

TARGET := blink
BUILD  := build

SRCS := main.c
OBJS := $(SRCS:%.c=$(BUILD)/%.o)

# Cortex-M4F (STM32F446RE)
CPUFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS  += -O0 -g3 -Wall -Wextra
CFLAGS  += $(CPUFLAGS)
CFLAGS  += -ffunction-sections -fdata-sections
CFLAGS  += -I$(LIBOPENCM3_DIR)/include
CFLAGS  += -DSTM32F4

# Generic libopencm3 linker script + our memory layout
GENERIC_LD := $(LIBOPENCM3_DIR)/lib/cortex-m-generic.ld
MEMORY_LD  := stm32f446re-memory.ld

LDFLAGS += -T$(MEMORY_LD) -T$(GENERIC_LD)
LDFLAGS += -nostartfiles
LDFLAGS += -Wl,--gc-sections
LDFLAGS += $(CPUFLAGS)
LDFLAGS += -Wl,-Map=$(BUILD)/$(TARGET).map

# libopencm3 library
LIBNAME := opencm3_stm32f4
LDLIBS  += -L$(LIBOPENCM3_DIR)/lib -l$(LIBNAME)

.PHONY: all clean libopencm3 flash

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

libopencm3:
	$(MAKE) -C $(LIBOPENCM3_DIR) TARGETS='stm32/f4' PREFIX=$(PREFIX)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: libopencm3 $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@


clean:
	rm -rf $(BUILD)

flash: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $(BUILD)/$(TARGET).elf verify reset exit"
