# ---- Toolchain prefix (GCC ARM Embedded) ----
PREFIX ?= arm-none-eabi

# ---- Path to libopencm3 submodule ----
LIBOPENCM3_DIR := lib/libopencm3

# ---- Build only STM32F4 family (covers STM32F446RE) ----
LIBOPENCM3_TARGETS := stm32/f4

.PHONY: all libopencm3 clean clean-libopencm3

all: libopencm3

# Build only the requested family, not the whole world
libopencm3:
	$(MAKE) -C $(LIBOPENCM3_DIR) PREFIX=$(PREFIX) TARGETS='$(LIBOPENCM3_TARGETS)'

clean-libopencm3:
	$(MAKE) -C $(LIBOPENCM3_DIR) clean

clean: clean-libopencm3
