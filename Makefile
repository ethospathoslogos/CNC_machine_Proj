# Toolchain (can be overridden by driver project)
CC ?= gcc
AR ?= ar

CFLAGS ?= -Wall -Wextra -std=c99 -g -Iinclude
BUILD_DIR = build
SRC_DIR   = src
LIB_DIR   = lib

LIB_NAME  = libgrblcore.a

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

# Default target builds the core library
all: dirs $(LIB_DIR)/$(LIB_NAME)

# Create static library from core objects
$(LIB_DIR)/$(LIB_NAME): $(OBJS)
	@echo "Archiving core library..."
	$(AR) rcs $@ $^

# Compile core source into object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "Compiling core $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Make required directories
dirs:
	@mkdir -p $(BUILD_DIR) $(LIB_DIR)

# Clean only core build artifacts
clean:
	@echo "Cleaning core build..."
	@rm -rf $(BUILD_DIR) $(LIB_DIR)

.PHONY: all clean dirs