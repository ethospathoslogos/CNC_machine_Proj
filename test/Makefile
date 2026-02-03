# Compiler and Flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

# Directories
SRC_DIR = ../src
TEST_DIR := $(CURDIR)
BUILD_DIR = build
BIN_DIR = bin

TEST_TARGET = $(BIN_DIR)/test_runner
PLANNER_TEST_TARGET = $(BIN_DIR)/planner_test_runner
GCODE_TEST_TARGET = $(BIN_DIR)/gcode_test_runner

# Source / objects
OBJS = $(BUILD_DIR)/parser.o $(BUILD_DIR)/input_test.o
PLANNER_OBJS = $(BUILD_DIR)/planner.o $(BUILD_DIR)/planner_test.o
GCODE_OBJS = $(BUILD_DIR)/gcode.o $(BUILD_DIR)/gcode_test.o

# Default target
all: dirs $(TEST_TARGET) $(PLANNER_TEST_TARGET) $(GCODE_TEST_TARGET)

# Link test runner  (THIS WAS MISSING)
$(TEST_TARGET): $(OBJS)
	@echo "Linking $@..."
	$(CC) $(CFLAGS) -o $@ $^

# Link planner test runner
$(PLANNER_TEST_TARGET): $(PLANNER_OBJS)
	@echo "Linking $@..."
	$(CC) $(CFLAGS) -o $@ $^

# Link gcode test runner
$(GCODE_TEST_TARGET): $(GCODE_OBJS)
	@echo "Linking $@..."
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Compile core source
$(BUILD_DIR)/parser.o: $(SRC_DIR)/parser.c
	@mkdir -p $(BUILD_DIR)
	@echo "Compiling $< -> $@..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile test source
$(BUILD_DIR)/input_test.o: $(TEST_DIR)/input_test.c
	@mkdir -p $(BUILD_DIR)
	@echo "Compiling $< -> $@..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile planner source
$(BUILD_DIR)/planner.o: $(SRC_DIR)/planner.c $(SRC_DIR)/planner.h
	@mkdir -p $(BUILD_DIR)
	@echo "Compiling $< -> $@..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile planner test source
$(BUILD_DIR)/planner_test.o: $(TEST_DIR)/planner_test.c
	@mkdir -p $(BUILD_DIR)
	@echo "Compiling $< -> $@..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile gcode source
$(BUILD_DIR)/gcode.o: $(SRC_DIR)/gcode.c $(SRC_DIR)/gcode.h
	@mkdir -p $(BUILD_DIR)
	@echo "Compiling $< -> $@..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile gcode test source
$(BUILD_DIR)/gcode_test.o: $(TEST_DIR)/gcode_test.c
	@mkdir -p $(BUILD_DIR)
	@echo "Compiling $< -> $@..."
	$(CC) $(CFLAGS) -c $< -o $@

# Ensure dirs exist
dirs:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

clean:
	@echo "Cleaning up..."
	@rm -rf $(BUILD_DIR) $(BIN_DIR)

run: all
	@echo "Running input tests..."
	./$(TEST_TARGET)
	@echo ""
	@echo "Running planner tests..."
	./$(PLANNER_TEST_TARGET)
	@echo ""
	@echo "Running gcode tests..."
	./$(GCODE_TEST_TARGET)

.PHONY: all clean dirs run