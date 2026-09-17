# ==============================================================================
# Makefile for STM32F767ZI Zenoh-Pico Project
# ==============================================================================

PROJECT_NAME := abeshitest
BUILD_DIR    := build
TEST_DIR     := tests
TEST_BUILD   := tests/build
TOOLCHAIN    := cmake/gcc-arm-none-eabi.cmake

# Python Code Generator
PYTHON       := python3
CODEGEN      := tools/msg2cdr.py
MSG_DIR      := test_msgs/msg
GEN_DIR      := Core/Inc/generated

# Flashing Tool Settings
FLASH_ADDR   := 0x08000000
BIN_FILE     := $(BUILD_DIR)/$(PROJECT_NAME).bin
ELF_FILE     := $(BUILD_DIR)/$(PROJECT_NAME).elf

.PHONY: all build flash flash-openocd test clean size msg setup help

# Default target: build firmware
all: build

## -----------------------------------------------------------------------------
## Environment Setup
## -----------------------------------------------------------------------------

setup:
	@echo "==> [Setup] Initializing Git submodules..."
	@git submodule update --init --recursive
	@echo "==> [Setup] Checking/Installing dependencies..."
	@if command -v apt-get >/dev/null 2>&1; then \
		sudo apt-get update && sudo apt-get install -y --no-install-recommends \
			gcc-arm-none-eabi \
			libnewlib-arm-none-eabi \
			libstdc++-arm-none-eabi-newlib \
			cmake \
			ninja-build \
			stlink-tools \
			python3; \
	else \
		echo "[Setup] Non-Debian system detected. Please ensure arm-none-eabi-gcc, cmake, ninja, and stlink are installed."; \
	fi
	@echo "==> [Setup] Generating message headers..."
	@$(MAKE) msg
	@echo "==> [Setup] Setup complete! Run 'make build' to compile or 'make flash' to flash."

## -----------------------------------------------------------------------------
## Build Targets
## -----------------------------------------------------------------------------

msg:
	@echo "==> [CodeGen] Generating Micro-CDR headers from .msg files..."
	@$(PYTHON) $(CODEGEN) --package robot_msgs --msg-dir $(MSG_DIR) --out-dir $(GEN_DIR)

build: msg
	@echo "==> [Build] Configuring CMake with Ninja..."
	@cmake -B $(BUILD_DIR) -G Ninja \
		-DCMAKE_BUILD_TYPE=Debug \
		-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)
	@echo "==> [Build] Compiling STM32 Firmware..."
	@cmake --build $(BUILD_DIR) --parallel
	@echo "==> [Size] Firmware Memory Usage:"
	@arm-none-eabi-size $(ELF_FILE)

size:
	@arm-none-eabi-size $(ELF_FILE)

## -----------------------------------------------------------------------------
## Flashing Targets (ST-LINK)
## -----------------------------------------------------------------------------

flash: build
	@echo "==> [Flash] Writing $(BIN_FILE) to STM32 via st-flash..."
	@st-flash --reset write $(BIN_FILE) $(FLASH_ADDR)

flash-openocd: build
	@echo "==> [Flash] Writing $(BIN_FILE) via OpenOCD..."
	@openocd -f interface/stlink.cfg -f target/stm32f7x.cfg \
		-c "program $(BIN_FILE) $(FLASH_ADDR) reset exit"

## -----------------------------------------------------------------------------
## Testing Targets
## -----------------------------------------------------------------------------

test: msg
	@echo "==> [Test] Building and running host unit tests..."
	@cmake -B $(TEST_BUILD) -S $(TEST_DIR) -G Ninja
	@cmake --build $(TEST_BUILD)
	@./$(TEST_BUILD)/test_serialization

## -----------------------------------------------------------------------------
## Clean Target
## -----------------------------------------------------------------------------

clean:
	@echo "==> [Clean] Removing build artifacts and generated files..."
	@rm -rf $(BUILD_DIR) $(TEST_BUILD) $(GEN_DIR)
	@echo "==> Clean complete."

## -----------------------------------------------------------------------------
## Help
## -----------------------------------------------------------------------------

help:
	@echo "Available commands:"
	@echo "  make setup         - Install toolchains/dependencies and init submodules"
	@echo "  make build         - Generate msg headers and build STM32 firmware (default)"
	@echo "  make flash         - Build and flash to STM32 using st-flash"
	@echo "  make flash-openocd - Build and flash using OpenOCD"
	@echo "  make test          - Build and run host unit tests"
	@echo "  make size          - Show firmware memory usage (Flash/RAM)"
	@echo "  make msg           - Generate C headers from .msg only"
	@echo "  make clean         - Delete all build files and generated headers"
