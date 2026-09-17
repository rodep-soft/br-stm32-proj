# ==============================================================================
# Makefile for STM32F767ZI Zenoh-Pico Project (Nix & Native Compatible)
# ==============================================================================

PROJECT_NAME := br-stm32
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

# Detect environment: check if tools are directly available or via Nix
HAS_ARM_GCC  := $(shell command -v arm-none-eabi-gcc 2>/dev/null)
HAS_NIX      := $(shell command -v nix 2>/dev/null)

.PHONY: all setup dev shell udev sub zenoh-sub msg build do-build flash do-flash flash-openocd do-flash-openocd test do-test size do-size clean help

# Default target
all: build

## -----------------------------------------------------------------------------
## Environment Setup & Nix Shell
## -----------------------------------------------------------------------------

# Setup dependencies and submodules
setup:
	@echo "==> [Setup] Initializing Git submodules..."
	@git submodule update --init --recursive
	@if [ -n "$(HAS_NIX)" ]; then \
		echo "==> [Setup] Nix detected! Allowing direnv if available..."; \
		command -v direnv >/dev/null 2>&1 && direnv allow || true; \
		echo "==> [Setup] Ready! You can run 'nix develop' (or use direnv) to enter the dev shell."; \
	elif command -v apt-get >/dev/null 2>&1; then \
		echo "==> [Setup] Debian/Ubuntu detected. Installing dependencies via apt..."; \
		sudo apt-get update && sudo apt-get install -y --no-install-recommends \
			gcc-arm-none-eabi \
			libnewlib-arm-none-eabi \
			libstdc++-arm-none-eabi-newlib \
			cmake \
			ninja-build \
			stlink-tools \
			python3; \
	else \
		echo "[Setup] Please install Nix (recommended) or native arm-none-eabi toolchain."; \
	fi
	@echo "==> [Setup] Generating message headers..."
	@$(MAKE) msg
	@echo "==> [Setup] Complete! Run 'make build' to compile."
	@echo "==> [Setup] (Tip: Run 'make udev' once if you need non-root ST-LINK access permissions)"

# Install ST-LINK udev rules to allow flashing without sudo
udev:
	@echo "==> [udev] Installing ST-LINK rules to /etc/udev/rules.d/..."
	@if [ -f /etc/NIXOS ]; then \
		echo "[udev] Note: On NixOS, consider setting 'services.udev.packages = [ pkgs.stlink ];' in your configuration.nix."; \
	fi
	@sudo cp tools/udev/49-stlink.rules /etc/udev/rules.d/
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger
	@echo "==> [udev] Rules installed and reloaded successfully!"
	@echo "==> [udev] If your ST-LINK is currently plugged in, please replug the USB cable."

# Enter Nix development shell
dev shell:
	@if [ -n "$(HAS_NIX)" ]; then \
		nix develop; \
	else \
		echo "Error: Nix is not installed on this system. See https://nixos.org/download"; \
		exit 1; \
	fi

## -----------------------------------------------------------------------------
## Code Generation (.msg -> Micro-CDR C Header)
## -----------------------------------------------------------------------------

msg:
	@echo "==> [CodeGen] Generating Micro-CDR headers from .msg files..."
	@$(PYTHON) $(CODEGEN) --package robot_msgs --msg-dir $(MSG_DIR) --out-dir $(GEN_DIR)

## -----------------------------------------------------------------------------
## Build Targets (Auto-delegates to Nix if tools not in PATH)
## -----------------------------------------------------------------------------

build: msg
ifeq ($(strip $(HAS_ARM_GCC)),)
ifneq ($(strip $(HAS_NIX)),)
	@echo "==> [Nix] arm-none-eabi-gcc not in PATH. Running build inside Nix shell..."
	@nix develop --command $(MAKE) do-build
else
	@$(MAKE) do-build
endif
else
	@$(MAKE) do-build
endif

do-build:
	@echo "==> [Build] Configuring CMake with Ninja..."
	@cmake -B $(BUILD_DIR) -G Ninja \
		-DCMAKE_BUILD_TYPE=Debug \
		-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)
	@echo "==> [Build] Compiling STM32 Firmware..."
	@cmake --build $(BUILD_DIR) --parallel
	@echo "==> [Size] Firmware Memory Usage:"
	@arm-none-eabi-size $(ELF_FILE)

size:
ifeq ($(strip $(HAS_ARM_GCC)),)
ifneq ($(strip $(HAS_NIX)),)
	@nix develop --command arm-none-eabi-size $(ELF_FILE)
else
	@arm-none-eabi-size $(ELF_FILE)
endif
else
	@arm-none-eabi-size $(ELF_FILE)
endif

## -----------------------------------------------------------------------------
## Flashing Targets (ST-LINK)
## -----------------------------------------------------------------------------

flash: build
ifeq ($(shell command -v st-flash 2>/dev/null),)
ifneq ($(strip $(HAS_NIX)),)
	@echo "==> [Nix] Running st-flash inside Nix shell..."
	@nix develop --command $(MAKE) do-flash
else
	@$(MAKE) do-flash
endif
else
	@$(MAKE) do-flash
endif

do-flash:
	@echo "==> [Flash] Writing $(BIN_FILE) to STM32 via st-flash..."
	@st-flash --reset write $(BIN_FILE) $(FLASH_ADDR)

flash-openocd: build
ifeq ($(shell command -v openocd 2>/dev/null),)
ifneq ($(strip $(HAS_NIX)),)
	@echo "==> [Nix] Running OpenOCD inside Nix shell..."
	@nix develop --command $(MAKE) do-flash-openocd
else
	@$(MAKE) do-flash-openocd
endif
else
	@$(MAKE) do-flash-openocd
endif

do-flash-openocd:
	@echo "==> [Flash] Writing $(BIN_FILE) via OpenOCD..."
	@openocd -f interface/stlink.cfg -f target/stm32f7x.cfg \
		-c "program $(BIN_FILE) $(FLASH_ADDR) reset exit"

## -----------------------------------------------------------------------------
## Zenoh Testing & Inspection
## -----------------------------------------------------------------------------

sub zenoh-sub:
	@$(PYTHON) tools/zenoh_sub.py

## -----------------------------------------------------------------------------
## Host Unit Testing
## -----------------------------------------------------------------------------

test: msg
ifeq ($(shell command -v cmake 2>/dev/null),)
ifneq ($(strip $(HAS_NIX)),)
	@echo "==> [Nix] Running host tests inside Nix shell..."
	@nix develop --command $(MAKE) do-test
else
	@$(MAKE) do-test
endif
else
	@$(MAKE) do-test
endif

do-test:
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
	@echo "  make dev / make shell - Enter the Nix development shell"
	@echo "  make setup            - Initialize submodules and dependencies / direnv"
	@echo "  make udev             - Install ST-LINK udev rules to allow flashing without sudo"
	@echo "  make build            - Generate headers and build STM32 firmware (default)"
	@echo "  make flash            - Build and flash to STM32 via ST-LINK (st-flash)"
	@echo "  make flash-openocd    - Build and flash to STM32 via OpenOCD"
	@echo "  make sub              - Run Zenoh router & subscriber to receive STM32 messages"
	@echo "  make test             - Build and run host unit tests"
	@echo "  make size             - Show firmware Flash/RAM consumption"
	@echo "  make msg              - Generate C headers from .msg only"
	@echo "  make clean            - Remove all build artifacts and generated headers"
