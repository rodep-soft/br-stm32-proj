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
HAS_NIX      := $(shell if command -v nix >/dev/null 2>&1 && [ -d /nix/store ]; then echo 1; fi)

.PHONY: all setup dev shell udev nixconf python-deps firewall-off router zenohd sub zenoh-sub msg build do-build flash do-flash flash-openocd do-flash-openocd test do-test size do-size clean help

# Default target
all: build

## -----------------------------------------------------------------------------
## Environment Setup & Nix Shell
## -----------------------------------------------------------------------------

# Setup all dependencies, submodules, configurations, and tools
setup:
	@echo "=========================================================="
	@echo "  Starting Full Environment Setup                         "
	@echo "=========================================================="
	@echo "==> [1/5] Initializing Git submodules..."
	@git submodule update --init --recursive
	@if [ -n "$(HAS_NIX)" ]; then \
		echo "==> [2/5] Nix detected! Configuring nix.conf & direnv..."; \
		$(MAKE) nixconf; \
	elif command -v apt-get >/dev/null 2>&1; then \
		echo "==> [2/5] Debian/Ubuntu detected. Installing dependencies via apt..."; \
		sudo apt-get update && sudo apt-get install -y --no-install-recommends \
			gcc-arm-none-eabi \
			libnewlib-arm-none-eabi \
			libstdc++-arm-none-eabi-newlib \
			cmake \
			ninja-build \
			stlink-tools \
			python3; \
	else \
		echo "==> [2/5] Toolchain notice: Please install Nix (recommended) or native arm-none-eabi toolchain."; \
	fi
	@echo "==> [3/5] Checking ST-LINK udev rules..."
	@if [ -f /etc/udev/rules.d/49-stlink.rules ] && cmp -s tools/udev/49-stlink.rules /etc/udev/rules.d/49-stlink.rules 2>/dev/null; then \
		echo "==> [udev] ST-LINK udev rules are already up-to-date."; \
	elif sudo -n true 2>/dev/null; then \
		$(MAKE) udev; \
	else \
		echo "==> [udev] Notice: Run 'make udev' once with sudo to enable non-root ST-LINK access."; \
	fi
	@echo "==> [4/5] Setting up Python dependencies (Zenoh & tools)..."
	@$(MAKE) python-deps
	@echo "==> [5/5] Generating Micro-CDR message headers..."
	@$(MAKE) msg
	@echo "=========================================================="
	@echo "  Setup Complete! All tools and configurations ready.     "
	@echo "  - Run 'make build'       to compile firmware"
	@echo "  - Run 'make flash'       to write to STM32"
	@echo "  - Run 'make sub'         to test Zenoh communication"
	@echo "=========================================================="

# Deploy user Nix configuration (Flakes & Cachix substituters)
nixconf:
	@bash tools/nix/setup-nix.sh

# Install Python tools for Zenoh testing
python-deps:
	@if ! $(PYTHON) -c "import zenoh" >/dev/null 2>&1; then \
		echo "==> [Python] Installing eclipse-zenoh and zenoh-cli..."; \
		$(PYTHON) -m pip install --user --break-system-packages -q eclipse-zenoh zenoh-cli 2>/dev/null || \
		$(PYTHON) -m pip install --user -q eclipse-zenoh zenoh-cli 2>/dev/null || \
		pip install --user -q eclipse-zenoh zenoh-cli 2>/dev/null || true; \
	else \
		echo "==> [Python] eclipse-zenoh is already installed."; \
	fi

# Install ST-LINK udev rules to allow flashing without sudo
udev:
	@if [ "$$(uname -s)" = "Linux" ]; then \
		if [ -f /etc/NIXOS ]; then \
			echo "==> [udev] Note: On NixOS, consider setting 'services.udev.packages = [ pkgs.stlink ];' in your configuration.nix."; \
		elif cmp -s tools/udev/49-stlink.rules /etc/udev/rules.d/49-stlink.rules 2>/dev/null; then \
			echo "==> [udev] ST-LINK udev rules are already up-to-date in /etc/udev/rules.d/"; \
		elif [ -w /etc/udev/rules.d ]; then \
			cp tools/udev/49-stlink.rules /etc/udev/rules.d/; \
			udevadm control --reload-rules 2>/dev/null || true; \
			udevadm trigger 2>/dev/null || true; \
			echo "==> [udev] Rules installed successfully!"; \
		elif command -v sudo >/dev/null 2>&1; then \
			echo "==> [udev] Installing ST-LINK rules to /etc/udev/rules.d/ (requires sudo)..."; \
			sudo cp tools/udev/49-stlink.rules /etc/udev/rules.d/; \
			sudo udevadm control --reload-rules; \
			sudo udevadm trigger; \
			echo "==> [udev] Rules installed and reloaded successfully!"; \
		fi; \
	fi

# Disable host firewall and flush packet filter rules (ACCEPT all)
firewall-off:
	@tools/firewall-off.sh

# Enter Nix development shell
dev shell:
	@if [ -n "$(HAS_NIX)" ]; then \
		nix develop; \
	elif [ -n "$$(command -v nix 2>/dev/null)" ] && [ ! -d /nix/store ]; then \
		echo "Error: 'nix' command is found, but '/nix/store' directory is missing or not mounted."; \
		echo "If using Docker or a container, please mount /nix/store or install standard build tools."; \
		exit 1; \
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

# Run standalone Zenoh router (auto-downloads official binary if not found)
router zenohd:
	@if ! command -v zenohd >/dev/null 2>&1; then \
		echo "==> [Zenoh] Downloading official zenohd binary..."; \
		mkdir -p $(HOME)/.local/bin; \
		curl -sL https://github.com/eclipse-zenoh/zenoh/releases/download/1.10.1/zenoh-1.10.1-x86_64-unknown-linux-gnu-standalone.zip -o /tmp/zenoh-bin.zip; \
		unzip -q -o /tmp/zenoh-bin.zip zenohd -d $(HOME)/.local/bin/; \
		chmod +x $(HOME)/.local/bin/zenohd; \
		rm -f /tmp/zenoh-bin.zip; \
		echo "==> [Zenoh] Installed zenohd to $(HOME)/.local/bin/zenohd"; \
	fi
	@echo "==> [Zenoh] Starting zenohd router on UDP & TCP 7447..."
	@zenohd --listen udp/0.0.0.0:7447 --listen tcp/0.0.0.0:7447

sub zenoh-sub: python-deps
	@$(PYTHON) tools/zenoh_sub.py $(ARGS)

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
	@./$(TEST_BUILD)/test_bridge_engine

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
	@echo "  make nixconf          - Configure user nix.conf (Flakes & Cachix binary caches)"
	@echo "  make udev             - Install ST-LINK udev rules to allow flashing without sudo"
	@echo "  make firewall-off     - Disable host firewall & packet filters (ACCEPT all, requires sudo)"
	@echo "  make build            - Generate headers and build STM32 firmware (default)"
	@echo "  make flash            - Build and flash to STM32 via ST-LINK (st-flash)"
	@echo "  make flash-openocd    - Build and flash to STM32 via OpenOCD"
	@echo "  make router           - Run standalone Zenoh router (zenohd) on UDP/TCP 7447"
	@echo "  make sub              - Run Zenoh router & subscriber to receive STM32 messages"
	@echo "  make test             - Build and run host unit tests"
	@echo "  make size             - Show firmware Flash/RAM consumption"
	@echo "  make msg              - Generate C headers from .msg only"
	@echo "  make clean            - Remove all build artifacts and generated headers"
