PROJECT_NAME := abu-bno085
PRESET       ?= debug_GCC_NUCLEO-C542RC
BUILD_DIR    := build/$(PRESET)
ELF_FILE     := $(BUILD_DIR)/$(PROJECT_NAME).elf
BIN_FILE     := $(BUILD_DIR)/$(PROJECT_NAME).bin
HEX_FILE     := $(BUILD_DIR)/$(PROJECT_NAME).hex
FLASH_ADDR   ?= 0x08000000

CMAKE        ?= cmake
OBJCOPY      ?= arm-none-eabi-objcopy
SIZE         ?= arm-none-eabi-size
ST_FLASH     ?= st-flash

.PHONY: all fix-cmake configure build rebuild clean size flash help

all: build

fix-cmake:
	@../fix_cmake.sh

configure: fix-cmake
	@echo "==> [CMake] Configuring preset '$(PRESET)'..."
	@$(CMAKE) --preset $(PRESET)

build: configure
	@echo "==> [Build] Building $(PROJECT_NAME)..."
	@$(CMAKE) --build --preset $(PRESET) --parallel
	@$(OBJCOPY) -O binary $(ELF_FILE) $(BIN_FILE)
	@$(OBJCOPY) -O ihex $(ELF_FILE) $(HEX_FILE)
	@echo "==> [Size] Firmware Memory Usage:"
	@$(SIZE) $(ELF_FILE)

rebuild: clean build

size:
	@$(SIZE) $(ELF_FILE)

flash: build
	@echo "==> [Flash] Writing $(BIN_FILE) to $(FLASH_ADDR)..."
	@$(ST_FLASH) --reset write $(BIN_FILE) $(FLASH_ADDR)

clean:
	@echo "==> [Clean] Removing build directory..."
	@rm -rf build

help:
	@echo "Usage:"
	@echo "  make            - Fix CMake files, configure, and build firmware (.elf, .bin, .hex)"
	@echo "  make fix-cmake  - Patch CubeMX-generated CMake files for CMake 3.28"
	@echo "  make rebuild    - Clean and rebuild firmware"
	@echo "  make flash      - Build and flash firmware via st-flash"
	@echo "  make size       - Show firmware memory usage"
	@echo "  make clean      - Remove build artifacts"
