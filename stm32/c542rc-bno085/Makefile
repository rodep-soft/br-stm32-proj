PROJECT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(PROJECT_DIR)/build/firmware
HOST_CC ?= cc

.PHONY: all build configure test
all: build

configure:
	cmake -S "$(PROJECT_DIR)/c542rc-bno085_cmake" -B "$(BUILD_DIR)" -G Ninja \
	  -DCMAKE_BUILD_TYPE=debug_GCC_NUCLEO-C542RC -DTOOLCHAIN_FILE=GCC.14.3.1.cmake \
	  -DCMAKE_PROJECT_c542rc-bno085_INCLUDE="$(PROJECT_DIR)/cmake/inject_application.cmake"

build: configure
	cmake --build "$(BUILD_DIR)" --parallel

test:
	mkdir -p "$(PROJECT_DIR)/build/tests"
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror \
	  -I"$(PROJECT_DIR)/App" -I"$(PROJECT_DIR)/Drivers/sh2/Inc" -I"$(PROJECT_DIR)/protocols" \
	  "$(PROJECT_DIR)/tests/test_imu_samples.c" "$(PROJECT_DIR)/App/imu_samples.c" \
	  "$(PROJECT_DIR)/protocols/bno085_can.c" -lm -o "$(PROJECT_DIR)/build/tests/test_imu_samples"
	"$(PROJECT_DIR)/build/tests/test_imu_samples"
	python3 "$(PROJECT_DIR)/tests/check_config.py"

	$(HOST_CC) -std=c11 -g -fsanitize=address,undefined -fno-sanitize-recover=all \
	  "$(PROJECT_DIR)/tests/test_shtp_bounds.c" -o "$(PROJECT_DIR)/build/tests/test_shtp_bounds"
	"$(PROJECT_DIR)/build/tests/test_shtp_bounds"
	$(HOST_CC) -std=c11 -g -fsanitize=address,undefined -fno-sanitize-recover=all \
	  "$(PROJECT_DIR)/tests/test_sh2_bounds.c" "$(PROJECT_DIR)/Drivers/sh2/Src/shtp.c" \
	  "$(PROJECT_DIR)/Drivers/sh2/Src/sh2_util.c" -o "$(PROJECT_DIR)/build/tests/test_sh2_bounds"
	"$(PROJECT_DIR)/build/tests/test_sh2_bounds"
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
	  -I"$(PROJECT_DIR)/tests/mocks" -I"$(PROJECT_DIR)/App" \
	  -I"$(PROJECT_DIR)/Drivers/sh2/Inc" -I"$(PROJECT_DIR)/protocols" \
	  "$(PROJECT_DIR)/tests/test_can_tx.c" -o "$(PROJECT_DIR)/build/tests/test_can_tx"
	"$(PROJECT_DIR)/build/tests/test_can_tx"
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
	  -I"$(PROJECT_DIR)/tests/mocks" -I"$(PROJECT_DIR)/App" -I"$(PROJECT_DIR)/Drivers/sh2/Inc" \
	  "$(PROJECT_DIR)/tests/test_bno_dma.c" -o "$(PROJECT_DIR)/build/tests/test_bno_dma"
	"$(PROJECT_DIR)/build/tests/test_bno_dma"
