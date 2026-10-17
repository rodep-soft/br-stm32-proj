cmake_minimum_required(VERSION 3.22)

# Set default toolchain file before project() if not already set
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/stm32-f303k8/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "Default toolchain file")
endif()

project(br-stm32-proj C ASM)

add_subdirectory(stm32-f303k8)
