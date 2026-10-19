cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32-f303k8)

# Set default toolchain file before project() if not already set
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "Default toolchain file")
endif()

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/bno08x/Src/sh2_hal_stm32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/bno08x/Src/bno08x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/sh2/Src/euler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/sh2/Src/sh2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/sh2/Src/sh2_SensorValue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/sh2/Src/sh2_util.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/sh2/Src/shtp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/protocols/bno085_can.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/bno08x/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/sh2/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/protocols
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Enable float formatting in printf
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

