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
add_compile_options(-Wno-error=incompatible-pointer-types -Wno-incompatible-pointer-types)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME br-stm32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Automatic Micro-CDR message header generation from ROS 2 .msg files
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_FOUND)
    set(MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test_msgs/msg")
    set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/generated")
    file(GLOB MSG_FILES "${MSG_DIR}/*.msg")
    
    add_custom_target(generate_messages
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tools/msg2cdr.py"
                --package can_msgs
                --msg-dir "${MSG_DIR}"
                --out-dir "${GEN_DIR}"
        DEPENDS ${MSG_FILES} "${CMAKE_CURRENT_SOURCE_DIR}/tools/msg2cdr.py"
        COMMENT "Generating Micro-CDR C headers from ROS 2 .msg files"
    )
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

if(TARGET generate_messages)
    add_dependencies(${CMAKE_PROJECT_NAME} generate_messages)
endif()

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
)


# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Use custom FindThreads module to prevent linking -lpthreads in bare-metal toolchains
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# Configure zenoh-pico for FreeRTOS + LwIP
set(ZP_PLATFORM "freertos_lwip" CACHE STRING "Zenoh-Pico platform" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static lib" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable testing" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
set(CHECK_THREADS OFF CACHE BOOL "Disable check threads" FORCE)

add_subdirectory(third_party/zenoh-pico)
target_link_libraries(zenohpico_static PUBLIC stm32cubemx)

# Configure Micro-CDR
set(UCDR_SUPERBUILD OFF CACHE BOOL "Disable superbuild" FORCE)
set(UCDR_BUILD_TESTS OFF CACHE BOOL "Disable tests" FORCE)
set(UCDR_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
set(UCDR_PIC OFF CACHE BOOL "Disable PIC" FORCE)
set(UCDR_ISOLATED_INSTALL OFF CACHE BOOL "Disable isolated install" FORCE)

add_subdirectory(third_party/micro-cdr)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/app_zenoh.c
    Core/Src/zenoh_ros2.c
    Core/Src/can_bridge.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob pthreads pthread)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    zenohpico_static
    microcdr
    # Add user defined libraries
)
