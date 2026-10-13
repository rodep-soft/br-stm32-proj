cmake_minimum_required(VERSION 3.15)
project(unit_tests C)

set(CMAKE_C_STANDARD 99)

# Configure Micro-CDR for host build
set(UCDR_SUPERBUILD OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(UCDR_PIC OFF CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/micro-cdr ${CMAKE_CURRENT_BINARY_DIR}/micro_cdr_build)

# Project include directory (set via -DPROJECT_INC_DIR=...)
if(NOT DEFINED PROJECT_INC_DIR)
  set(PROJECT_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc")
endif()

# 1. Serialization test
add_executable(test_serialization test_serialization.c)
target_include_directories(test_serialization PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/fixtures ${PROJECT_INC_DIR} ${PROJECT_INC_DIR}/generated)
target_link_libraries(test_serialization PRIVATE microcdr m)

# 2. Comprehensive Bridge Engine test (Multi-frame, Packet Loss, HW Filters, Edge Cases)
add_executable(test_bridge_engine test_bridge_engine.c app_config_test.c)
target_include_directories(test_bridge_engine PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/fixtures ${PROJECT_INC_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_bridge_engine PRIVATE microcdr m)

enable_testing()
add_test(NAME serialization_test COMMAND test_serialization)
add_test(NAME bridge_engine_test COMMAND test_bridge_engine)

# 3. Real firmware core logic (bridge_core.c) with sanitizers
add_executable(test_bridge_core test_bridge_core.c ${PROJECT_INC_DIR}/../Src/bridge_core.c)
target_include_directories(test_bridge_core PRIVATE ${PROJECT_INC_DIR})
target_compile_options(test_bridge_core PRIVATE -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=undefined -g)
target_link_options(test_bridge_core PRIVATE -fsanitize=address,undefined)
add_test(NAME bridge_core_test COMMAND test_bridge_core)
