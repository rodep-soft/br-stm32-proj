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
target_include_directories(test_serialization PRIVATE ${PROJECT_INC_DIR} ${PROJECT_INC_DIR}/generated)
target_link_libraries(test_serialization PRIVATE microcdr m)

# 2. Comprehensive Bridge Engine test (Multi-frame, Packet Loss, HW Filters, Edge Cases)
add_executable(test_bridge_engine test_bridge_engine.c)
target_include_directories(test_bridge_engine PRIVATE ${PROJECT_INC_DIR})
target_link_libraries(test_bridge_engine PRIVATE microcdr m)

enable_testing()
add_test(NAME serialization_test COMMAND test_serialization)
add_test(NAME bridge_engine_test COMMAND test_bridge_engine)
