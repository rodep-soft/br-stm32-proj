cmake_minimum_required(VERSION 3.15)
project(unit_tests C)

set(CMAKE_C_STANDARD 99)

# Configure Micro-CDR for host build
set(UCDR_SUPERBUILD OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(UCDR_PIC OFF CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/micro-cdr ${CMAKE_CURRENT_BINARY_DIR}/micro_cdr_build)

add_executable(test_serialization test_serialization.c)
target_include_directories(test_serialization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc)
target_link_libraries(test_serialization PRIVATE microcdr m)

enable_testing()
add_test(NAME serialization_test COMMAND test_serialization)
