# Dummy FindThreads for bare-metal ARM toolchain (FreeRTOS)
# zenoh-pico unconditionally calls find_package(Threads REQUIRED) for freertos_lwip,
# which injects -lpthreads into the linker flags. This dummy module satisfies the
# requirement while providing an empty Threads::Threads target.

set(Threads_FOUND TRUE)
set(CMAKE_THREAD_LIBS_INIT "")
set(CMAKE_USE_PTHREADS_INIT 0)
set(CMAKE_HP_PTHREADS_INIT 0)
set(CMAKE_USE_WIN32_THREADS_INIT 0)

if(NOT TARGET Threads::Threads)
  add_library(Threads::Threads INTERFACE IMPORTED)
endif()
