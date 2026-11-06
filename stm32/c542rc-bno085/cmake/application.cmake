get_filename_component(BNO_APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_target_property(firmware_sources c542rc-bno085 SOURCES)
list(FILTER firmware_sources EXCLUDE REGEX "(^|/)main\\.[ch]$")
set_property(TARGET c542rc-bno085 PROPERTY SOURCES "${firmware_sources}")
target_sources(c542rc-bno085 PRIVATE
  ${BNO_APP_ROOT}/App/main.c
  ${BNO_APP_ROOT}/App/dma_io.c
  ${BNO_APP_ROOT}/App/watchdog.c
  ${BNO_APP_ROOT}/App/can_tx.c
  ${BNO_APP_ROOT}/App/bno_transport.c
  ${BNO_APP_ROOT}/App/imu_samples.c
  ${BNO_APP_ROOT}/Drivers/sh2/Src/sh2.c
  ${BNO_APP_ROOT}/Drivers/sh2/Src/sh2_SensorValue.c
  ${BNO_APP_ROOT}/Drivers/sh2/Src/sh2_util.c
  ${BNO_APP_ROOT}/Drivers/sh2/Src/shtp.c
  ${BNO_APP_ROOT}/protocols/bno085_can.c)
target_include_directories(c542rc-bno085 PRIVATE
  ${BNO_APP_ROOT}/App ${BNO_APP_ROOT}/Drivers/sh2/Inc ${BNO_APP_ROOT}/protocols)
add_custom_command(TARGET c542rc-bno085 POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:c542rc-bno085> ${CMAKE_BINARY_DIR}/c542rc-bno085.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:c542rc-bno085> ${CMAKE_BINARY_DIR}/c542rc-bno085.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:c542rc-bno085>
  VERBATIM)

# Apply to HAL and application alike; never edit generated configuration.
target_compile_options(c542rc-bno085 PRIVATE
  "$<$<COMPILE_LANGUAGE:C,CXX>:-include${BNO_APP_ROOT}/App/app_hal_config.h>")

option(BNO_HARDWARE_SELFTEST "Inject recoverable faults on the bench; never enable for normal operation" OFF)
if(BNO_HARDWARE_SELFTEST)
  target_compile_definitions(c542rc-bno085 PRIVATE BNO_HARDWARE_SELFTEST=1)
endif()
