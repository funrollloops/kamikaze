cmake_minimum_required(VERSION 2.8)

# TODO: switch to avr-cmake
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake) # Arduino Toolchain
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O2" )

project( kamikaze-slave C CXX)
set(FIRMWARE_NAME kamikaze-slave-firmware )

set(${FIRMWARE_NAME}_BOARD pro328)               # Arduino Target board
set(${FIRMWARE_NAME}_SRCS  kamikaze.cc)

set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)            # Serial upload port
link_directories(${CMAKE_SOURCE_DIR}/../arduino)
set(${FIRMWARE_NAME}_NO_AUTOLIBS false)

generate_arduino_firmware(${FIRMWARE_NAME})
target_compile_options(${FIRMWARE_NAME} PRIVATE -Wall -pedantic -Werror -std=c++1y -O2 )

add_custom_target(
  upload-spi
  COMMAND sudo avrdude -p atmega328p -C +${CMAKE_SOURCE_DIR}/avrdude.conf -c pi_spi -v  -P /dev/spidev0.0 -U flash:w:${CMAKE_BINARY_DIR}/kamikaze-slave-firmware.hex -b 200000
  DEPENDS ${FIRMWARE_NAME}
)

add_custom_command(
  OUTPUT spi_master_example
  COMMAND gcc -o spi_master_example ${CMAKE_SOURCE_DIR}/examples/spi_master_example.c
)

add_custom_target(
  run-spi-test
  COMMAND ${CMAKE_BINARY_DIR}/spi_master_example -D /dev/spidev0.0 -s 100000
  DEPENDS spi_master_example
)
