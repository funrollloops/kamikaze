cmake_minimum_required(VERSION 3.5 )
project( kamikaze )

option(ENABLE_RASPICAM "Build with support for CSI-connected cameras" OFF)

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
add_subdirectory(controller)
