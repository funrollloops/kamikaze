cmake_minimum_required(VERSION 2.8)
project( kamikaze )

# Dependencies
# TODO: Fix this dependency to be auto-configured and installed

set(THIRD_PARTY ${CMAKE_BINARY_DIR}/third_party)
list(APPEND CMAKE_PREFIX_PATH ${THIRD_PARTY})
list(APPEND CMAKE_PREFIX_PATH "/home/sagarm/code/opencv/install-tree/" )
string(REPLACE ";" ":" CMAKE_PREFIX_PATH_STR "${CMAKE_PREFIX_PATH}")
find_package( OpenCV 3.2 REQUIRED )

include(ExternalProject)

function(Dependency name GIT_REPOSITORY GIT_TAG)
  ExternalProject_Add(${name}
    INSTALL_DIR ${THIRD_PARTY}
    UPDATE_COMMAND ""
    CMAKE_ARGS
      -Wno-dev
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    ${ARGN})
endfunction()

Dependency(external_gflags
  GIT_REPOSITORY "https://github.com/gflags/gflags.git"
  GIT_TAG "v2.2.1")
add_library( gflags STATIC IMPORTED )
set_target_properties( gflags PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${THIRD_PARTY}/lib/libgflags.a")
add_dependencies( gflags external_glog )

Dependency(external_glog
  GIT_REPOSITORY "https://github.com/google/glog.git"
  GIT_TAG "v0.3.5"
  DEPENDS external_gflags)
add_library( glog STATIC IMPORTED )
set_target_properties( glog PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${THIRD_PARTY}/lib/libglog.a")
add_dependencies( glog external_glog )

Dependency(external_raspicam
  GIT_REPOSITORY "https://github.com/cedricve/raspicam.git"
  GIT_TAG "3c07bd9d19e55e2fc23b3a2bd642b01a4cb400ee")
add_library( raspicam_cv SHARED IMPORTED )
set_target_properties( raspicam_cv PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${THIRD_PARTY}/lib/libraspicam_cv.so")
add_dependencies( raspicam_cv external_raspicam )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O2 -Wall -Werror")
include_directories( ${OpenCV_INCLUDE_DIRS} ${THIRD_PARTY}/include )
link_directories( ${THIRD_PARTY}/lib )

# Local targets
add_library( arduinoio STATIC arduinoio.cc )
target_link_libraries ( arduinoio pthread boost_system )

add_library( robot STATIC robot.cc )
target_link_libraries( robot arduinoio gflags glog )

add_executable( kamikaze main.cc )
target_link_libraries( kamikaze ${OpenCV_LIBS} pthread robot raspicam_cv gflags glog)

add_executable( robot_test robot_test.cc )
target_link_libraries( robot_test robot gflags glog )

find_package( Threads )
