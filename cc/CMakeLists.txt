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
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    ${ARGN})
endfunction()

Dependency(external_gflags
  GIT_REPOSITORY "https://github.com/gflags/gflags.git"
  GIT_TAG "v2.2.1")

Dependency(external_glog
  GIT_REPOSITORY "https://github.com/google/glog.git"
  GIT_TAG "v0.3.5"
  DEPENDS external_gflags)

Dependency(external_raspicam
  GIT_REPOSITORY "https://github.com/cedricve/raspicam.git"
  GIT_TAG "3c07bd9d19e55e2fc23b3a2bd642b01a4cb400ee")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O2 -Wall -Werror")
include_directories( ${OpenCV_INCLUDE_DIRS} ${THIRD_PARTY}/include )
link_directories( ${THIRD_PARTY}/lib )

# Local targets
add_library( arduinoio STATIC arduinoio.cc )
target_link_libraries ( arduinoio pthread boost_system )

add_library( robot STATIC robot.cc )
add_dependencies( robot external_gflags external_glog )
target_link_libraries( robot arduinoio gflags glog )

add_executable( kamikaze main.cc )
target_link_libraries( kamikaze ${OpenCV_LIBS} pthread robot raspicam_cv gflags glog)
add_dependencies( kamikaze external_raspicam external_glog external_gflags )

add_executable( robot_test robot_test.cc )
target_link_libraries( robot_test robot gflags glog )
add_dependencies( robot_test external_gflags external_glog )

find_package( Threads )
