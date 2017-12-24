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
    GIT_REPOSITORY ${GIT_REPOSITORY}
    GIT_TAG ${GIT_TAG}
    INSTALL_DIR ${THIRD_PARTY}
    DEPENDS ${ARGN}
    CONFIGURE_COMMAND CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_STR} cmake -DCMAKE_BUILD_TYPE=Release -G${CMAKE_GENERATOR} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
    UPDATE_COMMAND ""
    BUILD_COMMAND cmake --build <BINARY_DIR>
    INSTALL_COMMAND cmake --build <BINARY_DIR> -- install)
endfunction()

Dependency(external_gflags
  "https://github.com/gflags/gflags.git"
  "v2.2.1")

Dependency(external_glog
  "https://github.com/google/glog.git"
  "v0.3.5"
  external_gflags)

Dependency(external_raspicam
  "https://github.com/cedricve/raspicam.git"
  "3c07bd9d19e55e2fc23b3a2bd642b01a4cb400ee")

include_directories( ${OpenCV_INCLUDE_DIRS} ${THIRD_PARTY}/include )
link_directories( ${THIRD_PARTY}/lib )

add_executable( kamikaze main.cc robot.cc arduinoio.cc )
target_link_libraries( kamikaze ${OpenCV_LIBS} -lpthread -lboost_system raspicam_cv gflags glog)
target_compile_features( kamikaze PRIVATE cxx_constexpr cxx_decltype_auto )
add_dependencies( kamikaze external_raspicam external_glog external_gflags )

add_executable( robot_test robot_test.cc robot.cc arduinoio.cc )
target_link_libraries( robot_test -lpthread -lboost_system raspicam_cv gflags glog)
target_compile_features( robot_test PRIVATE cxx_constexpr cxx_decltype_auto )
add_dependencies( robot_test external_gflags external_glog )

find_package( Threads )
