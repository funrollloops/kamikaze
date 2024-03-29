cmake_minimum_required( VERSION 3.5 )
project(src C CXX)

option(ENABLE_RASPICAM "Build with support for CSI-connected cameras" OFF)

find_package( Boost 1.79.0 REQUIRED )
find_package( OpenCV 3.2.0 REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Threads REQUIRED )
find_package( gRPC REQUIRED )
find_package( gflags REQUIRED )
find_package( glog 0.3.5 REQUIRED )

if (ENABLE_RASPICAM)
  find_package( raspicam REQUIRED )
endif()

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O2 -Wall -Werror")
add_custom_command(
  OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.pb.cc"
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.pb.h"
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.grpc.pb.cc"
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.grpc.pb.h"
  COMMAND protobuf::protoc
  ARGS
    --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
    --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
    -I "${CMAKE_CURRENT_SOURCE_DIR}"
    --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN}"
    "${CMAKE_CURRENT_SOURCE_DIR}/protocol.proto"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/protocol.proto"
)

add_library( protocol
 "${CMAKE_CURRENT_BINARY_DIR}/protocol.pb.cc"
 "${CMAKE_CURRENT_BINARY_DIR}/protocol.grpc.pb.cc" )
target_link_libraries( protocol PUBLIC gRPC::grpc++_unsecure gRPC::gpr
    protobuf::libprotobuf )
target_include_directories( protocol PUBLIC "${CMAKE_CURRENT_BINARY_DIR}" )


# Local targets
add_library( arduinoio arduinoio.h arduinoio.cc )
target_link_libraries ( arduinoio pthread boost_system glog::glog )

add_library( robot robot.cc )
target_link_libraries( robot arduinoio glog::glog )

add_library( robot_proxy_wrapper robot_proxy_wrapper.cc )
target_link_libraries( robot_proxy_wrapper protocol glog::glog robot )

add_library( robot_from_flags robot_from_flags.cc )
target_link_libraries( robot_from_flags robot robot_proxy_wrapper gflags glog::glog )

add_library( capture capture.cc )
target_link_libraries( capture glog::glog opencv::opencv_videoio )
if (ENABLE_RASPICAM)
  target_link_libraries( capture raspicam_cv )
  target_compile_definitions( capture PUBLIC ENABLE_RASPICAM=1 )
endif()

add_executable( kamikaze main.cc )
target_link_libraries( kamikaze opencv::opencv_core opencv::opencv_highgui
    opencv::objdetect pthread robot_from_flags capture gflags glog::glog)

add_executable( robot_test robot_test.cc )
target_link_libraries( robot_test robot_from_flags gflags glog::glog )

add_executable( robot_proxy_server robot_proxy_server.cc )
target_link_libraries( robot_proxy_server protocol robot_from_flags )
