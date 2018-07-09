project(src C CXX)

find_package( OpenCV 3.2.0 REQUIRED )
find_package( Threads REQUIRED )
find_package( gflags REQUIRED )
find_package( glog 0.3.5 REQUIRED )
find_package( raspicam REQUIRED )

# Load Protobuf
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package( Protobuf CONFIG REQUIRED )
message(STATUS "Using protobuf ${Protobuf_VERSION}")
set(PROTOC $<TARGET_FILE:protobuf::protoc>)

# Load gRPC
find_package( gRPC CONFIG REQUIRED )
message(STATUS "Using gRPC ${gRPC_VERSION}")
set(GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O2 -Wall -Werror")

add_custom_command(
  OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.pb.cc"
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.pb.h"
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.grpc.pb.cc"
    "${CMAKE_CURRENT_BINARY_DIR}/protocol.grpc.pb.h"
  COMMAND ${PROTOC}
  ARGS
    --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
    --cpp_out "${CMAKE_BINARY_DIR}"
    -I "${CMAKE_SOURCE_DIR}"
    --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN}"
    "${CMAKE_SOURCE_DIR}/protocol.proto"
  DEPENDS "${CMAKE_SOURCE_DIR}/protocol.proto"
)

include_directories( "${Protobuf_INCLUDE_DIR}" )
include_directories( "${CMAKE_CURRENT_BINARY_DIR}" )

add_library( protocol STATIC
 "${CMAKE_CURRENT_BINARY_DIR}/protocol.pb.cc"
 "${CMAKE_CURRENT_BINARY_DIR}/protocol.grpc.pb.cc" )
target_link_libraries( protocol gRPC::grpc++_unsecure gRPC::gpr ${PROTOBUF_LIBRARY} )


# Local targets
add_library( arduinoio STATIC arduinoio.cc )
target_link_libraries ( arduinoio pthread boost_system glog::glog )

add_library( robot STATIC robot.cc )
target_link_libraries( robot arduinoio gflags glog::glog )

add_library( capture STATIC capture.cc )
target_link_libraries( capture raspicam_cv glog::glog ${OpenCV_LIBS})

add_executable( kamikaze main.cc )
target_link_libraries( kamikaze ${OpenCV_LIBS} pthread robot capture gflags glog::glog)

add_executable( robot_test robot_test.cc )
target_link_libraries( robot_test robot gflags glog::glog )

add_executable( proto_server proto_server.cc )
target_link_libraries( proto_server protocol )

add_executable( proto_client proto_client.cc )
target_link_libraries( proto_client protocol )

