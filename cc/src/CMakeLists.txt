find_package( OpenCV 3.2.0 REQUIRED )
find_package( Threads REQUIRED )
find_package( gflags REQUIRED )
find_package( glog 0.3.5 REQUIRED )
find_package( raspicam REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O2 -Wall -Werror")

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

