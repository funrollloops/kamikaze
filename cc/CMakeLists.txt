cmake_minimum_required(VERSION 3.5 )
project( kamikaze )

include(ExternalProject)

macro(CMakeDependency name)
  ExternalProject_Add(${name}
    INSTALL_DIR ${THIRD_PARTY}
    LIST_SEPARATOR |
    CMAKE_ARGS
      -Wno-dev
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_STR}
    ${ARGN})
  ExternalProject_Get_Property(${name} INSTALL_DIR)
  list(APPEND CMAKE_PREFIX_PATH ${INSTALL_DIR})
  string(REPLACE ";" "|" CMAKE_PREFIX_PATH_STR "${CMAKE_PREFIX_PATH}")
endmacro()

# TODO: TBB can provide a speed boost. Figure out why BUILD/WITH_TBB=on doesnt
# work or find a workaround.
CMakeDependency(OpenCV
    -DBUILD_TBB=OFF
    -DWITH_TIFF=OFF
    -DWITH_IPP=OFF
    -DWITH_FFMPEG=ON
    -DWITH_WEBP=OFF
    -DWITH_TBB=OFF
    -DWITH_V4L=ON
    -DWITH_QT=ON
    -DWITH_OPENGL=ON
    -DENABLE_FAST_MATH=1
    -DBUILD_JAVA=OFF
    -DBUILD_DOCS=OFF
    -DBUILD_opencv_calib3d=OFF
    -DBUILD_opencv_flann=OFF
    -DBUILD_opencv_highgui=ON
    -DBUILD_opencv_java=OFF
    -DBUILD_opencv_ml=ON
    -DBUILD_opencv_objdetect=ON
    -DBUILD_opencv_photo=OFF
    -DBUILD_opencv_python2=OFF
    -DBUILD_opencv_python3=OFF
    -DBUILD_opencv_shape=OFF
    -DBUILD_opencv_stitching=OFF
    -DBUILD_opencv_superres=OFF
    -DBUILD_opencv_ts=OFF
    -DBUILD_opencv_videostab=OFF
    -DWITH_CUDA=OFF
    -DWITH_CUFFT=OFF
    -DWITH_CUBLAS=OFF
  URL "https://github.com/opencv/opencv/archive/3.2.0.tar.gz"
  UPDATE_COMMAND ""
  URL_MD5 1ea44a1d98c126ad40079d8eb914a72e)

CMakeDependency(gflags
  URL "https://github.com/gflags/gflags/archive/v2.2.1.zip"
  UPDATE_COMMAND "")

CMakeDependency(glog
  URL "https://github.com/google/glog/archive/v0.3.5.zip"
  UPDATE_COMMAND ""
  DEPENDS gflags)

CMakeDependency(raspicam
  URL "https://github.com/cedricve/raspicam/archive/3c07bd9d19e55e2fc23b3a2bd642b01a4cb400ee.zip"
  UPDATE_COMMAND ""
  DEPENDS OpenCV)
ExternalProject_Get_Property(raspicam INSTALL_DIR)
set(raspicam_DIR ${INSTALL_DIR}/lib/cmake)

ExternalProject_Add(
  src
  DOWNLOAD_COMMAND ""
  LIST_SEPARATOR |
  CMAKE_ARGS
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_STR}
    -Draspicam_DIR=${raspicam_DIR}
  INSTALL_DIR ${CMAKE_BINARY_DIR}
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/src
  BINARY_DIR ${CMAKE_BINARY_DIR}/src-build
  INSTALL_COMMAND ln -sf <BINARY_DIR>/kamikaze <BINARY_DIR>/robot_test <INSTALL_DIR>/
  OUTPUT kamikaze robot_test
  DEPENDS OpenCV gflags glog raspicam)
