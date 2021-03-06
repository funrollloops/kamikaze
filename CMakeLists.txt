cmake_minimum_required(VERSION 3.5 )
project( kamikaze )

include(ExternalProject)

option(ENABLE_RASPICAM "Build with support for CSI-connected cameras" OFF)

macro(CMakeDependency name)
  ExternalProject_Add(${name}
    LIST_SEPARATOR |
    CMAKE_ARGS
      -Wno-dev
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
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
    -DBUILD_opencv_dnn=OFF
    -DBUILD_opencv_features2d=OFF
    -DBUILD_opencv_flann=OFF
    -DBUILD_opencv_highgui=ON
    -DBUILD_opencv_java_bindings_generator=OFF
    -DBUILD_opencv_java=OFF
    -DBUILD_opencv_ml=ON
    -DBUILD_opencv_objdetect=ON
    -DBUILD_opencv_photo=OFF
    -DBUILD_opencv_python2=OFF
    -DBUILD_opencv_python3=OFF
    -DBUILD_opencv_python_bindings_generator=OFF
    -DBUILD_opencv_shape=OFF
    -DBUILD_opencv_stitching=OFF
    -DBUILD_opencv_superres=OFF
    -DBUILD_opencv_ts=OFF
    -DBUILD_opencv_videostab=OFF
    -DWITH_CUDA=OFF
    -DWITH_CUFFT=OFF
    -DWITH_CUBLAS=OFF
    -DBUILD_SHARED_LIBS=OFF
  URL "https://github.com/opencv/opencv/archive/3.4.2.zip"
  URL_MD5 "9e9ebe9c1fe98c468f6e53f5c3c49716"
  UPDATE_COMMAND "")

CMakeDependency(gflags
  URL "https://github.com/gflags/gflags/archive/v2.2.1.zip"
  URL_MD5 "2d988ef0b50939fb50ada965dafce96b"
  UPDATE_COMMAND "")

CMakeDependency(glog
  URL "https://github.com/google/glog/archive/v0.3.5.zip"
  URL_MD5 "454766d0124951091c95bad33dafeacd"
  UPDATE_COMMAND ""
  DEPENDS gflags)

if (ENABLE_RASPICAM)
  CMakeDependency(raspicam
      -DBUILD_SHARED_LIBS=OFF
    URL "https://github.com/cedricve/raspicam/archive/3c07bd9d19e55e2fc23b3a2bd642b01a4cb400ee.zip"
    UPDATE_COMMAND ""
    DEPENDS OpenCV)
  ExternalProject_Get_Property(raspicam INSTALL_DIR)
  set(raspicam_DIR ${INSTALL_DIR}/lib/cmake)
  set(DEPEND_RASPICAM raspicam)
endif()

CMakeDependency(c-ares
    -DCARES_SHARED:BOOL=0
    -DCARES_STATIC:BOOL=1
    -DCARES_STATIC_PIC:BOOL=1
  URL "https://github.com/c-ares/c-ares/archive/cares-1_13_0.zip"
  URL_MD5 "9ff5c3db721b8450ec577f6823492ac2"
)

CMakeDependency(protobuf
    -Dprotobuf_BUILD_TESTS:BOOL=OFF
    -Dprotobuf_WITH_ZLIB:BOOL=OFF
    -Dprotobuf_MSVC_STATIC_RUNTIME:BOOL=OFF
  SOURCE_SUBDIR cmake/
  URL "https://github.com/google/protobuf/archive/v3.6.0.1.zip"
  URL_MD5 "0b33849480f5a469ba54b9ec10004e7b"
)
ExternalProject_Get_Property(protobuf INSTALL_DIR)
set(Protobuf_DIR ${INSTALL_DIR}/lib/cmake/protobuf)

# if OPENSSL_ROOT_DIR is set, propagate that hint path to the external projects with OpenSSL dependency.
set(_CMAKE_ARGS_OPENSSL_ROOT_DIR "")
if (OPENSSL_ROOT_DIR)
  set(_CMAKE_ARGS_OPENSSL_ROOT_DIR "-DOPENSSL_ROOT_DIR:PATH=${OPENSSL_ROOT_DIR}")
endif()

CMakeDependency(grpc
    -DgRPC_INSTALL:BOOL=ON
    -DgRPC_BUILD_TESTS:BOOL=OFF
    -DgRPC_PROTOBUF_PROVIDER:STRING=package
    -DgRPC_PROTOBUF_PACKAGE_TYPE:STRING=CONFIG
    -DgRPC_ZLIB_PROVIDER:STRING=package
    -DgRPC_CARES_PROVIDER:STRING=package
    -DgRPC_SSL_PROVIDER:STRING=package
    ${_CMAKE_ARGS_OPENSSL_ROOT_DIR}
  URL "https://github.com/grpc/grpc/archive/v1.13.0.zip"
  URL_MD5 "25907bbb2ce978b4434008aed98f47ea"
  DEPENDS c-ares protobuf #zlib
)


ExternalProject_Add(
  controller
  DOWNLOAD_COMMAND ""
  LIST_SEPARATOR |
  CMAKE_ARGS
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_STR}
    -DENABLE_RASPICAM=${ENABLE_RASPICAM}
    -Draspicam_DIR=${raspicam_DIR}
    -DProtobuf_DIR=${Protobuf_DIR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  INSTALL_DIR ${CMAKE_BINARY_DIR}
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/controller
  BINARY_DIR ${CMAKE_BINARY_DIR}/controller-build
  INSTALL_COMMAND ln -sf <BINARY_DIR>/kamikaze <BINARY_DIR>/robot_test <INSTALL_DIR>/
  DEPENDS OpenCV gflags glog protobuf grpc ${DEPEND_RASPICAM}
)


macro(TargetAlias subproject name)
  add_custom_target(
    ${name}
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}/${subproject}-build --target ${name}
    DEPENDS ${subproject})
endmacro()

TargetAlias(controller kamikaze)
TargetAlias(controller robot_test)
