cmake_minimum_required(VERSION 2.8)
project(camera-stream-callback-sample)

find_package( OpenCV REQUIRED)

###set(CMAKE_CXX_FLAGS "-fpermissive")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/osal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(GSTREAMERWEBRTC REQUIRED gstreamer-webrtc-1.0)
pkg_check_modules(SOUP REQUIRED libsoup-2.4)
pkg_check_modules(JSONGLIB REQUIRED json-glib-1.0)
pkg_check_modules(GSTREAMERSDP REQUIRED gstreamer-sdp-1.0)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DADVANCED_SENSING  -std=c++11 -fpermissive -fPIC -pthread -g -O0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DADVANCED_SENSING")



include_directories(${LIBUSB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS} ${GSTREAMERWEBRTC_INCLUDE_DIRS} ${SOUP_INCLUDE_DIRS} ${JSONGLIB_INCLUDE_DIRS} ${GSTREAMERSDP_INCLUDE_DIRS})


link_libraries(${OpenCV_LIBS} -lgstwebrtc-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lgio-2.0 -lgstsdp-1.0 -ljson-glib-1.0 -lsoup-2.4 -lusb-1.0)


add_executable(${PROJECT_NAME}
  src/gst_webrtc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/hal/osdkhal_linux.c
  ${CMAKE_CURRENT_SOURCE_DIR}/osal/osdkosal_linux.c
  ${CMAKE_CURRENT_SOURCE_DIR}/common/dji_linux_helpers.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/dji_linux_environment.cpp

)



target_link_libraries(${PROJECT_NAME} -ladvanced-sensing)
target_link_libraries(${PROJECT_NAME} -ldjiosdk-core -ldji-linker)
target_link_libraries(${PROJECT_NAME} -lgstwebrtc-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lgio-2.0 -lgstsdp-1.0 -ljson-glib-1.0 -lsoup-2.4 -lusb-1.0)
target_link_libraries(${PROJECT_NAME} -lavcodec -lavformat -lavutil -lswscale)


