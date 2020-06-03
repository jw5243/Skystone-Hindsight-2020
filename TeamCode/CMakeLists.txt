cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/src/main/jniLibs/opencv_contrib-master/modules")
include_directories(${CMAKE_SOURCE_DIR}/src/main/jniIncludes)

add_library(native-lib
            SHARED
            src/main/cpp/native-lib.cpp
            src/main/cpp/VisionUtil.cpp)

add_library(lib_opencv SHARED IMPORTED)

set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_java4.so)

find_library(log-lib
             log)

target_link_libraries(native-lib ${log-lib} lib_opencv)