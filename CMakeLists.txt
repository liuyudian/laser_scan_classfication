cmake_minimum_required(VERSION 2.8)
project(laser_scan_classfication)

# add c++11 standard
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")

# add output path
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# find opencv lib path
find_package(OpenCV 3.4 REQUIRED)
find_package(dlib REQUIRED)
include(/usr/local/lib/cmake/dlib/dlib.cmake)
# add head file
include_directories(${OpenCV_INCLUDE_DIRS} include)
message(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_LIBS})

set(THIRD_PARTY_LIBS ${OpenCV_LIBS})

############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )

add_executable(laser_scan_classfication main.cpp )
target_link_libraries( laser_scan_classfication ranging dlib::dlib)