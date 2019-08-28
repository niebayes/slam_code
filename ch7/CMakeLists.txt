cmake_minimum_required(VERSION 3.0)

project(ch6)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -g -march=native")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})
