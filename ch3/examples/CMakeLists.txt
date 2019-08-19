cmake_minimum_required(VERSION 3.0)
project(examples)

include_directories("/usr/include/eigen3")
add_executable(coordTransform coordTransform.cpp plotTraj.cpp)

find_package(Pangolin REQUIRED)
include_directories(${PANGOLIN_INCLUDE_DIR})
add_executable(plotTraj plotTraj.cpp)
target_link_libraries(plotTraj ${PANGOLIN_LIBRARIES})
