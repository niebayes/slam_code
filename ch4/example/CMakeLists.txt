cmake_minimum_required(VERSION 3.0)

project(trajError)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIR})
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIR})
include_directories("/usr/include/eigen3")

add_executable(trajError trajError.cpp)
target_link_libraries(trajError ${Pangolin_LIBRARIES})
target_link_libraries(trajError ${Sophus_LIBRARIES})
