cmake_minimum_required(VERSION 3.0)

project(useSophus)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIR})
include_directories("/usr/include/eigen3")

add_executable(useSophus useSophus.cpp)
