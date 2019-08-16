cmake_minimum_required(VERSION 2.8)

project(useEigen)

add_executable(eigenMatrix eigenMatrix.cpp)

include_directories("/usr/include/eigen3")
