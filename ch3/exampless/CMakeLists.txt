cmake_minimum_required(VERSION 3.0)

project(examples)

add_executable(examples coordTransform.cpp plotTraj.cpp)

include_directories("/usr/include/eigen3")
