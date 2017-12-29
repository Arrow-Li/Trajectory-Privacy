cmake_minimum_required(VERSION 3.5)
project(Work)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES Graph.h Matrix.h main.cpp en_tra.h Trajectory.h)
add_executable(Work ${SOURCE_FILES})