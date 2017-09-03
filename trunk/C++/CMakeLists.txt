cmake_minimum_required(VERSION 2.6)

project(dqrobotics)
set(CMAKE_BUILD_TYPE Release)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(dqrobotics SHARED src/DQ.cpp src/DQ_kinematics.cpp)
