cmake_minimum_required(VERSION 3.23)
project(RobotControl)

set(CMAKE_CXX_STANDARD 23)

find_package(ur_rtde REQUIRED)

add_executable(RobotControl main.cpp MotionControl.cpp MotionControl.h)

target_link_libraries(RobotControl PRIVATE ur_rtde::rtde)