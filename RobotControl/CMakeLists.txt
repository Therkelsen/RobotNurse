cmake_minimum_required(VERSION 3.23)
project(RobotControl)

set(CMAKE_CXX_STANDARD 23)

add_executable(RobotControl main.cpp MotionControl.cpp MotionControl.h)
