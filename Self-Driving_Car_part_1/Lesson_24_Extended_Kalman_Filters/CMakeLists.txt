cmake_minimum_required(VERSION 3.5)
project(Lesson_24)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 14)
add_subdirectory(24.13_laser_measurements)

add_executable(Lesson_24_7 main_24.7.cpp)
#Kalman filter equation

add_executable(Lesson_24_19 main_24.19.cpp)
#Jacobian matrix part 1

add_executable(Lesson_24_23 main_24.23.cpp)
#Jacobian matrix part 1
