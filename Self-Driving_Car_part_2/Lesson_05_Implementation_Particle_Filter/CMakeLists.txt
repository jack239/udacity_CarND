cmake_minimum_required(VERSION 3.5)
project(Lesson_05)

set(CMAKE_CXX_STANDARD 14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wpedantic")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(Lesson_05 main_5.05.cpp)
# normal_distribution

add_executable(Lesson_16 main_5.16.cpp)
# normal_distribution