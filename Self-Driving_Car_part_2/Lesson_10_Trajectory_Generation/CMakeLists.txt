cmake_minimum_required(VERSION 3.5)
project(Lesson_10)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(Lesson_10.14_hybrid_a_star)
add_subdirectory(Lesson_10.27_jerk_min)
