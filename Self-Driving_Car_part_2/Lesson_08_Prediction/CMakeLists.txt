cmake_minimum_required(VERSION 3.5)
project(Lesson_8)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(Lesson_8
        main.cpp
        classifier.cpp
        )

target_compile_definitions(Lesson_8 PUBLIC DATA_FOLDER="${CMAKE_SOURCE_DIR}/data")

