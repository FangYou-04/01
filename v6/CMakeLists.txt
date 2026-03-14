cmake_minimum_required(VERSION 3.15)
project(Armors)

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

add_subdirectory(main)