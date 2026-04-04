cmake_minimum_required(VERSION 3.15)
project(Armors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

set(HIK_MVS_PATH "/opt/MVS")
include_directories(${HIK_MVS_PATH}/include)
link_directories(${HIK_MVS_PATH}/lib/64)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(main)