cmake_minimum_required(VERSION 3.15)
# 限定最低cmake版本
project(dd)
# 项目命名dd
include_directories(${PROJECT_SOURCE_DIR}/include)
# 使用对于CMakeLists.txt目录对应的宏来找到头文件所在的绝对路径
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src srcfile)
# 使用对于CMakeLists.txt目录对应的宏来找到src的相对路径
add_executable(app ${srcfile})
# 根据以上资源构建app