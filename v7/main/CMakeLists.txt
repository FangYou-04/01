# 在CMakeLists.txt中添加调试编译选项（放在add_executable之前）
# set(CMAKE_BUILD_TYPE Debug)  # 调试模式（默认Release会优化代码，掩盖行号）
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  # -g生成调试信息，-O0关闭优化


add_executable(Armor Main2.cpp draw.cpp Armors.cpp PoseSlove.cpp Config.cpp Kalman.cpp)

target_link_libraries(Armor ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} MvCameraControl pthread)
