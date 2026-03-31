# 在CMakeLists.txt中添加调试编译选项（放在add_executable之前）
# set(CMAKE_BUILD_TYPE Debug)  # 调试模式（默认Release会优化代码，掩盖行号）
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  # -g生成调试信息，-O0关闭优化


# add_executable(Armor Main1.cpp draw.cpp Armors.cpp PoseSlove.cpp Config.cpp Kalman.cpp) # build

# add_executable(Armor Main2.cpp Armors.cpp PoseSlove.cpp Config.cpp Kalman.cpp) # build1

# add_executable(Armor Main3.cpp Armors.cpp PoseSlove.cpp Config.cpp Kalman.cpp DrawTrack.cpp HikCamera.cpp) # build2

# add_executable(Armor Main3.1.cpp Armors.cpp PoseSlove.cpp Config.cpp KalmanTracker.cpp DrawTracke1.cpp HikCamera.cpp ) # build3

add_executable(Armor Main4.cpp Armors.cpp PoseSlove.cpp Config.cpp ExtendedKalman.cpp DrawTracke2.cpp HikCamera.cpp) # build4

target_link_libraries(Armor ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} MvCameraControl pthread)
