cmake_minimum_required(VERSION 3.8)
project(armor_pose_solver)

# 关闭 Python 干扰
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

# C++ 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 强制使用 C++17（EKF 和 YOLO 需要）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找 OpenCV（必须包含 dnn 和 calib3d）
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc videoio imgcodecs calib3d dnn)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "未找到系统 OpenCV 4!请执行: sudo apt install libopencv-dev")
else()
  message(STATUS "找到系统 OpenCV: ${OpenCV_VERSION}")
endif()

# 2. 查找 ROS 2 核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 3. 可选：海康 MVS SDK（如果存在则启用）
set(MVS_DIR "/opt/MVS")
if(EXISTS "${MVS_DIR}/lib/libMvCameraControl.so")
  message(STATUS "发现海康 MVS SDK,将启用海康相机支持")
  include_directories(${MVS_DIR}/include)
  link_directories(${MVS_DIR}/lib)
  set(HIK_FOUND TRUE)
else()
  message(STATUS "未找到海康 MVS SDK,禁用海康相机支持")
endif()

# 4. 头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS}
)

# 5. 源文件列表（整合新旧模块）
set(SOURCES
  src/Main1.cpp                # 主程序（整合后的 YOLO+EKF+跟踪）
  src/Yolo1.cpp                # YOLO 检测器
  src/PoseSolve.cpp           # 位姿解算（从 YOLO 框解算装甲板位姿）
  src/ExtendedKalman.cpp      # 扩展卡尔曼滤波器
  src/Serial.cpp              # 串口通信（可选，保留原功能）
  src/Config.cpp              # 配置文件解析（从 YAML 加载参数）
)

# 如果启用海康相机，添加 HikCamera.cpp
if(HIK_FOUND)
  list(APPEND SOURCES src/HikCamera.cpp)
endif()

# 可选：原项目中的旧文件（Kalman.cpp, Draw.cpp）不再使用，故不添加
# 如果需要保留旧版兼容，可取消注释下一行
# list(APPEND SOURCES src/Kalman.cpp src/Draw.cpp)

# 6. 生成可执行文件
add_executable(armor_pose_node ${SOURCES})

# 7. 链接库
target_link_libraries(armor_pose_node
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 如果启用海康，链接 MVS 库
if(HIK_FOUND)
  target_link_libraries(armor_pose_node
    ${MVS_DIR}/lib/libMvCameraControl.so
    pthread
    dl
  )
endif()

# 8. 安装规则
install(TARGETS armor_pose_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 9. 测试（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 10. 导出包
ament_package()