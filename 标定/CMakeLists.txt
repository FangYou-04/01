# 指定CMake最低版本
cmake_minimum_required(VERSION 3.10)

# 项目名称（可根据你的项目修改，如ArmorDetect）
project(HikCameraCalib LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===================== 1. 查找OpenCV依赖 =====================
# 若系统未自动识别OpenCV，可手动指定OpenCV_DIR（如OpenCV安装在/usr/local/share/OpenCV）
# set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV版本: ${OpenCV_VERSION}")
    message(STATUS "OpenCV路径: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "未找到OpenCV，请先安装OpenCV")
endif()

# ===================== 2. 配置海康MVS SDK =====================
# 海康MVS SDK的安装根路径（Linux默认安装在/opt/MVS，若不同请修改）
set(MVS_SDK_PATH "/opt/MVS")
if(NOT EXISTS ${MVS_SDK_PATH})
    message(FATAL_ERROR "未找到海康MVS SDK，请检查路径: ${MVS_SDK_PATH}")
endif()

# 包含海康SDK头文件
include_directories(
    ${MVS_SDK_PATH}/include
    ${MVS_SDK_PATH}/include/GenICam
)

# 链接海康SDK库目录
link_directories(
    ${MVS_SDK_PATH}/lib/64
)

# ===================== 3. 源文件与可执行文件 =====================
# 你的主程序源文件（若有多个源文件，可添加到列表中，如src/calib.cpp src/camera.cpp）
set(SOURCE_FILES main.cpp)

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# ===================== 4. 链接依赖库 =====================
target_link_libraries(${PROJECT_NAME}
    # OpenCV库
    ${OpenCV_LIBS}
    # 海康MVS SDK库
    MvCameraControl
    # Linux系统库（线程、动态库加载等）
    pthread
    dl
)

# ===================== 5. 编译后配置（可选） =====================
# 将海康SDK的动态库拷贝到可执行文件目录（避免运行时库缺失）
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${MVS_SDK_PATH}/lib/64
    $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMENT "拷贝海康MVS SDK动态库到可执行文件目录"
)

# 打印编译信息
message(STATUS "项目编译配置完成，可执行文件名称: ${PROJECT_NAME}")
message(STATUS "可执行文件输出路径: ${CMAKE_BINARY_DIR}")