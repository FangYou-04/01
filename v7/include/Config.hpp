#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <opencv2/opencv.hpp>
#include <string>

// 定义颜色枚举
enum class ArmorColor {
    RED,
    BLUE
};

// 相机内参矩阵 (示例值，请根据实际相机参数进行修改)
const cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) << 
    1000.0, 0.0, 640.0,
    0.0, 1000.0, 360.0,
    0.0, 0.0, 1.0);
// 相机畸变系数 (示例值，请根据实际相机参数进行修改)
const cv::Mat DIST_COEFFS = (cv::Mat_<double>(1, 5) << 
    0.1, -0.25, 0.0, 0.0, 0.1);

// 装甲板实际尺寸 (单位：毫米)
const float SMALL_ARMOR_WIDTH = 140.0f;
const float SMALL_ARMOR_HEIGHT = 55.0f;
const float LARGE_ARMOR_WIDTH = 240.0f;
const float LARGE_ARMOR_HEIGHT = 55.0f;

// 预处理参数
const int GAUSSIAN_BLUR_SIZE = 7;
const float GAUSSIAN_BLUR_SIGMA = 1.5;
const int MORPH_SIZE = 3;
const int ELLIPSE_SIZE = 5;
const int THRESHOLD_BINARY = 230;


// 灯条检测参数
const float MIN_LIGHTBAR_AREA = 70.0;
const float MAX_LIGHTBAR_ANGLE = 30.0f; // 最大倾斜角
const float MIN_LIGHTBAR_RATIO = 2.0f; // 最小长宽比
const float MAX_LIGHTBAR_RATIO = 8.0f; // 最大长宽比

// 装甲板匹配参数
const float MAX_LIGHT_ANGLE_DIFF = 10.0f; // 最大灯条角度差
const float MAX_LIGHT_LENGTH_DIFF_RATIO = 0.2f; // 最大灯条长度差比例
const float ARMOR_ANGLE_MAX = 30.0f; // 装甲板最大倾斜角
const float DISTANCE_MAX = 10.0f; // 最大距离
const float DISTANCE_MIN = 3.0f; // 最小距离

// hsv阈值参数
const cv::Scalar RED_LOWER1 = cv::Scalar(0, 43, 46);
const cv::Scalar RED_UPPER1 = cv::Scalar(10, 255, 255);
const cv::Scalar RED_LOWER2 = cv::Scalar(156, 43, 46);
const cv::Scalar RED_UPPER2 = cv::Scalar(179, 255, 255);
const cv::Scalar BLUE_LOWER = cv::Scalar(100, 150, 0);
const cv::Scalar BLUE_UPPER = cv::Scalar(140, 255, 255);

#endif // CONFIG_HPP