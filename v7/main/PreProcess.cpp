#include "Config.hpp"
#include "Struct.hpp"
#include <opencv2/opencv.hpp>

// 图像预处理（二值化）
cv::Mat PreProcess(const cv::Mat& img, ArmorColor color) 
{
    cv::Mat HSV, gray, binary, result;

    cv::GaussianBlur(img, img, cv::Size(GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), GAUSSIAN_BLUR_SIGMA);

    cv::cvtColor(img, HSV, cv::COLOR_BGR2HSV);
    cv::split(HSV, channels);
    if (color == RED)
    {
        gray = channels.at(2) - channels.at(0);
        return gray;
    }
    else if (color == BLUE)
    {
        gray = channels.at(0) - channels.at(2);
        return gray;
    }

    cv::threshold(gray, binary, THRESHOLD_BINARY, 255, cv::THRESH_BINARY);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_SIZE, MORPH_SIZE));
    cv::morphologyEx(binary, result, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);

    cv::Mat kernel_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ELLIPSE_SIZE, ELLIPSE_SIZE));
    cv::dilate(result, result, kernel_element);
    return result;
}

// // 图像预处理 （mask掩码）
// cv::Mat preprocessImage(const cv::Mat &img) 
// {
//     // 高斯模糊和颜色空间转换
//     cv::Mat imgGaus, imgHSV;
//     cv::GaussianBlur(img, imgGaus, cv::Size(GAUSSIAN_SIZE, GAUSSIAN_SIZE), GAUSSIAN_SIGMA);
//     cv::cvtColor(imgGaus, imgHSV, cv::COLOR_BGR2HSV);

//     // 颜色阈值分割
//     cv::Mat mask1, mask2, maskRED;
//     cv::inRange(imgHSV, cv::Scalar(RED_LOWER1_H, RED_LOWER1_S, RED_LOWER1_V),
//                 cv::Scalar(RED_UPPER1_H, RED_UPPER1_S, RED_UPPER1_V), mask1);
//     cv::inRange(imgHSV, cv::Scalar(RED_LOWER2_H, RED_LOWER2_S, RED_LOWER2_V),
//                 cv::Scalar(RED_UPPER2_H, RED_UPPER2_S, RED_UPPER2_V), mask2);
//     // 合并两部分红色掩码
//     cv::bitwise_or(mask1, mask2, maskRED);

//     // 形态学操作
//     cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
//     cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_KERNEL_SIZE1, MORPH_KERNEL_SIZE1));
//     cv::Mat img_N;
//     cv::dilate(maskRED, maskRED, kernel1);
//     cv::morphologyEx(maskRED, img_N, cv::MORPH_CLOSE, kernel);
//     cv::morphologyEx(img_N, img_N, cv::MORPH_OPEN, kernel);
//     cv::dilate(img_N, img_N, kernel1);

//     return img_N;
// }