#pragma once
#include <opencv2/opencv.hpp>
#include "FourPlatesData.hpp"
#include <vector>
// 绘制
void DrawFour(cv::Mat& img,
              const Armor& detected_armor,
              const cv::Point3f& predicted_position,
              double predict_yaw,
              const cv::Mat& last_vaild_rvec,
              const cv::Mat& camera_martix,
              const cv::Mat& dist_coeffs,
              FourPlatesData* outplates = nullptr);