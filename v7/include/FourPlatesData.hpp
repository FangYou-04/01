#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

enum class PlateID {
    FRONT = 0,
    BACK = 1,
    LEFT = 2,
    RIGHT = 3
};

struct FourPlatesData
{
    PlateID id;
    // 相机坐标系下的三维位置
    cv::Point3f position3D; 
    // 投影后角点
    std::vector<cv::Point2f> corners2D;
    // 是否在视野范围内
    bool in_view;
    // 距离
    float distance;
    // yaw
    float yaw;
    // pitch
    float pitch;
};

using FourPlates = std::vector<FourPlatesData>;
