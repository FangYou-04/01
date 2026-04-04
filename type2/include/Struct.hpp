#ifndef STRUCT_H
#define STRUCT_H

// 1. 必须添加 memset 所需的头文件
#include <cstring>
// 2. OpenCV 头文件（修正为标准路径）
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

#include <vector>


// 装甲板结构体（适配YOLO+灯条检测）
struct Armors 
{
    cv::RotatedRect boundingRect;  // 装甲板外接旋转矩形
    cv::Point2f center;            // 装甲板中心坐标
    cv::Mat rvec, tvec;            // 旋转/平移向量

    std::vector<cv::Point2f> corners; // 装甲板四个角点（PnP解算用）
    
    float confidence;             // 装甲板置信度
    int classId;             // YOLO类别（0=蓝，1=红）

    float distance;                // 相机到装甲板距离
    float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;        // 欧拉角
    
    Armors() : rvec(cv::Mat::zeros(3,1,CV_64F)), tvec(cv::Mat::zeros(3,1,CV_64F)) {} // 构造函数初始化旋转/平移向量为零矩阵
};

#endif


