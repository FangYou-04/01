#ifndef STRUCT_H
#define STRUCT_H


#include <opencv4/opencv2/opencv.hpp>

struct Light
{
    cv::RotatedRect rect;
    float rat;
    float AbsAngle;
    cv::Point2f center;
    cv::Point2f vertices[4];

    Light() 
    {
        memset(vertices, 0, sizeof(vertices));
    }
};

#endif