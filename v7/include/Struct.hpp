#ifndef STRUCT_HPP
#define STRUCT_HPP
#include <opencv2/opencv.hpp>
#include <vector>

struct LightBar {
    cv::RotatedRect rect;
    float angle;
    float length;
    float width;
    cv::Point2f center;
    cv::Point2f verctices[4];

    LightBar() = default;
    LightBar(const cv::RotatedRect& r) : rect(r) {
        angle = r.angle;
        length = std::max(r.size.width, r.size.height);
        width = std::min(r.size.width, r.size.height);
        center = r.center;
        r.points(verctices);
    }
};